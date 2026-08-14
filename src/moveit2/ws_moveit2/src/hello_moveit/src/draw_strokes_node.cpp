/*
 * draw_strokes_node.cpp
 */

#include <memory>
#include <thread>
#include <mutex>
#include <map>
#include <string>
#include <vector>
#include <algorithm>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit_msgs/msg/joint_limits.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_trajectory/robot_trajectory.hpp>
#include <moveit/trajectory_processing/time_optimal_trajectory_generation.hpp>

#include <vision_interfaces/msg/stroke.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

struct Px { double u, v; };

struct PartData {
    std::string instance_label;
    std::vector<Px> points;
};

struct SegResult { double fraction; size_t points; double duration; bool executed; };

// ============================================================
// 카터시안 실행, 안전성 보충, RViz 시각화 헬퍼 함수
// ============================================================
static SegResult runCartesian(
    moveit::planning_interface::MoveGroupInterface & mg,
    moveit_visual_tools::MoveItVisualTools & visual_tools,
    const std::vector<geometry_msgs::msg::Pose> & wps,
    double eef_step, double vel, double acc, bool execute,
    const rclcpp::Logger & log, const std::string & label)
{
    moveit_msgs::msg::RobotTrajectory traj;
    
    // Joint Jump 방지를 위해 jump_threshold = 0.0 설정
    const double jump_threshold = 0.0;
    const double fraction = mg.computeCartesianPath(wps, eef_step, jump_threshold, traj);

    SegResult r{fraction, traj.joint_trajectory.points.size(), 0.0, false};
    if (fraction < 0.95) {
        RCLCPP_ERROR(log, "[%s] 달성률 %.1f%% — 경로가 끊겼습니다.", label.c_str(), fraction * 100.0);
        return r;
    }

    auto robot_model = mg.getRobotModel();
    robot_trajectory::RobotTrajectory rt(robot_model, mg.getName());
    rt.setRobotTrajectoryMsg(*mg.getCurrentState(), traj);

    // joint_limits 보충 로직
    const auto & joint_models = robot_model->getActiveJointModels();
    for (const auto * jm : joint_models) {
        auto bounds = jm->getVariableBounds();
        if (!bounds.empty()) {
            bool need_update = false;
            if (!bounds[0].velocity_bounded_ || bounds[0].max_velocity_ <= 0.0) {
                bounds[0].velocity_bounded_ = true;
                bounds[0].min_velocity_ = -3.14;
                bounds[0].max_velocity_ = 3.14;
                need_update = true;
            }
            if (!bounds[0].acceleration_bounded_ || bounds[0].max_acceleration_ <= 0.0) {
                bounds[0].acceleration_bounded_ = true;
                bounds[0].min_acceleration_ = -2.0;
                bounds[0].max_acceleration_ = 2.0;
                need_update = true;
            }
            if (need_update) {
                moveit_msgs::msg::JointLimits lim;
                lim.joint_name = jm->getName();
                lim.has_velocity_limits = bounds[0].velocity_bounded_;
                lim.max_velocity = bounds[0].max_velocity_;
                lim.has_acceleration_limits = bounds[0].acceleration_bounded_;
                lim.max_acceleration = bounds[0].max_acceleration_;

                std::vector<moveit_msgs::msg::JointLimits> lim_vec = {lim};
                const_cast<moveit::core::JointModel*>(jm)->setVariableBounds(lim_vec);
            }
        }
    }

    trajectory_processing::TimeOptimalTrajectoryGeneration totg;
    if (!totg.computeTimeStamps(rt, vel, acc)) {
        RCLCPP_ERROR(log, "[%s] 시간 매개변수화 실패", label.c_str());
        return r;
    }
    rt.getRobotTrajectoryMsg(traj);

    const auto jmg = robot_model->getJointModelGroup(mg.getName());
    const auto ee_link = robot_model->getLinkModel(mg.getEndEffectorLink());
    if (jmg && ee_link) {
        visual_tools.publishTrajectoryLine(traj, ee_link, jmg);
        visual_tools.trigger();
    }

    r.points = traj.joint_trajectory.points.size();
    r.duration = rclcpp::Duration(traj.joint_trajectory.points.back().time_from_start).seconds();

    if (!execute) {
        RCLCPP_INFO(log, "[%s] 계획만 — 구간 %zu개 · %.1f초", label.c_str(), r.points, r.duration);
        return r;
    }
    RCLCPP_INFO(log, "[%s] 실행 — 구간 %zu개 · %.1f초", label.c_str(), r.points, r.duration);
    r.executed = (mg.execute(traj) == moveit::core::MoveItErrorCode::SUCCESS);
    if (!r.executed) RCLCPP_ERROR(log, "[%s] 실행 실패", label.c_str());
    return r;
}

// ============================================================
// 노드 클래스
// ============================================================
class DrawStrokesNode : public rclcpp::Node
{
public:
    DrawStrokesNode() 
    : Node("draw_strokes_node", rclcpp::NodeOptions().automatically_declare_parameters_from_overrides(true))
    {
        rclcpp::QoS qos(20);
        qos.reliable();
        qos.transient_local();

        sub_ = this->create_subscription<vision_interfaces::msg::Stroke>(
            "/vision/strokes", qos,
            std::bind(&DrawStrokesNode::on_stroke, this, std::placeholders::_1));

        this->get_parameter_or("draw_size", draw_size_, 0.15);
        this->get_parameter_or("eef_step", eef_step_, 0.002);
        this->get_parameter_or("hover", hover_, 0.03);
        this->get_parameter_or("vel_scale", vel_, 0.1);
        this->get_parameter_or("acc_scale", acc_, 0.1);
        this->get_parameter_or("execute", execute_, true);
        this->get_parameter_or("go_home", go_home_, true);

        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/hcr5_examples/target_shape", rclcpp::QoS(1).transient_local().reliable());

        RCLCPP_INFO(this->get_logger(), "draw_strokes_node 시작. /vision/strokes 데이터 수신 대기 중...");
    }

    void wait_and_draw()
    {
        rclcpp::Rate rate(10);

        // 1. 첫 번째 스트로크 데이터가 들어올 때까지 대기
        RCLCPP_INFO(this->get_logger(), "첫 번째 스트로크 데이터 대기 중...");
        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (!parts_.empty()) break;
            }
            rate.sleep();
        }

        RCLCPP_INFO(this->get_logger(), "데이터 수신 시작됨. 추가 데이터 수신을 관찰합니다.");

        // 2. 데이터 수신 후, "마지막 데이터 수신 시점"으로부터 5초간 새 데이터가 없으면 수집 완료 처리
        auto last_received_time = this->now();
        size_t last_count = 0;

        while (rclcpp::ok()) {
            size_t current_count = 0;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                current_count = parts_.size();
            }

            // 새로운 스트로크가 추가 수신된 경우 -> 타이머 리셋
            if (current_count > last_count) {
                last_count = current_count;
                last_received_time = this->now();
                RCLCPP_INFO(this->get_logger(), "새 스트로크 수신됨 (현재 총 %zu개). 5초 대기 타이머 재설정.", last_count);
            }

            // 마지막 수신으로부터 5.0초 경과 시 수집 완료
            double elapsed = (this->now() - last_received_time).seconds();
            if (elapsed >= 5.0) {
                RCLCPP_INFO(this->get_logger(), "5초 동안 추가 데이터 없음. 총 %zu개 스트로크 수집 완료!", last_count);
                break;
            }

            rate.sleep();
        }

        std::vector<PartData> parts;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            for (auto & kv : parts_) parts.push_back(kv.second);
        }

        if (parts.empty()) {
            RCLCPP_ERROR(this->get_logger(), "수집된 스트로크가 없습니다.");
            return;
        }

        draw(parts);
    }

private:
    void on_stroke(const vision_interfaces::msg::Stroke::SharedPtr msg)
    {
        PartData pd;
        pd.instance_label = msg->instance_label;
        pd.points.reserve(msg->points.size());
        for (const auto & pt : msg->points) {
            pd.points.push_back({static_cast<double>(pt.u), static_cast<double>(pt.v)});
        }

        {
            std::lock_guard<std::mutex> lock(mutex_);
            parts_[pd.instance_label] = pd;
        }
        RCLCPP_INFO(this->get_logger(), "  받음: %s (점 %zu개)", pd.instance_label.c_str(), pd.points.size());
    }

    // 이미지 Pixel 좌표(u, v) -> 로봇 Base Cartesian 좌표(X, Y, Z) 변환
    geometry_msgs::msg::Pose toPose(const Px & p, double dz,
                                     const geometry_msgs::msg::Pose & origin,
                                     double u_c, double v_c, double s)
    {
        geometry_msgs::msg::Pose q = origin;
        // 이미지 u(가로) -> 로봇 Y축(좌우)
        // 이미지 v(세로) -> 로봇 X축(전후) : 이미지 v는 아래로 증가하므로 - 부호 적용
        q.position.x = origin.position.x - (p.v - v_c) * s;
        q.position.y = origin.position.y + (p.u - u_c) * s;
        q.position.z = origin.position.z + dz;
        return q;
    }

    void draw(const std::vector<PartData> & parts)
    {
        using moveit::planning_interface::MoveGroupInterface;
        MoveGroupInterface mg(shared_from_this(), "hcr_arm");

        // 기준 프레임 명시적 설정
        mg.setPoseReferenceFrame(mg.getPlanningFrame());

        moveit_visual_tools::MoveItVisualTools visual_tools(
            shared_from_this(), mg.getPlanningFrame(), rviz_visual_tools::RVIZ_MARKER_TOPIC,
            mg.getRobotModel());
        visual_tools.deleteAllMarkers();
        visual_tools.trigger();

        auto log = this->get_logger();

        // 1. 홈 자세 이동
        if (go_home_) {
            mg.setNamedTarget("hcr_home");
            if (mg.move() != moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_ERROR(log, "홈 자세 이동 실패 — SRDF의 group_state 'hcr_home' 확인");
                return;
            }
            RCLCPP_INFO(log, "홈 자세 도달");
        }

        // 2. 홈 자세의 Pose를 기준점(Origin)으로 사용
        const auto origin = mg.getCurrentPose().pose;

        RCLCPP_INFO(log, "그리기 기준점 설정 완료: z=%.4f (%.4f, %.4f)",
                    origin.position.z, origin.position.x, origin.position.y);

        // 3. 이미지 Scale 및 Center 계산
        double u_lo = 1e9, u_hi = -1e9, v_lo = 1e9, v_hi = -1e9;
        for (const auto & part : parts) {
            for (const auto & p : part.points) {
                u_lo = std::min(u_lo, p.u); u_hi = std::max(u_hi, p.u);
                v_lo = std::min(v_lo, p.v); v_hi = std::max(v_hi, p.v);
            }
        }
        const double w_px = u_hi - u_lo, h_px = v_hi - v_lo;
        const double s = draw_size_ / std::max(w_px, h_px);
        const double u_c = 0.5 * (u_lo + u_hi), v_c = 0.5 * (v_lo + v_hi);

        RCLCPP_INFO(log, "전체 bbox %.0f x %.0f px -> %.1f x %.1f mm (배율 %.5f m/px)",
                    w_px, h_px, w_px * s * 1000.0, h_px * s * 1000.0, s);

        // 4. RViz 마커 발행 (초록색 타겟 그림)
        {
            visualization_msgs::msg::MarkerArray arr;
            int id = 0;
            for (const auto & part : parts) {
                visualization_msgs::msg::Marker m;
                m.header.frame_id = mg.getPlanningFrame();
                m.header.stamp = this->now();
                m.ns = "target_shape";
                m.id = id++;
                m.type = visualization_msgs::msg::Marker::LINE_STRIP;
                m.action = visualization_msgs::msg::Marker::ADD;
                m.pose.orientation.w = 1.0;
                m.scale.x = 0.001;
                m.color.r = 0.1f; m.color.g = 1.0f; m.color.b = 0.1f; m.color.a = 1.0f;
                for (const auto & p : part.points) m.points.push_back(toPose(p, 0.0, origin, u_c, v_c, s).position);
                if (!part.points.empty()) m.points.push_back(m.points.front());
                arr.markers.push_back(m);
            }
            marker_pub_->publish(arr);
            RCLCPP_INFO(log, "입력 도형 발행 (초록, %zu개 스트로크)", parts.size());
        }

        // 5. 이전 펜 자취 초기화
        {
            auto cli = this->create_client<std_srvs::srv::Empty>("/pen_trail/clear");
            if (cli->wait_for_service(std::chrono::milliseconds(500))) {
                cli->async_send_request(std::make_shared<std_srvs::srv::Empty::Request>());
                RCLCPP_INFO(log, "이전 자취 삭제");
            }
        }

        // 6. 스트로크별 독립적 그리기 반복문
        for (const auto & part : parts) {
            if (part.points.empty()) continue;

            RCLCPP_INFO(log, ">>> [%s] 그리기 진행 중...", part.instance_label.c_str());

            // A. 바닥에 그릴 Pose 좌표 모음 생성
            std::vector<geometry_msgs::msg::Pose> ground_shape;
            for (const auto & p : part.points) {
                ground_shape.push_back(toPose(p, 0.0, origin, u_c, v_c, s));
            }
            // 닫힌 도형(원형 등)을 위해 첫 점을 끝에 추가
            ground_shape.push_back(ground_shape.front());

            // B. [접근 단계] 공중 시작점으로 이동 -> 바닥으로 수직 착지
            {
                std::vector<geometry_msgs::msg::Pose> approach_path;
                auto start_hover = toPose(part.points.front(), hover_, origin, u_c, v_c, s);
                auto start_ground = ground_shape.front();

                approach_path.push_back(start_hover);  // 1. 공중으로 이동
                approach_path.push_back(start_ground); // 2. 바닥으로 하강

                runCartesian(mg, visual_tools, approach_path, eef_step_, vel_, acc_, execute_, log, part.instance_label + "_접근");
            }

            // C. [그리기 단계] 바닥면을 따라 경로 복사 및 그리기
            {
                runCartesian(mg, visual_tools, ground_shape, eef_step_, vel_, acc_, execute_, log, part.instance_label + "_그리기");
            }

            // D. [이륙 단계] 그리기 완료 후 공중으로 수직 상승
            {
                std::vector<geometry_msgs::msg::Pose> retract_path;
                auto end_hover = toPose(part.points.back(), hover_, origin, u_c, v_c, s);

                retract_path.push_back(end_hover); // 1. 공중으로 띄우기

                runCartesian(mg, visual_tools, retract_path, eef_step_, vel_, acc_, execute_, log, part.instance_label + "_이륙");
            }
        }

        // 7. [종료 단계] 완성된 그림을 보여주기 위해 1번 관절을 +90도 회전하여 비킴
        RCLCPP_INFO(log, "그리기 완료! 완성된 그림이 보이도록 로봇팔을 비킵니다.");
        std::vector<double> joint_values = mg.getCurrentJointValues();
        if (!joint_values.empty()) {
            joint_values[0] += M_PI_2/3; // joint_1 +90도 회전
            
            mg.setJointValueTarget(joint_values);
            mg.setMaxVelocityScalingFactor(0.2);
            mg.setMaxAccelerationScalingFactor(0.2);
            
            if (mg.move() == moveit::core::MoveItErrorCode::SUCCESS) {
                RCLCPP_INFO(log, "로봇팔 회전 완료. 완성된 그림을 확인하세요!");
            }
        }

        RCLCPP_INFO(log, "전체 프로세스 종료");
    }

    rclcpp::Subscription<vision_interfaces::msg::Stroke>::SharedPtr sub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

    std::mutex mutex_;
    std::map<std::string, PartData> parts_;

    double draw_size_, eef_step_, hover_, vel_, acc_;
    bool execute_, go_home_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DrawStrokesNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    auto spinner = std::thread([&executor]() { executor.spin(); });

    node->wait_and_draw();

    executor.cancel();
    spinner.join();
    rclcpp::shutdown();
    return 0;
}