// 가짜 고양이 스트로크를 MoveIt2 로 그리고, 펜 자취를 RViz Marker 로 발행한다.
//
// 동작을 바꾸는 값은 전부 파라미터다 — config/draw_cat_params.yaml 을 고치면
// 되고 이 파일을 다시 컴파일할 필요는 없다. 각 값의 의미도 그 파일에 적혀 있다.
#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vision_interfaces/msg/stroke.hpp>
// F3.1(순서 최적화)과 Stroke 구조체는 여기 있다. ROS 에 의존하지 않는 순수 계산이라
// 따로 떼어 두었고, 그래서 로봇 없이 단위시험이 돈다 (test/test_optimizer.cpp).
#include "drawing_cat/optimizer.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <limits>
#include <map>
#include <mutex>
#include <set>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

namespace
{

using drawing_cat::Stroke;
using drawing_cat::optimizeStrokeOrder;
using drawing_cat::totalTravelMm;

/// 파라미터를 선언하고 그 값을 바로 돌려준다.
template <typename T>
T declare(const rclcpp::Node::SharedPtr& node, const std::string& name, const T& fallback)
{
    return node->declare_parameter<T>(name, fallback);
}


/// vision 의 map.csv 를 읽는다.
///
/// 형식:  stroke_idx,point_idx,x_mm,y_mm,closed   (헤더 줄은 있어도 되고 없어도 됨)
///
/// ⚠️ **vision_core 에 링크하지 않는다 — 의도된 것이다.** 두 모듈은 별도 컨테이너에서
/// 자라고 있고, moveit2 가 필요로 하는 것은 좌표뿐이다. 파일 형식 하나로 결합을
/// 줄이는 것이 팀 설계다 (simulation 의 planned_map.hpp 주석과 같은 근거).
/// 그래서 아래 파싱 규칙은 simulation/sim_core/src/planned_map.cpp 와 **일부러
/// 동일하게** 맞춰 두었다 — 한쪽만 고치면 두 소비자가 어긋난다.
std::vector<Stroke> loadMapCsv(const std::string& path, std::string& error)
{
    std::vector<Stroke> strokes;
    std::ifstream file(path);
    if (!file)
    {
        error = "계획 지도를 열 수 없습니다: " + path;
        return strokes;
    }

    std::string line;
    std::size_t line_number = 0;
    long current_stroke = -1;

    while (std::getline(file, line))
    {
        ++line_number;
        if (line.empty())
        {
            continue;
        }
        // 헤더 줄은 첫 필드가 숫자가 아니다.
        if (line_number == 1 && line.rfind("stroke_idx", 0) == 0)
        {
            continue;
        }

        std::vector<std::string> fields;
        {
            std::istringstream stream(line);
            std::string field;
            while (std::getline(stream, field, ','))
            {
                fields.push_back(field);
            }
        }
        if (fields.size() < 4)
        {
            error = "map.csv 형식이 맞지 않습니다 (" + path + " " +
                    std::to_string(line_number) +
                    "행): stroke_idx,point_idx,x_mm,y_mm,closed 를 기대합니다";
            return {};
        }

        try
        {
            const long stroke_index = std::stol(fields[0]);
            const double x_mm = std::stod(fields[2]);
            const double y_mm = std::stod(fields[3]);
            const bool closed = fields.size() >= 5 &&
                (fields[4] == "1" || fields[4] == "true" || fields[4] == "True");

            // 스트로크 인덱스가 바뀌면 새 스트로크. vision 은 순서대로 뱉는다.
            if (stroke_index != current_stroke)
            {
                strokes.push_back(Stroke{"stroke_" + std::to_string(stroke_index), {}, false});
                current_stroke = stroke_index;
            }
            strokes.back().flat_mm.push_back(x_mm);
            strokes.back().flat_mm.push_back(y_mm);
            strokes.back().closed = closed;
        }
        catch (const std::exception&)
        {
            error = "map.csv 의 숫자를 읽지 못했습니다 (" + path + " " +
                    std::to_string(line_number) + "행)";
            return {};
        }
    }
    return strokes;
}

/// 쿼터니언 q 로 로컬 +z 축 벡터 (0, 0, length) 를 회전시킨 결과.
///
/// 펜은 플랜지 로컬 +z 로 뻗어 있으므로(hcr_robot_pen.xacro 의 pen_joint 가
/// rpy 0 0 0), 펜 끝의 월드 오프셋이 곧 이 값이다. 회전행렬 3 번째 열만 쓰면
/// 되므로 tf2 를 끌어오지 않는다.
geometry_msgs::msg::Point rotatedZAxis(const geometry_msgs::msg::Quaternion& q, double length)
{
    geometry_msgs::msg::Point v;
    v.x = length * 2.0 * (q.x * q.z + q.w * q.y);
    v.y = length * 2.0 * (q.y * q.z - q.w * q.x);
    v.z = length * (1.0 - 2.0 * (q.x * q.x + q.y * q.y));
    return v;
}

/// A4 letterbox 매핑 계수 — vision 의 image_to_map.cpp `ScaleToPaper` 와 **같은 식**이다.
///
/// ⚠️ 축척 기준이 컨투어 bbox 가 아니라 **이미지 전체 크기**라는 점이 핵심이다.
/// 그래서 Stroke 메시지의 image_width/image_height 만 있으면 vision 이 map.csv 에
/// 넣었을 값과 동일한 mm 좌표를 복원할 수 있다 — 축척을 사람이 정할 필요가 없다.
struct PaperFit
{
    double scale{1.0};      ///< mm / px
    double offset_x{0.0};   ///< mm
    double offset_y{0.0};   ///< mm
};

PaperFit computePaperFit(int image_w, int image_h,
                         double paper_w_mm, double paper_h_mm, double margin_mm)
{
    const double drawable_w = paper_w_mm - 2.0 * margin_mm;
    const double drawable_h = paper_h_mm - 2.0 * margin_mm;
    PaperFit f;
    // 종횡비를 유지한 채 작화영역에 맞춰 축소하고 중앙 정렬한다 (letterbox).
    f.scale = std::min(drawable_w / image_w, drawable_h / image_h);
    f.offset_x = margin_mm + (drawable_w - image_w * f.scale) * 0.5;
    f.offset_y = margin_mm + (drawable_h - image_h * f.scale) * 0.5;
    return f;
}

/// `/vision/strokes` 를 모아 한 이미지분(프레임)을 만든다.
///
/// ⚠️ **메시지에 Header 도 프레임 번호도 없다.** 어느 이미지의 부위인지 알 방법이
/// 메시지 안에 없어서, 받는 쪽이 경계를 판정해야 한다. 두 신호를 쓴다:
///
///   ① instance_label 중복 — 이미 받은 라벨이 또 오면 새 이미지가 시작된 것이다.
///      타이밍과 무관해서 가장 믿을 만하다.
///   ② 발행 간격 — 한 이미지의 부위들은 "짧은 시간 안에 연달아" 온다. 마지막
///      수신 후 frame_timeout_ms 가 지나면 그 프레임이 끝났다고 본다.
///
/// image_width/height 가 바뀌면 확실히 다른 이미지지만, 같은 카메라면 안 바뀌므로
/// 보조 신호로만 쓴다.
///
/// ⚠️ 구독 QoS 의 durability 는 **VOLATILE** 이다. 발행자는 TRANSIENT_LOCAL 이지만
/// "구독자가 덜 요구하는" 방향이라 호환되고, **밀린 이력을 받지 않는다**(실측).
/// 이게 없으면 늦게 구독했을 때 이전 이미지 부위와 새 이미지 부위가 섞인다.
class StrokeCollector : public rclcpp::Node
{
public:
    StrokeCollector(const std::string& topic, int64_t frame_timeout_ms, bool use_latched)
    : Node("draw_cat_stroke_collector"), frame_timeout_ms_(frame_timeout_ms)
    {
        rclcpp::QoS qos(20);
        qos.reliable();
        // ⚠️ 둘 다 발행자(TRANSIENT_LOCAL)와 호환된다 — durability 는 "구독자가 덜
        // 요구하는" 방향이 호환이다. 차이는 **밀린 이력을 받느냐**뿐이다.
        //   volatile        : 살아 있는 발행분만. 이전 이미지가 섞일 여지가 없다.
        //   transient_local : 마지막 발행분을 받는다. vision 이 한 번 쏘고 끝나는
        //                     운용이면 이게 필요하다. 대신 오래된 프레임이 올 수 있다.
        if (use_latched) { qos.transient_local(); } else { qos.durability_volatile(); }
        sub_ = create_subscription<vision_interfaces::msg::Stroke>(
            topic, qos,
            [this](const vision_interfaces::msg::Stroke::SharedPtr msg) { onStroke(msg); });
    }

    /// 프레임 하나가 완성될 때까지 기다린다. 타임아웃이면 빈 벡터.
    ///
    /// 완성 판정은 **조용해질 때까지 기다리는 것**이다 — 마지막 수신 후
    /// frame_timeout_ms 동안 새 메시지가 없으면 그 프레임이 끝났다고 본다.
    /// 그동안 여러 프레임이 쌓였으면(TRANSIENT_LOCAL 로 밀린 이력을 한꺼번에 받은
    /// 경우가 대표적) **가장 최신 것**을 쓰고 나머지는 버린다.
    std::vector<vision_interfaces::msg::Stroke> waitForFrame(double wait_timeout_s, int& skipped)
    {
        skipped = 0;
        const auto deadline = std::chrono::steady_clock::now() +
                              std::chrono::duration<double>(wait_timeout_s);
        while (rclcpp::ok() && std::chrono::steady_clock::now() < deadline)
        {
            {
                std::lock_guard<std::mutex> lk(mu_);
                if (!done_.empty() || !current_.empty())
                {
                    const auto idle = std::chrono::duration_cast<std::chrono::milliseconds>(
                        std::chrono::steady_clock::now() - last_rx_).count();
                    if (idle >= frame_timeout_ms_)
                    {
                        if (!current_.empty())
                        {
                            done_.push_back(std::move(current_));
                            current_.clear();
                            labels_.clear();
                        }
                        auto out = std::move(done_.back());
                        skipped = static_cast<int>(done_.size()) - 1;
                        done_.clear();
                        return out;
                    }
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
        return {};
    }

private:
    void onStroke(const vision_interfaces::msg::Stroke::SharedPtr& msg)
    {
        std::lock_guard<std::mutex> lk(mu_);
        // ① 같은 라벨이 다시 왔다 = 새 이미지 시작. 지금까지 모은 것을 확정한다.
        if (labels_.count(msg->instance_label))
        {
            done_.push_back(std::move(current_));
            current_.clear();
            labels_.clear();
        }
        labels_.insert(msg->instance_label);
        current_.push_back(*msg);
        last_rx_ = std::chrono::steady_clock::now();
    }

    rclcpp::Subscription<vision_interfaces::msg::Stroke>::SharedPtr sub_;
    std::mutex mu_;
    std::vector<vision_interfaces::msg::Stroke> current_;
    std::vector<std::vector<vision_interfaces::msg::Stroke>> done_;
    std::set<std::string> labels_;
    std::chrono::steady_clock::time_point last_rx_{std::chrono::steady_clock::now()};
    int64_t frame_timeout_ms_;
};

/// 변환된 스트로크를 vision 의 map.csv 형식으로 저장한다.
///
/// 이걸 남겨야 simulation 의 trace_cli 로 "계획 대비 실제"를 볼 수 있다
/// (drawing_cat/README.md 7절). 토픽으로 받아도 파일 기반 검증 경로가 끊기지 않는다.
bool writeMapCsv(const std::string& path, const std::vector<Stroke>& strokes, std::string& error)
{
    std::ofstream f(path);
    if (!f)
    {
        error = "map.csv 를 쓸 수 없다: " + path;
        return false;
    }
    f << "stroke_idx,point_idx,x_mm,y_mm,closed\n";
    f << std::fixed << std::setprecision(3);
    for (size_t s = 0; s < strokes.size(); ++s)
    {
        const auto& flat = strokes[s].flat_mm;
        const int closed = strokes[s].closed ? 1 : 0;
        for (size_t i = 0, p = 0; i + 1 < flat.size(); i += 2, ++p)
        {
            f << s << ',' << p << ',' << flat[i] << ',' << flat[i + 1] << ',' << closed << '\n';
        }
    }
    return true;
}

/// 궤적의 **계획 소요시간**(초). 점이 없으면 0.
///
/// ⚠️ **MoveIt 이 계획한 시간이지 실제로 걸린 시간이 아니다.** 이게 장점이다 —
/// MuJoCo 의 관절 한계 눌림이나 추종 지연(joint_2 최대 14.96°)에 오염되지 않아서,
/// 순서·속도 배율을 바꿨을 때 **그 변경의 효과만** 깨끗하게 비교할 수 있다.
/// 실제로 걸린 시간은 따로 벽시계로 재서 나란히 찍는다 (둘의 차이가 곧 로봇 쪽 문제다).
double trajDurationS(const moveit_msgs::msg::RobotTrajectory& traj)
{
    const auto& pts = traj.joint_trajectory.points;
    if (pts.empty())
    {
        return 0.0;
    }
    const auto& t = pts.back().time_from_start;
    return static_cast<double>(t.sec) + static_cast<double>(t.nanosec) * 1e-9;
}

}  // namespace

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("draw_cat_node");
    const auto logger = node->get_logger();

    // ── 파라미터 읽기 ──────────────────────────────────────────────────────
    const auto group_name    = declare<std::string>(node, "planning.group", "hcr_arm");
    const auto home_pose     = declare<std::string>(node, "planning.home_pose", "hcr_home");
    const auto eef_step      = declare<double>(node, "planning.eef_step", 0.002);
    const auto min_fraction  = declare<double>(node, "planning.min_fraction", 0.9);
    const auto vel_scaling   = declare<double>(node, "planning.velocity_scaling", 0.1);
    const auto acc_scaling   = declare<double>(node, "planning.acceleration_scaling", 0.1);

    // ── F3.1 순서 최적화 ───────────────────────────────────────────────────
    const auto optimize_order = declare<bool>(node, "planning.optimize_order", true);
    const auto rotate_closed  = declare<bool>(node, "planning.rotate_closed_start", true);

    // ── 펜업 이동 (그리기와 분리) ──────────────────────────────────────────
    // 펜이 종이에서 떨어져 있는 구간이라 경로 모양이 그림에 영향을 주지 않는다.
    // 그리기와 같은 배율로 갈 이유가 없어서 따로 뗀다.
    const auto travel_mode      = declare<std::string>(node, "planning.travel_mode", "cartesian");
    const auto travel_vel       = declare<double>(node, "planning.travel_velocity_scaling", 0.5);
    // ⚠️ 속도가 아니라 **가속**이 손잡이다 (실측). 펜업 이동이 실제 4.5~24 mm 라 최고
    //    속도에 도달할 일이 없어서 travel_velocity_scaling 은 효과가 없다.
    // 🔴 1.0 은 관절 가속 한계의 100 % 다. 실물 이관 전 하드웨어 확인 필요 —
    //    docs/Pipeline Integration Status.md §5 "하드웨어" 참고.
    const auto travel_acc       = declare<double>(node, "planning.travel_acceleration_scaling", 1.0);
    const auto travel_eef_step  = declare<double>(node, "planning.travel_eef_step", 0.01);

    const auto use_current_origin = declare<bool>(node, "paper.use_current_pose_as_origin", true);
    const auto origin_x     = declare<double>(node, "paper.origin_x", 0.0);
    const auto origin_y     = declare<double>(node, "paper.origin_y", 0.0);
    const auto origin_z     = declare<double>(node, "paper.origin_z", 0.0);
    const auto auto_center  = declare<bool>(node, "paper.auto_center", false);
    auto center_x_mm        = declare<double>(node, "paper.center_x_mm", 81.0);
    auto center_y_mm        = declare<double>(node, "paper.center_y_mm", 51.0);
    const auto scale        = declare<double>(node, "paper.scale", 0.00025);
    const auto pen_lift     = declare<double>(node, "paper.pen_lift", 0.008);

    const auto tip_offset_z = declare<double>(node, "pen.tip_offset_z", 0.0);

    const auto marker_topic = declare<std::string>(node, "marker.topic", "pen_trace");
    const auto marker_ns    = declare<std::string>(node, "marker.ns", "cat_strokes");
    const auto line_width   = declare<double>(node, "marker.line_width", 0.001);
    const auto color        = declare<std::vector<double>>(node, "marker.color_rgba",
                                                           {1.0, 0.0, 0.0, 1.0});
    const auto hold_seconds = declare<double>(node, "marker.hold_seconds", 5.0);

    const auto stroke_source = declare<std::string>(node, "strokes.source", "params");
    const auto csv_path      = declare<std::string>(node, "strokes.csv_path", "");
    const auto stroke_names  = declare<std::vector<std::string>>(node, "strokes.names", {});
    const auto closed_flags  = declare<std::vector<bool>>(node, "strokes.closed", {});
    const auto strokes_topic = declare<std::string>(node, "strokes.topic", "/vision/strokes");
    const auto frame_timeout = declare<int>(node, "strokes.frame_timeout_ms", 1500);
    const auto wait_timeout  = declare<double>(node, "strokes.wait_timeout_s", 30.0);
    const auto use_latched   = declare<bool>(node, "strokes.use_latched", false);

    // A4 규격 — vision 의 params.hpp 기본값(210 × 297, 여백 15)과 같게 둔다.
    // 픽셀 → mm 변환에만 쓰이므로 source: "topic" 일 때만 의미가 있다.
    const auto paper_w_mm   = declare<double>(node, "paper.width_mm", 210.0);
    const auto paper_h_mm   = declare<double>(node, "paper.height_mm", 297.0);
    const auto paper_margin = declare<double>(node, "paper.margin_mm", 15.0);

    const auto map_csv_out  = declare<std::string>(node, "output.map_csv_path", "");

    if (color.size() != 4)
    {
        RCLCPP_ERROR(logger, "marker.color_rgba 는 원소가 4 개여야 한다 (현재 %zu 개) - 종료",
                     color.size());
        rclcpp::shutdown();
        return 1;
    }

    if (travel_mode != "cartesian" && travel_mode != "joint")
    {
        RCLCPP_ERROR(logger, "planning.travel_mode 는 'cartesian' 또는 'joint' 여야 한다 "
                             "(현재 '%s') - 종료", travel_mode.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // ── 그릴 것 불러오기 ───────────────────────────────────────────────────
    //
    // ⚠️ vision 규약상 **스트로크들 사이에는 순서가 없다.** 여기서 담기는 순서(csv 는
    // 파일 순, params 는 names 순, topic 은 수신 순)에는 아무 의미가 없고, 실제 그리는
    // 순서는 아래 F3.1(optimizeStrokeOrder)이 다시 정한다.
    std::vector<Stroke> strokes;

    if (stroke_source == "csv")
    {
        if (csv_path.empty())
        {
            RCLCPP_ERROR(logger, "strokes.source 가 'csv' 인데 strokes.csv_path 가 비어 있다 - 종료");
            rclcpp::shutdown();
            return 1;
        }
        std::string error;
        strokes = loadMapCsv(csv_path, error);
        if (!error.empty())
        {
            RCLCPP_ERROR(logger, "%s - 종료", error.c_str());
            rclcpp::shutdown();
            return 1;
        }
        RCLCPP_INFO(logger, "map.csv 로드: %s | 스트로크 %zu 개", csv_path.c_str(), strokes.size());
    }
    else if (stroke_source == "params")
    {
        if (stroke_names.empty())
        {
            RCLCPP_ERROR(logger, "strokes.names 가 비어 있다 - 파라미터 파일이 실제로 "
                                 "로드됐는지 확인할 것 (노드 이름 키가 draw_cat_node 인지) - 종료");
            rclcpp::shutdown();
            return 1;
        }
        // 좌표는 [x1,y1, x2,y2, ...] 로 평탄하게 들어온다 (ROS 2 파라미터가 중첩
        // 배열을 지원하지 않는다).
        for (size_t i = 0; i < stroke_names.size(); ++i)
        {
            Stroke s;
            s.name = stroke_names[i];
            s.flat_mm = declare<std::vector<double>>(node, "strokes." + s.name, {});
            s.closed = i < closed_flags.size() ? closed_flags[i] : false;
            strokes.push_back(std::move(s));
        }
    }
    else if (stroke_source == "topic")
    {
        // vision 이 픽셀 좌표를 토픽으로 보낸다. mm 변환은 **여기가 유일한 지점**이다
        // (F4.1 이 moveit2 담당인 이유이기도 하다).
        auto collector = std::make_shared<StrokeCollector>(strokes_topic, frame_timeout,
                                                           use_latched);
        rclcpp::executors::SingleThreadedExecutor cex;
        cex.add_node(collector);
        std::thread cthread([&cex]() { cex.spin(); });

        RCLCPP_INFO(logger, "'%s' 구독 중 (%s) — 최대 %.0f초 대기",
                    strokes_topic.c_str(),
                    use_latched ? "TRANSIENT_LOCAL: 밀린 이력도 받음"
                                : "VOLATILE: 살아 있는 발행분만",
                    wait_timeout);
        int skipped = 0;
        const auto frame = collector->waitForFrame(wait_timeout, skipped);
        cex.cancel();
        cthread.join();
        if (skipped > 0)
        {
            RCLCPP_WARN(logger, "프레임이 %d 개 더 있었다 — 가장 최신 것만 쓰고 버린다",
                        skipped);
        }

        if (frame.empty())
        {
            RCLCPP_ERROR(logger, "'%s' 에서 스트로크를 받지 못했다 - 종료.\n"
                                 "  · 발행 중인지: ros2 topic info %s --verbose\n"
                                 "  · 타입 해시가 우리 vision_interfaces 와 같은지 확인할 것 "
                                 "(다르면 토픽은 보이는데 수신이 0 이다)",
                         strokes_topic.c_str(), strokes_topic.c_str());
            rclcpp::shutdown();
            return 1;
        }

        const auto fit = computePaperFit(frame.front().image_width, frame.front().image_height,
                                         paper_w_mm, paper_h_mm, paper_margin);
        RCLCPP_INFO(logger,
                    "부위 %zu 개 수신 | 이미지 %d×%d px | letterbox %.5f mm/px, "
                    "오프셋 (%.1f, %.1f) mm  [용지 %.0f×%.0f, 여백 %.0f]",
                    frame.size(), frame.front().image_width, frame.front().image_height,
                    fit.scale, fit.offset_x, fit.offset_y, paper_w_mm, paper_h_mm, paper_margin);

        for (const auto& msg : frame)
        {
            Stroke s;
            s.name = msg.instance_label;
            // ⚠️ vision 규약대로 **항상 닫힌 윤곽**이다. 메시지에 closed 필드가 없다.
            s.closed = true;
            s.flat_mm.reserve(msg.points.size() * 2);
            for (const auto& p : msg.points)
            {
                s.flat_mm.push_back(fit.offset_x + p.u * fit.scale);
                s.flat_mm.push_back(fit.offset_y + p.v * fit.scale);
            }
            RCLCPP_INFO(logger, "  · '%s' %zu 점", s.name.c_str(), msg.points.size());
            strokes.push_back(std::move(s));
        }
    }
    else
    {
        RCLCPP_ERROR(logger, "strokes.source 는 'params' · 'csv' · 'topic' 중 하나여야 한다 "
                             "(현재 '%s') - 종료", stroke_source.c_str());
        rclcpp::shutdown();
        return 1;
    }

    // 점 개수가 홀수이거나 2점 미만인 스트로크는 그릴 수 없다.
    strokes.erase(std::remove_if(strokes.begin(), strokes.end(),
        [&](const Stroke& s) {
            if (s.flat_mm.size() < 4 || s.flat_mm.size() % 2 != 0)
            {
                RCLCPP_WARN(logger, "스트로크 '%s' 좌표 개수가 %zu 개 - 짝수이면서 4 개 이상이어야 "
                                    "한다. 건너뛴다.", s.name.c_str(), s.flat_mm.size());
                return true;
            }
            return false;
        }), strokes.end());

    if (strokes.empty())
    {
        RCLCPP_ERROR(logger, "그릴 수 있는 스트로크가 없다 - 종료");
        rclcpp::shutdown();
        return 1;
    }

    // ── 자동 중심 맞추기 ───────────────────────────────────────────────────
    // vision 이 넘겨준 좌표는 범위를 미리 알 수 없다. auto_center 를 켜면 실제
    // 좌표의 경계상자 중심을 원점에 맞춘다 — center_x_mm/center_y_mm 를 손으로
    // 맞출 필요가 없어진다.
    {
        double min_x = std::numeric_limits<double>::max(), max_x = -min_x;
        double min_y = std::numeric_limits<double>::max(), max_y = -min_y;
        size_t total_points = 0;
        for (const auto& s : strokes)
        {
            for (size_t i = 0; i + 1 < s.flat_mm.size(); i += 2)
            {
                min_x = std::min(min_x, s.flat_mm[i]);
                max_x = std::max(max_x, s.flat_mm[i]);
                min_y = std::min(min_y, s.flat_mm[i + 1]);
                max_y = std::max(max_y, s.flat_mm[i + 1]);
                ++total_points;
            }
        }
        if (auto_center)
        {
            center_x_mm = 0.5 * (min_x + max_x);
            center_y_mm = 0.5 * (min_y + max_y);
        }
        RCLCPP_INFO(logger,
                    "스트로크 %zu 개 · 점 %zu 개 | 범위 %.1f~%.1f × %.1f~%.1f mm "
                    "| 중심 %.1f, %.1f mm | 축척 후 %.1f × %.1f mm",
                    strokes.size(), total_points, min_x, max_x, min_y, max_y,
                    center_x_mm, center_y_mm,
                    (max_x - min_x) * scale * 1000.0, (max_y - min_y) * scale * 1000.0);
    }

    // ── F3.1 순서 최적화 ───────────────────────────────────────────────────
    //
    // ⚠️ **자동 중심 맞추기 뒤에 와야 한다.** 시작점으로 쓰는 center_x_mm/center_y_mm 이
    //    auto_center 블록에서 덮어써지기 때문이다.
    // ⚠️ **map.csv 저장 앞에 와야 한다.** 저장된 stroke_idx 가 실제 실행 순서와 같아야
    //    trace_cli 로 "계획 대비 실제" 를 볼 때 인덱스가 맞는다.
    //
    // 시작점이 (center_x_mm, center_y_mm) 인 이유: 아래 flangePose() 를 보면
    // mx_mm == center_x_mm 일 때 p.x = px0 - tip_offset.x 이고, px0 는 현재(home) 자세의
    // 펜 끝이므로 결국 home 플랜지 위치로 환원된다. 즉 **종이 좌표계에서 home 이 놓인
    // 자리가 곧 (center_x_mm, center_y_mm)** 이다 — 로봇 실제 자세를 몰라도 된다.
    //
    // ⚠️ 이 등식은 `paper.use_current_pose_as_origin: true` 일 때만 성립한다. false 면
    //    원점이 파라미터로 고정되어 home 이 종이 어디에 놓이는지 알 수 없다. 다만 그
    //    경우에도 틀어지는 것은 **첫 스트로크 선택 하나뿐**이고(이후 홉은 스트로크
    //    끝점끼리라 무관), 순서 자체가 무효가 되지는 않는다.
    if (optimize_order)
    {
        const double before_mm = totalTravelMm(strokes, center_x_mm, center_y_mm);
        double after_mm = 0.0;
        strokes = optimizeStrokeOrder(std::move(strokes), center_x_mm, center_y_mm,
                                      rotate_closed, after_mm);

        std::ostringstream order;
        for (std::size_t i = 0; i < strokes.size(); ++i)
        {
            order << (i ? " → " : "") << strokes[i].name;
        }
        const double saved_pct = before_mm > 0.0 ? (before_mm - after_mm) / before_mm * 100.0 : 0.0;
        RCLCPP_INFO(logger,
                    "F3.1 순서 최적화 (NN%s) | 펜업 이동 %.1f → %.1f 종이mm (%.1f%% 감소) "
                    "| 실제 %.1f → %.1f mm | 순서: %s",
                    rotate_closed ? ", 폐곡선 시작점 회전" : ", 회전 없음",
                    before_mm, after_mm, saved_pct,
                    before_mm * scale * 1000.0, after_mm * scale * 1000.0,
                    order.str().c_str());
    }
    else
    {
        RCLCPP_INFO(logger, "F3.1 순서 최적화 꺼짐 — 주어진 순서 그대로 그린다 "
                            "| 펜업 이동 %.1f 종이mm",
                    totalTravelMm(strokes, center_x_mm, center_y_mm));
    }

    // 계획 지도를 파일로 남긴다 — simulation 의 trace_cli 가 이걸 받아 "계획 대비 실제"
    // 를 SVG 로 겹쳐 그린다. 토픽으로 받았더라도 파일 기반 검증 경로가 유지된다.
    // ⚠️ 최적화 **뒤**라서 stroke_idx 가 실제 실행 순서다.
    if (!map_csv_out.empty())
    {
        std::string err;
        if (writeMapCsv(map_csv_out, strokes, err))
        {
            RCLCPP_INFO(logger, "계획 지도 저장: %s", map_csv_out.c_str());
        }
        else
        {
            RCLCPP_WARN(logger, "%s (그리기는 계속한다)", err.c_str());
        }
    }

    // ⚠️ MoveGroupInterface 가 /joint_states 구독 등 콜백을 처리하려면 노드가
    // 별도 스레드에서 spin 되고 있어야 한다. 없으면 getCurrentPose() 가
    // (0,0,0) 을 돌려주고 fraction 이 전부 0.00 이 된다.
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spin_thread([&executor]() { executor.spin(); });

    moveit::planning_interface::MoveGroupInterface move_group(node, group_name);
    move_group.setMaxVelocityScalingFactor(vel_scaling);
    move_group.setMaxAccelerationScalingFactor(acc_scaling);

    auto marker_pub = node->create_publisher<visualization_msgs::msg::Marker>(marker_topic, 10);

    rclcpp::sleep_for(std::chrono::seconds(1));

    auto shutdown = [&](int code) {
        rclcpp::shutdown();
        spin_thread.join();
        return code;
    };

    // ── 1) 안전한 홈 자세로 이동 ───────────────────────────────────────────
    move_group.setNamedTarget(home_pose);
    moveit::planning_interface::MoveGroupInterface::Plan home_plan;
    if (move_group.plan(home_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "'%s' 계획 실패 - 종료", home_pose.c_str());
        return shutdown(1);
    }
    // ⚠️ plan() 뿐 아니라 execute() 결과도 봐야 한다. 실행이 실패했는데 계속
    // 진행하면 아래에서 잡는 기준 자세가 통째로 틀어진다.
    if (move_group.execute(home_plan) != moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_ERROR(logger, "'%s' 실행 실패 - 시뮬레이터/컨트롤러가 떠 있는지 확인할 것 - 종료",
                     home_pose.c_str());
        return shutdown(1);
    }
    // 순서 최적화와 무관한 **고정 비용**이라 아래 비율 계산에서는 뺀다.
    const double home_s = trajDurationS(home_plan.trajectory);
    RCLCPP_INFO(logger, "'%s' 자세로 이동 완료 | 계획 %.2f s", home_pose.c_str(), home_s);
    rclcpp::sleep_for(std::chrono::milliseconds(500));

    // ── 2) 기준 자세 확보 ──────────────────────────────────────────────────
    // 이 방향(orientation)을 그대로 유지한 채 평면 위에서만 움직인다.
    const geometry_msgs::msg::PoseStamped base = move_group.getCurrentPose();
    const auto orientation = base.pose.orientation;

    // 플랜지 → 펜 끝 오프셋. tip_offset_z 가 0 이면 영벡터라 아래 식이 전부
    // 플랜지 기준으로 환원된다 (지금 확인된 동작).
    const auto tip_offset = rotatedZAxis(orientation, tip_offset_z);

    // 종이 원점을 **펜 끝 기준**으로 잡는다.
    double px0, py0, pz0;
    if (use_current_origin)
    {
        px0 = base.pose.position.x + tip_offset.x;
        py0 = base.pose.position.y + tip_offset.y;
        pz0 = base.pose.position.z + tip_offset.z;
        RCLCPP_INFO(logger, "종이 원점 = 현재 자세 기준 | 펜 끝: x=%.3f y=%.3f z=%.3f",
                    px0, py0, pz0);
    }
    else
    {
        px0 = origin_x;
        py0 = origin_y;
        pz0 = origin_z;
        RCLCPP_INFO(logger, "종이 원점 = 파라미터 지정 | 펜 끝: x=%.3f y=%.3f z=%.3f",
                    px0, py0, pz0);
    }

    // 펜 끝 목표점 → 플랜지 목표 자세. 플래닝은 그룹 끝 링크(플랜지) 기준이므로
    // 펜 길이만큼 되빼야 한다.
    auto flangePose = [&](double mx_mm, double my_mm, double lift) {
        geometry_msgs::msg::Pose p;
        p.position.x = px0 + (mx_mm - center_x_mm) * scale - tip_offset.x;
        p.position.y = py0 + (my_mm - center_y_mm) * scale - tip_offset.y;
        p.position.z = pz0 + lift - tip_offset.z;
        p.orientation = orientation;
        return p;
    };

    // ── 시간 계측 ──────────────────────────────────────────────────────────
    //
    // "순서를 바꿔서 시간을 줄인다" 를 주장하려면 **펜업 이동이 전체의 몇 % 인지**를
    // 알아야 한다. 그 비율이 곧 F3.1 이 줄일 수 있는 시간의 **상한**이다 — 펜업이
    // 20 % 면 순서를 아무리 잘 짜도 전체 20 % 이상은 줄지 않는다.
    //
    // 두 가지를 따로 잰다:
    //   planned — MoveIt 이 계획한 궤적 길이. 로봇이 못 따라와도 값이 흔들리지 않아
    //             **변경의 효과만** 보기에 적합하다. 비교의 기준은 이쪽이다.
    //   wall    — execute() 가 실제로 걸린 시간. 계획과 벌어지는 만큼이 로봇 쪽 문제다.
    double travel_s = 0.0, travel_wall_s = 0.0;
    double draw_s   = 0.0, draw_wall_s   = 0.0;
    int travel_n = 0;

    /// 궤적을 실행하고, 성공했을 때만 계획 시간과 실측 시간을 누적한다.
    /// (실패한 실행을 섞으면 평균이 오염된다.)
    auto runTimed = [&](const moveit_msgs::msg::RobotTrajectory& traj,
                        double& planned_acc, double& wall_acc, int& count) {
        moveit::planning_interface::MoveGroupInterface::Plan p;
        p.trajectory = traj;
        const auto t0 = std::chrono::steady_clock::now();
        const bool ok = move_group.execute(p) == moveit::core::MoveItErrorCode::SUCCESS;
        const double dt = std::chrono::duration<double>(
            std::chrono::steady_clock::now() - t0).count();
        if (ok)
        {
            planned_acc += trajDurationS(traj);
            wall_acc += dt;
            ++count;
        }
        return ok;
    };

    // ── 3) 스트로크마다 [펜업 접근 → 펜다운 그리기 → 펜업 이탈] ────────────
    int stroke_id = 0;
    int drawn = 0;
    for (const auto& stroke : strokes)
    {
        const auto& flat = stroke.flat_mm;
        const auto& name = stroke.name;

        // 폐곡선이면 마지막 점 뒤에 첫 점을 한 번 더 찍어 윤곽을 닫는다.
        // vision 은 첫 점을 끝에 중복해 넣지 않으므로 여기서 더해야 한다.
        const double end_x = stroke.closed ? flat[0] : flat[flat.size() - 2];
        const double end_y = stroke.closed ? flat[1] : flat[flat.size() - 1];

        // ── 3-a) 펜업 이동 ─────────────────────────────────────────────────
        //
        // 펜이 종이에서 떨어져 있으므로 **어떤 경로로 가든 그림에 흔적이 남지 않는다.**
        // 그래서 그리기와 같은 배율로 조심스럽게 갈 이유가 없다 — 배율만 올려도
        // 이 구간의 시간이 그대로 줄어든다.
        //
        // ⚠️ 예전에는 이 이동이 아래 그리기 Cartesian 호출에 묶여 있었다. MoveIt 은
        //    waypoints[0] 으로 가는 구간도 알아서 만들어 주는데, 같은 호출이라 그리기와
        //    똑같은 속도·보간 규칙을 받았다.
        //
        // ⚠️ waypoints[0] 은 아래에서도 접근점 그대로 둔다. 여기서 실패해도 그리기
        //    호출이 알아서 접근점까지 데려가므로 동작이 깨지지 않는다 (길이 0 구간이
        //    되는 것뿐이다).
        const auto approach = flangePose(flat[0], flat[1], pen_lift);
        const double travel_s_before = travel_s;   // 이 스트로크 몫만 떼어내려고
        move_group.setMaxVelocityScalingFactor(travel_vel);
        move_group.setMaxAccelerationScalingFactor(travel_acc);

        if (travel_mode == "joint")
        {
            // 관절공간 계획 — 직선 제약이 없어 더 짧은 경로를 잡을 수 있다.
            // ⚠️ 대신 OMPL RRT 는 샘플링 기반이라 짧은 경로를 **보장하지 않는다.**
            //    돌아가는 경로가 나오면 배율을 올린 이득이 상쇄될 수 있다.
            move_group.setPoseTarget(approach);
            moveit::planning_interface::MoveGroupInterface::Plan travel_plan;
            if (move_group.plan(travel_plan) == moveit::core::MoveItErrorCode::SUCCESS)
            {
                if (!runTimed(travel_plan.trajectory, travel_s, travel_wall_s, travel_n))
                {
                    RCLCPP_WARN(logger, "  → 펜업 이동 실행 실패 - 그리기 호출에 맡긴다");
                }
            }
            else
            {
                RCLCPP_WARN(logger, "  → 펜업 이동 계획 실패 - 그리기 호출에 맡긴다");
            }
            move_group.clearPoseTargets();
        }
        else
        {
            // 직선 이동. 경로가 예측 가능하고(항상 최단 직선) 배율 이득은 그대로 얻는다.
            // 보간 간격은 그리기보다 성기게 둔다 — 종이에 안 닿으니 촘촘할 이유가 없다.
            moveit_msgs::msg::RobotTrajectory travel_traj;
            const double tf = move_group.computeCartesianPath({approach}, travel_eef_step,
                                                              travel_traj);
            if (tf >= min_fraction)
            {
                if (!runTimed(travel_traj, travel_s, travel_wall_s, travel_n))
                {
                    RCLCPP_WARN(logger, "  → 펜업 이동 실행 실패 - 그리기 호출에 맡긴다");
                }
            }
            else
            {
                RCLCPP_WARN(logger, "  → 펜업 이동 fraction %.2f - 그리기 호출에 맡긴다", tf);
            }
        }

        // ── 3-b) 그리기 ────────────────────────────────────────────────────
        // 여기부터는 종이에 닿는다. 원래의 느린 배율로 되돌린다.
        move_group.setMaxVelocityScalingFactor(vel_scaling);
        move_group.setMaxAccelerationScalingFactor(acc_scaling);

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(approach);                                 // 접근점 (보통 길이 0)
        for (size_t i = 0; i + 1 < flat.size(); i += 2)                // 펜다운 그리기
        {
            waypoints.push_back(flangePose(flat[i], flat[i + 1], 0.0));
        }
        if (stroke.closed)                                             // 닫는 구간
        {
            waypoints.push_back(flangePose(end_x, end_y, 0.0));
        }
        waypoints.push_back(flangePose(end_x, end_y, pen_lift));        // 펜업 이탈

        moveit_msgs::msg::RobotTrajectory trajectory;
        const double fraction = move_group.computeCartesianPath(waypoints, eef_step, trajectory);

        RCLCPP_INFO(logger, "스트로크 %d '%s' | 점 %zu 개%s | 성공률(fraction): %.2f",
                    stroke_id, name.c_str(), flat.size() / 2,
                    stroke.closed ? " · 폐곡선" : "", fraction);

        if (fraction < min_fraction)
        {
            RCLCPP_WARN(logger, "  → fraction 이 min_fraction(%.2f) 미만이라 실행하지 않는다",
                        min_fraction);
        }
        else
        {
            const double draw_s_before = draw_s;
            if (!runTimed(trajectory, draw_s, draw_wall_s, drawn))
            {
                RCLCPP_WARN(logger, "  → 실행 실패 - 다음 스트로크로 넘어간다");
            }
            else
            {
                RCLCPP_INFO(logger, "  → 계획 시간: 이동 %.2f s + 그리기 %.2f s = %.2f s",
                            travel_s - travel_s_before, draw_s - draw_s_before,
                            (travel_s - travel_s_before) + (draw_s - draw_s_before));
            }
        }

        // 자취 마커는 **펜 끝** 경로를 그린다 (플랜지가 아니라).
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = move_group.getPlanningFrame();
        marker.header.stamp = node->now();
        marker.ns = marker_ns;
        marker.id = stroke_id;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = line_width;
        marker.color.r = static_cast<float>(color[0]);
        marker.color.g = static_cast<float>(color[1]);
        marker.color.b = static_cast<float>(color[2]);
        marker.color.a = static_cast<float>(color[3]);
        marker.pose.orientation.w = 1.0;

        auto tipPoint = [&](double mx_mm, double my_mm) {
            geometry_msgs::msg::Point p;
            p.x = px0 + (mx_mm - center_x_mm) * scale;
            p.y = py0 + (my_mm - center_y_mm) * scale;
            p.z = pz0;
            return p;
        };
        for (size_t i = 0; i + 1 < flat.size(); i += 2)
        {
            marker.points.push_back(tipPoint(flat[i], flat[i + 1]));
        }
        if (stroke.closed)
        {
            marker.points.push_back(tipPoint(flat[0], flat[1]));
        }
        marker_pub->publish(marker);

        ++stroke_id;
        rclcpp::sleep_for(std::chrono::milliseconds(300));
    }

    RCLCPP_INFO(logger, "===== 고양이 그리기 완료 | %d/%zu 스트로크 실행 | 마커 %.1f초간 유지 =====",
                drawn, strokes.size(), hold_seconds);

    // ── 시간 분해 ──────────────────────────────────────────────────────────
    //
    // ⚠️ **펜업 비율이 F3.1 순서 최적화가 줄일 수 있는 시간의 상한이다.** 순서를
    //    바꿔서 줄어드는 것은 펜업 이동뿐이고, 그리는 시간은 좌표가 그대로인 한
    //    변하지 않기 때문이다. 펜업이 20 % 면 전체 20 % 가 이론적 최대치다.
    {
        const double motion_s = travel_s + draw_s;
        const double motion_wall_s = travel_wall_s + draw_wall_s;
        const auto pct = [&](double v) { return motion_s > 0.0 ? v / motion_s * 100.0 : 0.0; };

        RCLCPP_INFO(logger, "===== 시간 분해 (MoveIt 계획 기준) =====");
        RCLCPP_INFO(logger, "  홈 이동     %6.2f s          (1 회, 순서와 무관한 고정 비용 — 아래 비율에서 제외)",
                    home_s);
        RCLCPP_INFO(logger, "  펜업 이동   %6.2f s  %5.1f%%  (%d 회)", travel_s, pct(travel_s), travel_n);
        RCLCPP_INFO(logger, "  그리기      %6.2f s  %5.1f%%  (%d 회)", draw_s, pct(draw_s), drawn);
        RCLCPP_INFO(logger, "  ─────────────────────────────");
        RCLCPP_INFO(logger, "  이동+그리기 %6.2f s", motion_s);

        if (motion_wall_s > 0.0)
        {
            const double gap = motion_s > 0.0 ? (motion_wall_s - motion_s) / motion_s * 100.0 : 0.0;
            RCLCPP_INFO(logger, "  실측(벽시계) %5.2f s          (계획 대비 %+.1f%% — 벌어진 만큼이 로봇/컨트롤러 쪽 문제다)",
                        motion_wall_s, gap);
        }

        RCLCPP_INFO(logger,
                    "  ⚠️ 펜업이 전체의 %.1f%% 다 — **순서 최적화로 줄일 수 있는 시간의 상한**이 이 값이다.",
                    pct(travel_s));
    }

    const rclcpp::Time end_time = node->now() + rclcpp::Duration::from_seconds(hold_seconds);
    while (rclcpp::ok() && node->now() < end_time)
    {
        rclcpp::sleep_for(std::chrono::milliseconds(500));
    }

    return shutdown(0);
}
