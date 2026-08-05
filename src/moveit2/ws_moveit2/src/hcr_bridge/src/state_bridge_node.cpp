// hcr_state_bridge — 실기 HCR-5 의 관절 상태를 ROS2 로 흘린다 (읽기 전용 + 최소 제어).
//
// 이 노드가 하는 일
//   · MQTT motion/joint/position (~29Hz, 도 단위) → /joint_states (URDF 규약, 라디안)
//   · status/operation·event/collision 감시 → 경고 로그
//   · 서비스: 서보 ON/OFF, 홈 복귀 (동작 지령이므로 기본값은 전부 비활성)
//
// 안전 — PC 에서 보내는 MQTT 명령은 **펜던트 인에이블 스위치(데드맨)를 거치지 않는다.**
// 그래서 동작 지령 서비스는 파라미터 allow_motion 이 true 일 때만 열린다.

#include <array>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "hcr_bridge/joint_convention.hpp"
#include "hcr_bridge/mqtt_client.hpp"

namespace hcr_bridge
{

class StateBridgeNode : public rclcpp::Node
{
public:
  StateBridgeNode()
  : Node("hcr_state_bridge")
  {
    host_ = declare_parameter<std::string>("host", "192.168.0.20");
    port_ = declare_parameter<int>("port", 1883);
    allow_motion_ = declare_parameter<bool>("allow_motion", false);
    const auto publish_rate = declare_parameter<double>("publish_rate", 50.0);

    js_pub_ = create_publisher<sensor_msgs::msg::JointState>("joint_states", rclcpp::SensorDataQoS());

    mqtt_ = std::make_unique<MqttClient>(host_, port_);
    mqtt_->subscribe(
      "motion/joint/position",
      [this](const std::string &, const Json & j) { onJointPosition(j); });
    mqtt_->subscribe(
      "status/operation",
      [this](const std::string &, const Json & j) { onOperation(j); });
    mqtt_->subscribe(
      "event/collision",
      [this](const std::string &, const Json &) {
        RCLCPP_ERROR(
          get_logger(),
          "충돌 감지 — 로봇이 PAUSED 로 래치됐다. event/collision/clear 로 해제해야 재개된다");
      });

    if (!mqtt_->start()) {
      RCLCPP_FATAL(get_logger(), "MQTT 접속 실패: %s:%d", host_.c_str(), port_);
      throw std::runtime_error("MQTT 접속 실패");
    }
    RCLCPP_INFO(get_logger(), "MQTT 접속: %s:%d", host_.c_str(), port_);
    if (!allow_motion_) {
      RCLCPP_INFO(get_logger(), "allow_motion=false — 동작 지령 서비스는 거부한다 (읽기 전용)");
    }

    servo_srv_ = create_service<std_srvs::srv::SetBool>(
      "~/set_servo",
      [this](
        const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
        std::shared_ptr<std_srvs::srv::SetBool::Response> res) { onSetServo(req, res); });

    home_srv_ = create_service<std_srvs::srv::Trigger>(
      "~/go_home",
      [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) { onGoHome(res); });

    timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / publish_rate), [this] { publishJointState(); });
  }

private:
  void onJointPosition(const Json & env)
  {
    // 봉투: {"type":"pub","uuid":...,"data":{"thng_id":"1","data":{base:...,...}}}
    const auto & outer = env.value("data", Json::object());
    const auto & inner = outer.value("data", Json::object());
    if (!inner.contains("base")) { return; }

    std::array<double, kNumJoints> real_deg{};
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      if (!inner.contains(kMqttJointKeys[i])) { return; }
      real_deg[i] = inner[kMqttJointKeys[i]].get<double>();
    }
    const auto urdf_rad = realDegToUrdfRad(real_deg);

    std::lock_guard<std::mutex> lk(state_mutex_);
    latest_ = urdf_rad;
    have_state_ = true;
    last_rx_ = now();
  }

  void onOperation(const Json & env)
  {
    const auto & outer = env.value("data", Json::object());
    const auto & d = outer.value("data", Json::object());
    const auto fieldbus = d.value("controllerStatus", std::string{});
    if (!fieldbus.empty() && fieldbus != last_fieldbus_) {
      if (fieldbus != "FIELD_BUS_SW_STATE_CONNECTED") {
        RCLCPP_ERROR(get_logger(), "필드버스 상태: %s — 드라이브 통신 확인 필요", fieldbus.c_str());
      } else if (!last_fieldbus_.empty()) {
        RCLCPP_INFO(get_logger(), "필드버스 복구: %s", fieldbus.c_str());
      }
      last_fieldbus_ = fieldbus;
    }
    servo_on_ = (d.value("operationStatus", std::string{}) == "SERVO_ON");
  }

  void publishJointState()
  {
    std::array<double, kNumJoints> q{};
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      if (!have_state_) {
        if ((now() - node_start_).seconds() > 5.0 && !warned_no_state_) {
          RCLCPP_WARN(get_logger(), "motion/joint/position 을 아직 못 받았다 — 실기 전원·연결 확인");
          warned_no_state_ = true;
        }
        return;
      }
      // 상태가 끊기면 낡은 값을 계속 뿌리지 않는다.
      if ((now() - last_rx_).seconds() > 1.0) {
        if (!warned_stale_) {
          RCLCPP_ERROR(get_logger(), "상태 수신 중단(>1s) — joint_states 발행을 멈춘다");
          warned_stale_ = true;
        }
        return;
      }
      warned_stale_ = false;
      q = latest_;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = now();
    msg.name.assign(kUrdfJointNames.begin(), kUrdfJointNames.end());
    msg.position.assign(q.begin(), q.end());
    js_pub_->publish(msg);
  }

  void onSetServo(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res)
  {
    if (!allow_motion_) {
      res->success = false;
      res->message = "allow_motion=false 라 거부했다";
      return;
    }
    const Json d{{"operationStatus", req->data ? "SERVO_ON" : "SERVO_OFF"}};
    const auto r = mqtt_->request("set/operation", d);
    res->success = r.has_value();
    res->message = r ? "ok" : "ack 없음 또는 실패";
    RCLCPP_INFO(get_logger(), "서보 %s → %s", req->data ? "ON" : "OFF", res->message.c_str());
  }

  void onGoHome(std::shared_ptr<std_srvs::srv::Trigger::Response> res)
  {
    if (!allow_motion_) {
      res->success = false;
      res->message = "allow_motion=false 라 거부했다";
      return;
    }
    if (!servo_on_) {
      res->success = false;
      res->message = "서보가 꺼져 있다";
      return;
    }
    // move/joint/home 은 관절값 없이 속도만 받는다 — 홈 자세는 컨트롤러가 안다.
    RCLCPP_WARN(get_logger(), "홈 복귀 시작 — 로봇이 움직인다. e-stop 대기할 것");
    const auto r = mqtt_->request("move/joint/home", Json{{"velocity", "22.50"}});
    res->success = r.has_value();
    res->message = r ? "명령 접수(도착은 event/motion 으로 판정)" : "ack 없음";
  }

  std::string host_;
  int port_{1883};
  bool allow_motion_{false};

  std::unique_ptr<MqttClient> mqtt_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr js_pub_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr servo_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr home_srv_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::mutex state_mutex_;
  std::array<double, kNumJoints> latest_{};
  bool have_state_{false};
  bool warned_stale_{false};
  bool warned_no_state_{false};
  bool servo_on_{false};
  std::string last_fieldbus_;
  rclcpp::Time last_rx_{0, 0, RCL_ROS_TIME};
  rclcpp::Time node_start_{now()};
};

}  // namespace hcr_bridge

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  try {
    rclcpp::spin(std::make_shared<hcr_bridge::StateBridgeNode>());
  } catch (const std::exception & e) {
    RCLCPP_FATAL(rclcpp::get_logger("hcr_state_bridge"), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
