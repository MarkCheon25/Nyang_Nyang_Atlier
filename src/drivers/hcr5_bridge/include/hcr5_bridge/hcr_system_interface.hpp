// HcrSystemInterface — HCR-5 실기를 ros2_control 하드웨어 컴포넌트로 노출한다.
//
// mock_components/GenericSystem 이 앉아 있던 자리를 실기로 바꾼다. 이 클래스가 붙으면
// MoveIt2 의 실행 경로(FollowJointTrajectory → JTC → 하드웨어)가 실기까지 이어진다.
//
// 계약은 상위 문서가 소유한다 —
//   src/drivers/ros2_control_hw_interface/중간결과물.md
//   §1 인터페이스 3종 · §2 주기 · §3 라이프사이클 · §5 안전게이트 7건
// 이 파일과 그 문서가 어긋나면 **문서가 이긴다.**
//
// ── 이 컴포넌트가 푸는 문제: 주기가 셋 다 다르다 ──────────────────────────
//   controller_manager   100Hz   ← 손대지 않는다
//   실기 상태 버스        29.1Hz  ← <ros2_control rw_rate="30"> 으로 분리
//   명령 RPC 왕복        115~137ms ← 워커 스레드로 분리 (read/write 를 절대 막지 않는다)
//
// read() 는 MQTT 콜백이 채워둔 캐시를 **복사만** 하고, write() 는 워커에 **큐잉만** 한다.
// 둘 다 블로킹이 없다.
//
// ⚠️ 관절 규약 변환(joint_convention.hpp)을 빼먹으면 에러 없이 로봇이 엉뚱한 자세로 간다.
#ifndef HCR5_BRIDGE__HCR_SYSTEM_INTERFACE_HPP_
#define HCR5_BRIDGE__HCR_SYSTEM_INTERFACE_HPP_

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/duration.hpp>
#include <rclcpp/time.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "hcr5_bridge/joint_convention.hpp"
#include "hcr5_bridge/mqtt_client.hpp"

namespace hcr5_bridge
{

using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class HcrSystemInterface : public hardware_interface::SystemInterface
{
public:
  HcrSystemInterface() = default;
  ~HcrSystemInterface() override;

  // ── 라이프사이클 (중간결과물 §3) ────────────────────────────────────────
  // export_state_interfaces()·export_command_interfaces() 는 **재정의하지 않는다.**
  // ros2_control 4.45.2 에서 deprecated 이고, 프레임워크가 URDF 선언대로 자동 export 한다.
  CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
  CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  using Clock = std::chrono::steady_clock;
  using JointArray = std::array<double, kNumJoints>;

  // ── MQTT 구독 콜백 (수신 스레드에서 돈다) ───────────────────────────────
  void onJointPosition(const Json & envelope);
  void onOperation(const Json & envelope);
  void onCollision();
  void onCollisionClear();

  // ── 명령 발행 워커 ─────────────────────────────────────────────────────
  void workerLoop();
  void startWorker();
  void stopWorker();
  /// move/stop 을 잠금과 무관하게 즉시 발행한다 (정지는 게이트에 걸리면 안 된다).
  void publishStop();

  bool waitForFirstSample(std::chrono::milliseconds timeout);
  /// status/operation 이 필드버스 CONNECTED 를 세울 때까지 상한을 걸고 기다린다.
  /// 그 토픽은 51.5ms 주기라 on_configure 직후엔 아직 안 와 있다 — 즉시 요구하면 경합이 난다.
  bool waitForFieldbus(std::chrono::milliseconds timeout);
  /// URDF 선언이 계약(§1)과 맞는지 대조. 어긋나면 사유를 로그에 남기고 false.
  bool validateInterfaces();

  // ── 파라미터 (URDF <param> — on_init 에서 파싱) ─────────────────────────
  std::string host_{"192.168.0.20"};
  int port_{1883};
  bool allow_motion_{false};
  double deadband_deg_{0.01};
  std::chrono::milliseconds connect_timeout_{3000};   // 첫 표본 대기 상한
  std::chrono::milliseconds state_timeout_{1000};     // 무수신 → DEACTIVATE
  std::chrono::milliseconds min_publish_interval_{137};  // 재발행 간격 하한 (ack 왕복 실측)

  std::unique_ptr<MqttClient> mqtt_;

  // ── 상태 캐시: MQTT 수신 스레드 → read() ───────────────────────────────
  mutable std::mutex state_mutex_;
  std::condition_variable state_cv_;
  JointArray cache_rad_{};        // 규약 변환까지 끝낸 URDF 라디안
  JointArray prev_cache_rad_{};
  Clock::time_point cache_stamp_{};
  Clock::time_point prev_cache_stamp_{};
  std::uint64_t sample_seq_{0};   // 표본 일련번호 — read() 가 '새 표본인가'를 이걸로 판정
  bool have_state_{false};

  // ── read() 산출물 ──────────────────────────────────────────────────────
  std::uint64_t last_read_seq_{0};
  JointArray vel_rad_s_{};        // 새 표본이 온 주기에만 갱신 (§1)
  bool warned_stale_{false};

  // ── 실기 상태 감시 (§5 게이트 3·4) ─────────────────────────────────────
  std::atomic<bool> fieldbus_ok_{false};
  std::atomic<bool> collision_latched_{false};
  std::mutex fieldbus_mutex_;
  std::condition_variable fieldbus_cv_;
  std::string last_fieldbus_;

  // ── 명령: write() → 워커 ───────────────────────────────────────────────
  std::mutex cmd_mutex_;
  std::condition_variable cmd_cv_;
  JointArray pending_real_deg_{};   // 워커가 아직 안 보낸 최신 목표 (합쳐진다)
  bool pending_valid_{false};
  JointArray last_sent_real_deg_{};
  bool have_sent_{false};
  JointArray prev_cmd_real_deg_{};  // 직전 주기의 명령 — '목표가 멎었는가' 판정용
  bool have_prev_cmd_{false};
  bool cmd_moving_{false};

  // 발행 잠금. on_activate 에서만 풀리고, on_deactivate/on_error/충돌에서 잠긴다.
  std::atomic<bool> publish_locked_{true};
  std::atomic<bool> publish_permanently_locked_{false};

  std::thread worker_;
  std::atomic<bool> worker_running_{false};

  // 한 번만 내는 경고들
  bool warned_deadman_{false};
  std::atomic<bool> warned_limits_{false};
  std::atomic<bool> warned_fieldbus_gate_{false};
};

}  // namespace hcr5_bridge

#endif  // HCR5_BRIDGE__HCR_SYSTEM_INTERFACE_HPP_
