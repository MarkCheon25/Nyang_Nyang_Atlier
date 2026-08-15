#include "hcr5_bridge/hcr_system_interface.hpp"

#include <netdb.h>
#include <poll.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <stdexcept>
#include <vector>

#include <hardware_interface/lexical_casts.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>
#include <rclcpp/logging.hpp>

namespace hcr5_bridge
{

namespace
{

constexpr const char * kTopicJointPosition = "motion/joint/position";
constexpr const char * kTopicOperation = "status/operation";
constexpr const char * kTopicCollision = "event/collision";
constexpr const char * kTopicCollisionClear = "event/collision/clear";
constexpr const char * kTopicMoveHere = "move/joint/here";
constexpr const char * kTopicMoveStop = "move/stop";
constexpr const char * kTopicGetPos = "get/command/pos";
constexpr const char * kFieldBusConnected = "FIELD_BUS_SW_STATE_CONNECTED";

/// 목표가 '멎었다'로 볼 변화량(도). 부동소수 지터만 흡수하는 크기다.
constexpr double kSettleEpsDeg = 1e-9;

/// on_activate 가 필드버스 CONNECTED 를 기다리는 상한.
/// status/operation 실측 주기 51.5ms 의 약 9.7배 — 몇 주기 놓쳐도 통과하되,
/// 필드버스가 정말 죽어 있으면 여기서 끊고 ERROR 로 간다.
constexpr std::chrono::milliseconds kFieldbusWait{500};

/// 브로커에 TCP 로 닿는지 **시간 상한을 걸어** 확인한다.
///
/// mosquitto_connect 는 블로킹이고 취소가 안 된다 — 응답 없는 호스트를 물면 OS 기본
/// 타임아웃(수십 초~2분)까지 잡혀 있는다. on_configure 가 "얌전히 ERROR" 로 죽으려면
/// 그 앞에서 먼저 끊어야 한다.
bool tcpProbe(const std::string & host, int port, std::chrono::milliseconds timeout)
{
  addrinfo hints{};
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;

  const std::string port_str = std::to_string(port);
  addrinfo * res = nullptr;
  if (::getaddrinfo(host.c_str(), port_str.c_str(), &hints, &res) != 0 || res == nullptr) {
    return false;
  }

  bool ok = false;
  for (addrinfo * ai = res; ai != nullptr && !ok; ai = ai->ai_next) {
    const int fd = ::socket(ai->ai_family, ai->ai_socktype | SOCK_NONBLOCK, ai->ai_protocol);
    if (fd < 0) { continue; }

    if (::connect(fd, ai->ai_addr, ai->ai_addrlen) == 0) {
      ok = true;
    } else if (errno == EINPROGRESS) {
      pollfd pfd{fd, POLLOUT, 0};
      if (::poll(&pfd, 1, static_cast<int>(timeout.count())) > 0) {
        int err = 0;
        socklen_t len = sizeof(err);
        ok = (::getsockopt(fd, SOL_SOCKET, SO_ERROR, &err, &len) == 0) && (err == 0);
      }
    }
    ::close(fd);
  }
  ::freeaddrinfo(res);
  return ok;
}

/// 봉투 두 겹({"data":{"data":{...}}})을 벗긴다 — 로봇 상태 발행의 공통 형태다.
Json unwrap(const Json & envelope)
{
  const auto outer = envelope.value("data", Json::object());
  return outer.value("data", Json::object());
}

}  // namespace

HcrSystemInterface::~HcrSystemInterface()
{
  stopWorker();
  if (mqtt_) { mqtt_->stop(); }
}

// ══ on_init ══════════════════════════════════════════════════════════════
// URDF <param> 파싱 + 인터페이스 선언이 계약과 맞는지 대조. 실기는 아직 안 만진다.
CallbackReturn HcrSystemInterface::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (hardware_interface::SystemInterface::on_init(params) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  const auto & p = info_.hardware_parameters;
  const auto get = [&p](const std::string & key) -> const std::string * {
    const auto it = p.find(key);
    return it == p.end() ? nullptr : &it->second;
  };

  try {
    if (const auto * v = get("host")) { host_ = *v; }
    if (const auto * v = get("port")) { port_ = hardware_interface::stoi32(*v); }
    if (const auto * v = get("allow_motion")) { allow_motion_ = hardware_interface::parse_bool(*v); }
    if (const auto * v = get("deadband_deg")) { deadband_deg_ = hardware_interface::stod(*v); }
    if (const auto * v = get("connect_timeout_ms")) {
      connect_timeout_ = std::chrono::milliseconds(hardware_interface::stoi32(*v));
    }
    if (const auto * v = get("state_timeout_ms")) {
      state_timeout_ = std::chrono::milliseconds(hardware_interface::stoi32(*v));
    }
    if (const auto * v = get("min_publish_interval_ms")) {
      min_publish_interval_ = std::chrono::milliseconds(hardware_interface::stoi32(*v));
    }
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "하드웨어 <param> 파싱 실패: %s", e.what());
    return CallbackReturn::ERROR;
  }

  if (deadband_deg_ < 0.0) {
    RCLCPP_ERROR(get_logger(), "deadband_deg 가 음수다: %.6f", deadband_deg_);
    return CallbackReturn::ERROR;
  }

  if (!validateInterfaces()) { return CallbackReturn::ERROR; }

  RCLCPP_INFO(
    get_logger(),
    "HcrSystemInterface on_init — %s:%d · allow_motion=%s · deadband=%.4f° · "
    "rw_rate=%uHz · is_async=%s",
    host_.c_str(), port_, allow_motion_ ? "true" : "false", deadband_deg_,
    info_.rw_rate, info_.is_async ? "true" : "false");

  if (!allow_motion_) {
    RCLCPP_INFO(get_logger(), "allow_motion=false — write() 는 아무것도 발행하지 않는다 (읽기 전용)");
  } else {
    // §5 게이트 5 — 데드맨 우회 고지. 이 경고는 기동 시 한 번만 낸다.
    RCLCPP_WARN(
      get_logger(),
      "⚠️ allow_motion=true — 로봇이 실제로 움직인다. PC 의 MQTT 명령은 펜던트 인에이블"
      "(데드맨) 스위치를 거치지 않는다. e-stop 대기·반경 정리 상태에서만 쓸 것");
    warned_deadman_ = true;
  }
  return CallbackReturn::SUCCESS;
}

// URDF 선언 ↔ 계약(§1) 대조. 여기서 걸러야 규약 변환이 조용히 엇나가는 걸 막는다.
bool HcrSystemInterface::validateInterfaces()
{
  if (info_.joints.size() != kNumJoints) {
    RCLCPP_ERROR(
      get_logger(), "관절 수가 %zu 다 — 계약은 %zu 개(joint_1~joint_6)",
      info_.joints.size(), kNumJoints);
    return false;
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto & j = info_.joints[i];

    // 이름과 **순서**가 둘 다 맞아야 한다. 순서가 틀리면 규약 변환이 축을 바꿔 적용한다.
    if (j.name != kUrdfJointNames[i]) {
      RCLCPP_ERROR(
        get_logger(), "관절 %zu 의 이름이 '%s' 다 — 계약은 '%s' (선언 순서가 축 순서다)",
        i, j.name.c_str(), kUrdfJointNames[i].c_str());
      return false;
    }

    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_ERROR(
        get_logger(), "'%s' 의 command_interface 가 계약과 다르다 — position 1개여야 한다",
        j.name.c_str());
      return false;
    }

    bool has_pos = false;
    bool has_vel = false;
    for (const auto & si : j.state_interfaces) {
      has_pos |= (si.name == hardware_interface::HW_IF_POSITION);
      has_vel |= (si.name == hardware_interface::HW_IF_VELOCITY);
    }
    if (!has_pos || !has_vel) {
      RCLCPP_ERROR(
        get_logger(), "'%s' 의 state_interface 가 계약과 다르다 — position·velocity 둘 다 필요",
        j.name.c_str());
      return false;
    }
  }
  return true;
}

// ══ on_configure ═════════════════════════════════════════════════════════
// MQTT 접속 · 구독 · **첫 상태 표본 수신까지 대기**. 실기가 없으면 여기서 얌전히 죽는다.
CallbackReturn HcrSystemInterface::on_configure(const rclcpp_lifecycle::State &)
{
  publish_locked_ = true;
  collision_latched_ = false;
  fieldbus_ok_ = false;
  {
    // 리셋하지 않으면 on_activate 의 실패 메시지가 자기모순이 된다
    // ("CONNECTED 가 아니다 (현재 'CONNECTED')") — 직전 세션 값이 그대로 남기 때문이다.
    std::lock_guard<std::mutex> lk(fieldbus_mutex_);
    last_fieldbus_.clear();
  }
  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    have_state_ = false;
    sample_seq_ = 0;
    last_read_seq_ = 0;
    vel_rad_s_.fill(0.0);
  }

  if (!tcpProbe(host_, port_, connect_timeout_)) {
    RCLCPP_ERROR(
      get_logger(), "브로커에 TCP 로 닿지 않는다: %s:%d (%lldms 안). 랜선·전원·"
      "`nmcli connection up hcr5-probe` 확인",
      host_.c_str(), port_, static_cast<long long>(connect_timeout_.count()));
    return CallbackReturn::ERROR;
  }

  mqtt_ = std::make_unique<MqttClient>(host_, port_, "hcr-ros2-control");
  mqtt_->subscribe(
    kTopicJointPosition, [this](const std::string &, const Json & j) { onJointPosition(j); });
  mqtt_->subscribe(kTopicOperation, [this](const std::string &, const Json & j) { onOperation(j); });
  mqtt_->subscribe(kTopicCollision, [this](const std::string &, const Json &) { onCollision(); });
  // 해제는 **사람이** event/collision/clear 를 발행한다(§5). 플러그인은 그걸 버스에서
  // 보고 래치를 푼다 — 스스로 풀지 않는다.
  mqtt_->subscribe(
    kTopicCollisionClear, [this](const std::string &, const Json &) { onCollisionClear(); });

  if (!mqtt_->start()) {
    RCLCPP_ERROR(get_logger(), "MQTT 접속 실패: %s:%d", host_.c_str(), port_);
    mqtt_.reset();
    return CallbackReturn::ERROR;
  }

  if (!waitForFirstSample(connect_timeout_)) {
    RCLCPP_ERROR(
      get_logger(),
      "%s 첫 표본을 %lldms 안에 못 받았다 — 브로커는 붙었지만 로봇이 상태를 안 낸다",
      kTopicJointPosition, static_cast<long long>(connect_timeout_.count()));
    mqtt_->stop();
    mqtt_.reset();
    return CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_logger(), "MQTT 접속·첫 표본 수신: %s:%d", host_.c_str(), port_);
  return CallbackReturn::SUCCESS;
}

bool HcrSystemInterface::waitForFirstSample(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lk(state_mutex_);
  return state_cv_.wait_for(lk, timeout, [this] { return have_state_; });
}

bool HcrSystemInterface::waitForFieldbus(std::chrono::milliseconds timeout)
{
  std::unique_lock<std::mutex> lk(fieldbus_mutex_);
  return fieldbus_cv_.wait_for(lk, timeout, [this] { return fieldbus_ok_.load(); });
}

// ══ on_activate ══════════════════════════════════════════════════════════
// 필드버스·상태 신선도 확인 후 **명령 버퍼를 현재 실기 자세로 초기화**한다.
// 이걸 빼면 첫 write() 가 0 이나 낡은 값으로 도약한다.
CallbackReturn HcrSystemInterface::on_activate(const rclcpp_lifecycle::State &)
{
  if (!mqtt_ || !mqtt_->connected()) {
    RCLCPP_ERROR(get_logger(), "MQTT 가 붙어 있지 않다");
    return CallbackReturn::ERROR;
  }

  // on_configure 는 motion/joint/position 첫 표본만 기다린다(29Hz). 필드버스 플래그를 세우는
  // status/operation 은 51.5ms 주기라 configure 직후엔 아직 안 와 있을 수 있다 — 즉시 요구하면
  // configure→activate 간격이 짧을 때(재활성은 1ms 미만) 반드시 실패한다.
  if (!waitForFieldbus(kFieldbusWait)) {
    std::lock_guard<std::mutex> lk(fieldbus_mutex_);
    RCLCPP_ERROR(
      get_logger(),
      "필드버스가 %s 가 아니다 (%lldms 대기, 현재 '%s') — 드라이브 통신을 먼저 살려야 한다",
      kFieldBusConnected, static_cast<long long>(kFieldbusWait.count()),
      last_fieldbus_.empty() ? "미수신" : last_fieldbus_.c_str());
    return CallbackReturn::ERROR;
  }

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (!have_state_ || (Clock::now() - cache_stamp_) > state_timeout_) {
      RCLCPP_ERROR(get_logger(), "상태가 신선하지 않다 — 활성화하지 않는다");
      return CallbackReturn::ERROR;
    }
  }

  // 명령 버퍼 초기화. 우선 컨트롤러가 아는 '지령 자세'(get/command/pos)를 쓰고,
  // RPC 가 실패하면 스트리밍 상태로 대신한다.
  JointArray init_real_deg{};
  bool from_rpc = false;
  if (const auto r = mqtt_->request(kTopicGetPos, Json::object())) {
    const auto it = r->find("joint");
    if (it != r->end() && it->is_array() && it->size() == kNumJoints) {
      for (std::size_t i = 0; i < kNumJoints; ++i) { init_real_deg[i] = (*it)[i].get<double>(); }
      from_rpc = true;
    }
  }
  if (!from_rpc) {
    JointArray rad{};
    {
      std::lock_guard<std::mutex> lk(state_mutex_);
      rad = cache_rad_;
    }
    init_real_deg = urdfRadToRealDeg(rad);
    RCLCPP_WARN(get_logger(), "%s 응답이 없어 스트리밍 상태로 명령 버퍼를 초기화한다", kTopicGetPos);
  }

  const auto init_rad = realDegToUrdfRad(init_real_deg);
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    set_command(kUrdfJointNames[i] + "/position", init_rad[i]);
  }

  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    pending_valid_ = false;
    have_sent_ = false;
    cmd_moving_ = false;
    prev_cmd_real_deg_ = init_real_deg;
    have_prev_cmd_ = true;
    last_sent_real_deg_ = init_real_deg;
  }

  warned_stale_ = false;
  warned_limits_ = false;
  warned_fieldbus_gate_ = false;

  if (allow_motion_ && !publish_permanently_locked_) {
    startWorker();
    publish_locked_ = false;
    RCLCPP_WARN(get_logger(), "⚠️ 동작 지령 잠금 해제 — 이제 write() 가 실기를 움직인다");
  } else if (publish_permanently_locked_) {
    RCLCPP_ERROR(get_logger(), "발행이 영구 잠겨 있다(on_error 이력) — 읽기만 한다");
  }

  RCLCPP_INFO(
    get_logger(), "활성화 — 명령 버퍼 = 실기 [%.3f %.3f %.3f %.3f %.3f %.3f]° (%s)",
    init_real_deg[0], init_real_deg[1], init_real_deg[2], init_real_deg[3], init_real_deg[4],
    init_real_deg[5], from_rpc ? kTopicGetPos : "스트리밍 상태");
  return CallbackReturn::SUCCESS;
}

// ══ on_deactivate ════════════════════════════════════════════════════════
// move/stop 을 내고 발행을 잠근다. **서보는 끄지 않는다** — 브레이크 판단은 사람 몫이다.
CallbackReturn HcrSystemInterface::on_deactivate(const rclcpp_lifecycle::State &)
{
  publish_locked_ = true;
  publishStop();
  stopWorker();
  RCLCPP_INFO(get_logger(), "비활성화 — move/stop 발행·발행 잠금. 서보는 그대로 둔다");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HcrSystemInterface::on_cleanup(const rclcpp_lifecycle::State &)
{
  publish_locked_ = true;
  stopWorker();
  if (mqtt_) {
    mqtt_->stop();
    mqtt_.reset();
  }
  RCLCPP_INFO(get_logger(), "정리 — MQTT 종료·워커 회수");
  return CallbackReturn::SUCCESS;
}

CallbackReturn HcrSystemInterface::on_error(const rclcpp_lifecycle::State &)
{
  publish_locked_ = true;
  publish_permanently_locked_ = true;   // 이 인스턴스는 다시 발행하지 않는다
  publishStop();
  stopWorker();
  RCLCPP_ERROR(get_logger(), "에러 — move/stop 발행·발행 영구 잠금");
  return CallbackReturn::SUCCESS;
}

// ══ read ═════════════════════════════════════════════════════════════════
// 캐시 복사만 한다. MQTT 수신 스레드가 이미 규약 변환까지 끝내 뒀다.
hardware_interface::return_type HcrSystemInterface::read(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  JointArray q{};
  bool fresh_sample = false;
  double dt = 0.0;
  JointArray q_prev{};

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (!have_state_) {
      RCLCPP_ERROR(get_logger(), "상태를 한 번도 못 받았다");
      return hardware_interface::return_type::DEACTIVATE;
    }

    const auto age = Clock::now() - cache_stamp_;
    if (age > state_timeout_) {
      if (!warned_stale_) {
        RCLCPP_ERROR(
          get_logger(), "상태 수신 중단(%lldms > %lldms) — 낡은 자세를 참으로 믿게 두지 않는다. "
          "비활성으로 내린다",
          static_cast<long long>(
            std::chrono::duration_cast<std::chrono::milliseconds>(age).count()),
          static_cast<long long>(state_timeout_.count()));
        warned_stale_ = true;
      }
      return hardware_interface::return_type::DEACTIVATE;
    }
    warned_stale_ = false;

    q = cache_rad_;
    if (sample_seq_ != last_read_seq_) {
      fresh_sample = true;
      q_prev = prev_cache_rad_;
      dt = std::chrono::duration<double>(cache_stamp_ - prev_cache_stamp_).count();
      last_read_seq_ = sample_seq_;
    }
  }

  // velocity 는 실기 원천이 없는 **산출값**이다 (§1). 새 표본이 온 주기에만 갱신하고
  // 그 사이에는 직전 값을 유지한다 — 표본 없이 0 을 넣으면 JTC 피드백이 톱니가 된다.
  if (fresh_sample && dt > 0.0) {
    for (std::size_t i = 0; i < kNumJoints; ++i) { vel_rad_s_[i] = (q[i] - q_prev[i]) / dt; }
  }

  for (std::size_t i = 0; i < kNumJoints; ++i) {
    set_state(kUrdfJointNames[i] + "/position", q[i]);
    set_state(kUrdfJointNames[i] + "/velocity", vel_rad_s_[i]);
  }
  return hardware_interface::return_type::OK;
}

// ══ write ════════════════════════════════════════════════════════════════
// 게이트 5종을 통과하면 워커에 큐잉만 한다. 여기서 블로킹하면 rw 루프가 멎는다.
hardware_interface::return_type HcrSystemInterface::write(
  const rclcpp::Time &, const rclcpp::Duration &)
{
  // 게이트 1 — 동작 지령 잠금. 기본값이 잠김이고, 여기서 조용히 끝난다.
  if (!allow_motion_ || publish_locked_ || publish_permanently_locked_) {
    return hardware_interface::return_type::OK;
  }

  // 게이트 4 — 충돌 래치. 사람이 event/collision/clear 를 낼 때까지 안 움직인다.
  if (collision_latched_) { return hardware_interface::return_type::OK; }

  // 게이트 3 — 필드버스. 드라이브가 죽은 채로 목표를 밀어넣지 않는다.
  if (!fieldbus_ok_) {
    if (!warned_fieldbus_gate_.exchange(true)) {
      RCLCPP_ERROR(get_logger(), "필드버스 두절 — 발행을 보류한다");
    }
    return hardware_interface::return_type::OK;
  }
  warned_fieldbus_gate_ = false;

  JointArray cmd_rad{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    cmd_rad[i] = get_command(kUrdfJointNames[i] + "/position");
    if (!std::isfinite(cmd_rad[i])) { return hardware_interface::return_type::OK; }
  }
  const auto cmd_real_deg = urdfRadToRealDeg(cmd_rad);

  // 게이트 6 — 관절한계. 공식 가동범위와 URDF 한계를 **둘 다** 본다.
  if (!withinLimits(cmd_real_deg)) {
    if (!warned_limits_.exchange(true)) {
      RCLCPP_ERROR(
        get_logger(), "목표가 공식 가동범위 밖이다 (±360°, J3 ±165°) — 발행하지 않는다");
    }
    return hardware_interface::return_type::OK;
  }
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto it = info_.limits.find(kUrdfJointNames[i]);
    if (it == info_.limits.end() || !it->second.has_position_limits) { continue; }
    if (cmd_rad[i] < it->second.min_position || cmd_rad[i] > it->second.max_position) {
      if (!warned_limits_.exchange(true)) {
        RCLCPP_ERROR(
          get_logger(), "'%s' 목표 %.4frad 가 URDF 한계 [%.4f, %.4f] 밖이다 — 발행하지 않는다",
          kUrdfJointNames[i].c_str(), cmd_rad[i], it->second.min_position,
          it->second.max_position);
      }
      return hardware_interface::return_type::OK;
    }
  }
  warned_limits_ = false;

  // 게이트 7 — 데드밴드. 단, **목표가 멎으면 데드밴드와 무관하게 마지막 값을 1회** 낸다.
  // 분해능은 병목이 아니므로(hcr5_comm README §5.3) 전정밀도 그대로 싣는다.
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);

    double moved = 0.0;
    if (have_prev_cmd_) {
      for (std::size_t i = 0; i < kNumJoints; ++i) {
        moved = std::max(moved, std::fabs(cmd_real_deg[i] - prev_cmd_real_deg_[i]));
      }
    }
    double from_sent = 0.0;
    for (std::size_t i = 0; i < kNumJoints; ++i) {
      from_sent = std::max(from_sent, std::fabs(cmd_real_deg[i] - last_sent_real_deg_[i]));
    }

    bool enqueue = false;
    if (!have_prev_cmd_ || moved > kSettleEpsDeg) {
      cmd_moving_ = true;                                  // 목표가 아직 움직이는 중
      enqueue = (!have_sent_ || from_sent >= deadband_deg_);
    } else if (cmd_moving_) {
      cmd_moving_ = false;                                 // 방금 멎었다 → 최종값 확정 발행
      enqueue = (!have_sent_ || from_sent > 0.0);
    }

    prev_cmd_real_deg_ = cmd_real_deg;
    have_prev_cmd_ = true;

    if (enqueue) {
      // 아직 안 나간 목표가 있으면 **덮어쓴다** — 낡은 목표를 뒤늦게 보내면 안 된다.
      pending_real_deg_ = cmd_real_deg;
      pending_valid_ = true;
      cmd_cv_.notify_one();
    }
  }
  return hardware_interface::return_type::OK;
}

// ══ 워커 — 유일하게 블로킹이 허용된 곳 ═══════════════════════════════════
void HcrSystemInterface::startWorker()
{
  if (worker_running_.exchange(true)) { return; }
  worker_ = std::thread(&HcrSystemInterface::workerLoop, this);
}

void HcrSystemInterface::stopWorker()
{
  if (!worker_running_.exchange(false)) { return; }
  cmd_cv_.notify_all();
  if (worker_.joinable()) { worker_.join(); }
}

void HcrSystemInterface::workerLoop()
{
  auto last_publish = Clock::now() - min_publish_interval_;

  while (worker_running_) {
    JointArray target{};
    {
      std::unique_lock<std::mutex> lk(cmd_mutex_);
      cmd_cv_.wait_for(
        lk, std::chrono::milliseconds(100),
        [this] { return pending_valid_ || !worker_running_; });
      if (!worker_running_) { break; }
      if (!pending_valid_) { continue; }
      target = pending_real_deg_;
      pending_valid_ = false;
    }

    if (publish_locked_ || publish_permanently_locked_ || collision_latched_) { continue; }

    // 게이트 7 — 재발행 간격 하한. ack 왕복이 115~137ms 라 그보다 촘촘하면 겹친다.
    const auto since = Clock::now() - last_publish;
    if (since < min_publish_interval_) { std::this_thread::sleep_for(min_publish_interval_ - since); }
    if (!worker_running_ || publish_locked_ || publish_permanently_locked_) { continue; }

    Json angles = Json::array();
    for (const double d : target) { angles.push_back(d); }

    const auto reply = mqtt_->request(kTopicMoveHere, Json{{"jointAngle", angles}});
    last_publish = Clock::now();

    if (reply) {
      std::lock_guard<std::mutex> lk(cmd_mutex_);
      last_sent_real_deg_ = target;
      have_sent_ = true;
      RCLCPP_DEBUG(
        get_logger(), "%s [%.4f %.4f %.4f %.4f %.4f %.4f]°", kTopicMoveHere, target[0], target[1],
        target[2], target[3], target[4], target[5]);
    } else {
      // ack 가 없으면 보냈다고 치지 않는다 — 다음 목표가 데드밴드에 먹히면 안 된다.
      RCLCPP_WARN(get_logger(), "%s ack 없음 — 발행 실패로 본다", kTopicMoveHere);
    }
  }
}

void HcrSystemInterface::publishStop()
{
  if (mqtt_ && mqtt_->connected()) { mqtt_->publish(kTopicMoveStop, Json::object()); }
}

// ══ MQTT 구독 콜백 (수신 스레드) ═════════════════════════════════════════
void HcrSystemInterface::onJointPosition(const Json & envelope)
{
  const auto inner = unwrap(envelope);

  JointArray real_deg{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    const auto it = inner.find(kMqttJointKeys[i]);
    if (it == inner.end() || !it->is_number()) { return; }
    real_deg[i] = it->get<double>();
  }

  // 규약 변환을 여기서 끝내 둔다 — read() 는 복사만 하게.
  const auto rad = realDegToUrdfRad(real_deg);
  const auto now = Clock::now();

  {
    std::lock_guard<std::mutex> lk(state_mutex_);
    if (have_state_) {
      prev_cache_rad_ = cache_rad_;
      prev_cache_stamp_ = cache_stamp_;
    } else {
      prev_cache_rad_ = rad;
      prev_cache_stamp_ = now;
    }
    cache_rad_ = rad;
    cache_stamp_ = now;
    have_state_ = true;
    ++sample_seq_;
  }
  state_cv_.notify_all();
}

void HcrSystemInterface::onOperation(const Json & envelope)
{
  const auto d = unwrap(envelope);
  const auto status = d.value("controllerStatus", std::string{});
  if (status.empty()) { return; }

  {
    // 플래그를 뮤텍스 **안에서** 세운다 — 밖에서 세우면 on_activate 의 대기가 갱신을 놓칠 수 있다.
    std::lock_guard<std::mutex> lk(fieldbus_mutex_);
    fieldbus_ok_ = (status == kFieldBusConnected);
    if (status != last_fieldbus_) {
      if (!fieldbus_ok_) {
        RCLCPP_ERROR(get_logger(), "필드버스 상태: %s — 드라이브 통신 확인 필요", status.c_str());
      } else if (!last_fieldbus_.empty()) {
        RCLCPP_INFO(get_logger(), "필드버스 복구: %s", status.c_str());
      }
      last_fieldbus_ = status;
    }
  }
  fieldbus_cv_.notify_all();
}

void HcrSystemInterface::onCollision()
{
  if (collision_latched_.exchange(true)) { return; }
  RCLCPP_ERROR(
    get_logger(),
    "충돌 감지 — 발행을 래치했다. event/collision/clear 를 **사람이** 발행해야 풀린다");
  publishStop();
}

void HcrSystemInterface::onCollisionClear()
{
  if (!collision_latched_.exchange(false)) { return; }
  // 래치를 풀되 큐에 남은 낡은 목표는 버린다 — 충돌 시점의 목표를 재개하면 다시 박는다.
  {
    std::lock_guard<std::mutex> lk(cmd_mutex_);
    pending_valid_ = false;
  }
  RCLCPP_WARN(get_logger(), "%s 관측 — 충돌 래치 해제. 대기 중이던 목표는 버렸다", kTopicCollisionClear);
}

}  // namespace hcr5_bridge

PLUGINLIB_EXPORT_CLASS(hcr5_bridge::HcrSystemInterface, hardware_interface::SystemInterface)
