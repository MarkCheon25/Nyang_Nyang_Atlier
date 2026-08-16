// 게이트 ⑤ — 서보가 켜져 있는가. `program/play` 를 쏘기 전에 본다.
//
// 🔴 **`program/play` 는 서보가 꺼져 있어도 `code:0` 으로 정상 ack 한다.**
// 2026-08-16 실측: `program/clear` · `program/plan` · `program/play` 세 ack 가 전부 성공인데
// 로봇은 1mm 도 움직이지 않았고, 자세를 되읽으니 소수점 끝자리까지 발행 전과 같았다.
// 유일한 신호는 **60초 타임아웃**이었다 — 로그만 보면 성공한 실행과 구별되지 않는다.
// 자율 운전 층에 그대로 올리면 "가끔 60초씩 멈춘다" 로 나타날 자리라 발행 전에 막는다.
//
// `hcr5_comm/README.md` §6.1.3 이 *"서보 ON 필요"* 라고 적어 둔 조건인데,
// 실행기 어디에도 그것을 보는 코드가 없었다. 게이트 ⓪~④ 는 전부 **내가 보낼 값**만 검사하고
// **로봇이 받을 준비가 됐는지**는 아무도 안 봤다. 이 헤더가 그 칸이다.
#ifndef HCR5_BRIDGE__SERVO_GATE_HPP_
#define HCR5_BRIDGE__SERVO_GATE_HPP_

#include <chrono>
#include <condition_variable>
#include <memory>
#include <mutex>
#include <string>

#include "hcr5_bridge/mqtt_client.hpp"

namespace hcr5_bridge
{

/// `status/operation` 을 한 판 받아 `operationStatus` 를 확인한다.
/// 이 토픽은 로봇이 약 20Hz 로 계속 올리므로 기본 2초면 충분히 잡힌다.
///
/// @param state_out 관찰된 상태 문자열. 한 판도 못 받으면 빈 문자열.
/// @return `SERVO_ON` 일 때만 true.
///
/// ⚠️ `MqttClient` 에는 구독 해지가 없다. 콜백이 함수 반환 뒤에도 20Hz 로 계속 불리므로
/// 상태를 스택이 아니라 `shared_ptr` 에 담아 **콜백이 값으로 붙잡게** 한다.
/// 지역 변수를 참조로 캡처하면 반환 직후부터 해제된 스택을 쓴다.
inline bool waitServoOn(MqttClient & mqtt, std::string & state_out, double timeout_sec = 2.0)
{
  struct State
  {
    std::mutex m;
    std::condition_variable cv;
    std::string status;
  };
  auto st = std::make_shared<State>();

  mqtt.subscribe(
    "status/operation", [st](const std::string &, const Json & env) {
      // 콜백은 **봉투 원문**을 받는다 (mqtt_client.cpp::dispatch — RPC 응답만 한 겹 벗긴다).
      // 알맹이는 data.data 안쪽이다.
      const auto d = env.find("data");
      if (d == env.end()) { return; }
      const auto dd = d->find("data");
      if (dd == d->end()) { return; }
      const auto s = dd->find("operationStatus");
      if (s == dd->end() || !s->is_string()) { return; }
      {
        std::lock_guard<std::mutex> lk(st->m);
        st->status = s->get<std::string>();
      }
      st->cv.notify_all();
    });

  std::unique_lock<std::mutex> lk(st->m);
  st->cv.wait_for(
    lk, std::chrono::duration<double>(timeout_sec), [&] { return !st->status.empty(); });
  state_out = st->status;
  return state_out == "SERVO_ON";
}

}  // namespace hcr5_bridge

#endif  // HCR5_BRIDGE__SERVO_GATE_HPP_
