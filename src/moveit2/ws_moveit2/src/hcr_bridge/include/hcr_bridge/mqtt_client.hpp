// HCR-5 컨트롤러의 네이티브 MQTT 버스 클라이언트.
//
// 봉투 규약 (2026-07-29·08-05 역설계):
//   보낼 때  {"type":"pub"|"pubWithAck", "uuid":U, "data":{"thng_id":1, ...}}
//   pubWithAck 는 **uuid 와 같은 이름의 토픽**으로 응답이 온다:
//            {"type":"pub", "uuid":..., "data":{"code":0,"data":{...},"msg":"success"}}
//   → 응답의 알맹이는 한 겹 안쪽(data.data)이다.
//
// ⚠️ ack 는 '명령 접수'이지 '동작 완료'가 아니다. 이동 완료는 event/motion 으로 판정한다.
#ifndef HCR_BRIDGE__MQTT_CLIENT_HPP_
#define HCR_BRIDGE__MQTT_CLIENT_HPP_

#include <chrono>
#include <condition_variable>
#include <functional>
#include <map>
#include <mutex>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

struct mosquitto;

namespace hcr_bridge
{

using Json = nlohmann::json;
using TopicCallback = std::function<void (const std::string & topic, const Json & payload)>;

class MqttClient
{
public:
  MqttClient(std::string host, int port, std::string client_id_prefix = "hcr-ros2");
  ~MqttClient();

  MqttClient(const MqttClient &) = delete;
  MqttClient & operator=(const MqttClient &) = delete;

  /// 브로커 접속 + 백그라운드 수신 루프 시작. 실패하면 false.
  bool start();
  void stop();
  bool connected() const { return connected_; }

  /// 토픽 구독. 같은 토픽에 여러 콜백을 걸면 마지막 것이 이긴다.
  void subscribe(const std::string & topic, TopicCallback cb);

  /// 단방향 발행 (type:"pub"). data 에 thng_id 를 자동으로 넣는다.
  void publish(const std::string & topic, const Json & data);

  /// pubWithAck RPC. 응답의 알맹이(data.data)를 돌려준다.
  /// 타임아웃이거나 code!=0 이면 nullopt.
  std::optional<Json> request(
    const std::string & topic, const Json & data,
    std::chrono::milliseconds timeout = std::chrono::milliseconds(3000));

  /// thng_id — 로봇 식별자. 관찰된 값은 항상 1이다.
  void setThngId(int id) { thng_id_ = id; }

private:
  static void onMessage(mosquitto * m, void * self, const struct mosquitto_message * msg);
  void dispatch(const std::string & topic, const std::string & payload);
  static std::string makeUuidV1();

  std::string host_;
  int port_;
  std::string client_id_;
  int thng_id_{1};

  mosquitto * mosq_{nullptr};
  bool connected_{false};

  std::mutex cb_mutex_;
  std::map<std::string, TopicCallback> callbacks_;

  // RPC 대기: uuid → 응답 슬롯
  std::mutex rpc_mutex_;
  std::condition_variable rpc_cv_;
  std::map<std::string, std::optional<Json>> pending_;
};

}  // namespace hcr_bridge

#endif  // HCR_BRIDGE__MQTT_CLIENT_HPP_
