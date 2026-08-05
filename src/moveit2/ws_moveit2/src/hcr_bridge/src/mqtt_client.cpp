#include "hcr_bridge/mqtt_client.hpp"

#include <mosquitto.h>
#include <uuid/uuid.h>

#include <atomic>
#include <cstring>

namespace hcr_bridge
{

namespace
{
// mosquitto_lib_init/cleanup 은 프로세스당 한 번이어야 한다.
std::atomic<int> g_lib_refs{0};

void libRef()
{
  if (g_lib_refs.fetch_add(1) == 0) { mosquitto_lib_init(); }
}
void libUnref()
{
  if (g_lib_refs.fetch_sub(1) == 1) { mosquitto_lib_cleanup(); }
}
}  // namespace

std::string MqttClient::makeUuidV1()
{
  // 로봇이 발행하는 uuid 도 v1(시간+MAC)이다. 응답 토픽명으로 쓰이므로 형식만 맞으면 되지만
  // 관찰된 것과 같은 종류를 쓴다.
  uuid_t u;
  uuid_generate_time(u);
  char buf[37];
  uuid_unparse_lower(u, buf);
  return std::string(buf);
}

MqttClient::MqttClient(std::string host, int port, std::string client_id_prefix)
: host_(std::move(host)), port_(port)
{
  libRef();
  client_id_ = client_id_prefix + "-" + makeUuidV1().substr(0, 8);
}

MqttClient::~MqttClient()
{
  stop();
  if (mosq_) { mosquitto_destroy(mosq_); mosq_ = nullptr; }
  libUnref();
}

bool MqttClient::start()
{
  // clean session. 브로커는 무인증이다(Mosquitto 1.4.7).
  mosq_ = mosquitto_new(client_id_.c_str(), true, this);
  if (!mosq_) { return false; }
  mosquitto_message_callback_set(mosq_, &MqttClient::onMessage);

  if (mosquitto_connect(mosq_, host_.c_str(), port_, 30) != MOSQ_ERR_SUCCESS) { return false; }
  if (mosquitto_loop_start(mosq_) != MOSQ_ERR_SUCCESS) { return false; }
  connected_ = true;

  // start() 전에 걸어둔 구독을 재적용한다.
  std::lock_guard<std::mutex> lk(cb_mutex_);
  for (const auto & [topic, _] : callbacks_) {
    mosquitto_subscribe(mosq_, nullptr, topic.c_str(), 0);
  }
  return true;
}

void MqttClient::stop()
{
  if (!mosq_ || !connected_) { return; }
  connected_ = false;
  mosquitto_disconnect(mosq_);
  mosquitto_loop_stop(mosq_, false);
}

void MqttClient::subscribe(const std::string & topic, TopicCallback cb)
{
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    callbacks_[topic] = std::move(cb);
  }
  if (connected_) { mosquitto_subscribe(mosq_, nullptr, topic.c_str(), 0); }
}

void MqttClient::onMessage(::mosquitto *, void * self, const ::mosquitto_message * msg)
{
  if (!msg || !msg->payload) { return; }
  auto * c = static_cast<MqttClient *>(self);
  c->dispatch(msg->topic, std::string(static_cast<char *>(msg->payload), msg->payloadlen));
}

void MqttClient::dispatch(const std::string & topic, const std::string & payload)
{
  Json j;
  try {
    j = Json::parse(payload);
  } catch (const std::exception &) {
    return;  // 로봇이 보내는 것은 전부 JSON 이다. 아니면 버린다.
  }

  // RPC 응답인가 — 대기 중인 uuid 와 토픽명이 같으면.
  {
    std::lock_guard<std::mutex> lk(rpc_mutex_);
    auto it = pending_.find(topic);
    if (it != pending_.end()) {
      it->second = j;
      rpc_cv_.notify_all();
      return;
    }
  }

  TopicCallback cb;
  {
    std::lock_guard<std::mutex> lk(cb_mutex_);
    auto it = callbacks_.find(topic);
    if (it == callbacks_.end()) { return; }
    cb = it->second;
  }
  cb(topic, j);
}

void MqttClient::publish(const std::string & topic, const Json & data)
{
  if (!connected_) { return; }
  Json d = data;
  d["thng_id"] = thng_id_;
  const Json env{{"type", "pub"}, {"uuid", makeUuidV1()}, {"data", d}};
  const std::string s = env.dump();
  mosquitto_publish(mosq_, nullptr, topic.c_str(), static_cast<int>(s.size()), s.data(), 0, false);
}

std::optional<Json> MqttClient::request(
  const std::string & topic, const Json & data, std::chrono::milliseconds timeout)
{
  if (!connected_) { return std::nullopt; }

  const std::string uuid = makeUuidV1();
  // ★ 순서가 중요하다 — 발행 전에 응답 토픽을 먼저 구독해야 응답을 놓치지 않는다.
  {
    std::lock_guard<std::mutex> lk(rpc_mutex_);
    pending_[uuid] = std::nullopt;
  }
  mosquitto_subscribe(mosq_, nullptr, uuid.c_str(), 0);

  Json d = data;
  d["thng_id"] = thng_id_;
  const Json env{{"type", "pubWithAck"}, {"uuid", uuid}, {"data", d}};
  const std::string s = env.dump();
  mosquitto_publish(mosq_, nullptr, topic.c_str(), static_cast<int>(s.size()), s.data(), 0, false);

  std::optional<Json> reply;
  {
    std::unique_lock<std::mutex> lk(rpc_mutex_);
    rpc_cv_.wait_for(lk, timeout, [&] { return pending_[uuid].has_value(); });
    reply = pending_[uuid];
    pending_.erase(uuid);
  }
  // 구독을 남겨두면 장수명 노드에서 누수가 된다.
  mosquitto_unsubscribe(mosq_, nullptr, uuid.c_str());

  if (!reply) { return std::nullopt; }
  // 봉투 한 겹 + 결과 한 겹을 벗긴다: {"data":{"code":0,"data":{...}}}
  const auto & body = (*reply).value("data", Json::object());
  if (!body.contains("code") || body["code"].get<int>() != 0) { return std::nullopt; }
  return body.value("data", Json::object());
}

}  // namespace hcr_bridge
