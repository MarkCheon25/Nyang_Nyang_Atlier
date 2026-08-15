// servo — 서보 ON/OFF 를 MQTT 로 직접 낸다.
//
//   ros2 run hcr5_bridge servo on|off [--host H] [--port P]
//
// ROS 를 안 쓴다(rclcpp 의존 없음). launch 가 기동·종료 훅에서 부르고,
// 스택이 죽은 뒤에도 단독으로 돌아야 하기 때문이다 — 정지 사다리 3순위
// (운전절차.md §4)가 ROS 와 무관해야 하는 것이 그 이유다.
//
// tools/mqtt_cmd.py servo 와 같은 일을 한다. 그쪽은 사람이 쓰는 도구이고
// 이쪽은 launch 가 부르는 실행기다 — 소스 경로가 아니라 패키지로 찾힌다.

#include <cstdio>
#include <cstring>
#include <string>

#include "hcr5_bridge/mqtt_client.hpp"

int main(int argc, char ** argv)
{
  using hcr5_bridge::MqttClient;
  using hcr5_bridge::Json;

  std::string host = "192.168.0.20";
  int port = 1883;
  std::string action;

  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--host" && i + 1 < argc) { host = argv[++i]; }
    else if (a == "--port" && i + 1 < argc) { port = std::atoi(argv[++i]); }
    else if (a == "on" || a == "off") { action = a; }
    else if (a == "-h" || a == "--help") {
      std::printf("사용법: ros2 run hcr5_bridge servo on|off [--host H] [--port P]\n");
      return 0;
    } else {
      std::fprintf(stderr, "✗ 모르는 인자: %s\n", a.c_str());
      return 2;
    }
  }

  if (action.empty()) {
    std::fprintf(stderr, "✗ on 또는 off 가 필요하다\n");
    return 2;
  }

  MqttClient mqtt(host, port, "hcr5-servo");
  if (!mqtt.start()) {
    std::fprintf(stderr, "✗ MQTT 접속 실패: %s:%d\n", host.c_str(), port);
    return 3;
  }

  const char * status = (action == "on") ? "SERVO_ON" : "SERVO_OFF";
  const auto r = mqtt.request("set/operation", Json{{"operationStatus", status}});
  if (!r) {
    std::fprintf(stderr, "✗ %s 응답이 없다\n", status);
    return 3;
  }
  std::printf("서보 %s — ack 수신\n", action == "on" ? "ON" : "OFF");
  return 0;
}
