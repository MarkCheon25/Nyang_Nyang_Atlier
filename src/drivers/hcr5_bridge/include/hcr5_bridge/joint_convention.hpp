// 실기 HCR-5 ↔ URDF 관절 규약 변환.
//
// 두 모델은 영점 규약이 다르다 (2026-08-05 실측):
//   실기 zero = 팔이 수평으로 뻗은 자세   (flange z = -2.5mm)
//   URDF zero = 팔이 수직으로 선 자세     (flange z = +1063mm)
//
//   q_URDF[i](도) = SIGN[i] * q_real[i](도) + DELTA[i]
//
// 이 변환과 URDF origin 교정(hcr_robot.xacro ★ 표시)을 함께 적용하면
// 109개 자세에서 위치오차 RMS 0.0060mm 로 일치한다. 근거: 업무목록 T16.
//
// ⚠️ 이 변환을 빼먹으면 로봇이 **조용히 전혀 다른 자세로 간다** — 에러도 안 난다.
#ifndef HCR5_BRIDGE__JOINT_CONVENTION_HPP_
#define HCR5_BRIDGE__JOINT_CONVENTION_HPP_

#include <array>
#include <string>
#include <cmath>

namespace hcr5_bridge
{

inline constexpr std::size_t kNumJoints = 6;

// MQTT 상태 버스(motion/joint/position)의 키 순서.
// jogJoint/start 의 joint 인덱스 1~6 과 같은 순서임을 6축 조그 실측으로 확인했다.
inline const std::array<std::string, kNumJoints> kMqttJointKeys{
  "base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"};

// URDF/SRDF 의 관절 이름. 위 키와 1:1 대응한다.
inline const std::array<std::string, kNumJoints> kUrdfJointNames{
  "joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"};

// 부호: J3(elbow) 만 반전이다.
inline constexpr std::array<double, kNumJoints> kSign{1.0, 1.0, -1.0, 1.0, 1.0, 1.0};
// 영점 오프셋(도).
inline constexpr std::array<double, kNumJoints> kDeltaDeg{90.0, 90.0, 0.0, 90.0, 0.0, 0.0};

inline constexpr double deg2rad(double d) { return d * M_PI / 180.0; }
inline constexpr double rad2deg(double r) { return r * 180.0 / M_PI; }

/// 실기 관절각(도) → URDF 관절각(라디안)
inline std::array<double, kNumJoints> realDegToUrdfRad(const std::array<double, kNumJoints> & real_deg)
{
  std::array<double, kNumJoints> out{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    out[i] = deg2rad(kSign[i] * real_deg[i] + kDeltaDeg[i]);
  }
  return out;
}

/// URDF 관절각(라디안) → 실기 관절각(도)
inline std::array<double, kNumJoints> urdfRadToRealDeg(const std::array<double, kNumJoints> & urdf_rad)
{
  std::array<double, kNumJoints> out{};
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    out[i] = (rad2deg(urdf_rad[i]) - kDeltaDeg[i]) / kSign[i];
  }
  return out;
}

/// 실기 홈 자세(도). 펜던트 확인값 — flange (490.0, -170.5, 441.5), rx=-180.
inline constexpr std::array<double, kNumJoints> kHomeRealDeg{0.0, -90.0, -90.0, -90.0, 90.0, 0.0};

/// 공식 가동범위(도). Appendix F: ±360°, J3 만 ±165°.
inline constexpr std::array<double, kNumJoints> kLimitLoDeg{-360.0, -360.0, -165.0, -360.0, -360.0, -360.0};
inline constexpr std::array<double, kNumJoints> kLimitHiDeg{ 360.0,  360.0,  165.0,  360.0,  360.0,  360.0};

/// 실기 각도가 공식 가동범위 안인지. 범위 밖이면 로봇이 자체 정지를 건다.
inline bool withinLimits(const std::array<double, kNumJoints> & real_deg)
{
  for (std::size_t i = 0; i < kNumJoints; ++i) {
    if (real_deg[i] < kLimitLoDeg[i] || real_deg[i] > kLimitHiDeg[i]) { return false; }
  }
  return true;
}

}  // namespace hcr5_bridge

#endif  // HCR5_BRIDGE__JOINT_CONVENTION_HPP_
