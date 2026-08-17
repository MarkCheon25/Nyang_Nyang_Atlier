// 실기 HCR-5 의 직교 포즈 규약 — 위치(mm)와 자세(rx, ry, rz 도).
//
// ⚠️ 이 파일이 있는 이유 —
// 회전 표현 규약은 2026-08-16 까지 **문서 어디에도 정의돼 있지 않았다**. 알려진 것은
// 홈 자세에서 rx=-180 이라는 점 하나뿐이었고(hcr5_comm/README.md §7.2), 그래서
// movel.md §2.3 은 자세 변경 기능을 통째로 닫아 두고 `tool` 좌표계를 미구현으로 남겼다.
//
//   2026-08-16 (세션 260816-컨스터블) 실측으로 확정했다:
//
//     R = Rz(rz) · Ry(ry) · Rx(rx)        ← ZYX 오일러 (= XYZ 고정축, roll-pitch-yaw)
//
// 실측 방법 — FK RPC(`robot/convertPose`) 만 썼다. **로봇은 움직이지 않았다.**
//   J6(wrist3) 만 θ 돌리면 플랜지는 자기 z축(툴축) 둘레로만 돈다  → R(θ) = R(0)·Rz(θ)
//   J1(base)   만 θ 돌리면 플랜지는 월드 z축   둘레로만 돈다  → R(θ) = Rz(θ)·R(0)
// 후보 7종에 두 항등식의 잔차를 재어 ZYX 만 0 이었다. 두 자세(홈 · ry≠0 인 임의 자세)
// × 두 축 × 표본 10개에서 잔차 **6.9e-16**, 나머지 6종은 전부 1.28 이상.
//   ⚠️ 홈 자세만으로는 부족하다 — 홈은 ry=0 이라 ZYX 와 ZXY 가 구별되지 않는다.
//      가르려면 ry≠0 인 자세가 표본에 있어야 한다.
//
// 이 규약이 있어야 여는 것: `movel tool` 의 Δ 회전(툴 축 방향으로 밀기).
// 자세를 **유지**만 하는 경로(base·world)는 이 규약을 몰라도 성립한다 — 받은 orientation 을
// 그대로 되돌려 주면 되기 때문이다. 규약이 필요한 것은 방향을 **합성**할 때다.
#ifndef HCR5_BRIDGE__POSE_CONVENTION_HPP_
#define HCR5_BRIDGE__POSE_CONVENTION_HPP_

#include <array>
#include <cmath>

#include "hcr5_bridge/joint_convention.hpp"

namespace hcr5_bridge
{

/// 직교 위치(mm).
using Vec3 = std::array<double, 3>;
/// 행 우선 3×3 회전행렬.
using Mat3 = std::array<std::array<double, 3>, 3>;

/// 실기 orientation(rx, ry, rz 도) → 회전행렬.
/// R = Rz(rz)·Ry(ry)·Rx(rx) — 2026-08-16 실측 확정(파일 머리주석).
inline Mat3 rotationFromRealDeg(const Vec3 & rpy_deg)
{
  const double x = deg2rad(rpy_deg[0]);
  const double y = deg2rad(rpy_deg[1]);
  const double z = deg2rad(rpy_deg[2]);
  const double cx = std::cos(x), sx = std::sin(x);
  const double cy = std::cos(y), sy = std::sin(y);
  const double cz = std::cos(z), sz = std::sin(z);

  return Mat3{{
    {{cz * cy, cz * sy * sx - sz * cx, cz * sy * cx + sz * sx}},
    {{sz * cy, sz * sy * sx + cz * cx, sz * sy * cx - cz * sx}},
    {{-sy,     cy * sx,                cy * cx}}}};
}

/// R · v — 툴 좌표계 Δ 를 base 좌표계로 옮길 때 쓴다.
/// ⚠️ 이름을 `apply` 로 두면 안 된다 — 인자가 둘 다 std::array 라 ADL 이 `std::apply` 를
/// 후보에 올려 오버로드 해석이 그쪽으로 끌려간다.
inline Vec3 applyRotation(const Mat3 & R, const Vec3 & v)
{
  Vec3 out{};
  for (std::size_t i = 0; i < 3; ++i) {
    out[i] = R[i][0] * v[0] + R[i][1] * v[1] + R[i][2] * v[2];
  }
  return out;
}

inline Vec3 add(const Vec3 & a, const Vec3 & b) { return Vec3{a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
inline Vec3 sub(const Vec3 & a, const Vec3 & b) { return Vec3{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

inline double norm(const Vec3 & v)
{
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}

}  // namespace hcr5_bridge

#endif  // HCR5_BRIDGE__POSE_CONVENTION_HPP_
