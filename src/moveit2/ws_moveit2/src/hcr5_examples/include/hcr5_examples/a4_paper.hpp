// ═══ A4 종이 · 입력 도형 · 좌표 매핑 (헤더 온리) ═══════════════════════════
//
// 05(연속 데카르트)와 06(movel 점대점)이 **같은 종이 위에 같은 그림**을 그린다.
// 실행 전략만 예제마다 다르고, 기하는 전부 이 파일이다.
//
// ⚠️ 지금은 **06 만 이 헤더를 쓴다.** 05 는 같은 값을 자기 파일 안에 그대로 들고
//    있다 (05 를 건드리지 않기로 해서 그대로 두었다). 즉 teach 기록이 **두 벌**이다 —
//    종이를 다시 재면 05_draw_contour.cpp 의 kLL/kLU/kRU/kRL 과 아래 값을 **둘 다**
//    고쳐야 한다. 05 를 이 헤더로 옮기면 이 위험이 사라진다 (동작은 완전히 동일).
//    옮기기 전까지는 두 곳이 같은지 확인하는 것이 사람 몫이다.

#ifndef HCR5_EXAMPLES__A4_PAPER_HPP_
#define HCR5_EXAMPLES__A4_PAPER_HPP_

#include <algorithm>
#include <cmath>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>

namespace hcr5_examples
{

// ── 입력 도형 ────────────────────────────────────────────────────────────────
// 고양이 이미지에서 뽑은 윤곽선. OpenCV `findContours` 산출물 형태 그대로 —
// **픽셀 좌표**이고, 원점은 이미지 **좌상단**, v(세로)는 **아래로** 증가한다.
// 마지막 점에서 첫 점으로 돌아가는 닫힌 경로다(각 예제가 자동으로 이어 붙인다).
// 나중에 vision 이 토픽/서비스로 넘겨주면 이 배열만 갈아끼우면 된다.
struct Px { int u, v; };

inline const std::vector<Px> kContour = {
  { 56,   0}, { 47,  12}, { 46,  84}, { 38, 103}, { 36, 127}, {  5, 131}, {  4, 139},
  {  7, 142}, { 37, 142}, { 42, 153}, { 15, 167}, { 17, 175}, { 50, 167}, { 61, 179},
  { 51, 234}, { 50, 272}, { 69, 337}, { 85, 480}, { 70, 489}, { 62, 500}, { 60, 512},
  { 64, 523}, { 83, 535}, {322, 535}, {340, 529}, {386, 525}, {417, 511}, {447, 478},
  {459, 435}, {455, 399}, {437, 344}, {435, 317}, {448, 294}, {479, 281}, {479, 246},
  {462, 237}, {432, 241}, {405, 260}, {389, 285}, {382, 314}, {383, 339}, {403, 405},
  {407, 435}, {395, 463}, {371, 475}, {374, 445}, {371, 388}, {358, 338}, {343, 305},
  {304, 257}, {234, 208}, {211, 178}, {217, 167}, {250, 175}, {253, 165}, {227, 156},
  {231, 142}, {260, 142}, {264, 138}, {264, 133}, {258, 129}, {233, 128}, {232, 106},
  {224,  83}, {223,   8}, {215,   0}, {206,   0}, {160,  41}, {111,  42}, { 66,   0},
};

// ── 종이 (A4) ────────────────────────────────────────────────────────────────
// 실기에서 teach 한 네 모서리 [m, 로봇 베이스 프레임]. **이 파일이 정본이다** —
// 종이를 옮기거나 다시 재면 여기를 고치고 커밋한다 (측정 기록이므로 런치 인자로
// 빼지 않았다). 이름은 위에서 내려다본 기준 — +X 가 오른쪽, +Y 가 위쪽이다.
//
//        LU (432.09, 107.84)      RU (639.24, 107.82)     ← +Y (위)
//              ┌──────────────────────┐
//              │                      │
//              │        A4            │  긴 변 297 → +Y
//              │                      │
//              └──────────────────────┘
//        LL (435.07,−186.48)      RL (644.00,−186.14)
//                     짧은 변 210 → +X
//
// 실측 검토
//   변 길이   LL→RL 209.01 · LU→RU 207.23 (공칭 210)
//             LL→LU 294.34 · RL→RU 294.00 (공칭 297)
//     → XY 는 1~3mm 짧게 찍혔다. 모서리 직각도도 ±0.9° 흔들린다.
//   평면성   네 점의 최소제곱 평면 이탈이 ±0.008mm — **Z 는 서로 완벽히 일관된다.**
//   기울기   좌변 −12.36/−12.39, 우변 −18.28/−18.29.
//            X 로 209mm 가는 동안 Z 가 5.92mm 내려간다 = 1.63° 기울어진 책상이다.
//
//   즉 "Z 오차"로 보였던 5.9mm 는 측정 잡음이 아니라 **실제 기울기**다.
//   ±0.008mm 로 평면에 붙어 있는 값이 잡음일 수는 없다.
struct Corner { double x, y, z; };

inline constexpr Corner kLL{0.43507, -0.18648, -0.01236};   // 왼쪽 아래
inline constexpr Corner kLU{0.43209,  0.10784, -0.01239};   // 왼쪽 위
inline constexpr Corner kRU{0.63924,  0.10782, -0.01829};   // 우측 위
inline constexpr Corner kRL{0.64400, -0.18614, -0.01828};   // 우측 아래

// teach 할 때 티치펜던트가 보여준 자세는 네 점 모두 Rx=179.9° Ry=0 Rz=−90° 였다.
// 우리 URDF 의 홈에서 pen_tip 자세는 RPY(180°, 0, 0) 이다. 둘의 차이는 **펜 축을
// 중심으로 한 90° 회전뿐**이고 (툴 Z 축은 양쪽 다 정확히 (0,0,−1) = 수직 하향),
// 펜은 축 대칭이라 그려지는 선에는 영향이 없다. 실기 TCP 프레임의 X 축 정의가
// 우리 tool0 과 90° 다른 것으로 보이며, 네 점의 Rz 가 모두 같으므로 어느 쪽이든
// 그리기 결과는 동일하다. 손목 자세만 달라진다 — 필요하면 pen_yaw_deg 로 준다.

inline double cornerDistance(const Corner & p, const Corner & q)
{
  return std::sqrt((p.x - q.x) * (p.x - q.x) +
                   (p.y - q.y) * (p.y - q.y) +
                   (p.z - q.z) * (p.z - q.z));
}

// ── 종이 평면 ────────────────────────────────────────────────────────────────
// teach 한 XY 는 그대로 쓰고, Z 만 z_left / z_right 로 갈아끼운다.
// use_measured_z 면 teach 값을 그대로 쓴다 (= 실제 종이면을 그대로 따라간다).
class A4Paper
{
public:
  A4Paper(double z_left, double z_right, bool use_measured_z, double pen_yaw_deg)
  : ll_(kLL), lu_(kLU), ru_(kRU), rl_(kRL)
  {
    if (!use_measured_z) {
      ll_.z = lu_.z = z_left;
      rl_.z = ru_.z = z_right;
    }
    // 펜 자세: R = Rz(yaw)·Rx(180°) → 쿼터니언 (x,y,z,w) = (cos(yaw/2), sin(yaw/2), 0, 0)
    // yaw=0 이면 (1,0,0,0) 으로 홈의 pen_tip 자세와 정확히 같다 (툴 Z = (0,0,−1)).
    const double h = 0.5 * pen_yaw_deg * M_PI / 180.0;
    pen_.x = std::cos(h); pen_.y = std::sin(h); pen_.z = 0.0; pen_.w = 0.0;

    width_  = 0.5 * (cornerDistance(kLL, kRL) + cornerDistance(kLU, kRU));   // ≈ 0.208
    height_ = 0.5 * (cornerDistance(kLL, kLU) + cornerDistance(kRL, kRU));   // ≈ 0.294
  }

  // 종이 정규좌표 (a,b) ∈ [0,1]² → 로봇 좌표. a 는 좌→우(+X), b 는 아래→위(+Y).
  // **겹선형 보간**이라 네 모서리를 정확히 지난다. 종이가 완전한 직사각형이
  // 아니어도(실측 직각도 ±0.9°) 네 변을 그대로 따라가며, Z 기울기도 자동으로 실린다.
  geometry_msgs::msg::Pose at(double a, double b, double dz = 0.0) const
  {
    const double w00 = (1 - a) * (1 - b), w10 = a * (1 - b);
    const double w11 = a * b,             w01 = (1 - a) * b;
    geometry_msgs::msg::Pose q;
    q.orientation = pen_;
    q.position.x = w00 * ll_.x + w10 * rl_.x + w11 * ru_.x + w01 * lu_.x;
    q.position.y = w00 * ll_.y + w10 * rl_.y + w11 * ru_.y + w01 * lu_.y;
    q.position.z = w00 * ll_.z + w10 * rl_.z + w11 * ru_.z + w01 * lu_.z + dz;
    return q;
  }

  double width()  const { return width_; }    // 짧은 변 [m]
  double height() const { return height_; }   // 긴 변 [m]
  const geometry_msgs::msg::Quaternion & penOrientation() const { return pen_; }

  // teach 평면 대비 편차 [m]. + 면 펜이 종이 위로 뜬 것 = 안 그려진다.
  static double gapLeft(double z_left)   { return z_left  - 0.5 * (kLL.z + kLU.z); }
  static double gapRight(double z_right) { return z_right - 0.5 * (kRL.z + kRU.z); }

private:
  Corner ll_, lu_, ru_, rl_;
  geometry_msgs::msg::Quaternion pen_;
  double width_{}, height_{};
};

// ── 픽셀 → 종이 좌표 ─────────────────────────────────────────────────────────
// 이미지는 좌상단 원점 · u 오른쪽 · v 아래로 증가.
// 종이는 (위에서 내려다볼 때) a 오른쪽 · b 위쪽.
//
//     a = 0.5 + (u − u_c)·s / W       u 증가 → 오른쪽
//     b = 0.5 − (v − v_c)·s / H       v 증가 → 아래쪽   ← 부호 하나만 뒤집는다
//
// 거울상이 아닌 이유: +Z 에서 내려다보는 시점에서 a 는 화면 오른쪽, b 는 화면
// 위쪽이다. 여기에 u→오른쪽, v→아래쪽을 그대로 대응시켰으므로 이미지가 보이는
// 그대로 놓인다. 펜은 종이 **윗면**에 그리고 사람도 위에서 보므로 뒤집히지 않는다.
//
// 배율은 여백을 뺀 종이 안에 꽉 채운다. draw_size > 0 이면 그 값을 쓰되, 종이를
// 넘으면 잘라내는 대신 줄여서 맞춘다 (clamped() 가 true 가 된다).
class ContourFit
{
public:
  ContourFit(const std::vector<Px> & contour, const A4Paper & paper,
             double margin, double draw_size)
  : paper_(paper)
  {
    int u_lo = contour.front().u, u_hi = u_lo;
    int v_lo = contour.front().v, v_hi = v_lo;
    for (const auto & p : contour) {
      u_lo = std::min(u_lo, p.u); u_hi = std::max(u_hi, p.u);
      v_lo = std::min(v_lo, p.v); v_hi = std::max(v_hi, p.v);
    }
    w_px_ = u_hi - u_lo;  h_px_ = v_hi - v_lo;
    u_c_  = 0.5 * (u_lo + u_hi);  v_c_ = 0.5 * (v_lo + v_hi);

    s_fit_ = std::min((paper.width()  - 2 * margin) / w_px_,
                      (paper.height() - 2 * margin) / h_px_);
    s_ = s_fit_;
    if (draw_size > 0.0) {
      s_ = draw_size / std::max(w_px_, h_px_);
      if (s_ > s_fit_) { s_ = s_fit_; clamped_ = true; }
    }
  }

  geometry_msgs::msg::Pose pose(const Px & p, double dz = 0.0) const
  {
    return paper_.at(0.5 + (p.u - u_c_) * s_ / paper_.width(),
                     0.5 - (p.v - v_c_) * s_ / paper_.height(), dz);
  }

  double scale()    const { return s_; }        // [m/px]
  double fitScale() const { return s_fit_; }    // 종이에 꽉 채웠을 때의 배율
  bool   clamped()  const { return clamped_; }  // draw_size 가 종이를 넘어 줄었나
  double widthPx()  const { return w_px_; }
  double heightPx() const { return h_px_; }
  double drawnWidth()  const { return w_px_ * s_; }   // [m]
  double drawnHeight() const { return h_px_ * s_; }   // [m]

private:
  A4Paper paper_;
  double w_px_{}, h_px_{}, u_c_{}, v_c_{};
  double s_{}, s_fit_{};
  bool   clamped_{false};
};

}  // namespace hcr5_examples

#endif  // HCR5_EXAMPLES__A4_PAPER_HPP_
