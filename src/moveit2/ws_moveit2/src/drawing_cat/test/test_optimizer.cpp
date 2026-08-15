// F3.1 순서 최적화 단위시험 — 로봇도 ROS 도 필요 없다.
//
//   colcon test --packages-select drawing_cat
//   colcon test-result --verbose          ← 실패 내용 보기
//
// ⚠️ 여기서 보는 것은 "얼마나 빨라지나" 가 아니라 **"의도대로 동작하나"** 다.
//    시간 측정은 로봇이 있어야 하므로 docs/Trajectory Optimization.md 로 간다.
//
// 확인하는 것:
//   · NN 이 가까운 것부터 고르는가
//   · 열린 스트로크를 가까운 끝에서 진입하려고 뒤집는가
//   · 닫힌 스트로크를 가장 가까운 정점으로 회전시키는가
//   · 뒤집거나 회전한 뒤에도 **좌표를 잃거나 만들지 않는가**
//   · 이탈점 규칙이 draw_cat.cpp 의 end_x/end_y 와 같은가
#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>
#include <utility>
#include <vector>

#include "drawing_cat/optimizer.hpp"

using drawing_cat::optimizeStrokeOrder;
using drawing_cat::Stroke;
using drawing_cat::strokeExit;
using drawing_cat::totalTravelMm;

namespace
{

/// 회전·반전이 점을 잃거나 만들지 않았는지 (순서만 달라야 한다).
::testing::AssertionResult SamePointSet(const Stroke& a, const Stroke& b)
{
    if (a.flat_mm.size() != b.flat_mm.size())
    {
        return ::testing::AssertionFailure()
               << "좌표 개수가 다르다: " << a.flat_mm.size() << " vs " << b.flat_mm.size();
    }
    std::vector<std::pair<double, double>> pa, pb;
    for (std::size_t i = 0; i + 1 < a.flat_mm.size(); i += 2)
    {
        pa.emplace_back(a.flat_mm[i], a.flat_mm[i + 1]);
    }
    for (std::size_t i = 0; i + 1 < b.flat_mm.size(); i += 2)
    {
        pb.emplace_back(b.flat_mm[i], b.flat_mm[i + 1]);
    }
    std::sort(pa.begin(), pa.end());
    std::sort(pb.begin(), pb.end());
    return pa == pb ? ::testing::AssertionSuccess()
                    : ::testing::AssertionFailure() << "좌표 집합이 달라졌다";
}

}  // namespace

// ── 열린 스트로크 — NN 순서 + 방향 반전 ─────────────────────────────────────
//
// 펜은 (0,0). A 는 멀리 있고, B 는 가깝지만 **꼬리 쪽이** 더 가깝다.
TEST(OptimizeStrokeOrder, OpenStrokesPickNearestEndAndReverse)
{
    const std::vector<Stroke> in = {
        {"A", {100.0, 0.0, 110.0, 0.0}, false},
        {"B", {50.0, 0.0, 10.0, 0.0}, false},   // 꼬리 (10,0) 가 펜에 더 가깝다
    };

    const double before = totalTravelMm(in, 0.0, 0.0);
    double after = 0.0;
    const auto out = optimizeStrokeOrder(in, 0.0, 0.0, true, after);

    ASSERT_EQ(out.size(), 2u);
    EXPECT_EQ(out[0].name, "B") << "가까운 B 를 먼저 골라야 한다";
    EXPECT_DOUBLE_EQ(out[0].flat_mm[0], 10.0) << "꼬리에서 진입 = 뒤집혀 있어야 한다";
    EXPECT_EQ(out[1].name, "A");
    EXPECT_TRUE(SamePointSet(in[1], out[0]));

    // before: (0,0)→A머리(100)=100, A끝(110)→B머리(50)=60  ⇒ 160
    // after : (0,0)→B꼬리(10)= 10, B끝(50)→A머리(100)=50   ⇒  60
    EXPECT_DOUBLE_EQ(before, 160.0);
    EXPECT_DOUBLE_EQ(after, 60.0);
    EXPECT_LT(after, before);
}

// ── 닫힌 스트로크 — 시작점 회전 ─────────────────────────────────────────────
//
// 폐곡선에는 "첫 점" 이 원래 없다. flat[0] 을 펜에서 **가장 먼** 모서리에 두고,
// 회전이 가장 가까운 정점을 골라내는지 본다.
TEST(OptimizeStrokeOrder, ClosedStrokeRotatesToNearestVertex)
{
    // 정점 순서: (100,100) 먼쪽 → (0,100) → (0,0) 가까운쪽 → (100,0)
    const std::vector<Stroke> in = {
        {"square", {100.0, 100.0, 0.0, 100.0, 0.0, 0.0, 100.0, 0.0}, true},
    };

    double with_rot = 0.0, without_rot = 0.0;
    const auto rotated = optimizeStrokeOrder(in, 0.0, 0.0, true, with_rot);
    const auto plain = optimizeStrokeOrder(in, 0.0, 0.0, false, without_rot);

    EXPECT_DOUBLE_EQ(rotated[0].flat_mm[0], 0.0);
    EXPECT_DOUBLE_EQ(rotated[0].flat_mm[1], 0.0) << "펜에 가장 가까운 (0,0) 에서 진입";
    EXPECT_DOUBLE_EQ(plain[0].flat_mm[0], 100.0);
    EXPECT_DOUBLE_EQ(plain[0].flat_mm[1], 100.0) << "회전을 끄면 flat[0] 그대로";

    EXPECT_DOUBLE_EQ(with_rot, 0.0);
    EXPECT_DOUBLE_EQ(without_rot, std::hypot(100.0, 100.0));
    EXPECT_LT(with_rot, without_rot) << "회전이 이동거리를 줄여야 한다";

    EXPECT_TRUE(SamePointSet(in[0], rotated[0]));
    EXPECT_EQ(rotated[0].flat_mm.size(), 8u) << "점을 더하거나 빼면 안 된다";
}

// ── 이탈점 규칙 ─────────────────────────────────────────────────────────────
//
// ⚠️ draw_cat.cpp 의 end_x/end_y 계산과 **반드시 같아야 한다.** 어긋나면 다음
//    스트로크까지의 이동 거리를 잘못 재고, 그리기 자체는 멀쩡해 보인다.
TEST(OptimizeStrokeOrder, ClosedStrokeExitEqualsEntry)
{
    std::vector<Stroke> in = {
        {"c1", {0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0}, true},
        {"c2", {0.0, 0.0, 10.0, 0.0, 10.0, 10.0, 0.0, 10.0}, true},
    };
    for (auto& v : in[1].flat_mm) { v += 200.0; }   // 두 번째를 멀리 옮긴다

    double travel = 0.0;
    const auto out = optimizeStrokeOrder(in, 0.0, 0.0, true, travel);

    double ex = 0.0, ey = 0.0;
    strokeExit(out[0], ex, ey);
    EXPECT_DOUBLE_EQ(ex, out[0].flat_mm[0]);
    EXPECT_DOUBLE_EQ(ey, out[0].flat_mm[1]) << "폐곡선은 한 바퀴 돌아 제자리로 온다";

    EXPECT_EQ(out[0].name, "c1") << "가까운 것부터";
    EXPECT_EQ(out[1].name, "c2");
}

// ── 실제 파라미터 데이터 회귀 ───────────────────────────────────────────────
//
// config/draw_cat_params.yaml 의 좌표 그대로. 값이 바뀌면 알아채라고 넣어 둔다.
TEST(OptimizeStrokeOrder, RealParamsDataDoesNotGetWorse)
{
    const std::vector<Stroke> in = {
        {"left_ear", {50, 30, 52, 28, 55, 27, 58, 28, 60, 30}, false},
        {"right_ear", {100, 30, 103, 28, 106, 27, 109, 28, 112, 30}, false},
        {"left_eye", {70, 50, 75, 50}, false},
        {"right_eye", {85, 50, 90, 50}, false},
        {"mouth", {60, 70, 70, 75, 80, 75, 90, 70}, false},
    };

    // (81, 51) = draw_cat_params.yaml 의 center_x_mm / center_y_mm 기본값
    const double before = totalTravelMm(in, 81.0, 51.0);
    double after = 0.0;
    const auto out = optimizeStrokeOrder(in, 81.0, 51.0, true, after);

    EXPECT_EQ(out.size(), 5u) << "스트로크를 잃으면 안 된다";
    EXPECT_LE(after, before) << "최적화가 이동거리를 늘리면 안 된다";
    EXPECT_NEAR(before, 170.0, 0.5);
    EXPECT_NEAR(after, 123.9, 0.5);
}

// ── 엣지 케이스 ─────────────────────────────────────────────────────────────
TEST(OptimizeStrokeOrder, EdgeCases)
{
    // 스트로크 1 개
    const std::vector<Stroke> one = {{"only", {5.0, 5.0, 6.0, 6.0}, false}};
    double t1 = 0.0;
    const auto out1 = optimizeStrokeOrder(one, 0.0, 0.0, true, t1);
    ASSERT_EQ(out1.size(), 1u);
    EXPECT_DOUBLE_EQ(out1[0].flat_mm[0], 5.0) << "더 가까운 (5,5) 에서 진입";

    // 빈 입력 — 크래시 없이 빈 결과
    const std::vector<Stroke> none;
    double t2 = 0.0;
    const auto out2 = optimizeStrokeOrder(none, 0.0, 0.0, true, t2);
    EXPECT_TRUE(out2.empty());
    EXPECT_DOUBLE_EQ(t2, 0.0);

    // 2 점짜리 폐곡선 — 최소 크기의 폐곡선도 회전이 되는가
    const std::vector<Stroke> tiny = {{"cc", {1.0, 1.0, 2.0, 2.0}, true}};
    double t3 = 0.0;
    const auto out3 = optimizeStrokeOrder(tiny, 3.0, 3.0, true, t3);
    ASSERT_EQ(out3.size(), 1u);
    EXPECT_DOUBLE_EQ(out3[0].flat_mm[0], 2.0) << "가까운 (2,2) 정점으로 회전";
}
