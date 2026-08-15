// F3.1 — 스트로크 순서 최적화.
//
// ⚠️ **이 헤더는 ROS·MoveIt 에 의존하지 않는다 — 의도된 것이다.** 순수 기하 계산만
// 담아야 로봇 없이 단위시험을 돌릴 수 있다 (test/test_optimizer.cpp).
// 좌표 변환·궤적 실행처럼 ROS 가 필요한 것은 src/draw_cat.cpp 에 둔다.
//
// 측정 결과와 설계 근거: docs/Trajectory Optimization.md
#ifndef DRAWING_CAT__OPTIMIZER_HPP_
#define DRAWING_CAT__OPTIMIZER_HPP_

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <string>
#include <vector>

namespace drawing_cat
{

/// 스트로크 하나. 좌표는 [x1,y1, x2,y2, ...] 로 평탄하게 담는다 (mm).
struct Stroke
{
    std::string name;
    std::vector<double> flat_mm;
    /// 폐곡선 여부. true 면 마지막 점에서 첫 점으로 돌아오는 구간까지 그린다.
    ///
    /// ⚠️ vision 규약: **첫 점을 끝에 중복해 넣지 않는다.** 닫는 구간을 만드는
    /// 것은 소비자 몫이다 (simulation 쪽 planned_map.cpp 도 같은 처리를 한다).
    bool closed{false};
};

// ── F3.1 스트로크 순서 최적화 ──────────────────────────────────────────────
//
// 줄이려는 것은 **펜업 이동 거리**다 — 펜을 든 채 다음 스트로크로 날아가는 구간이고,
// 그림에 아무 흔적도 남기지 않으므로 짧을수록 순수하게 이득이다.
//
// ⚠️ 여기서 다루는 좌표는 전부 **종이 mm** 다. 로봇 실제 이동거리가 아니라
//    `paper.scale` 을 곱하기 전 값이다 (scale 0.00025 면 종이 100 mm ≈ 실제 25 mm).
//    mm 공간과 world 공간은 아핀 변환 관계라 **거리의 대소가 보존**되므로,
//    mm 에서 고른 순서가 world 에서도 같은 순서다. 그래서 로봇 실제 자세를 몰라도 된다.

inline double dist2d(double x1, double y1, double x2, double y2)
{
    return std::hypot(x1 - x2, y1 - y2);
}

/// 스트로크를 다 그린 뒤 펜이 있게 되는 위치(이탈점).
///
/// ⚠️ main() 의 `end_x`/`end_y` 계산과 **같은 규칙이어야 한다.** 닫힌 스트로크는
/// 한 바퀴 돌아 첫 점으로 돌아오므로 이탈점 = 진입점이고, 열린 스트로크는 마지막 점이다.
inline void strokeExit(const Stroke& s, double& ex, double& ey)
{
    const auto& f = s.flat_mm;
    ex = s.closed ? f[0] : f[f.size() - 2];
    ey = s.closed ? f[1] : f[f.size() - 1];
}

/// 주어진 순서대로 그릴 때의 펜업 이동 거리 합 (종이 mm).
///
/// 최적화 전후를 같은 자로 재기 위한 것이다 — 이 값이 줄어든 만큼이 F3.1 의 성과다.
inline double totalTravelMm(const std::vector<Stroke>& strokes, double start_x_mm, double start_y_mm)
{
    double total = 0.0;
    double cur_x = start_x_mm, cur_y = start_y_mm;
    for (const auto& s : strokes)
    {
        total += dist2d(cur_x, cur_y, s.flat_mm[0], s.flat_mm[1]);
        strokeExit(s, cur_x, cur_y);
    }
    return total;
}

/// F3.1 — 스트로크 순서 최적화 (Nearest Neighbor).
///
/// 매번 "펜에서 가장 가까운 아직 안 그린 스트로크"를 고른다. O(n²·점개수) 이고
/// 최적해를 보장하지는 않지만, 스트로크 수가 적을 때는 최적해와 거의 같다.
///
/// 진입점을 고르는 방법이 스트로크 종류에 따라 다르다:
///
///   열린 스트로크 (눈·입 같은 선) — 양 끝점 둘 중 가까운 쪽으로 진입한다. 끝점에서
///     들어가려면 좌표를 뒤집어야 하므로 `reverse` 로 처리한다. 선택지는 **2 개**다.
///
///   닫힌 스트로크 (윤곽선) — 폐곡선에는 "첫 점" 이라는 것이 원래 없다. 어느 정점에서
///     출발하든 한 바퀴 돌아 그 자리로 돌아오므로 결과가 같다. 그래서 **모든 정점**이
///     진입 후보이고, 가장 가까운 정점이 맨 앞에 오도록 배열을 회전시킨다.
///     `flat[0]` 은 비전의 컨투어 추적기가 우연히 시작한 점일 뿐이라 우리에게 유리할
///     이유가 없다 — 이걸 고르는 것이 회전(`rotate_closed`)이다.
///     ⚠️ 닫힌 스트로크는 뒤집어도 진입점이 그대로라 **반전이 의미가 없다.**
///
/// @param rotate_closed  false 면 닫힌 스트로크도 `flat[0]` 진입으로 고정한다 (효과 비교용).
/// @param out_travel_mm  최적화된 순서의 펜업 이동 거리 합 (종이 mm).
inline std::vector<Stroke> optimizeStrokeOrder(std::vector<Stroke> strokes,
                                        double start_x_mm, double start_y_mm,
                                        bool rotate_closed,
                                        double& out_travel_mm)
{
    std::vector<Stroke> ordered;
    ordered.reserve(strokes.size());
    std::vector<bool> used(strokes.size(), false);
    double cur_x = start_x_mm, cur_y = start_y_mm;
    out_travel_mm = 0.0;

    for (size_t picked = 0; picked < strokes.size(); ++picked)
    {
        std::size_t best_idx = 0;
        std::size_t best_entry = 0;      ///< 진입할 정점 번호 (점 단위, flat 인덱스 아님)
        bool best_reverse = false;
        double best_dist = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < strokes.size(); ++i)
        {
            if (used[i]) { continue; }
            const auto& flat = strokes[i].flat_mm;
            const std::size_t n_pts = flat.size() / 2;

            auto consider = [&](std::size_t entry, bool reverse) {
                const double d = dist2d(cur_x, cur_y, flat[entry * 2], flat[entry * 2 + 1]);
                if (d < best_dist)
                {
                    best_dist = d; best_idx = i; best_entry = entry; best_reverse = reverse;
                }
            };

            if (strokes[i].closed)
            {
                // 폐곡선: 모든 정점이 진입 후보다 (회전을 끄면 첫 점만).
                if (rotate_closed)
                {
                    for (std::size_t k = 0; k < n_pts; ++k) { consider(k, false); }
                }
                else
                {
                    consider(0, false);
                }
            }
            else
            {
                // 열린 선: 머리에서 정방향, 또는 꼬리에서 역방향.
                consider(0, false);
                consider(n_pts - 1, true);
            }
        }

        Stroke chosen = std::move(strokes[best_idx]);
        used[best_idx] = true;

        if (best_reverse)
        {
            // (x,y) 쌍의 순서만 뒤집는다 — 쌍 안의 x,y 는 그대로 둬야 한다.
            std::vector<double> rev;
            rev.reserve(chosen.flat_mm.size());
            for (std::size_t i = chosen.flat_mm.size(); i >= 2; i -= 2)
            {
                rev.push_back(chosen.flat_mm[i - 2]);
                rev.push_back(chosen.flat_mm[i - 1]);
            }
            chosen.flat_mm = std::move(rev);
        }
        else if (best_entry > 0)
        {
            // 폐곡선 회전: 고른 정점이 맨 앞에 오도록 왼쪽으로 민다. 순환이라 모양은 같다.
            std::rotate(chosen.flat_mm.begin(),
                        chosen.flat_mm.begin() + static_cast<std::ptrdiff_t>(best_entry * 2),
                        chosen.flat_mm.end());
        }

        out_travel_mm += best_dist;
        strokeExit(chosen, cur_x, cur_y);
        ordered.push_back(std::move(chosen));
    }
    return ordered;
}


}  // namespace drawing_cat

#endif  // DRAWING_CAT__OPTIMIZER_HPP_
