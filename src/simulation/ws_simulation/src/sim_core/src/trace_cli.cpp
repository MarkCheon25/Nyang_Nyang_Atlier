// trace_cli — 접촉 궤적 확인 하네스.
//
// **이 도구가 있는 이유는 "계획대로 그어졌는가"를 눈으로 보기 위해서다.**
// 시뮬이 남긴 궤적 CSV 와 vision 이 만든 계획 지도(map.csv)를 한 장에 겹쳐 SVG 로
// 뽑는다. mm 를 그대로 쓰므로 브라우저에서 A4 실제 비율로 보이고 인쇄 실측도 된다.
//
//   ros2 run sim_core trace_cli --dummy --case weak-force --out-dir ~/data/out
//   ros2 run sim_core trace_cli --trace ~/data/trace.csv --plan ~/data/map.csv --out-dir ~/data/out
//
// `--dummy` 는 MuJoCo 없이 하네스만 검증한다 — 시뮬 연동이 막혀도 확인 경로가
// 서 있는지 따로 확인할 수 있게 한 것이다 (vision 의 stroke_map_cli --dummy 와 같은 뜻).
#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "atlier/sim/contact_trace.hpp"
#include "atlier/sim/planned_map.hpp"
#include "atlier/sim/trace_io.hpp"
#include "atlier/sim/trace_svg.hpp"

namespace
{

using atlier::sim::ContactSample;
using atlier::sim::DrawnTrace;
using atlier::sim::PlannedMap;
using atlier::sim::PlannedStroke;
using atlier::sim::Point2;

struct Options
{
  std::string trace_path;
  std::string plan_path;
  std::string out_dir;
  bool dummy{false};
  std::string dummy_case{"ideal"};
  double step_mm{1.0};       ///< vision Params::resample_step_mm 과 같은 뜻
  double dt_s{0.02};
  bool show_force{false};
  bool show_travel{false};
};

const std::vector<std::string> kCases = {"ideal", "weak-force", "drift"};

/// 하네스 검증용 계획 지도 — 여백 15mm 안의 테두리 · 대각선 · 원.
/// vision 의 MakeDummyMap 과 같은 뜻이며, 형식만 map.csv 경유로 맞춰져 있다.
PlannedMap MakeDummyPlan()
{
  PlannedMap map;
  const double margin = 15.0;
  const double w = map.paper_w_mm;
  const double h = map.paper_h_mm;

  PlannedStroke border;
  border.closed = true;
  for (const auto & corner : {
      std::pair<double, double>{margin, margin},
      std::pair<double, double>{w - margin, margin},
      std::pair<double, double>{w - margin, h - margin},
      std::pair<double, double>{margin, h - margin}})
  {
    border.points.push_back(Point2{corner.first, corner.second});
  }
  map.strokes.push_back(border);

  PlannedStroke diagonal;
  diagonal.points.push_back(Point2{margin, margin});
  diagonal.points.push_back(Point2{w - margin, h - margin});
  map.strokes.push_back(diagonal);

  PlannedStroke circle;
  circle.closed = true;
  const double cx = w / 2.0;
  const double cy = h / 2.0;
  const double radius = 45.0;
  for (int step = 0; step < 72; ++step) {
    const double angle = 2.0 * M_PI * step / 72.0;
    circle.points.push_back(Point2{cx + radius * std::cos(angle), cy + radius * std::sin(angle)});
  }
  map.strokes.push_back(circle);
  return map;
}

/// 계획을 따라 "실제로 그은" 궤적을 흉내낸다.
///
/// 시나리오가 셋인 이유는 R2(필압 제어 실패)가 어떤 모습으로 나타나는지를 미리
/// 눈에 익히기 위해서다 — 실제 MuJoCo 결과를 볼 때 무엇을 찾아야 하는지 안다.
DrawnTrace MakeDummyTrace(const PlannedMap & plan, const Options & options)
{
  DrawnTrace trace;
  trace.paper_w_mm = plan.paper_w_mm;
  trace.paper_h_mm = plan.paper_h_mm;

  double now_s = 0.0;
  double drift_mm = 0.0;
  std::size_t global_index = 0;

  for (const PlannedStroke & stroke : plan.strokes) {
    std::vector<Point2> points = stroke.points;
    if (stroke.closed && points.size() >= 2) {
      points.push_back(points.front());
    }

    for (std::size_t index = 0; index + 1 < points.size(); ++index) {
      const Point2 & from = points[index];
      const Point2 & to = points[index + 1];
      const double dx = to.x_mm - from.x_mm;
      const double dy = to.y_mm - from.y_mm;
      const double length = std::sqrt(dx * dx + dy * dy);
      const int steps = std::max(1, static_cast<int>(length / options.step_mm));

      for (int step = 0; step < steps; ++step) {
        const double ratio = static_cast<double>(step) / steps;
        ContactSample sample;
        sample.t_s = now_s;
        sample.x_mm = from.x_mm + dx * ratio;
        sample.y_mm = from.y_mm + dy * ratio;

        // 기본 필압 — 테이프 고정이라 z 완충이 없어 값이 튄다는 전제를 흉내낸다.
        sample.force_n = 0.45 + 0.05 * std::sin(global_index * 0.05);

        if (options.dummy_case == "weak-force") {
          // 특정 구간에서 필압이 무너져 펜이 뜬다 → 한 스트로크가 여러 조각으로 쪼개진다.
          if ((global_index / 40) % 5 == 3) {
            sample.force_n = 0.01;
          }
        } else if (options.dummy_case == "drift") {
          // 위치 오차가 누적된다 (캘리브레이션 오차 R5 의 모습).
          drift_mm += 0.0015;
          sample.x_mm += drift_mm;
          sample.y_mm += drift_mm * 0.5;
        }

        sample.in_contact = sample.force_n > 0.05;
        trace.samples.push_back(sample);
        now_s += options.dt_s;
        ++global_index;
      }
    }

    // 스트로크 사이 — pen-up 후 다음 시작점으로 이동 (접촉 없음).
    for (int step = 0; step < 10; ++step) {
      ContactSample sample;
      sample.t_s = now_s;
      sample.x_mm = points.empty() ? 0.0 : points.back().x_mm;
      sample.y_mm = points.empty() ? 0.0 : points.back().y_mm;
      sample.force_n = 0.0;
      sample.in_contact = false;
      trace.samples.push_back(sample);
      now_s += options.dt_s;
    }
  }
  return trace;
}

void PrintHelp()
{
  std::cout <<
    "trace_cli — 접촉 궤적 확인 하네스 (MuJoCo 시뮬 결과를 눈으로 본다)\n\n"
    "  --trace FILE    시뮬이 남긴 궤적 CSV (t_s,x_mm,y_mm,force_n,in_contact)\n"
    "  --plan FILE     vision 이 만든 계획 지도 map.csv — 겹쳐 그린다\n"
    "  --out-dir DIR   산출물 저장 위치. 없으면 stdout 만\n"
    "  --dummy         MuJoCo 없이 가짜 궤적 생성 (하네스 검증용)\n"
    "  --case NAME     더미 시나리오: ideal | weak-force | drift\n"
    "  --step-mm F     더미 샘플 간격 (기본 1.0 — vision resample_step_mm 과 같은 뜻)\n"
    "  --show-force    필압을 선 굵기에 반영\n"
    "  --show-travel   공중 이동(pen-up)을 점선으로\n"
    "  --help\n\n"
    "산출물: drawn.svg (계획+실제 겹침) · trace.csv · summary.txt\n";
}

}  // namespace

int main(int argc, char ** argv)
{
  Options options;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto next = [&]() -> std::string {
        if (index + 1 >= argc) {
          throw std::invalid_argument(argument + " 에 값이 없습니다");
        }
        return argv[++index];
      };
    try {
      if (argument == "--help" || argument == "-h") {
        PrintHelp();
        return 0;
      } else if (argument == "--trace") {
        options.trace_path = next();
      } else if (argument == "--plan") {
        options.plan_path = next();
      } else if (argument == "--out-dir") {
        options.out_dir = next();
      } else if (argument == "--dummy") {
        options.dummy = true;
      } else if (argument == "--case") {
        options.dummy_case = next();
      } else if (argument == "--step-mm") {
        options.step_mm = std::stod(next());
      } else if (argument == "--show-force") {
        options.show_force = true;
      } else if (argument == "--show-travel") {
        options.show_travel = true;
      } else {
        std::cerr << "알 수 없는 인자: " << argument << "\n";
        PrintHelp();
        return 2;
      }
    } catch (const std::exception & error) {
      std::cerr << "인자 오류: " << error.what() << "\n";
      return 2;
    }
  }

  if (!options.dummy && options.trace_path.empty()) {
    std::cerr << "--trace 나 --dummy 중 하나는 있어야 합니다. --help 참조\n";
    return 2;
  }
  if (options.dummy &&
    std::find(kCases.begin(), kCases.end(), options.dummy_case) == kCases.end())
  {
    std::cerr << "--case 는 ideal | weak-force | drift 중 하나여야 합니다\n";
    return 2;
  }

  try {
    PlannedMap plan;
    bool has_plan = false;

    if (!options.plan_path.empty()) {
      plan = atlier::sim::LoadPlannedMapCsv(options.plan_path);
      has_plan = true;
    } else if (options.dummy) {
      // 더미일 때는 계획도 같이 만든다 — 겹쳐 봐야 "계획대로인가"가 보인다.
      plan = MakeDummyPlan();
      has_plan = true;
    }

    const DrawnTrace trace = options.dummy
      ? MakeDummyTrace(plan, options)
      : atlier::sim::LoadTraceCsv(options.trace_path);

    if (options.dummy) {
      std::cout << "더미 시나리오: " << options.dummy_case << "\n\n";
    }
    std::cout << atlier::sim::RenderSummary(trace, has_plan ? &plan : nullptr) << "\n";

    if (options.out_dir.empty()) {
      return 0;
    }
    std::filesystem::create_directories(options.out_dir);

    atlier::sim::SvgStyle style;
    style.width_by_force = options.show_force;
    style.show_travel = options.show_travel;

    const bool ok =
      atlier::sim::SaveSvg(trace, has_plan ? &plan : nullptr, options.out_dir + "/drawn.svg", style) &&
      atlier::sim::SaveTraceCsv(trace, options.out_dir + "/trace.csv") &&
      atlier::sim::SaveText(
      options.out_dir + "/summary.txt",
      atlier::sim::RenderSummary(trace, has_plan ? &plan : nullptr));

    std::cout << (ok ? "저장: " : "⚠️ 저장 실패: ") << options.out_dir
              << "/{drawn.svg, trace.csv, summary.txt}\n";
    return ok ? 0 : 1;
  } catch (const std::exception & error) {
    std::cerr << "실패: " << error.what() << "\n";
    return 1;
  }
}
