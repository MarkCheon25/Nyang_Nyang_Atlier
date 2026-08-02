// job_cli — 작업 생애주기 확인 하네스.
//
// **이 도구가 있는 이유는 상태 머신이 맞게 도는지 눈으로 보기 위해서다.**
// 로봇도 웹도 없이, 가짜 파이프라인이 스트로크를 흉내내며 Machine 을 관통시킨다.
// 시나리오 9 종이 상태 머신의 모든 간선을 한 번씩 밟는다.
//
//   ros2 run job_core job_cli --scenario all --out-dir ~/data/out
//
// 시간은 가상 시계다 — 15분짜리 작업이 순식간에 재생된다. Machine 이 시계를
// 들고 있지 않은 이유가 이것이다.
#include <algorithm>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "atlier/job/job.hpp"
#include "atlier/job/job_json.hpp"
#include "atlier/job/job_machine.hpp"
#include "atlier/job/job_timeline.hpp"
#include "atlier/job/policy.hpp"

namespace
{

using atlier::job::Command;
using atlier::job::Machine;
using atlier::job::PlanSummary;
using atlier::job::Policy;
using atlier::job::State;

struct Options
{
  std::string scenario{"happy"};
  int strokes{12};
  double stroke_time_s{2.0};   ///< N2 근거: 300~500 스트로크 / 15분 = 스트로크당 1.8~3.0 s
  double plan_time_s{3.0};
  std::string out_dir;
};

const std::vector<std::string> kScenarios = {
  "happy", "reject", "plan-fail", "estop", "pause",
  "review-continue", "review-abort", "timeout", "stroke-fail",
};

/// 가짜 계획 요약. 실제 값은 vision 의 지도와 moveit2 의 F3.3 로그가 채운다.
/// 여기서는 AC2(20% 이상 단축)를 통과하는 비율로 만들어 판정 경로까지 검증한다.
PlanSummary MakePlan(const Options & options)
{
  PlanSummary plan;
  plan.stroke_count = options.strokes;
  plan.draw_len_mm = options.strokes * 45.0;
  plan.travel_after_s = options.strokes * options.stroke_time_s;
  plan.travel_before_s = plan.travel_after_s * 1.35;   // → 25.9% 단축
  return plan;
}

/// 스트로크를 진행시킨다. 목표에 닿거나 Drawing 이 아니게 되면(F8.1 확인 지점 등) 멈춘다.
void Draw(
  Machine & machine, double & now_s, const Options & options, int target,
  const std::vector<int> & fail_at = {})
{
  while (machine.record().strokes_done < target && machine.state() == State::Drawing) {
    now_s += options.stroke_time_s;
    const int index = machine.record().strokes_done;   // 지금 그릴 스트로크
    const bool fails =
      std::find(fail_at.begin(), fail_at.end(), index) != fail_at.end();
    if (fails) {
      machine.StrokeFailed(index, now_s);
    } else {
      machine.StrokeDone(now_s);
    }
  }
}

/// 업로드 → 검사 통과 → 시작 → 계획 완료까지. 모든 정상 시나리오의 공통 앞부분.
void OpenJob(Machine & machine, double & now_s, const Options & options)
{
  machine.InputAccepted(now_s);
  machine.Apply(Command::Start, now_s);
  now_s += options.plan_time_s;
  machine.PlanReady(MakePlan(options), now_s);
}

Machine RunScenario(const std::string & name, const Options & options)
{
  Policy policy;
  double now_s = 0.0;

  if (name == "happy") {
    // MVP(sim First Stroke) 설정 — SA 는 ⑤ 모니터링을 `이후`로 두었고 MuJoCo 에는
    // 카메라가 없다(PF §5.1). 둘 다 꺼야 sim 완주가 AC1 을 그대로 만족한다.
    policy.review_enabled = false;
    policy.capture_enabled = false;
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes);
    machine.DrawingDone(now_s);
    return machine;
  }

  if (name == "reject") {
    Machine machine(name, policy);
    machine.InputRejected("흰 배경이 아닙니다 — 배경 부적합 (PF §3.1)", now_s);
    // 거부된 작업에 시작을 걸어 본다 — 거절 경로도 이력에 남아야 한다.
    machine.Apply(Command::Start, now_s);
    return machine;
  }

  if (name == "plan-fail") {
    Machine machine(name, policy);
    machine.InputAccepted(now_s);
    machine.Apply(Command::Start, now_s);
    now_s += options.plan_time_s;
    machine.PlanFailed("지도에 스트로크가 0 개 — vision F2.1~F2.3 미구현", now_s);
    return machine;
  }

  if (name == "estop") {
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes / 3);
    machine.Apply(Command::EmergencyStop, now_s);
    // F6.4 는 재개가 없다 — 시도가 거절되는 것까지 확인한다.
    machine.Apply(Command::Resume, now_s);
    return machine;
  }

  if (name == "pause") {
    policy.review_enabled = false;   // 정지·재개만 보려고 중간 확인은 끈다
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes / 4);
    machine.Apply(Command::Pause, now_s);
    now_s += 30.0;                   // 사람이 멈춰 세워 둔 시간
    machine.Apply(Command::Resume, now_s);
    Draw(machine, now_s, options, options.strokes);
    machine.DrawingDone(now_s);
    now_s += 8.0;
    machine.CaptureDone("results/pause.png", now_s);
    return machine;
  }

  if (name == "review-continue") {
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes);          // 50% 에서 멈춘다
    now_s += 12.0;                                            // 사람이 차이 정보를 보는 시간
    machine.Apply(Command::ReviewContinue, now_s);
    Draw(machine, now_s, options, options.strokes);
    machine.DrawingDone(now_s);
    now_s += 8.0;
    machine.CaptureDone("results/review-continue.png", now_s);
    return machine;
  }

  if (name == "review-abort") {
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes);
    now_s += 12.0;
    machine.Apply(Command::ReviewAbort, now_s);
    return machine;
  }

  if (name == "timeout") {
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes);
    now_s += policy.review_timeout_s + 1.0;                   // 아무도 응답하지 않는다
    machine.Tick(now_s);                                      // → 자동 계속, 개입 아님
    Draw(machine, now_s, options, options.strokes);
    machine.DrawingDone(now_s);
    now_s += 8.0;
    machine.CaptureDone("results/timeout.png", now_s);
    return machine;
  }

  if (name == "stroke-fail") {
    policy.review_enabled = false;
    Machine machine(name, policy);
    OpenJob(machine, now_s, options);
    Draw(machine, now_s, options, options.strokes, {2, 7});   // F4.4 — 기록 후 계속
    machine.DrawingDone(now_s);
    now_s += 8.0;
    machine.CaptureDone("results/stroke-fail.png", now_s);
    return machine;
  }

  throw std::invalid_argument("알 수 없는 시나리오: " + name);
}

void Report(const std::string & name, const Machine & machine, const Options & options)
{
  std::cout << "\n══════════ 시나리오: " << name << " ══════════\n\n";
  std::cout << atlier::job::RenderText(machine.record(), machine.history()) << "\n";
  std::cout << atlier::job::RenderSummary(machine.record()) << "\n";

  if (options.out_dir.empty()) {
    return;
  }
  std::filesystem::create_directories(options.out_dir);
  const std::string base = options.out_dir + "/" + name;
  const bool ok =
    atlier::job::SaveText(
    base + ".job.json",
    atlier::job::ToJson(machine.record(), machine.policy(), machine.history())) &&
    atlier::job::SaveText(base + ".timeline.csv", atlier::job::RenderCsv(machine.history())) &&
    atlier::job::SaveText(
    base + ".timeline.md",
    atlier::job::RenderMermaid(machine.record(), machine.history()));
  std::cout << (ok ? "  저장: " : "  ⚠️ 저장 실패: ") << base << ".{job.json,timeline.csv,timeline.md}\n";
}

void PrintHelp()
{
  std::cout <<
    "job_cli — 작업 생애주기 확인 하네스 (⑥ 작업 관리)\n\n"
    "  --scenario NAME   시나리오 (기본 happy). `all` 이면 전부 실행\n"
    "  --strokes N       가짜 스트로크 개수 (기본 12)\n"
    "  --stroke-time S   스트로크 1개 소요 초 (기본 2.0 — N2 근거 1.8~3.0)\n"
    "  --out-dir PATH    산출물 저장 위치. 없으면 stdout 만\n"
    "  --list            시나리오 목록\n"
    "  --help\n\n"
    "산출물: <name>.job.json (F6.3 레코드 원형) · <name>.timeline.csv ·\n"
    "        <name>.timeline.md (mermaid — 실제로 밟은 경로)\n";
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
      } else if (argument == "--list") {
        for (const std::string & name : kScenarios) {
          std::cout << name << "\n";
        }
        return 0;
      } else if (argument == "--scenario") {
        options.scenario = next();
      } else if (argument == "--strokes") {
        options.strokes = std::stoi(next());
      } else if (argument == "--stroke-time") {
        options.stroke_time_s = std::stod(next());
      } else if (argument == "--out-dir") {
        options.out_dir = next();
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

  if (options.strokes < 4) {
    std::cerr << "--strokes 는 4 이상이어야 합니다 (중간 확인 지점이 성립하려면)\n";
    return 2;
  }

  try {
    if (options.scenario == "all") {
      for (const std::string & name : kScenarios) {
        Report(name, RunScenario(name, options), options);
      }
    } else {
      Report(options.scenario, RunScenario(options.scenario, options), options);
    }
  } catch (const std::exception & error) {
    std::cerr << "실패: " << error.what() << "\n";
    return 1;
  }
  return 0;
}
