// 운영 계층 경계 타입 — "작업 1건"이 무엇인가.
//
// 이 헤더는 ROS·웹·DB 어디에도 의존하지 않는다. 작업을 소비하는 쪽(웹 백엔드,
// 나중에 붙을 이력 저장소)이 이 헤더 하나에 대응하는 형식만 알면 되게 하기
// 위함이다. 의존을 늘리지 말 것.
//
// ⚠️ 네임스페이스가 `atlier::operator` 가 아닌 이유 — `operator` 는 C++ 예약어다.
// 디렉터리는 `src/operator`(SA 의 모듈명)지만 코드상 이름은 이 모듈의 중심
// 개념인 `job` 을 쓴다.
#ifndef ATLIER_JOB_JOB_HPP
#define ATLIER_JOB_JOB_HPP

#include <string>
#include <vector>

namespace atlier::job
{

/// 작업 1건이 놓일 수 있는 상태.
///
/// BRD F6.2(모니터링)가 보여주는 것, F6.3(이력)이 저장하는 것, F6.4·F6.5가
/// 가르는 것이 전부 이 한 열거형의 다른 면이다.
enum class State
{
  Received,        ///< F1.1 업로드 수신 — 적합성 검사 대기
  Ready,           ///< F1.2 통과. **AC1 의 출발선** — 사람이 종이를 놓고 Start 를 누르기 직전
  Rejected,        ///< F1.2 부적합 — 종료. 로봇은 한 번도 움직이지 않는다 (PF §3.1)
  Planning,        ///< F2 지도 생성 + F3 순서 최적화·펜업다운
  Drawing,         ///< F4.3 궤적 실행 중
  AwaitingReview,  ///< F8.1 중간 확인 — 사람 판단 대기. **작업당 1회만**
  Paused,          ///< F6.5 상태 보존 정지 — 재개하면 멈추기 직전 상태로 돌아간다
  Capturing,       ///< F8.2 완성작 촬영
  Completed,       ///< 종료 ✓
  Aborted,         ///< F6.4 긴급정지 또는 F8.1 사람 중단 — 종료 ✗ (상태 폐기)
  Failed,          ///< 계획·실행 실패 — 종료 ✗
};

/// 사람이 운영 UI 에서 내리는 명령.
///
/// Start 를 뺀 나머지는 모두 작화 흐름에 손대는 행위라 개입 로그(Intervention)에
/// 남는다. Start 는 AC1 의 출발선 자체이므로 개입이 아니다.
enum class Command
{
  Start,           ///< 작화 시작 — AC1 측정 시작점
  EmergencyStop,   ///< F6.4 — 즉시 정지, 상태 **폐기**, 재개 없음
  Pause,           ///< F6.5 — pen-up 후 정지, 상태 **보존**
  Resume,          ///< F6.5 — 중단 스트로크부터 이어서
  ReviewContinue,  ///< F8.1 — 차이 정보를 보고 "계속"
  ReviewAbort,     ///< F8.1 — 차이 정보를 보고 "중단"
};

/// 파이프라인(vision·moveit2)이 올려보내는 사실. 사람의 뜻이 아니라 일어난 일이다.
enum class Event
{
  InputAccepted,   ///< F1.2 통과 (판정 주체는 vision — README "F1.2 는 누가 하는가")
  InputRejected,   ///< F1.2 거부 — 사유 동반
  PlanReady,       ///< F2+F3 완료 — PlanSummary 동반
  PlanFailed,
  StrokeDone,      ///< F4.3 스트로크 1개 완료
  StrokeFailed,    ///< F4.4 실패 — 기록만 하고 흐름은 멈추지 않는다
  ReviewTimeout,   ///< 확인 무응답. **사람 개입이 아니다** (AC1 해석 ① — README 미결 참조)
  DrawingDone,     ///< 전량 완료
  CaptureDone,     ///< F8.2 완성작 촬영 완료
};

/// 사람이 작화 흐름에 손댄 사건.
///
/// **AC1 을 로그로 증명하려면 이것이 필요하다** — "종이가 놓인 뒤부터 완성까지
/// 사람이 손대면 불합격"인데, BRD F6.3 의 저장 항목(입력·계획·결과·소요시간)에는
/// 개입 기록이 없어 완주 여부를 사후에 확인할 근거가 없었다.
struct Intervention
{
  double at_s{0.0};                        ///< Start 기준 경과 시간
  int at_stroke{-1};                       ///< 그 시점 스트로크 인덱스 (-1 = 작화 전)
  Command command{Command::EmergencyStop};
};

/// 계획 요약 — F3.3 이 남기는 로그가 그대로 AC2 측정 근거가 된다.
struct PlanSummary
{
  int stroke_count{0};          ///< 지도의 스트로크 개수 (vision StrokeMap::strokes.size())
  double draw_len_mm{0.0};      ///< 펜을 내린 채 이동하는 총 거리 (vision TotalDrawLengthMm)
  double travel_before_s{0.0};  ///< F3.3 — 무최적화(입력 순서 그대로) 총 이동시간
  double travel_after_s{0.0};   ///< F3.3 — 최적화 후 총 이동시간
};

/// 작업 1건. **BRD F6.3 이 저장할 대상의 원형** — 웹 백엔드는 이 구조를 그대로
/// 받아 DB 한 행으로 옮긴다 (경계 형식은 job_json.hpp).
struct Record
{
  std::string id;
  State state{State::Received};

  // ── 입력 (F1) ──
  std::string input_image_path;
  std::string reject_reason;              ///< F1.2 — "사유와 함께 거부"

  // ── 계획 (F3.3 → AC2) ──
  PlanSummary plan;

  // ── 실행 (F4) ──
  int strokes_done{0};
  std::vector<int> failed_strokes;        ///< F4.4 — 기록 후 계속, 재시도 없음
  int aborted_at_stroke{-1};              ///< F6.4 — 이력에 남길 "중단 + 스트로크 번호"

  // ── 시간 (N2 15분) ──
  // **음수가 "아직 없음"이다.** 0 을 센티널로 쓰면 시각 0 에 시작한 작업(시뮬·테스트에서
  // 흔하다)이 미시작으로 읽혀 소요 시간이 통째로 0 이 된다.
  double started_at_s{-1.0};              ///< Command::Start 시각. AC1 측정 시작선
  double ended_at_s{-1.0};                ///< 종료 상태 진입 시각

  // ── 개입 (AC1) ──
  std::vector<Intervention> interventions;

  // ── 결과 (F8.2 → AC3) ──
  std::string result_image_path;
  std::string quality_verdict;            ///< CLIP 판정. SA §4 DB 항목엔 있고 BRD F6.3 엔 없다
};

// ── 판정 유틸 ───────────────────────────────────────────────────────────────
// 수락 기준을 코드가 직접 답하게 둔다. 운영 계층이 세 기준의 증거를 만드는
// 유일한 자리라서다 (README "운영 계층이 만드는 증거").

/// 더 이상 전이가 없는 상태인가.
bool IsTerminal(State state);

const char * ToString(State state);
const char * ToString(Command command);
const char * ToString(Event event);

/// 작화 소요 시간(초). 아직 끝나지 않았으면 0 을 돌려준다.
double ElapsedS(const Record & record);

/// **N2** — 스트로크 300~500개 기준 1장 15분 이내.
bool MeetsN2(const Record & record, double budget_s = 900.0);

/// **AC2** — 무최적화 대비 총 이동시간 단축률. before 가 0 이면 0 을 돌려준다.
double OptimizationGainRatio(const PlanSummary & plan);
bool MeetsAc2(const PlanSummary & plan, double threshold = 0.20);

/// 작화 도중 사람 개입이 없었는가. 개입 로그가 비어 있으면 true.
///
/// ReviewTimeout(무응답 자동 계속)은 애초에 Intervention 으로 남지 않으므로 여기서
/// 걸리지 않는다 — 그것이 AC1 해석 ①이다.
bool WasUnattended(const Record & record);

/// **AC1** — "사람 개입 없이 A4 선화 **완성**".
///
/// 개입이 없는 것만으로는 부족하다. 거부·실패·중단된 작업도 개입이 없을 수 있지만
/// AC1 은 완주를 요구한다 — 두 조건을 함께 본다.
bool MeetsAc1(const Record & record);

}  // namespace atlier::job

#endif  // ATLIER_JOB_JOB_HPP
