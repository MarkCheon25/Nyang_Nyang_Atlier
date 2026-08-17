// 작업 레코드의 경계 형식 — 운영 계층 밖으로 나가는 유일한 표현.
//
// **저장하지 않는다.** SA §5.2 가 "DB 에 직접 붙는 것은 하나"로 정했고 그 하나는
// 웹 백엔드(Node.js)다. 이 모듈은 레코드를 직렬화까지만 하고, SQLite 에 넣는 일은
// rosbridge 를 구독하는 Node 가 한다. C++ 쪽에 DB 드라이버를 붙이지 말 것.
//
// 외부 JSON 라이브러리에 의존하지 않는다 — 내보내는 형식이 이 정도로 단순한데
// 경계 헤더에 의존을 하나 더 얹을 이유가 없다.
#ifndef ATLIER_JOB_JOB_JSON_HPP
#define ATLIER_JOB_JOB_JSON_HPP

#include <string>
#include <vector>

#include "atlier/job/job.hpp"
#include "atlier/job/job_machine.hpp"
#include "atlier/job/policy.hpp"

namespace atlier::job
{

/// 작업 1건 → JSON. 웹 백엔드가 DB 한 행으로 옮길 대상이다.
///
/// 수락 기준 판정(ac1_unattended · ac2_gain · n2_within_budget)을 함께 실어
/// 보낸다 — 판정 규칙이 두 언어에 중복 구현되면 반드시 어긋난다.
std::string ToJson(const Record & record, int indent = 2);

/// 정책 스냅샷. 어떤 설정으로 돌린 작업인지가 남아야 사후 비교가 된다.
std::string ToJson(const Policy & policy, int indent = 2);

/// 전이 이력까지 포함한 전체 덤프 (CLI 하네스 산출물 `job.json`).
std::string ToJson(
  const Record & record, const Policy & policy,
  const std::vector<Transition> & history, int indent = 2);

/// 문자열을 JSON 리터럴로 이스케이프한다 (따옴표 포함).
std::string JsonQuote(const std::string & text);

/// 텍스트 파일로 저장. 실패 시 false.
bool SaveText(const std::string & path, const std::string & content);

}  // namespace atlier::job

#endif  // ATLIER_JOB_JOB_JSON_HPP
