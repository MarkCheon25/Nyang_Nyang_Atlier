#include "atlier/job/job_json.hpp"

#include <cstdio>
#include <fstream>
#include <sstream>

namespace atlier::job
{
namespace
{

std::string Indent(int level, int width)
{
  return std::string(static_cast<std::size_t>(level * width), ' ');
}

/// 소수점 자리를 고정한다 — 기본 스트림 출력은 정밀도가 들쭉날쭉해 diff 가 지저분해진다.
std::string Num(double value, int precision = 3)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

std::string Bool(bool value)
{
  return value ? "true" : "false";
}

}  // namespace

std::string JsonQuote(const std::string & text)
{
  std::string out = "\"";
  for (const char character : text) {
    switch (character) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        // 제어문자는 \u 로. 한글은 UTF-8 바이트를 그대로 흘려보낸다 (JSON 은 UTF-8 이 기본).
        if (static_cast<unsigned char>(character) < 0x20) {
          char buffer[8];
          std::snprintf(buffer, sizeof(buffer), "\\u%04x", character);
          out += buffer;
        } else {
          out += character;
        }
    }
  }
  out += "\"";
  return out;
}

std::string ToJson(const Record & record, int indent)
{
  std::ostringstream out;
  const std::string i1 = Indent(1, indent);
  const std::string i2 = Indent(2, indent);

  out << "{\n";
  out << i1 << "\"id\": " << JsonQuote(record.id) << ",\n";
  out << i1 << "\"state\": " << JsonQuote(ToString(record.state)) << ",\n";
  out << i1 << "\"terminal\": " << Bool(IsTerminal(record.state)) << ",\n";

  out << i1 << "\"input\": {\n";
  out << i2 << "\"image_path\": " << JsonQuote(record.input_image_path) << ",\n";
  out << i2 << "\"reject_reason\": " << JsonQuote(record.reject_reason) << "\n";
  out << i1 << "},\n";

  out << i1 << "\"plan\": {\n";
  out << i2 << "\"stroke_count\": " << record.plan.stroke_count << ",\n";
  out << i2 << "\"draw_len_mm\": " << Num(record.plan.draw_len_mm) << ",\n";
  out << i2 << "\"travel_before_s\": " << Num(record.plan.travel_before_s) << ",\n";
  out << i2 << "\"travel_after_s\": " << Num(record.plan.travel_after_s) << "\n";
  out << i1 << "},\n";

  out << i1 << "\"execution\": {\n";
  out << i2 << "\"strokes_done\": " << record.strokes_done << ",\n";
  out << i2 << "\"failed_strokes\": [";
  for (std::size_t index = 0; index < record.failed_strokes.size(); ++index) {
    out << (index ? ", " : "") << record.failed_strokes[index];
  }
  out << "],\n";
  out << i2 << "\"aborted_at_stroke\": " << record.aborted_at_stroke << "\n";
  out << i1 << "},\n";

  out << i1 << "\"timing\": {\n";
  out << i2 << "\"started_at_s\": " << Num(record.started_at_s) << ",\n";
  out << i2 << "\"ended_at_s\": " << Num(record.ended_at_s) << ",\n";
  out << i2 << "\"elapsed_s\": " << Num(ElapsedS(record)) << "\n";
  out << i1 << "},\n";

  out << i1 << "\"interventions\": [\n";
  for (std::size_t index = 0; index < record.interventions.size(); ++index) {
    const Intervention & intervention = record.interventions[index];
    out << i2 << "{\"at_s\": " << Num(intervention.at_s)
        << ", \"at_stroke\": " << intervention.at_stroke
        << ", \"command\": " << JsonQuote(ToString(intervention.command)) << "}"
        << (index + 1 < record.interventions.size() ? "," : "") << "\n";
  }
  out << i1 << "],\n";

  out << i1 << "\"result\": {\n";
  out << i2 << "\"image_path\": " << JsonQuote(record.result_image_path) << ",\n";
  out << i2 << "\"quality_verdict\": " << JsonQuote(record.quality_verdict) << "\n";
  out << i1 << "},\n";

  // 판정 규칙이 C++ 과 웹 양쪽에 중복 구현되면 반드시 어긋난다 — 결과를 실어 보낸다.
  out << i1 << "\"acceptance\": {\n";
  out << i2 << "\"ac1_met\": " << Bool(MeetsAc1(record)) << ",\n";
  out << i2 << "\"unattended\": " << Bool(WasUnattended(record)) << ",\n";
  out << i2 << "\"ac2_gain_ratio\": " << Num(OptimizationGainRatio(record.plan)) << ",\n";
  out << i2 << "\"ac2_met\": " << Bool(MeetsAc2(record.plan)) << ",\n";
  out << i2 << "\"n2_within_budget\": " << Bool(MeetsN2(record)) << "\n";
  out << i1 << "}\n";

  out << "}";
  return out.str();
}

std::string ToJson(const Policy & policy, int indent)
{
  std::ostringstream out;
  const std::string i1 = Indent(1, indent);
  out << "{\n";
  out << i1 << "\"review_enabled\": " << Bool(policy.review_enabled) << ",\n";
  out << i1 << "\"review_at_ratio\": " << Num(policy.review_at_ratio) << ",\n";
  out << i1 << "\"review_timeout_s\": " << Num(policy.review_timeout_s, 1) << ",\n";
  out << i1 << "\"review_default_continue\": " << Bool(policy.review_default_continue) << ",\n";
  out << i1 << "\"capture_enabled\": " << Bool(policy.capture_enabled) << ",\n";
  out << i1 << "\"stroke_retry_count\": " << policy.stroke_retry_count << ",\n";
  out << i1 << "\"pen_up_on_pause\": " << Bool(policy.pen_up_on_pause) << ",\n";
  out << i1 << "\"pen_up_on_estop\": " << Bool(policy.pen_up_on_estop) << "\n";
  out << "}";
  return out.str();
}

std::string ToJson(
  const Record & record, const Policy & policy,
  const std::vector<Transition> & history, int indent)
{
  std::ostringstream out;
  const std::string i1 = Indent(1, indent);
  const std::string i2 = Indent(2, indent);

  // 중첩 객체를 통째로 들여쓰기 위해 줄 단위로 다시 흘려 넣는다.
  auto embed = [&](const std::string & json, const std::string & pad) {
      std::istringstream in(json);
      std::string line;
      std::string result;
      bool first = true;
      while (std::getline(in, line)) {
        result += (first ? "" : "\n" + pad) + line;
        first = false;
      }
      return result;
    };

  out << "{\n";
  out << i1 << "\"record\": " << embed(ToJson(record, indent), i1) << ",\n";
  out << i1 << "\"policy\": " << embed(ToJson(policy, indent), i1) << ",\n";
  out << i1 << "\"history\": [\n";
  for (std::size_t index = 0; index < history.size(); ++index) {
    const Transition & transition = history[index];
    out << i2 << "{\"seq\": " << index
        << ", \"at_s\": " << Num(transition.at_s, 1)
        << ", \"accepted\": " << Bool(transition.accepted)
        << ", \"from\": " << JsonQuote(ToString(transition.from))
        << ", \"to\": " << JsonQuote(ToString(transition.to))
        << ", \"label\": " << JsonQuote(transition.label)
        << ", \"note\": " << JsonQuote(transition.note) << "}"
        << (index + 1 < history.size() ? "," : "") << "\n";
  }
  out << i1 << "]\n";
  out << "}\n";
  return out.str();
}

bool SaveText(const std::string & path, const std::string & content)
{
  std::ofstream file(path);
  if (!file) {
    return false;
  }
  file << content;
  return static_cast<bool>(file);
}

}  // namespace atlier::job
