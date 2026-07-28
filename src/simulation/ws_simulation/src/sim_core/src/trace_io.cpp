#include "atlier/sim/trace_io.hpp"

#include <fstream>
#include <iomanip>
#include <sstream>
#include <stdexcept>

namespace atlier::sim
{
namespace
{

std::string Fixed(double value, int precision = 2)
{
  std::ostringstream out;
  out.setf(std::ios::fixed);
  out.precision(precision);
  out << value;
  return out.str();
}

std::vector<std::string> SplitCsv(const std::string & line)
{
  std::vector<std::string> fields;
  std::istringstream stream(line);
  std::string field;
  while (std::getline(stream, field, ',')) {
    fields.push_back(field);
  }
  return fields;
}

}  // namespace

bool SaveTraceCsv(const DrawnTrace & trace, const std::string & path)
{
  std::ofstream file(path);
  if (!file) {
    return false;
  }
  file << "t_s,x_mm,y_mm,force_n,in_contact\n";
  for (const ContactSample & sample : trace.samples) {
    file << Fixed(sample.t_s, 4) << ","
         << Fixed(sample.x_mm, 4) << ","
         << Fixed(sample.y_mm, 4) << ","
         << Fixed(sample.force_n, 4) << ","
         << (sample.in_contact ? 1 : 0) << "\n";
  }
  return static_cast<bool>(file);
}

DrawnTrace LoadTraceCsv(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("궤적 파일을 열 수 없습니다: " + path);
  }

  DrawnTrace trace;
  std::string line;
  std::size_t line_number = 0;
  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    if (line_number == 1 && line.rfind("t_s", 0) == 0) {
      continue;
    }
    const std::vector<std::string> fields = SplitCsv(line);
    if (fields.size() < 5) {
      throw std::runtime_error(
              "trace.csv 형식이 맞지 않습니다 (" + path + " " + std::to_string(line_number) +
              "행): t_s,x_mm,y_mm,force_n,in_contact 를 기대합니다");
    }
    try {
      ContactSample sample;
      sample.t_s = std::stod(fields[0]);
      sample.x_mm = std::stod(fields[1]);
      sample.y_mm = std::stod(fields[2]);
      sample.force_n = std::stod(fields[3]);
      sample.in_contact = fields[4] == "1" || fields[4] == "true";
      trace.samples.push_back(sample);
    } catch (const std::exception &) {
      throw std::runtime_error(
              "trace.csv 의 숫자를 읽지 못했습니다 (" + path + " " +
              std::to_string(line_number) + "행)");
    }
  }
  return trace;
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

std::string RenderSummary(const DrawnTrace & trace, const PlannedMap * planned)
{
  std::ostringstream out;
  const std::size_t segments = SegmentCount(trace);
  const double drawn_mm = DrawnLengthMm(trace);

  out << "실제로 그려진 것\n";
  out << "  샘플        : " << SampleCount(trace) << " 개\n";
  out << "  선 조각     : " << segments << " 개\n";
  out << "  그린 길이   : " << Fixed(drawn_mm, 1) << " mm\n";
  out << "  접촉 비율   : " << Fixed(ContactDutyRatio(trace) * 100.0, 1)
      << " %  (나머지는 공중 이동 — N2 예산)\n";

  double min_n = 0.0;
  double max_n = 0.0;
  double mean_n = 0.0;
  if (ForceStats(trace, min_n, max_n, mean_n)) {
    out << "  필압        : 최소 " << Fixed(min_n, 3) << " · 평균 " << Fixed(mean_n, 3)
        << " · 최대 " << Fixed(max_n, 3) << " N   ← R2 판단 근거\n";
  } else {
    out << "  필압        : — (접촉 샘플 없음)\n";
  }

  Point2 min_point;
  Point2 max_point;
  if (BoundingBox(trace, min_point, max_point)) {
    out << "  바운딩박스  : (" << Fixed(min_point.x_mm, 1) << ", " << Fixed(min_point.y_mm, 1)
        << ") ~ (" << Fixed(max_point.x_mm, 1) << ", " << Fixed(max_point.y_mm, 1) << ") mm\n";
  }

  if (planned == nullptr) {
    return out.str();
  }

  const double planned_mm = PlannedLengthMm(*planned);
  out << "\n계획 대비\n";
  out << "  스트로크    : 계획 " << planned->strokes.size() << " 개 → 실제 조각 "
      << segments << " 개";
  if (segments > planned->strokes.size()) {
    // 계획보다 조각이 많다 = 한 스트로크가 도중에 끊겼다는 뜻.
    out << "   ⚠️ **선 끊김 " << (segments - planned->strokes.size())
        << " 회** — 필압 부족 의심 (BRD R2)";
  } else if (segments < planned->strokes.size()) {
    out << "   ⚠️ 덜 그림 " << (planned->strokes.size() - segments) << " 개";
  }
  out << "\n";
  out << "  선 길이     : 계획 " << Fixed(planned_mm, 1) << " → 실제 " << Fixed(drawn_mm, 1)
      << " mm";
  if (planned_mm > 0.0) {
    out << "  (" << Fixed(drawn_mm / planned_mm * 100.0, 1) << " %)";
  }
  out << "\n";
  return out.str();
}

}  // namespace atlier::sim
