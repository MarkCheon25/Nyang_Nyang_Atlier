#include "atlier/sim/planned_map.hpp"

#include <cmath>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace atlier::sim
{
namespace
{

/// 한 줄을 쉼표로 자른다. vision 의 map.csv 는 따옴표를 쓰지 않으므로 단순 분할로 족하다.
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

PlannedMap LoadPlannedMapCsv(const std::string & path)
{
  std::ifstream file(path);
  if (!file) {
    throw std::runtime_error("계획 지도를 열 수 없습니다: " + path);
  }

  PlannedMap map;
  std::string line;
  std::size_t line_number = 0;
  long current_stroke = -1;

  while (std::getline(file, line)) {
    ++line_number;
    if (line.empty()) {
      continue;
    }
    // 헤더 줄은 첫 필드가 숫자가 아니다.
    if (line_number == 1 && line.rfind("stroke_idx", 0) == 0) {
      continue;
    }

    const std::vector<std::string> fields = SplitCsv(line);
    if (fields.size() < 4) {
      throw std::runtime_error(
              "map.csv 형식이 맞지 않습니다 (" + path + " " + std::to_string(line_number) +
              "행): stroke_idx,point_idx,x_mm,y_mm,closed 를 기대합니다");
    }

    try {
      const long stroke_index = std::stol(fields[0]);
      Point2 point;
      point.x_mm = std::stod(fields[2]);
      point.y_mm = std::stod(fields[3]);
      const bool closed = fields.size() >= 5 &&
        (fields[4] == "1" || fields[4] == "true" || fields[4] == "True");

      // 스트로크 인덱스가 바뀌면 새 스트로크. vision 은 순서대로 뱉으므로 이걸로 족하다.
      if (stroke_index != current_stroke) {
        map.strokes.push_back(PlannedStroke{});
        current_stroke = stroke_index;
      }
      map.strokes.back().points.push_back(point);
      map.strokes.back().closed = closed;
    } catch (const std::exception &) {
      throw std::runtime_error(
              "map.csv 의 숫자를 읽지 못했습니다 (" + path + " " +
              std::to_string(line_number) + "행)");
    }
  }
  return map;
}

double PlannedLengthMm(const PlannedMap & map)
{
  double total = 0.0;
  for (const PlannedStroke & stroke : map.strokes) {
    for (std::size_t index = 1; index < stroke.points.size(); ++index) {
      const double dx = stroke.points[index].x_mm - stroke.points[index - 1].x_mm;
      const double dy = stroke.points[index].y_mm - stroke.points[index - 1].y_mm;
      total += std::sqrt(dx * dx + dy * dy);
    }
    // 폐곡선은 마지막 점에서 첫 점으로 돌아오는 구간까지 그린다 (vision 규약 —
    // 첫 점을 끝에 중복해 넣지 않으므로 여기서 더해야 한다).
    if (stroke.closed && stroke.points.size() >= 2) {
      const double dx = stroke.points.front().x_mm - stroke.points.back().x_mm;
      const double dy = stroke.points.front().y_mm - stroke.points.back().y_mm;
      total += std::sqrt(dx * dx + dy * dy);
    }
  }
  return total;
}

}  // namespace atlier::sim
