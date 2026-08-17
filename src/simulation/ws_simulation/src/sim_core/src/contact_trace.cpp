#include "atlier/sim/contact_trace.hpp"

#include <algorithm>
#include <cmath>

namespace atlier::sim
{

Point2 WorldToPaper(const PaperFrame & frame, double world_x_m, double world_y_m)
{
  const double dx = world_x_m - frame.origin_x_m;
  const double dy = world_y_m - frame.origin_y_m;

  // 종이 +x 축(월드 기준)과, 그것을 시계방향 90° 돌린 +y 축.
  // **종이 y 는 아래로 증가**한다(vision 과 같은 규약)는 것이 ey 의 부호에 들어 있다 —
  // 위에서 내려다볼 때 화면 아래쪽이 종이의 +y 다.
  const double ex_x = std::cos(frame.yaw_rad);
  const double ex_y = std::sin(frame.yaw_rad);
  const double ey_x = std::sin(frame.yaw_rad);
  const double ey_y = -std::cos(frame.yaw_rad);

  Point2 point;
  point.x_mm = (dx * ex_x + dy * ex_y) * 1000.0;
  point.y_mm = (dx * ey_x + dy * ey_y) * 1000.0;
  return point;
}

std::size_t SampleCount(const DrawnTrace & trace)
{
  return trace.samples.size();
}

std::size_t SegmentCount(const DrawnTrace & trace)
{
  std::size_t count = 0;
  bool previous = false;
  for (const ContactSample & sample : trace.samples) {
    // 떨어져 있다가 닿는 순간마다 새 조각이 시작된다.
    if (sample.in_contact && !previous) {
      ++count;
    }
    previous = sample.in_contact;
  }
  return count;
}

double DrawnLengthMm(const DrawnTrace & trace)
{
  double total = 0.0;
  for (std::size_t index = 1; index < trace.samples.size(); ++index) {
    const ContactSample & previous = trace.samples[index - 1];
    const ContactSample & current = trace.samples[index];
    // 양 끝이 모두 닿아 있어야 그 구간에 선이 남는다.
    if (!previous.in_contact || !current.in_contact) {
      continue;
    }
    const double dx = current.x_mm - previous.x_mm;
    const double dy = current.y_mm - previous.y_mm;
    total += std::sqrt(dx * dx + dy * dy);
  }
  return total;
}

bool ForceStats(const DrawnTrace & trace, double & min_n, double & max_n, double & mean_n)
{
  double sum = 0.0;
  std::size_t count = 0;
  for (const ContactSample & sample : trace.samples) {
    if (!sample.in_contact) {
      continue;
    }
    if (count == 0 || sample.force_n < min_n) {
      min_n = sample.force_n;
    }
    if (count == 0 || sample.force_n > max_n) {
      max_n = sample.force_n;
    }
    sum += sample.force_n;
    ++count;
  }
  if (count == 0) {
    return false;
  }
  mean_n = sum / static_cast<double>(count);
  return true;
}

double ContactDutyRatio(const DrawnTrace & trace)
{
  if (trace.samples.empty()) {
    return 0.0;
  }
  // 샘플 간격이 균일하다는 전제로 개수 비율을 쓴다 — 시뮬은 고정 timestep 으로 돈다.
  std::size_t contact = 0;
  for (const ContactSample & sample : trace.samples) {
    if (sample.in_contact) {
      ++contact;
    }
  }
  return static_cast<double>(contact) / static_cast<double>(trace.samples.size());
}

bool BoundingBox(const DrawnTrace & trace, Point2 & min_out, Point2 & max_out)
{
  bool found = false;
  for (const ContactSample & sample : trace.samples) {
    // 떠 있는 동안의 위치는 종이에 아무것도 남기지 않으므로 범위에서 뺀다.
    if (!sample.in_contact) {
      continue;
    }
    if (!found) {
      min_out = {sample.x_mm, sample.y_mm};
      max_out = {sample.x_mm, sample.y_mm};
      found = true;
      continue;
    }
    min_out.x_mm = std::min(min_out.x_mm, sample.x_mm);
    min_out.y_mm = std::min(min_out.y_mm, sample.y_mm);
    max_out.x_mm = std::max(max_out.x_mm, sample.x_mm);
    max_out.y_mm = std::max(max_out.y_mm, sample.y_mm);
  }
  return found;
}

}  // namespace atlier::sim
