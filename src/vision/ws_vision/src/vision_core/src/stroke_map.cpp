#include "atlier/vision/stroke_map.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace atlier::vision
{
namespace
{

double Distance(const Point2 & a, const Point2 & b)
{
  const double dx = a.x_mm - b.x_mm;
  const double dy = a.y_mm - b.y_mm;
  return std::sqrt(dx * dx + dy * dy);
}

}  // namespace

std::size_t TotalPointCount(const StrokeMap & map)
{
  std::size_t count = 0;
  for (const auto & stroke : map.strokes) {
    count += stroke.points.size();
  }
  return count;
}

double TotalDrawLengthMm(const StrokeMap & map)
{
  double total = 0.0;
  for (const auto & stroke : map.strokes) {
    if (stroke.points.size() < 2) {
      continue;
    }
    for (std::size_t i = 1; i < stroke.points.size(); ++i) {
      total += Distance(stroke.points[i - 1], stroke.points[i]);
    }
    if (stroke.closed) {
      total += Distance(stroke.points.back(), stroke.points.front());
    }
  }
  return total;
}

bool BoundingBox(const StrokeMap & map, Point2 & min_out, Point2 & max_out)
{
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  bool found = false;

  for (const auto & stroke : map.strokes) {
    for (const auto & point : stroke.points) {
      min_x = std::min(min_x, point.x_mm);
      min_y = std::min(min_y, point.y_mm);
      max_x = std::max(max_x, point.x_mm);
      max_y = std::max(max_y, point.y_mm);
      found = true;
    }
  }

  if (!found) {
    return false;
  }
  min_out = Point2{min_x, min_y};
  max_out = Point2{max_x, max_y};
  return true;
}

}  // namespace atlier::vision
