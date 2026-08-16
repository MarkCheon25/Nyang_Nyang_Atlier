#include "vision_stylize/prompt_preset.hpp"

#include <algorithm>
#include <fstream>
#include <sstream>

#include "vision_stylize/types.hpp"

namespace vision_stylize
{

std::string load_prompt(const std::filesystem::path & dir, const std::string & name)
{
  const std::filesystem::path path = dir / (name + ".txt");

  std::error_code ec;
  if (!std::filesystem::exists(path, ec)) {
    std::ostringstream os;
    os << "프롬프트 파일이 없다: " << path.string();
    const auto available = list_presets(dir);
    if (!available.empty()) {
      os << " (있는 것: ";
      for (std::size_t i = 0; i < available.size(); ++i) {
        os << (i ? ", " : "") << available[i];
      }
      os << ")";
    }
    throw PreflightError(os.str());
  }

  std::ifstream in(path);
  if (!in) {
    throw PreflightError("프롬프트 파일을 열 수 없다: " + path.string());
  }

  std::ostringstream buffer;
  buffer << in.rdbuf();
  std::string text = buffer.str();

  // 끝의 개행은 떼어낸다 — 요청 해시가 편집기 습관에 흔들리지 않게.
  while (!text.empty() && (text.back() == '\n' || text.back() == '\r')) {
    text.pop_back();
  }
  if (text.empty()) {
    throw PreflightError("프롬프트 파일이 비어 있다: " + path.string());
  }
  return text;
}

std::vector<std::string> list_presets(const std::filesystem::path & dir)
{
  std::vector<std::string> names;

  std::error_code ec;
  if (!std::filesystem::is_directory(dir, ec)) {
    return names;
  }
  for (const auto & entry : std::filesystem::directory_iterator(dir, ec)) {
    if (!entry.is_regular_file()) {continue;}
    if (entry.path().extension() != ".txt") {continue;}
    names.push_back(entry.path().stem().string());
  }
  std::sort(names.begin(), names.end());
  return names;
}

}  // namespace vision_stylize
