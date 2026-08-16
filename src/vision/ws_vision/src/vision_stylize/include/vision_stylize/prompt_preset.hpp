// prompt_preset.hpp — 프롬프트 불러오기
//
// 프롬프트를 소스에 문자열로 박지 않고 prompts/ 안의 텍스트 파일로 두고 읽는다.
// 프롬프트는 실험으로만 정해지는데, 코드에 있으면 한 글자 고칠 때마다 다시 빌드해야
// 한다. 파일이면 빌드가 없고 git 에서 프롬프트 변화만 따로 읽힌다.
//
// 설치 경로를 찾는 일은 **명령줄 프로그램 쪽에서 한다.** 라이브러리는 경로를 받기만
// 한다 — 설치 경로 조회에는 ament_index 가 필요한데 그걸 라이브러리에 넣으면
// "ROS 없이도 빌드된다" 가 깨진다.

#ifndef VISION_STYLIZE__PROMPT_PRESET_HPP_
#define VISION_STYLIZE__PROMPT_PRESET_HPP_

#include <filesystem>
#include <string>
#include <vector>

namespace vision_stylize
{

/// dir/<name>.txt 를 읽어 문자열로. 없으면 PreflightError.
std::string load_prompt(const std::filesystem::path & dir, const std::string & name);

/// dir 안의 *.txt 프리셋 이름들 (확장자 뺀 이름, 사전순).
std::vector<std::string> list_presets(const std::filesystem::path & dir);

}  // namespace vision_stylize

#endif  // VISION_STYLIZE__PROMPT_PRESET_HPP_
