// stylize_cli.cpp — 명령줄 프로그램
//
// 라이브러리들을 순서대로 엮어 실제로 돌린다. 성공 판정을 사람이 눈으로 하므로,
// 이 프로그램의 목적은 **프롬프트 여러 종의 결과를 같은 사진으로 만들어 나란히
// 놓는 것**이다. 숫자는 화면에, 그림은 파일로 나간다.
//
// 검사와 비용 계산을 전부 앞에 몰아둔 것이 요점이다. 프롬프트마다
// "계산 → 동의 → 전송" 을 반복하면 사용자가 여러 번 답해야 하고, 두 번째에서
// 실패하면 첫 번째 돈만 나간 어중간한 상태가 된다.

#include <cstdlib>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// 설치 경로 조회는 **명령줄 프로그램만** 한다. 라이브러리에 넣으면
// "ROS 없이도 빌드·실행된다" 가 깨진다 (llm_readme §2).
#include <ament_index_cpp/get_package_share_directory.hpp>

#include "vision_stylize/api_client.hpp"
#include "vision_stylize/cost_guard.hpp"
#include "vision_stylize/image_io.hpp"
#include "vision_stylize/image_ops.hpp"
#include "vision_stylize/prompt_preset.hpp"
#include "vision_stylize/types.hpp"

namespace fs = std::filesystem;
using namespace vision_stylize;

namespace
{

struct Options
{
  fs::path input;
  fs::path out_dir;
  fs::path reference;
  fs::path prompt_dir;
  std::string preset = "all";
  StylizeParams params;
  PreprocessOptions preprocess;
  bool binarize = true;
  bool dry_run = false;
  bool assume_yes = false;
  bool use_cache = true;
  int max_attempts = 3;
};

void print_usage()
{
  std::cout <<
    R"(사용법:
  stylize_cli --in <사진> --out-dir <경로> [옵션]

필수
  --in <경로>                입력 사진 (PNG · JPEG · WebP, 50MB 미만)
  --out-dir <경로>           산출물이 떨어질 디렉터리

선택
  --ref <경로>               참조 그림. 없으면 프롬프트만으로 진행한다
  --preset <이름|all>        기본 all — prompts/ 안의 모든 프리셋을 돈다
  --quality low|medium|high|auto   기본 low
  --size auto|WxH            기본 auto
  --model <이름>             기본 gpt-image-2
  --max-edge <픽셀>          기본 1024. 입력 토큰의 유일한 손잡이
  --no-grayscale             색을 지우지 않는다
  --no-trim                  가장자리 여백을 잘라내지 않는다
  --fit contain|cover|stretch      기본 contain (--size 가 WxH 일 때만 쓰인다)
  --no-binarize              이진화 참고본을 만들지 않는다
  --prompt-dir <경로>        프롬프트 디렉터리를 직접 지정
  --dry-run                  비용만 계산하고 끝낸다. 아무 것도 보내지 않는다
  --yes                      동의를 묻지 않는다
  --no-cache                 이미 만든 결과가 있어도 다시 보낸다
  --retries <횟수>           기본 3 (429 · 5xx · 전송 오류에만 쓰인다)

API 키는 환경변수 OPENAI_API_KEY 에서만 읽는다.
인자로 받는 경로는 두지 않는다 — ps 와 셸 히스토리에 그대로 남는다.
)";
}

std::string next_value(int argc, char ** argv, int * i, const char * flag)
{
  if (*i + 1 >= argc) {
    throw PreflightError(std::string(flag) + " 에 값이 없다");
  }
  return argv[++(*i)];
}

Options parse_args(int argc, char ** argv)
{
  Options options;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    if (arg == "--in") {
      options.input = next_value(argc, argv, &i, "--in");
    } else if (arg == "--out-dir") {
      options.out_dir = next_value(argc, argv, &i, "--out-dir");
    } else if (arg == "--ref") {
      options.reference = next_value(argc, argv, &i, "--ref");
    } else if (arg == "--preset") {
      options.preset = next_value(argc, argv, &i, "--preset");
    } else if (arg == "--quality") {
      options.params.quality = next_value(argc, argv, &i, "--quality");
    } else if (arg == "--size") {
      options.params.size = next_value(argc, argv, &i, "--size");
    } else if (arg == "--model") {
      options.params.model = next_value(argc, argv, &i, "--model");
    } else if (arg == "--background") {
      options.params.background = next_value(argc, argv, &i, "--background");
    } else if (arg == "--max-edge") {
      options.preprocess.max_edge = std::stoi(next_value(argc, argv, &i, "--max-edge"));
    } else if (arg == "--no-grayscale") {
      options.preprocess.grayscale = false;
    } else if (arg == "--no-trim") {
      options.preprocess.trim = false;
    } else if (arg == "--fit") {
      options.preprocess.fit = parse_fit_mode(next_value(argc, argv, &i, "--fit"));
    } else if (arg == "--no-binarize") {
      options.binarize = false;
    } else if (arg == "--prompt-dir") {
      options.prompt_dir = next_value(argc, argv, &i, "--prompt-dir");
    } else if (arg == "--dry-run") {
      options.dry_run = true;
    } else if (arg == "--yes" || arg == "-y") {
      options.assume_yes = true;
    } else if (arg == "--no-cache") {
      options.use_cache = false;
    } else if (arg == "--retries") {
      options.max_attempts = std::stoi(next_value(argc, argv, &i, "--retries"));
    } else if (arg == "--help" || arg == "-h") {
      print_usage();
      std::exit(0);
    } else {
      throw PreflightError("모르는 인자다: " + arg);
    }
  }

  if (options.input.empty()) {
    throw PreflightError("--in 이 없다");
  }
  if (options.out_dir.empty()) {
    throw PreflightError("--out-dir 이 없다");
  }
  return options;
}

/// 실행 파일이 있는 디렉터리. 설치 경로를 상대로 찾기 위한 기준점이다.
fs::path executable_dir()
{
  std::error_code ec;
  const fs::path exe = fs::read_symlink("/proc/self/exe", ec);
  if (ec) {
    return fs::current_path();
  }
  return exe.parent_path();
}

/// 프롬프트 디렉터리 찾기 — --prompt-dir → STYLIZE_PROMPT_DIR → 실행파일 상대.
fs::path resolve_prompt_dir(const Options & options)
{
  if (!options.prompt_dir.empty()) {
    if (!fs::is_directory(options.prompt_dir)) {
      throw PreflightError("--prompt-dir 이 디렉터리가 아니다: " + options.prompt_dir.string());
    }
    return options.prompt_dir;
  }

  if (const char * from_env = std::getenv("STYLIZE_PROMPT_DIR")) {
    const fs::path dir = from_env;
    if (fs::is_directory(dir)) {
      return dir;
    }
    throw PreflightError(
      "STYLIZE_PROMPT_DIR 이 디렉터리가 아니다: " + std::string(from_env));
  }

  // ament 가 아는 설치 위치. --symlink-install 이면 실행 파일이 build/ 안에 있어서
  // 실행 파일 상대 경로로는 share/ 를 못 찾는다 — 그 경우를 이쪽이 받아낸다.
  try {
    const fs::path share = ament_index_cpp::get_package_share_directory("vision_stylize");
    const fs::path dir = share / "prompts";
    std::error_code ec;
    if (fs::is_directory(dir, ec)) {
      return dir;
    }
  } catch (const std::exception &) {
    // ROS 환경이 source 되지 않았을 뿐이다. 아래 상대 경로로 넘어간다.
  }

  const fs::path exe_dir = executable_dir();
  const std::vector<fs::path> candidates = {
    // 설치 위치 — lib/vision_stylize/stylize_cli 에서 share/vision_stylize/prompts 로
    exe_dir / ".." / ".." / "share" / "vision_stylize" / "prompts",
    exe_dir / ".." / "share" / "vision_stylize" / "prompts",
    exe_dir / "prompts",
    // 소스 트리에서 바로 돌릴 때 (build/vision_stylize/stylize_cli 기준)
    exe_dir / ".." / ".." / "src" / "vision_stylize" / "prompts",
  };
  for (const auto & candidate : candidates) {
    std::error_code ec;
    if (fs::is_directory(candidate, ec)) {
      return fs::weakly_canonical(candidate, ec);
    }
  }

  throw PreflightError(
    "프롬프트 디렉터리를 찾을 수 없다. --prompt-dir 로 알려주거나 "
    "환경변수 STYLIZE_PROMPT_DIR 을 설정할 것");
}

/// 사진만의 해시 — 전처리한 사진 파일 이름에 쓴다. 프롬프트가 섞이면 같은 사진에
/// 대해 프리셋 수만큼 사본이 생긴다.
std::string photo_hash(const ImageBlob & photo, const std::optional<ImageBlob> & reference)
{
  StylizeRequest only_photo;
  only_photo.photo = photo;
  only_photo.reference = reference;
  only_photo.prompt.clear();
  only_photo.params = StylizeParams{};
  return request_hash(only_photo);
}

bool ask_yes_no(const std::string & question)
{
  std::cout << question << " [y/N] " << std::flush;
  std::string answer;
  if (!std::getline(std::cin, answer)) {
    return false;
  }
  return answer == "y" || answer == "Y" || answer == "yes";
}

/// 프롬프트 하나에 대한 계획 한 줄
struct Plan
{
  std::string preset;
  std::string prompt;
  std::string hash;
  fs::path png_path;
  fs::path bw_path;
  fs::path text_path;
  CostEstimate cost;
  bool cached = false;
};

std::string summary_text(
  const Plan & plan, const Options & options, const PreprocessReport & report,
  const fs::path & sent_path, bool has_reference)
{
  std::ostringstream os;
  os << "프리셋   : " << plan.preset << "\n"
     << "모델     : " << options.params.model << "\n"
     << "크기     : " << options.params.size << "\n"
     << "품질     : " << options.params.quality << "\n"
     << "참조 그림: " << (has_reference ? options.reference.string() : "(없음)") << "\n"
     << "전처리   : " << report.summary() << "\n"
     << "보낸 사진: " << sent_path.filename().string() << "\n"
     << "요청 해시: " << plan.hash << "\n"
     << "예상 비용: " << plan.cost.summary() << "   ⚠️ 토큰 계수 미확인 — 자릿수 감각용\n"
     << "\n"
     << "다음 단계(SAM3) 입력으로 쓸 파일: " << plan.png_path.filename().string()
     << "  ← 이진화본이 아니라 이쪽이다\n"
     << "\n"
     << "── 프롬프트 원문 ──\n"
     << plan.prompt << "\n";
  return os.str();
}

}  // namespace

int main(int argc, char ** argv)
{
  try {
    // 1. 인자 해석
    const Options options = parse_args(argc, argv);

    // 2. API 키 확인 — 없으면 여기서 끝. 돈 쓰기 전에 실패한다.
    const char * api_key_env = std::getenv("OPENAI_API_KEY");
    const std::string api_key = api_key_env ? api_key_env : "";
    if (api_key.empty() && !options.dry_run) {
      throw PreflightError(
        "환경변수 OPENAI_API_KEY 가 없다. --dry-run 은 키 없이도 돈다.\n"
        "  export OPENAI_API_KEY=\"$(cat ~/.openai_key)\"");
    }

    // 3. 사진 읽기 (+ 참조 그림)
    const ImageBlob photo = read_image(options.input);
    std::optional<ImageBlob> reference;
    if (!options.reference.empty()) {
      reference = read_image(options.reference);
    }

    // 4. 전처리 → 보낼 바이트 확정 + 기록
    PreprocessOptions preprocess_options = options.preprocess;
    const auto parsed_size = parse_size(options.params.size);
    if (parsed_size) {
      preprocess_options.target_aspect =
        static_cast<double>(parsed_size->width) / static_cast<double>(parsed_size->height);
    }
    PreprocessReport report;
    const ImageBlob sent = preprocess(photo, preprocess_options, &report);

    // 5. 전처리 결과를 파일로 저장 — 결과가 이상할 때 "무엇을 보냈길래" 를 되짚는
    //    유일한 방법이다.
    const std::string input_hash = photo_hash(sent, reference);
    const fs::path sent_path = options.out_dir / ("input_" + input_hash + "_sent.png");
    write_bytes(sent_path, sent.bytes);

    // 6. 출력 크기 규칙 검사 (규칙은 출력에만 걸린다)
    if (parsed_size) {
      validate_output_size(*parsed_size);
    }

    // 7. 쓸 프롬프트 목록 정하기
    const fs::path prompt_dir = resolve_prompt_dir(options);
    std::vector<std::string> presets;
    if (options.preset == "all") {
      presets = list_presets(prompt_dir);
      if (presets.empty()) {
        throw PreflightError("프롬프트 파일이 하나도 없다: " + prompt_dir.string());
      }
    } else {
      presets.push_back(options.preset);
    }

    std::cout << "입력      : " << options.input.string() << "\n"
              << "전처리    : " << report.summary() << "\n"
              << "보낸 사진 : " << sent_path.string() << "\n"
              << "참조 그림 : "
              << (reference ? options.reference.string() : "(없음 — 프롬프트만으로 진행)") << "\n"
              << "프롬프트  : " << prompt_dir.string() << "\n"
              << "프리셋    : " << presets.size() << "종\n\n";

    // 8~11. 프롬프트마다 읽기 · 해시 · 캐시 확인 · 비용 더하기
    std::vector<Plan> plans;
    double total_cost = 0.0;
    int to_send = 0;

    for (const auto & preset : presets) {
      Plan plan;
      plan.preset = preset;
      plan.prompt = load_prompt(prompt_dir, preset);

      StylizeRequest request;
      request.photo = sent;
      request.reference = reference;
      request.prompt = plan.prompt;
      request.params = options.params;

      plan.hash = request_hash(request);

      const std::string stem = preset + "_" + options.params.quality + "_" + plan.hash;
      plan.png_path = options.out_dir / (stem + ".png");
      plan.bw_path = options.out_dir / (stem + "_bw.png");
      plan.text_path = options.out_dir / (stem + ".txt");

      plan.cost = estimate_cost(request);
      plan.cached = options.use_cache && fs::exists(plan.png_path);

      if (!plan.cached) {
        total_cost += plan.cost.usd;
        ++to_send;
      }
      plans.push_back(plan);
    }

    std::cout << std::fixed << std::setprecision(4);
    for (const auto & plan : plans) {
      std::cout << "  " << std::setw(14) << std::left << plan.preset
                << (plan.cached ? "이미 있음 — 건너뜀  " : "보낼 것            ")
                << "$" << plan.cost.usd << "  " << plan.hash << "\n";
    }
    std::cout << "\n보낼 요청 " << to_send << "건 · 예상 총액 $" << total_cost
              << "   ⚠️ 토큰 계수 미확인 — 자릿수 감각이지 청구서가 아니다\n\n";

    // 12. 총액을 한 번에 보여주고 동의 받기 — 미리보기 모드면 여기서 끝
    if (options.dry_run) {
      std::cout << "--dry-run 이라 아무 것도 보내지 않았다.\n";
      return 0;
    }
    if (to_send == 0) {
      std::cout << "보낼 것이 없다 (전부 이미 있다). 다시 보내려면 --no-cache.\n";
      return 0;
    }
    if (!options.assume_yes && !ask_yes_no("보낼까?")) {
      std::cout << "보내지 않았다.\n";
      return 0;
    }

    // 13~16. 프롬프트마다 보내기 · 저장 · 이진화본 · 조건 기록
    //
    // 주소를 환경변수로도 받는다 — 시험할 때 가짜 서버를 물리기 위해서다.
    // 명령줄 옵션으로 두지 않은 것은 실사용에서 잘못 쓸 자리를 늘리지 않기 위해서.
    const char * base_url_env = std::getenv("OPENAI_BASE_URL");
    const ApiClient client(
      api_key,
      base_url_env ? std::string(base_url_env) : std::string("https://api.openai.com"));
    if (base_url_env) {
      std::cout << "⚠️  주소를 바꿔 보낸다 (OPENAI_BASE_URL): " << base_url_env << "\n";
    }
    int sent_count = 0;
    int failed_count = 0;

    for (auto & plan : plans) {
      if (plan.cached) {
        continue;
      }

      StylizeRequest request;
      request.photo = sent;
      request.reference = reference;
      request.prompt = plan.prompt;
      request.params = options.params;

      std::cout << "[" << plan.preset << "] 보내는 중 ..." << std::flush;
      try {
        const auto png = client.edit_image_with_retry(request, options.max_attempts);
        write_bytes(plan.png_path, png);

        if (options.binarize) {
          // ⚠️ 참고용이다. 다음 단계(SAM3) 입력은 위의 원본 PNG.
          write_bytes(plan.bw_path, binarize(png));
        }
        write_text(
          plan.text_path,
          summary_text(plan, options, report, sent_path, reference.has_value()));

        std::cout << " 받았다 (" << png.size() / 1024 << " KB)\n";
        ++sent_count;
      } catch (const ApiError & e) {
        std::cout << " 실패\n";
        std::cerr << "  [" << plan.preset << "] " << e.what()
                  << (e.retryable() ? "  (재시도했으나 계속 실패)" : "  (재시도하지 않았다)")
                  << "\n";
        ++failed_count;
      }
    }

    // 17. 결과 표
    std::cout << "\n── 결과 ──\n";
    for (const auto & plan : plans) {
      const bool made = fs::exists(plan.png_path);
      std::cout << "  " << std::setw(14) << std::left << plan.preset
                << (made ? plan.png_path.filename().string() : std::string("(없음)"));
      if (made && options.binarize && fs::exists(plan.bw_path)) {
        std::cout << "   참고: " << plan.bw_path.filename().string();
      }
      std::cout << "\n";
    }
    std::cout << "\n보냄 " << sent_count << "건 · 실패 " << failed_count << "건\n"
              << "산출물: " << options.out_dir.string() << "\n"
              << "다음 단계(SAM3) 입력은 _bw 가 붙지 않은 원본 PNG 다.\n";

    return failed_count == 0 ? 0 : 1;

  } catch (const PreflightError & e) {
    std::cerr << "보내기 전 실패: " << e.what() << "\n";
    return 2;
  } catch (const ApiError & e) {
    std::cerr << "API 실패: " << e.what() << "\n";
    return 3;
  } catch (const std::exception & e) {
    std::cerr << "오류: " << e.what() << "\n";
    return 4;
  }
}
