// stroke_map_cli — 이미지 한 장을 지도로 바꾸고 결과를 눈으로 확인한다.
//
// 결과 확인이 이 도구의 존재 이유다. 숫자(스트로크·점·선길이)는 stdout 으로,
// 그림은 SVG·PNG 로, 좌표는 CSV 로 떨어뜨린다. 출력 디렉터리는 호스트에
// 마운트되어 있으므로 컨테이너 밖에서 바로 열어 보면 된다.
#include <opencv2/imgcodecs.hpp>

#include <cstdlib>
#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include "atlier/vision/debug_dump.hpp"
#include "atlier/vision/image_to_map.hpp"
#include "atlier/vision/params.hpp"
#include "atlier/vision/render_svg.hpp"
#include "atlier/vision/stroke_map.hpp"

namespace
{

void PrintUsage(const char * program)
{
  std::cout
    << "사용법: " << program << " [옵션]\n"
    << "\n"
    << "  --in <경로>            입력 이미지 (--dummy 가 아니면 필수)\n"
    << "  --dummy                이미지 없이 더미 지도를 만든다 (하네스 검증용)\n"
    << "  --out-dir <경로>       출력 디렉터리 (기본: out)\n"
    << "  --dump-stages          단계별 PNG 를 함께 저장한다\n"
    << "  --show-points          SVG 에 점을 찍는다 (재샘플링 간격 확인)\n"
    << "\n"
    << "  --paper <W>x<H>        용지 크기 mm (기본: 210x297)\n"
    << "  --margin <mm>          사방 여백 (기본: 15)\n"
    << "  --resample-step <mm>   재샘플링 간격 (기본: 1.0)\n"
    << "  --min-stroke-len <mm>  이보다 짧은 스트로크는 버린다 (기본: 2.0)\n"
    << "  --invert               반전된 입력(검은 배경)일 때\n"
    << "  --no-thinning          centerline 추출을 건너뛴다\n"
    << "  --dpi <n>              04_paper.png 해상도 (기본: 300)\n"
    << "  -h, --help             이 도움말\n";
}

/// 다음 인자를 값으로 꺼낸다. 없으면 사용법을 알리고 종료한다.
std::string TakeValue(int argc, char ** argv, int & i, const std::string & option)
{
  if (i + 1 >= argc) {
    throw std::runtime_error(option + " 에 값이 필요하다");
  }
  return argv[++i];
}

void ParsePaper(const std::string & text, atlier::vision::Params & params)
{
  const std::size_t separator = text.find('x');
  if (separator == std::string::npos) {
    throw std::runtime_error("--paper 형식은 <W>x<H> 다 (예: 210x297)");
  }
  params.paper_w_mm = std::stod(text.substr(0, separator));
  params.paper_h_mm = std::stod(text.substr(separator + 1));
}

}  // namespace

int main(int argc, char ** argv)
{
  using namespace atlier::vision;

  Params params;
  DumpOptions dump;
  std::string input_path;
  bool use_dummy = false;
  bool dump_stages = false;
  bool show_points = false;

  try {
    for (int i = 1; i < argc; ++i) {
      const std::string arg = argv[i];
      if (arg == "-h" || arg == "--help") {
        PrintUsage(argv[0]);
        return 0;
      } else if (arg == "--in") {
        input_path = TakeValue(argc, argv, i, arg);
      } else if (arg == "--dummy") {
        use_dummy = true;
      } else if (arg == "--out-dir") {
        dump.out_dir = TakeValue(argc, argv, i, arg);
      } else if (arg == "--dump-stages") {
        dump_stages = true;
      } else if (arg == "--show-points") {
        show_points = true;
      } else if (arg == "--paper") {
        ParsePaper(TakeValue(argc, argv, i, arg), params);
      } else if (arg == "--margin") {
        params.margin_mm = std::stod(TakeValue(argc, argv, i, arg));
      } else if (arg == "--resample-step") {
        params.resample_step_mm = std::stod(TakeValue(argc, argv, i, arg));
      } else if (arg == "--min-stroke-len") {
        params.min_stroke_len_mm = std::stod(TakeValue(argc, argv, i, arg));
      } else if (arg == "--invert") {
        params.invert = true;
      } else if (arg == "--no-thinning") {
        params.use_thinning = false;
      } else if (arg == "--dpi") {
        dump.render_dpi = std::stoi(TakeValue(argc, argv, i, arg));
      } else {
        throw std::runtime_error("알 수 없는 옵션: " + arg);
      }
    }

    if (!use_dummy && input_path.empty()) {
      PrintUsage(argv[0]);
      return 2;
    }

    if (!EnsureOutDir(dump)) {
      throw std::runtime_error("출력 디렉터리를 만들지 못했다: " + dump.out_dir);
    }

    StrokeMap map;
    if (use_dummy) {
      std::cout << "[vision] 입력       : (더미 지도 — 이미지를 읽지 않음)\n";
      map = MakeDummyMap(params);
    } else {
      const cv::Mat image = cv::imread(input_path, cv::IMREAD_COLOR);
      if (image.empty()) {
        throw std::runtime_error("이미지를 읽지 못했다: " + input_path);
      }
      std::cout << "[vision] 입력       : " << input_path << " (" << image.cols << "×"
                << image.rows << ")\n";

      if (dump_stages) {
        // 단계별로 직접 불러 중간 결과를 떨어뜨린다. BuildStrokeMap 과 같은 순서다.
        const cv::Mat binary = Preprocess(image, params);
        const std::vector<ContourPx> contours = ExtractContours(binary, params);
        const std::vector<PolylinePx> polylines = ToPolylines(contours, params);
        DumpPreprocess(binary, dump);
        DumpContours(image, contours, dump);
        DumpPolylines(image, polylines, dump);
        map = ScaleToPaper(polylines, image.size(), params);
      } else {
        map = BuildStrokeMap(image, params);
      }
    }

    // ── 통계 ──────────────────────────────────────────────────────────────
    std::cout << "[vision] 파라미터   : paper=" << params.paper_w_mm << "×" << params.paper_h_mm
              << "mm margin=" << params.margin_mm << "mm resample=" << params.resample_step_mm
              << "mm\n"
              << "[vision] 스트로크   : " << map.strokes.size() << " 개\n"
              << "[vision] 점         : " << TotalPointCount(map) << " 개\n"
              << "[vision] 총 선길이  : " << TotalDrawLengthMm(map) << " mm\n";

    Point2 min_corner;
    Point2 max_corner;
    if (BoundingBox(map, min_corner, max_corner)) {
      std::cout << "[vision] 바운딩박스 : (" << min_corner.x_mm << ", " << min_corner.y_mm
                << ") ~ (" << max_corner.x_mm << ", " << max_corner.y_mm << ") mm\n";
      const bool inside = min_corner.x_mm >= params.margin_mm - 1e-6 &&
                          min_corner.y_mm >= params.margin_mm - 1e-6 &&
                          max_corner.x_mm <= params.paper_w_mm - params.margin_mm + 1e-6 &&
                          max_corner.y_mm <= params.paper_h_mm - params.margin_mm + 1e-6;
      std::cout << "[vision] 여백 검사  : " << (inside ? "통과" : "⚠️  작화영역을 벗어남")
                << "\n";
    }

    // ── 결과물 ────────────────────────────────────────────────────────────
    SvgStyle style;
    style.show_points = show_points;
    style.margin_mm = params.margin_mm;

    const std::string svg_path = dump.out_dir + "/map.svg";
    const std::string csv_path = dump.out_dir + "/map.csv";
    if (!SaveSvg(map, svg_path, style)) {
      throw std::runtime_error("SVG 를 쓰지 못했다: " + svg_path);
    }
    if (!SaveCsv(map, csv_path)) {
      throw std::runtime_error("CSV 를 쓰지 못했다: " + csv_path);
    }
    DumpPaper(map, params, dump);

    std::cout << "[vision] 저장       : " << svg_path << " · " << csv_path << " · "
              << dump.out_dir << "/04_paper.png\n";

    if (map.strokes.empty()) {
      std::cout << "\n⚠️  스트로크가 0 개입니다 — F2.1~F2.3 이 아직 TODO 라 정상입니다.\n"
                << "    좌표 규약·렌더·CSV 하네스를 검증하려면 --dummy 를 쓰세요.\n";
    }
  } catch (const std::exception & error) {
    std::cerr << "[vision] 오류: " << error.what() << "\n";
    return 1;
  }

  return 0;
}
