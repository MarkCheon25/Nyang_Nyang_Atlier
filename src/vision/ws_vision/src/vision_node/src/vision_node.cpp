// vision_node — vision_core 의 ROS 2 래퍼.
//
// ⚠️ **경계 인터페이스(토픽/서비스/메시지 타입)는 아직 정하지 않았다.**
// 지도를 어떤 형식으로 moveit2 에 넘길지는 moveit2 쪽 착수 시점에 함께 정한다.
// 코어를 ROS 무관 라이브러리로 분리해 둔 이유가 그 결정을 늦출 수 있어서다.
//
// 지금 이 노드가 하는 일은 하나다 — ROS 파라미터로 받은 설정을 Params 로 옮겨
// 코어를 호출하고, 결과 통계를 로그로 남긴다. 파라미터·빌드·링크 경로가 살아
// 있는지 확인하는 용도다.
#include <rclcpp/rclcpp.hpp>

#include <exception>
#include <memory>
#include <string>

#include "atlier/vision/image_to_map.hpp"
#include "atlier/vision/params.hpp"
#include "atlier/vision/render_svg.hpp"
#include "atlier/vision/stroke_map.hpp"

namespace atlier::vision
{

class VisionNode : public rclcpp::Node
{
public:
  VisionNode()
  : Node("vision_node")
  {
    params_.paper_w_mm = declare_parameter("paper_w_mm", params_.paper_w_mm);
    params_.paper_h_mm = declare_parameter("paper_h_mm", params_.paper_h_mm);
    params_.margin_mm = declare_parameter("margin_mm", params_.margin_mm);
    params_.resample_step_mm = declare_parameter("resample_step_mm", params_.resample_step_mm);
    params_.min_stroke_len_mm = declare_parameter("min_stroke_len_mm", params_.min_stroke_len_mm);
    params_.blur_ksize = static_cast<int>(declare_parameter("blur_ksize", params_.blur_ksize));
    params_.invert = declare_parameter("invert", params_.invert);
    params_.use_thinning = declare_parameter("use_thinning", params_.use_thinning);

    image_path_ = declare_parameter("image_path", std::string{});
    svg_path_ = declare_parameter("svg_path", std::string{});
  }

  /// image_path 파라미터가 주어졌으면 한 번 처리한다.
  /// 생성자가 아니라 밖에서 부른다 — 실패 시 노드 생성 자체를 무너뜨리지 않기 위함.
  void ProcessOnce()
  {
    if (image_path_.empty()) {
      RCLCPP_INFO(
        get_logger(),
        "image_path 가 비어 있어 대기합니다. 경계 인터페이스(토픽/서비스)는 아직 "
        "미정이라, 지금은 --ros-args -p image_path:=<경로> 로만 동작합니다.");
      return;
    }

    try {
      const StrokeMap map = BuildStrokeMapFromFile(image_path_, params_);
      RCLCPP_INFO(
        get_logger(), "지도 생성: 스트로크 %zu 개 · 점 %zu 개 · 총 선길이 %.1f mm",
        map.strokes.size(), TotalPointCount(map), TotalDrawLengthMm(map));

      if (map.strokes.empty()) {
        RCLCPP_WARN(
          get_logger(),
          "스트로크가 0 개입니다 — F2.1~F2.3 이 아직 TODO 라 정상입니다. "
          "하네스 검증은 stroke_map_cli --dummy 를 쓰세요.");
      }

      if (!svg_path_.empty()) {
        SvgStyle style;
        style.margin_mm = params_.margin_mm;
        if (SaveSvg(map, svg_path_, style)) {
          RCLCPP_INFO(get_logger(), "SVG 저장: %s", svg_path_.c_str());
        } else {
          RCLCPP_ERROR(get_logger(), "SVG 를 쓰지 못했습니다: %s", svg_path_.c_str());
        }
      }
    } catch (const std::exception & error) {
      RCLCPP_ERROR(get_logger(), "지도 생성 실패: %s", error.what());
    }
  }

private:
  Params params_;
  std::string image_path_;
  std::string svg_path_;
};

}  // namespace atlier::vision

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<atlier::vision::VisionNode>();
  node->ProcessOnce();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
