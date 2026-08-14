/*
 * contour_pixel_node.cpp
 *
 * 실행 위치: 비전 컨테이너 (OpenCV만 있으면 됨, MoveIt/json 필요 없음)
 *
 * 역할:
 *   /vision/parts (vision_interfaces/MaskImage 커스텀 메시지) 를 구독해서
 *   컨투어+approxPolyDP로 픽셀 좌표를 뽑아 /vision/strokes 로 재발행한다.
 *
 * 이번 버전 변경점:
 *   - 입력(/vision/parts)도 출력(/vision/strokes)도 둘 다 커스텀 메시지가 되면서
 *     nlohmann_json, base64 디코더가 전부 필요 없어졌다.
 *   - mask_data(uint8[])를 cv::Mat으로 감싸기만 하면 되므로 PNG 디코딩도 불필요.
 *
 * label별 근사 정밀도 분기:
 *   "cat"(고양이 전체 윤곽)은 다른 부위보다 큰 영역이라 더 촘촘한 epsilon(0.0025)을
 *   쓰고, 나머지 부위는 기본값(0.01)을 쓴다.
 *
 * 의존성: rclcpp, vision_interfaces, OpenCV   (nlohmann_json 없음!)
 */

#include <rclcpp/rclcpp.hpp>
#include <vision_interfaces/msg/mask_image.hpp>
#include <vision_interfaces/msg/stroke.hpp>
#include <vision_interfaces/msg/pixel_point.hpp>

#include <opencv2/opencv.hpp>

#include <string>
#include <vector>
#include <algorithm>

class ContourPixelNode : public rclcpp::Node
{
public:
    ContourPixelNode() : Node("contour_pixel_node")
    {
        rclcpp::QoS qos(20);
        qos.reliable();
        qos.transient_local();

        sub_ = this->create_subscription<vision_interfaces::msg::MaskImage>(
            "/vision/parts", qos,
            std::bind(&ContourPixelNode::on_part, this, std::placeholders::_1));

        pub_ = this->create_publisher<vision_interfaces::msg::Stroke>("/vision/strokes", qos);

        default_epsilon_ratio_ = this->declare_parameter<double>("epsilon_ratio", 0.01);
        cat_epsilon_ratio_ = this->declare_parameter<double>("cat_epsilon_ratio", 0.0025);

        RCLCPP_INFO(this->get_logger(),
            "contour_pixel_node 시작. /vision/parts(MaskImage) 구독 -> /vision/strokes(Stroke) 발행 "
            "(기본 epsilon_ratio=%.4f, cat 전용=%.4f)",
            default_epsilon_ratio_, cat_epsilon_ratio_);
    }

private:
    void on_part(const vision_interfaces::msg::MaskImage::SharedPtr msg)
    {
        const std::string & label = msg->instance_label;

        if (msg->mask_data.empty() || msg->image_width == 0 || msg->image_height == 0) {
            RCLCPP_WARN(this->get_logger(), "[%s] 빈 마스크 메시지", label.c_str());
            return;
        }

        const size_t expected_size =
            static_cast<size_t>(msg->image_width) * static_cast<size_t>(msg->image_height);
        if (msg->mask_data.size() != expected_size) {
            RCLCPP_ERROR(this->get_logger(),
                "[%s] mask_data 크기 불일치: 받은 %zu, 기대 %zu (width*height)",
                label.c_str(), msg->mask_data.size(), expected_size);
            return;
        }

        // ---- uint8[] 원본 바이트를 cv::Mat으로 감싸기 (PNG 디코딩 불필요) ----
        // 메시지 버퍼를 그대로 참조만 하면 findContours가 내부에서 수정할 수 있어
        // clone()으로 복사본을 만들어 안전하게 처리한다.
        cv::Mat mask(msg->image_height, msg->image_width, CV_8UC1,
                     const_cast<unsigned char*>(msg->mask_data.data()));
        mask = mask.clone();

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (contours.empty()) {
            RCLCPP_WARN(this->get_logger(), "[%s] 컨투어 없음", label.c_str());
            return;
        }

        auto largest = std::max_element(contours.begin(), contours.end(),
            [](const auto & a, const auto & b) { return cv::contourArea(a) < cv::contourArea(b); });

        // ---- label 분기: cat일 때는 더 촘촘한 epsilon_ratio 사용 ----
        double current_epsilon_ratio = default_epsilon_ratio_;  // 기본값 (0.01)
        if (label == "cat") {
            current_epsilon_ratio = cat_epsilon_ratio_;          // 0.0025
        }

        double epsilon = current_epsilon_ratio * cv::arcLength(*largest, true);
        std::vector<cv::Point> approx;
        cv::approxPolyDP(*largest, approx, epsilon, true);

        vision_interfaces::msg::Stroke stroke_msg;
        stroke_msg.instance_label = label;
        stroke_msg.image_width = msg->image_width;
        stroke_msg.image_height = msg->image_height;
        stroke_msg.points.reserve(approx.size());
        for (const auto & p : approx) {
            vision_interfaces::msg::PixelPoint pt;
            pt.u = p.x;
            pt.v = p.y;
            stroke_msg.points.push_back(pt);
        }

        pub_->publish(stroke_msg);

        RCLCPP_INFO(this->get_logger(),
            "[%s] 픽셀점 %zu개 (epsilon_ratio=%.4f) -> /vision/strokes 발행",
            label.c_str(), approx.size(), current_epsilon_ratio);
    }

    rclcpp::Subscription<vision_interfaces::msg::MaskImage>::SharedPtr sub_;
    rclcpp::Publisher<vision_interfaces::msg::Stroke>::SharedPtr pub_;
    double default_epsilon_ratio_;
    double cat_epsilon_ratio_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ContourPixelNode>());
    rclcpp::shutdown();
    return 0;
}
