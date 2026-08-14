"""
sam3_extract_ros_node.py

역할:
  SAM3로 고양이 사진에서 부위별 마스크를 뽑고(NMS/좌우라벨링 로직),
  각 부위를 커스텀 메시지 vision_interfaces/MaskImage 로 만들어 발행한다.

  이전 버전(JSON+base64+PNG)과 다른 점:
  PNG 인코딩도, base64도, JSON도 전혀 쓰지 않는다. 마스크의 원본 바이트
  (numpy uint8 배열을 그대로 .tobytes())를 커스텀 메시지의 uint8[] 필드에
  담아 보낸다. 그래서 이 노드도, 이걸 받는 C++ 노드도 json 라이브러리가
  필요 없어진다.

토픽: /vision/parts (vision_interfaces/msg/MaskImage)
QoS : RELIABLE + TRANSIENT_LOCAL
"""

import numpy as np
import cv2
import rclpy
import os
import sys  # 수정됨: 키 입력 감지를 위해 추가
import select  # 수정됨: 비차단 키 입력 감지를 위해 추가
import tty  # 수정됨: 터미널 모드 변경을 위해 추가
import termios  # 수정됨: 터미널 모드 변경을 위해 추가
from pathlib import Path
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from vision_interfaces.msg import MaskImage
from ultralytics.models.sam import SAM3SemanticPredictor



CURRENT_DIR = Path(__file__).resolve().parent

# ============================================================
# 설정 (기존과 동일)
# ============================================================
PROMPTS = ["cat", "eye of cat", "nose of cat", "mouth of cat"]

EXPECTED_COUNTS = {
    "cat": 1,
    "eye of cat": 2,
    "nose of cat": 1,
    "mouth of cat": 1,
}

NMS_IOU_THRESHOLD = 0.5

SAM3_OVERRIDES = {
    "conf": 0.25,
    "task": "segment",
    "mode": "predict",
    "model": str(CURRENT_DIR / "sam3.pt"),
    "quantize": 32,
    "save": False,
    "retina_masks": True,
}


# ============================================================
# SAM3 추론 + 후처리 (기존 로직 그대로, 변경 없음)
# ============================================================
def run_sam3(image_path, prompts):
    predictor = SAM3SemanticPredictor(overrides=SAM3_OVERRIDES)
    predictor.set_image(image_path)
    if not hasattr(predictor.model, "mask_threshold"):
        predictor.model.mask_threshold = 0.5
    results = predictor(text=prompts)
    return results, predictor.model.names


def collect_detections(results, names, img_shape):
    detections = []
    for r in results:
        if r.masks is None:
            continue
        masks = r.masks.data.cpu().numpy()
        cls_ids = r.boxes.cls.cpu().numpy().astype(int)
        confs = r.boxes.conf.cpu().numpy()

        for mask, cls_id, conf in zip(masks, cls_ids, confs):
            class_name = names[cls_id]
            mask_uint8 = (mask * 255).astype(np.uint8)
            if mask_uint8.shape[:2] != img_shape[:2]:
                mask_uint8 = cv2.resize(
                    mask_uint8, (img_shape[1], img_shape[0]),
                    interpolation=cv2.INTER_LINEAR,
                )
                _, mask_uint8 = cv2.threshold(mask_uint8, 127, 255, cv2.THRESH_BINARY)

            ys, xs = np.where(mask_uint8 > 0)
            if len(xs) == 0:
                continue
            centroid = (float(xs.mean()), float(ys.mean()))

            detections.append({
                "class_name": class_name,
                "conf": float(conf),
                "mask": mask_uint8,
                "centroid": centroid,
            })
    return detections


def mask_iou(mask_a, mask_b):
    inter = np.logical_and(mask_a > 0, mask_b > 0).sum()
    union = np.logical_or(mask_a > 0, mask_b > 0).sum()
    return inter / union if union else 0.0


def nms_same_class(dets, iou_threshold):
    dets_sorted = sorted(dets, key=lambda d: d["conf"], reverse=True)
    kept = []
    for cand in dets_sorted:
        if not any(mask_iou(cand["mask"], k["mask"]) > iou_threshold for k in kept):
            kept.append(cand)
    return kept


def apply_expected_count(dets, class_name, expected_counts):
    limit = expected_counts.get(class_name)
    if limit is None:
        return dets
    return sorted(dets, key=lambda d: d["conf"], reverse=True)[:limit]


def label_left_right(dets):
    if len(dets) != 2:
        for i, d in enumerate(dets):
            d["instance_label"] = f"{d['class_name']}_{i}"
        return dets
    dets_sorted = sorted(dets, key=lambda d: d["centroid"][0])
    dets_sorted[0]["instance_label"] = f"{dets_sorted[0]['class_name']}_left"
    dets_sorted[1]["instance_label"] = f"{dets_sorted[1]['class_name']}_right"
    return dets_sorted


def finalize_labels(dets):
    if len(dets) == 1:
        dets[0]["instance_label"] = dets[0]["class_name"]
    return dets


def build_final_detections(image_path):
    orig_img = cv2.imread(image_path)
    if orig_img is None:
        raise FileNotFoundError(f"이미지를 읽을 수 없습니다: {image_path}")

    results, names = run_sam3(image_path, PROMPTS)
    all_dets = collect_detections(results, names, orig_img.shape)

    by_class = {}
    for d in all_dets:
        by_class.setdefault(d["class_name"], []).append(d)

    final_dets = []
    for class_name, dets in by_class.items():
        dets = nms_same_class(dets, NMS_IOU_THRESHOLD)
        dets = apply_expected_count(dets, class_name, EXPECTED_COUNTS)
        expected = EXPECTED_COUNTS.get(class_name)
        dets = label_left_right(dets) if expected == 2 else finalize_labels(dets)
        final_dets.extend(dets)

    return final_dets, orig_img.shape


# ============================================================
# ROS2 노드
# ============================================================
class Sam3ExtractNode(Node):
    def __init__(self, image_path):
        super().__init__("sam3_extract_node")

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(MaskImage, "/vision/parts", qos)
        self.image_path = image_path

    def process_and_publish(self):
        final_dets, img_shape = build_final_detections(self.image_path)
        img_h, img_w = img_shape[:2]

        self.get_logger().info(f"{len(final_dets)}개 부위 검출, publish 시작")

        for d in final_dets:
            msg = MaskImage()
            msg.instance_label = d["instance_label"]
            msg.class_name = d["class_name"]
            msg.confidence = float(d["conf"])
            msg.image_width = img_w
            msg.image_height = img_h
            # numpy uint8 배열을 그대로 bytes로: PNG/base64/JSON 전혀 없음
            msg.mask_data = d["mask"].tobytes()

            self.publisher.publish(msg)
            self.get_logger().info(f"  publish: {d['instance_label']} (conf={d['conf']:.3f})")


# ============================================================
# 수정됨: 키 입력 관련 헬퍼 함수 추가
# ============================================================
def is_key_pressed():
    """비차단(Non-blocking) 방식으로 키 입력이 있었는지 확인합니다."""
    return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])


def main():
    rclpy.init()

    image_path = str(CURRENT_DIR / "cat_char.png")
    print(image_path)
    node = Sam3ExtractNode(image_path)
    node.process_and_publish()

    # ============================================================
    # 수정됨: 터미널 설정을 변경하여 'q' 입력 대기 로직 추가
    # ============================================================
    node.get_logger().info("발행 완료. 'q' 키를 누르면 노드가 종료됩니다...")

    # 현재 터미널 설정 저장
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        # 터미널을 raw 모드로 변경 (엔터 없이 키 입력 인식)
        tty.setcbreak(sys.stdin.fileno())

        # 무한 루프 돌며 'spin_once'와 '키 입력 확인' 병행
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)

            # 키 입력이 있었는지 확인
            if is_key_pressed():
                key = sys.stdin.read(1)
                if key == 'q':
                    node.get_logger().info("'q' 키 입력 감지. 노드를 종료합니다.")
                    break
    except Exception as e:
        node.get_logger().error(f"루프 중 오류 발생: {e}")
    finally:
        # 반드시 터미널 설정을 원복해야 함
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()