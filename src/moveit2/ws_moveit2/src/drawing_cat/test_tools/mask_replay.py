#!/usr/bin/env python3
"""SAM3 대신 `/vision/parts` 를 발행한다 — 비전팀이 올린 마스크 PNG 를 그대로 재생.

⚠️ **확인 전용이다. 제품 코드가 아니다.** 지우는 법은 test_tools/README.md.

## 왜 필요한가

비전 파이프라인은 두 토막이다.

    [SAM3 파이썬]  →  /vision/parts  →  [contour_pixel C++]  →  /vision/strokes
     torch + 3GB 모델                    OpenCV 만

앞 토막이 무겁다(모델 3.3GB, pip 설치 5~10분). 그런데 비전팀이 SAM3 **출력물**
(`sam3_output/` 의 마스크 PNG 5장 + metadata.json)을 같이 올려 두었으므로, 그것을
그대로 발행하면 앞 토막을 대신할 수 있다.

## ⚠️ moveit2 개발 PC 에서는 SAM3 자체를 돌릴 수 없다 (2026-08-16 확인)

편의가 아니라 **제약**이다. 실제로 설치까지 해 보고 막힌 지점 셋:

    ① GPU 커널 없음   torch 2.13.0+cu130 지원 = sm_75 이상
                      GTX 1050 = sm_61 (Pascal). CUDA 13 이 이 세대를 잘라냈다
                      → GPU 를 넘겨줘도 CPU 로 떨어진다
    ② VRAM 부족       모델 3.3GB vs VRAM 2.0GB
                      ① 을 우회하려 구버전 torch 로 낮춰도 여기서 다시 막힌다
    ③ RAM 부족        총 7.5GB 중 가용 4.2GB. CPU 추론이 스왑으로 넘어가
                      멈춘 것처럼 보인다 (실제로 2회 발생)

①②는 이 PC 에서 고칠 수 없다. **따라서 이 노드는 임시 대역이 아니라 우리 쪽의
상시 상류다.** 실물 SAM3 검증은 비전팀 몫이다.

## 그래도 하류 검증이 유효한 이유

`/vision/parts` 발행자의 계약은 **전부 밖에서 관찰 가능한 것**뿐이다. 아래를 맞추면
하류(`contour_pixel_node` → `draw_cat`)는 재생인지 진짜인지 구분할 수 없다.

    토픽/타입   /vision/parts · vision_interfaces/msg/MaskImage
    QoS         RELIABLE + TRANSIENT_LOCAL, depth 20
    발행 방식   추론이 끝난 뒤 전 부위를 **sleep 없이 몰아서**
    노드 수명   발행 후에도 살아 있음 (SAM3 는 'q' 입력까지)
    필드        label · class_name · confidence · w/h · mono8 raw bytes

이 값들은 `sam3_extract_ros_node.py` 를 읽어 그대로 맞췄다 (2026-08-15).

**2026-08-16 에 이것으로 끝에서 끝까지 관통했다.** `contour_pixel` 부터는 전부 비전팀
실제 코드이며, 축척·좌표 방향·타입 해시·프레임 경계·전 스트로크 실행이 실물 DDS 에서
검증됐다 — 상세는 `docs/Vision Integration Result.md`.

## 재생으로 덮지 못하는 것 — 비전팀 확인 대상

마스크가 고정이라 아래는 여전히 실물 확인 대상이다. 그중 넷은 아래 파라미터로
**일부러 비틀어서** 미리 눌러볼 수 있다.

| 실물에서 달라지는 것        | 이 노드의 대응            |
|----------------------------|--------------------------|
| 부위 개수 변동              | `drop`                   |
| 라벨 중복                   | `duplicate_label`        |
| 부위 간 발행 간격           | `delay_between_ms`       |
| 프레임이 여러 번 오는 경우   | `repeat`                 |
| **마스크 품질**(구멍·여러 덩어리) | ❌ 못 함 — 비전팀 몫 |

마스크 품질만 남는데, 그것은 위 ①②③ 때문에 **여기서는 확인할 방법이 없다.**

## 쓰는 법 (비전 컨테이너 안)

    source ~/ws_vision/install/setup.bash
    python3 ~/ws_vision/mask_replay.py

    # 부위 간 3 초 간격 — 받는 쪽 frame_timeout_ms(기본 1500) 를 넘겨 본다
    python3 ~/ws_vision/mask_replay.py --ros-args -p delay_between_ms:=3000

    # 코를 빼고 발행 — 부위 개수가 줄었을 때
    python3 ~/ws_vision/mask_replay.py --ros-args -p drop:="nose of cat"

    # 첫 부위를 같은 라벨로 두 번 — 프레임 경계 판정을 흔든다
    python3 ~/ws_vision/mask_replay.py --ros-args -p duplicate_label:=true
"""
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from vision_interfaces.msg import MaskImage

# 기본 경로 — 비전 컨테이너 기준. sam3_output 은 ws_vision 바로 아래에 있다.
DEFAULT_DIR = str(Path.home() / "ws_vision" / "sam3_output")


def load_mask(path: Path) -> np.ndarray:
    """PNG 를 mono8 (0/255) 2차원 배열로 읽는다.

    cv2 를 먼저 쓰고 없으면 PIL 로 떨어진다 — 컨테이너에 무엇이 있는지 확실하지 않아서다.
    """
    try:
        import cv2
        img = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if img is None:
            raise RuntimeError(f"cv2 가 읽지 못했다: {path}")
        return img
    except ImportError:
        pass
    try:
        from PIL import Image
        return np.array(Image.open(path).convert("L"))
    except ImportError:
        sys.exit(
            "PNG 를 읽을 라이브러리가 없다. 컨테이너에서 하나만 설치할 것:\n"
            "    sudo apt install -y python3-opencv\n"
            "  또는\n"
            "    pip install pillow"
        )


class MaskReplayNode(Node):
    def __init__(self):
        super().__init__("mask_replay_node")

        self.declare_parameter("dir", DEFAULT_DIR)
        self.declare_parameter("delay_between_ms", 0)
        self.declare_parameter("drop", "")
        self.declare_parameter("duplicate_label", False)
        self.declare_parameter("repeat", 1)

        # ⚠️ SAM3 노드와 **똑같은** QoS 여야 한다. 다르면 하류가 다르게 반응해서
        #    재생으로 검증한 의미가 없어진다.
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.pub = self.create_publisher(MaskImage, "/vision/parts", qos)

    def run(self):
        root = Path(self.get_parameter("dir").value)
        meta_path = root / "metadata.json"
        if not meta_path.is_file():
            sys.exit(f"metadata.json 이 없다: {meta_path}\n"
                     f"  --ros-args -p dir:=<sam3_output 경로> 로 지정할 수 있다")

        meta = json.loads(meta_path.read_text())
        img_w, img_h = meta["image_width"], meta["image_height"]
        parts = meta["parts"]

        drop = self.get_parameter("drop").value
        if drop:
            before = len(parts)
            parts = [p for p in parts if p["instance_label"] != drop]
            self.get_logger().warn(f"drop='{drop}' → 부위 {before} → {len(parts)} 개")

        delay_s = self.get_parameter("delay_between_ms").value / 1000.0
        duplicate = self.get_parameter("duplicate_label").value
        repeat = self.get_parameter("repeat").value

        self.get_logger().info(
            f"{meta_path.parent} | 이미지 {img_w}x{img_h} | 부위 {len(parts)} 개"
            + (f" | 부위 간 {delay_s:.1f}s" if delay_s else " | 몰아서 발행(실제 SAM3 와 동일)")
            + (" | 라벨 중복 주입" if duplicate else "")
            + (f" | {repeat} 회 반복" if repeat > 1 else "")
        )

        for r in range(repeat):
            if repeat > 1:
                self.get_logger().info(f"── 프레임 {r + 1}/{repeat} ──")
            for i, part in enumerate(parts):
                self.publish_part(root, part, img_w, img_h)
                # 첫 부위를 같은 라벨로 한 번 더 — 받는 쪽 프레임 경계 판정을 흔든다
                if duplicate and i == 0:
                    self.get_logger().warn(f"  ↳ 같은 라벨로 한 번 더: '{part['instance_label']}'")
                    self.publish_part(root, part, img_w, img_h)
                if delay_s:
                    time.sleep(delay_s)
            if repeat > 1 and r + 1 < repeat:
                time.sleep(max(delay_s, 2.0))

        self.get_logger().info("발행 완료. Ctrl-C 로 종료 "
                               "(TRANSIENT_LOCAL 이라 살아 있어야 늦게 뜬 구독자도 받는다)")

    def publish_part(self, root: Path, part: dict, img_w: int, img_h: int):
        mask = load_mask(root / part["mask_file"])
        if mask.shape != (img_h, img_w):
            self.get_logger().warn(
                f"  '{part['instance_label']}' 마스크 크기 {mask.shape[1]}x{mask.shape[0]} 가 "
                f"metadata 의 {img_w}x{img_h} 와 다르다 — 마스크 크기를 따른다")
        h, w = mask.shape

        msg = MaskImage()
        msg.instance_label = part["instance_label"]
        msg.class_name = part["class_name"]
        msg.confidence = float(part["confidence"])
        msg.image_width = w
        msg.image_height = h
        # SAM3 노드와 같은 형식 — mono8 raw bytes, row-major. PNG/base64/JSON 없음.
        msg.mask_data = np.ascontiguousarray(mask, dtype=np.uint8).tobytes()

        self.pub.publish(msg)
        nz = int(np.count_nonzero(mask))
        self.get_logger().info(
            f"  publish: {msg.instance_label} "
            f"(conf={msg.confidence:.3f}, 흰 픽셀 {nz})")


def main():
    rclpy.init()
    node = MaskReplayNode()
    try:
        node.run()
        rclpy.spin(node)          # 살아 있어야 TRANSIENT_LOCAL 이 의미가 있다
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
