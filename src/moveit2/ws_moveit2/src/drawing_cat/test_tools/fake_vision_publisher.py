#!/usr/bin/env python3
"""vision 을 흉내내어 `/vision/strokes` 를 발행한다 — **연결 확인 전용**.

⚠️ **실제 vision 이 붙으면 이 폴더째 지운다.** 제품 코드가 아니다.
   지우는 법은 test_tools/README.md 참조.

vision 담당자가 공유한 인터페이스 문서 스펙을 그대로 따른다:
  · 부위 하나 = 메시지 하나. 짧은 간격으로 연달아 발행
  · QoS: RELIABLE + TRANSIENT_LOCAL + depth 20
  · 좌표는 이미지 픽셀 (좌상단 원점, v 는 아래로 증가)
  · 닫힌 윤곽이지만 첫 점을 끝에 중복해 넣지 않는다

쓰는 법:

    # 기본 — 고양이 한 장(부위 5 개)을 한 번 발행하고 노드는 살아 있는다
    ros2 run drawing_cat fake_vision_publisher

    # 프레임 경계 판정을 시험한다 — 이미지 3 장을 3 초 간격으로
    ros2 run drawing_cat fake_vision_publisher --ros-args -p frames:=3

    # 발행자를 VOLATILE 로 (늦게 구독하면 못 받는 상황 재현)
    ros2 run drawing_cat fake_vision_publisher --ros-args -p latched:=false
"""
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile,
                       QoSReliabilityPolicy)

from vision_interfaces.msg import PixelPoint, Stroke


def ring(cx, cy, r, n, ears=False):
    """중심 (cx, cy) 둘레를 n 등분한 폐곡선. ears=True 면 위쪽에 귀 두 개가 솟는다."""
    peaks = (math.radians(125.0), math.radians(55.0))
    half = math.radians(18.0)
    pts = []
    for i in range(n):
        th = 2.0 * math.pi * i / n
        rr = r
        if ears:
            for peak in peaks:
                d = abs((th - peak + math.pi) % (2.0 * math.pi) - math.pi)
                if d < half:
                    rr += r * 0.55 * (1.0 - d / half)
        # v 는 아래로 증가하므로 y 를 뒤집는다 (이미지 좌표계)
        pts.append((cx + rr * math.cos(th), cy - rr * math.sin(th)))
    return pts


def cat_parts(w, h):
    """이미지 크기에 맞춰 고양이 부위를 만든다. 라벨은 vision 문서의 예시 형식."""
    cx, cy = w * 0.5, h * 0.5
    r = min(w, h) * 0.3125
    return [
        ('cat',              ring(cx, cy, r, 48, ears=True)),
        ('eye of cat_left',  ring(cx - r * 0.33, cy - r * 0.17, r * 0.15, 14)),
        ('eye of cat_right', ring(cx + r * 0.33, cy - r * 0.17, r * 0.15, 14)),
        ('nose of cat',      ring(cx, cy + r * 0.13, r * 0.10, 12)),
        ('mouth of cat',     ring(cx, cy + r * 0.40, r * 0.23, 16)),
    ]


def main():
    rclpy.init()
    node = Node('fake_vision_publisher')

    topic     = node.declare_parameter('topic', '/vision/strokes').value
    width     = node.declare_parameter('image_width', 1280).value
    height    = node.declare_parameter('image_height', 960).value
    frames    = node.declare_parameter('frames', 1).value
    part_gap  = node.declare_parameter('part_interval_s', 0.15).value
    frame_gap = node.declare_parameter('frame_interval_s', 3.0).value
    latched   = node.declare_parameter('latched', True).value
    startup   = node.declare_parameter('startup_delay_s', 2.0).value

    qos = QoSProfile(
        depth=20,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=(QoSDurabilityPolicy.TRANSIENT_LOCAL if latched
                    else QoSDurabilityPolicy.VOLATILE),
    )
    pub = node.create_publisher(Stroke, topic, qos)
    log = node.get_logger()
    log.info(f"'{topic}' 발행 준비 | 이미지 {width}x{height} | "
             f"durability={'TRANSIENT_LOCAL' if latched else 'VOLATILE'}")

    # 구독자가 디스커버리를 마칠 시간을 준다.
    # ⚠️ VOLATILE 구독자는 이 시점 이전에 발행된 것을 받지 못한다.
    if startup > 0.0:
        log.info(f'구독자 대기 {startup:.1f}초...')
        time.sleep(startup)

    parts = cat_parts(width, height)
    for f in range(max(1, frames)):
        if not rclpy.ok():
            break
        if f > 0:
            log.info(f'--- {frame_gap:.1f}초 후 다음 이미지 ---')
            time.sleep(frame_gap)
        for label, pts in parts:
            msg = Stroke()
            msg.instance_label = label
            msg.image_width = width
            msg.image_height = height
            msg.points = [PixelPoint(u=int(round(x)), v=int(round(y))) for x, y in pts]
            pub.publish(msg)
            log.info(f'  발행 [{f + 1}/{frames}] {label} ({len(msg.points)}점)')
            time.sleep(part_gap)

    log.info('발행 완료 — 노드는 살아 있는다 (TRANSIENT_LOCAL 확인용). Ctrl-C 로 종료')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
