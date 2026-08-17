#!/usr/bin/env python3
"""base_link -> LINK 변환을 **전정밀도**로 찍는다. tf2_echo 는 소수 3자리라 도달오차(mm 이하)를 못 잰다.
사용: tf_read.py [LINK] [FRAME]"""
import sys, math
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from rclpy.duration import Duration

LINK = sys.argv[1] if len(sys.argv) > 1 else "link6_1"
FRAME = sys.argv[2] if len(sys.argv) > 2 else "base_link"

rclpy.init()
n = Node("tf_read")
buf = Buffer()
TransformListener(buf, n)

t = None
for _ in range(200):
    rclpy.spin_once(n, timeout_sec=0.05)
    try:
        t = buf.lookup_transform(FRAME, LINK, rclpy.time.Time(),
                                 timeout=Duration(seconds=0.1))
        break
    except Exception:
        continue

if t is None:
    print("LOOKUP_FAIL"); sys.exit(2)

tr, q = t.transform.translation, t.transform.rotation
# 회전행렬 — 링크 축이 base 에서 어디를 보는지 (열이 축이다)
x, y, z, w = q.x, q.y, q.z, q.w
R = [[1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
     [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
     [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]]

print(f"POS {tr.x:.9f} {tr.y:.9f} {tr.z:.9f}")
print(f"QUAT {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}")
print(f"AXIS_X {R[0][0]:+.6f} {R[1][0]:+.6f} {R[2][0]:+.6f}")
print(f"AXIS_Y {R[0][1]:+.6f} {R[1][1]:+.6f} {R[2][1]:+.6f}")
print(f"AXIS_Z {R[0][2]:+.6f} {R[1][2]:+.6f} {R[2][2]:+.6f}")
rclpy.shutdown()
