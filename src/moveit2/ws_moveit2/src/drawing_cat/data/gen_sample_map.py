"""vision 의 map.csv 형식으로 고양이 머리 외곽선(폐곡선) 하나를 만든다.

형식: stroke_idx,point_idx,x_mm,y_mm,closed
A4(210x297) 중앙에 놓고, 귀 두 개가 솟은 원형 윤곽을 균등 간격으로 리샘플링한다.
vision 규약대로 **첫 점을 끝에 중복해 넣지 않는다**.
"""
import math

CX, CY = 105.0, 148.5     # A4 중앙 (mm)
R = 40.0                  # 머리 반지름
EAR_H = 22.0              # 귀 높이
EAR_HALF = math.radians(18)   # 귀 각폭의 절반
N = 96                    # 점 개수

# 귀 두 개의 중심 각도 (위쪽 좌/우)
EARS = [math.radians(125.0), math.radians(55.0)]


def radius_at(theta):
    r = R
    for peak in EARS:
        d = abs((theta - peak + math.pi) % (2 * math.pi) - math.pi)
        if d < EAR_HALF:
            # 삼각형 형태로 솟았다가 내려온다
            r += EAR_H * (1.0 - d / EAR_HALF)
    return r


rows = []
for i in range(N):
    th = 2.0 * math.pi * i / N
    r = radius_at(th)
    x = CX + r * math.cos(th)
    y = CY - r * math.sin(th)   # 화면 좌표계(아래로 +y)에 맞춰 뒤집는다
    rows.append((0, i, x, y, 1))

with open("map.csv", "w") as f:
    f.write("stroke_idx,point_idx,x_mm,y_mm,closed\n")
    for s, p, x, y, c in rows:
        f.write(f"{s},{p},{x:.3f},{y:.3f},{c}\n")

xs = [r[2] for r in rows]
ys = [r[3] for r in rows]
print(f"점 {len(rows)}개 | x {min(xs):.1f}~{max(xs):.1f} mm | y {min(ys):.1f}~{max(ys):.1f} mm")
