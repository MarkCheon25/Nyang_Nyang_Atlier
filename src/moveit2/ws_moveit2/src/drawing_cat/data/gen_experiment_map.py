"""A/B 실험용 map.csv — **닫힌 윤곽 5 개**를 만든다.

`gen_sample_map.py` 는 윤곽선 하나뿐이라 **순서 최적화를 잴 수 없다** (고를 것이 없다).
이 파일은 비전이 실제로 보내는 형태 — 부위 여러 개, 전부 폐곡선 — 를 흉내 낸다.

## 일부러 "최적화 안 된" 상태로 만든다

측정 대상이 최적화의 효과이므로, 출발점이 이미 최적이면 아무것도 안 보인다.
다만 **억지로 나쁘게 만들지는 않는다** — 비전이 실제로 내놓는 상태를 따른다:

1. **부위 순서 = 라벨 순서.** SAM3 가 부위를 내놓는 순서에는 그리기 효율과 아무 관계가
   없다 (vision 규약: "스트로크들 사이에는 순서가 없다").
2. **각 윤곽의 첫 점 = 가장 위쪽 정점.** `cv2.findContours` 가 컨투어를 그 근처에서
   시작하기 때문이다. 펜 위치와는 무관하게 정해지는 값이라, 폐곡선 회전이 고칠 여지가
   여기서 생긴다.

형식은 vision 과 같다: `stroke_idx,point_idx,x_mm,y_mm,closed`
vision 규약대로 **첫 점을 끝에 중복해 넣지 않는다.**
"""
import math

CX, CY = 105.0, 148.5   # A4(210x297) 중앙 (mm)


def ellipse(cx, cy, rx, ry, n):
    """중심 (cx,cy), 반지름 (rx,ry) 인 타원을 n 점으로."""
    return [(cx + rx * math.cos(2 * math.pi * i / n),
             cy + ry * math.sin(2 * math.pi * i / n)) for i in range(n)]


def head(n=96, r=40.0, ear_h=22.0, ear_half=math.radians(18)):
    """귀 두 개가 솟은 머리 윤곽 (gen_sample_map.py 와 같은 모양)."""
    ears = [math.radians(125.0), math.radians(55.0)]
    pts = []
    for i in range(n):
        th = 2 * math.pi * i / n
        rr = r
        for peak in ears:
            d = abs((th - peak + math.pi) % (2 * math.pi) - math.pi)
            if d < ear_half:
                rr += ear_h * (1.0 - d / ear_half)
        pts.append((CX + rr * math.cos(th), CY + rr * math.sin(th)))
    return pts


def start_at_topmost(pts):
    """가장 위쪽(y 최소) 정점이 맨 앞에 오도록 회전. cv2.findContours 흉내."""
    k = min(range(len(pts)), key=lambda i: (pts[i][1], pts[i][0]))
    return pts[k:] + pts[:k]


# 라벨 순서대로 — 그리기 효율과 무관한 순서다.
PARTS = [
    ("cat",             head()),
    ("eye of cat_left", ellipse(CX - 15.0, CY - 8.0,  6.0, 6.0, 24)),
    ("eye of cat_right", ellipse(CX + 15.0, CY - 8.0,  6.0, 6.0, 24)),
    ("nose of cat",     ellipse(CX,        CY + 6.0,  4.0, 3.0, 16)),
    ("mouth of cat",    ellipse(CX,        CY + 19.0, 14.0, 7.0, 32)),
]

rows = []
for idx, (label, pts) in enumerate(PARTS):
    for p_idx, (x, y) in enumerate(start_at_topmost(pts)):
        rows.append(f"{idx},{p_idx},{x:.3f},{y:.3f},1")

out = "experiment_map.csv"
with open(out, "w") as f:
    f.write("stroke_idx,point_idx,x_mm,y_mm,closed\n")
    f.write("\n".join(rows) + "\n")

xs = [x for _, pts in PARTS for x, _ in pts]
ys = [y for _, pts in PARTS for _, y in pts]
print(f"{out}: 스트로크 {len(PARTS)} 개 / 총 {len(rows)} 점 (전부 closed=1)")
print(f"  범위 {min(xs):.1f}~{max(xs):.1f} x {min(ys):.1f}~{max(ys):.1f} mm "
      f"= {max(xs)-min(xs):.1f} x {max(ys)-min(ys):.1f} mm")
for idx, (label, pts) in enumerate(PARTS):
    head_pt = start_at_topmost(pts)[0]
    print(f"  {idx} {label:18s} {len(pts):3d}점  첫점 ({head_pt[0]:6.1f}, {head_pt[1]:6.1f})")
