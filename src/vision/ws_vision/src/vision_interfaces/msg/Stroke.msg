# 부위 하나(예: eye_left)를 구성하는 픽셀 좌표 목록.
# contour_pixel_node(비전 컨테이너)가 만들어서 발행하고,
# draw_strokes_node(MoveIt 컨테이너)가 구독해서 그린다.

string instance_label      # 예: "eye_left", "cat", "nose"
int32 image_width          # 원본 이미지 가로 크기(px), 배율 계산에 사용
int32 image_height         # 원본 이미지 세로 크기(px)
vision_interfaces/PixelPoint[] points   # approxPolyDP로 근사된 픽셀 좌표들 (닫힌 컨투어)
