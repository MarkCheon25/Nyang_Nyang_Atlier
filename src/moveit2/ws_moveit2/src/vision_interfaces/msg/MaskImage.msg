# SAM3(파이썬)가 뽑은 부위 하나의 이진 마스크를 그대로 담아 보낸다.
# PNG 인코딩/base64/JSON을 전혀 쓰지 않고, mono8(0 또는 255) 픽셀 바이트를
# row-major로 그대로 담는다. 크기 = image_width * image_height.

string instance_label     # 예: "eye_left", "cat", "nose"
string class_name         # 예: "eye of cat" (원본 SAM3 프롬프트)
float32 confidence
int32 image_width
int32 image_height
uint8[] mask_data         # mono8 raw bytes, row-major
