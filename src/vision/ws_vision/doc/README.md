# 업로드 내용 설명

업로드된 패키지에는 vision 기본 노드 2개와 인터페이스, moveit2 초안 코드가 포함 되어있습니다. vision의 노드는 python, cpp 두 개로 각각 입력 이미지의 특징 추출과 Opencv를 활용한 픽셀 좌표 추출 및 토픽 발행의 기능을 수행합니다.

비전 노드의 경우, 좌표 픽셀 목록이 각 추출된 영역별로 모아져 보내집니다
(현재 프롬포트에 따라 cat, eye, nose, mouth)

moveit2 초안 코드의 경우 비전 노드로부터 픽셀 좌표를 받아 이를 비례식으로 변환 후, rviz상에 이동하는 것을 구현하고자 하였으나 아직 미완성입니다. 
그림을 그리진 못하지만 픽셀 좌표 송수신은 작동됨을 확인되었습니다. 


## 현재 구현 상태

- SAM3 마스크 추출 및 `/vision_parts` 발행: 완료
- Contour 기반 픽셀 좌표 추출 및 `/vision/strokes` 발행: 완료
- MoveIt2에서 픽셀 좌표 수신 및 Cartesian Path 계산: 완료
- 로봇 그리기 자세 및 실제 경로 실행: 수정 필요


### vision_interfaces
- MoveIt 컨테이너와 Vision 컨테이너가 별도의 workspace를 사용하므로 각 workspace에 동일한 vision_interfaces 패키지가 필요

## 비전 - moveit2가 주고 받는 메시지 형식

 `vision_interfaces/msg/PixelPoint`
int32 u    # 이미지 가로축 픽셀 좌표 (왼쪽이 0)
int32 v    # 이미지 세로축 픽셀 좌표 (위쪽이 0, 아래로 갈수록 증가)
``

 `vision_interfaces/msg/Stroke`
string instance_label      # 부위 이름. 예: "cat", "eye of cat_left", "nose of cat"
int32 image_width          # 원본 이미지 가로 크기 (px)
int32 image_height         # 원본 이미지 세로 크기 (px)
PixelPoint[] points         # 이 부위의 윤곽선을 이루는 점들 (아래 참고)


# 다음의 경우, 비전 노드끼리 주고 받는 추출 영역 토픽
 `vision_interfaces/msg/MaskImage`
string instance_label     # 예: "eye_left", "cat", "nose"
string class_name         # 예: "eye of cat" (원본 SAM3 프롬프트)
float32 confidence
int32 image_width
int32 image_height
uint8[] mask_data         # mono8 raw bytes, row-major



### vision 노드
(1) `python sam3_extract_ros_node.py`
- 가상환경을 만든 후, 그 안에서 실행
- 코드 내 설정된 경로의 이미지 영역을 추출
- PROMPTS 넣은 단어에 맞는 영역 추출하여 MaskImage 형식으로 토픽 발송

토픽 내용은 어떤 부분인지 라벨과 sam 프롬프트 라벨, 신뢰도, 이미지 크기 및 추출한 영역의 마스크 데이터


(2) `cpp contour_pixel_node.cpp`
- sam3에서 보낸 토픽 vision_parts를 구독
- 영역을 컨투어 및 점의 갯수를 줄임

- 추출한 컨투어 선의 점 갯수 줄이는 정도를 조절하는 파라미터
default_epsilon_ratio_ -> 눈,코,입
cat_epsilon_ratio_ -> 고양이 몸 전체 (점을 더 많이 추출하게금 다르게 설정)

각 부위별 점 갯수가 컨투어 실행 시 출력됨
이를 보고 조절할 수 있음

- vision/strokes로 픽셀 점 좌표들 및 라벨, 이미지 크기 토픽 전송 -> moveit2 노드




### moveit2 노드 (미완성 - 수정 필요)
(3) `draw_strokes_node.cpp`
- 픽셀 좌표를 받아 변환 후 카터시안 경로를 계산
- 경로를 시각화하고 그리는 동작 수행
(add로 rviz에서 토픽 추가하여 시각화 활성화)
=> 픽셀 좌표를 받아와서 카터시안 경로 계산까지는 성공

* 수정해야할 내용
-> 그리는 자세 지정이 되지 않음 (홈 자세 그대로 움직임)
-> 그리는 동작이 예상 경로와 다름




## 환경 구성 및 실행

# [1] `사전 sam3 실행을 위한 가상환경 빌드` **필수**

(1) 도커 환경 내에 필요한 라이브러리 설치
sudo apt update
sudo apt install python3-venv python3-dev

위 명령어로 정상 설치 되지 않을 경우 아래 명령어로 실행
sudo apt install -y python3-venv python3.12-venv
혹은
sudo apt install -y python3-venv python3-pip


(2) `가상환경 만들기 | ros2 환경과 같이 쓰기 위한 설정 추가

python3 -m venv --system-site-packages ~/sam3_env_jazzy

가상환경 활성화
source ~/sam3_env_jazzy/bin/activate
which pip   # /home/rosuser/sam3_env_jazzy/bin/pip 가 출력되면 성공

가상환경 종료 방법
deactivate


sam3 관련 패키지를 가상환경에 설치
pip install --upgrade pip

# 컨테이너 안, venv 활성화 상태에서 - 설치 시간 꽤 오래 걸림(5~10분) + 용량 부족 유의
pip install numpy==2.2.6 torch==2.13.0 torchvision==0.28.0 ultralytics==8.4.107
pip install git+https://github.com/ultralytics/CLIP.git

(3) 가상환경에서 sam3 파이썬 파일 실행
python3 sam3_extract_ros_node.py



## sam3.pt 다운로드 링크 **필수, 용량 큼 주의(3GB)**
https://drive.google.com/file/d/1uDDGdN8Jnu9_qB_AGL1S3BcMPAl3R2f9/view?usp=sharing
-> 설치 위치 : src/vision/ws_vision/src/vision_node/src

*참고 : sam3 실행이 어려울 경우를 대비해 sam3_output 예시 이미지를 업로드했습니다*





# [2] 실행 방법

***실행 전 가상환경 생성 및 sam3.pt 설치 필수***
*** 비전 및 moveit2 ws에 인터페이스 패키지 설치 확인***

moveit2 노드의 경우, 미완성이므로 참고용. 실행 필수 X

각 컨테이너에서 빌드 후 소스 실행
colcon build --symlink-install
source install/setup.bash


`(실행 순서) demo.launch.py -> contour_pixel -> sam3_extract_ros_node.py
-> (참고용. 필요 시 실행) draw_strokes.launch.py`


# ── MoveIt 컨테이너 ──────────────────────────────
# 터미널1: RViz + MoveIt 실행 (시뮬레이션/실물 포함)
 ros2 launch hcr_moveit_config demo.launch.py

# 터미널2 그리기 노드 (참고용. 필요 시 실행)
ros2 launch hello_moveit draw_strokes.launch.py

# ── 비전 컨테이너 ────────────────────────────────
# 터미널3: 컨투어 추출 노드
ros2 run vision_node contour_pixel

# ── 비전 컨테이너 내 가상환경 실행 ──────────────────
# 터미널4: 가상환경에서 SAM3 마스크 추출 + 발행
source ~/sam3_env_jazzy/bin/activate
python3 sam3_extract_ros_node.py






