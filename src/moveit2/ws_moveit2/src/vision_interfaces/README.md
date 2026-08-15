# vision_interfaces (미러)

⚠️ **이 패키지는 vision 팀 정의의 복제본이다. 원본이 아니다.**

vision 컨테이너가 `/vision/strokes` 를 발행할 때 쓰는 메시지 정의를, moveit2 컨테이너가
구독하기 위해 같은 내용으로 다시 선언해 둔 것이다.

## 왜 복제하나

vision 과 moveit2 는 **별도 컨테이너·별도 워크스페이스**다. moveit2 워크스페이스는
`ws_moveit2` 와 `hanwha_robot_arm` 만 마운트하므로 vision 의 패키지를 볼 수 없다.
ROS 2 는 메시지 정의가 양쪽에 각각 있어도, **구조가 같으면 타입 해시가 같아져** 통신이
된다. 그 성질에 기대는 것이다.

## ⚠️ 어긋나면 조용히 실패한다

필드 이름·타입·순서 중 **하나라도** 다르면 타입 해시가 달라진다. 그러면:

- `ros2 topic list` 에는 토픽이 **보인다**
- `ros2 topic info /vision/strokes` 에 발행자도 **보인다**
- 그런데 메시지는 **한 개도 안 들어온다**

에러 메시지가 없어서 QoS 문제로 오해하기 쉽다.

## 확인 방법

vision 이 발행 중일 때 양쪽 해시를 비교한다.

```bash
# moveit2 컨테이너에서 — 발행자(vision)의 타입 해시가 보인다
ros2 topic info /vision/strokes --verbose | grep -i "type hash"

# 우리 정의의 해시
ros2 interface show vision_interfaces/msg/Stroke     # 내용 확인
```

`RIHS01_...` 값이 같아야 한다. 다르면 아래 정의를 vision 원본과 맞춰 다시 빌드한다.

### 기준 해시값 — **이 값의 주인은 이 문서 한 곳이다**

아래 정의로 빌드했을 때의 값 (2026-08-13):

```
RIHS01_d8c0b1c8ffdbe600b7b59775cc2460b82c96be241faf49fea80c7393e77506d7
```

⚠️ 다른 문서에 베껴 두지 말 것. 정의가 바뀌면 해시도 바뀌는데, 여러 곳에 흩어져 있으면
어느 값이 맞는지 알 수 없게 된다. **정의를 고치면 이 값도 함께 갱신한다.**

## 현재 정의

```
# Stroke.msg
string instance_label      # 부위 이름. 예: "cat", "eye of cat_left"
int32 image_width          # 원본 이미지 가로 크기 (px)
int32 image_height         # 원본 이미지 세로 크기 (px)
PixelPoint[] points        # 이 부위의 닫힌 윤곽선을 이루는 점들

# PixelPoint.msg
int32 u    # 이미지 가로축 픽셀 좌표 (왼쪽이 0)
int32 v    # 이미지 세로축 픽셀 좌표 (위쪽이 0, 아래로 갈수록 증가)
```

출처: vision 담당자가 공유한 인터페이스 문서 (2026-08-13).

## 없애야 할 것

vision 의 `vision_interfaces` 가 저장소 `main` 에 병합되고 moveit2 컨테이너가 그것을
볼 수 있게 되면 **이 미러는 지워야 한다.** 두 정의가 공존하면 언젠가 갈라진다.
