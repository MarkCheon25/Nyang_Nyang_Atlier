#!/usr/bin/env python3
"""펜 끝 자취 시각화 — TF 에서 pen_tip 을 따라가며 지나간 선을 RViz 에 그린다.

왜 필요한가
-----------
지금까지는 데카르트 경로가 잘 나왔는지를 **달성률 숫자**로만 판단했다.
그런데 그리기 작업의 결과물은 결국 "펜이 지나간 선" 그 자체다. 그것이 보이지
않으면 다음을 알 수 없다.

  · 사각형이 진짜 사각형인가 (모서리가 둥글게 뭉개지지 않았는가)
  · A4 크기로 키웠을 때 **어디서** 일그러지는가
  · 계획한 도형과 실제 자취가 얼마나 벌어지는가  ← 실기 선 품질 지표

이 노드는 로봇을 제어하지 않는다. **읽기 전용**이다. TF 를 구독해 선만 그린다.

동작
----
    TF (base_frame → tip_frame) 조회
      → 직전 점에서 min_point_distance 이상 움직였으면 점 추가
        → MarkerArray 발행 (LINE_STRIP = 자취, SPHERE = 현재 펜 끝)

RViz 에서 Add → MarkerArray → Topic 을 /hcr5_viz/pen_trail 로 잡으면 보인다.

자취 지우기
-----------
    ros2 service call /hcr5_viz/clear std_srvs/srv/Empty
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

import tf2_ros
from geometry_msgs.msg import Point
from std_srvs.srv import Empty
from visualization_msgs.msg import Marker, MarkerArray


class PenTrail(Node):
    def __init__(self):
        super().__init__("pen_trail")

        # ── 파라미터 ──────────────────────────────────────────────────────────
        # 전부 launch 나 커맨드라인에서 바꿀 수 있다. 코드를 고칠 필요가 없다.
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("tip_frame", "pen_tip")
        # min_point_distance: 이만큼 움직여야 점을 하나 찍는다 [m].
        #   0 으로 두면 정지 상태에서도 같은 점이 계속 쌓여 메모리를 먹는다.
        #   너무 크면 곡선이 각져 보인다. 2mm 정도가 그리기 스케일에 맞다.
        self.declare_parameter("min_point_distance", 0.002)
        # max_points: 오래된 점부터 버린다. 무한정 쌓이지 않게 하는 안전장치.
        self.declare_parameter("max_points", 20000)
        self.declare_parameter("publish_rate", 30.0)     # [Hz]
        self.declare_parameter("line_width", 0.002)      # [m] 선 굵기
        self.declare_parameter("trail_color", [1.0, 0.2, 0.2, 1.0])   # RGBA 빨강
        self.declare_parameter("tip_color", [1.0, 1.0, 0.2, 1.0])     # RGBA 노랑
        self.declare_parameter("tip_size", 0.008)        # [m] 현재 위치 구슬 지름

        g = self.get_parameter
        self.base_frame = g("base_frame").value
        self.tip_frame = g("tip_frame").value
        self.min_dist = g("min_point_distance").value
        self.max_points = g("max_points").value
        rate = g("publish_rate").value
        self.line_width = g("line_width").value
        self.trail_color = g("trail_color").value
        self.tip_color = g("tip_color").value
        self.tip_size = g("tip_size").value

        # ── TF ────────────────────────────────────────────────────────────────
        # Buffer 가 변환을 저장하고, TransformListener 가 /tf·/tf_static 을 구독해 채운다.
        # 둘 다 필요하다. listener 를 self 에 붙여두지 않으면 GC 되어 조용히 죽는다.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── 발행 · 서비스 ─────────────────────────────────────────────────────
        # ~/ 는 노드 이름으로 치환된다 → /pen_trail/trail
        self.pub = self.create_publisher(MarkerArray, "~/trail", 1)
        self.srv_clear = self.create_service(Empty, "~/clear", self._on_clear)

        self.points: list[Point] = []
        self._warned = False   # TF 경고를 한 번만 내기 위한 플래그

        # ★ 기동 시 이전 자취를 지운다.
        #   Marker 는 lifetime 을 지정하지 않으면 기본값 0 = **영구 유지**다.
        #   그래서 노드를 껐다 켜도 RViz 는 이전 실행의 선을 계속 들고 있고,
        #   새 자취가 그 위에 덧그려져 두 실험 결과가 겹쳐 보인다.
        #   (노드가 죽어도 선이 남아 있어서 "아직 도는 중"으로 오인하기도 쉽다)
        #
        #   지연을 두는 이유: 발행자가 막 생겼을 때 구독자(RViz)와의 연결이
        #   아직 안 맺어져 있으면 이 메시지가 유실된다. 0.5초 뒤 한 번만 쏜다.
        self._startup_clear = self.create_timer(0.5, self._clear_once)

        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"펜 자취 시각화 시작: {self.base_frame} → {self.tip_frame}, "
            f"{rate:.0f}Hz, 최소간격 {self.min_dist * 1000:.1f}mm"
        )
        self.get_logger().info("RViz: Add → MarkerArray → Topic = /pen_trail/trail")
        self.get_logger().info("자취 지우기: ros2 service call /pen_trail/clear std_srvs/srv/Empty")

    # ─────────────────────────────────────────────────────────────────────────
    def _clear_once(self):
        """기동 직후 1회 실행. 이전 실행이 남긴 마커를 지우고 타이머를 해제한다."""
        self._startup_clear.cancel()
        self._delete_all()
        self.get_logger().info("이전 자취 삭제 (기동 정리)")

    def _delete_all(self):
        """RViz 가 들고 있는 마커를 실제로 지운다.

        빈 LINE_STRIP 을 보내는 것만으로는 안 된다 — RViz 는 이전 마커를
        그대로 유지한다. DELETEALL 액션을 명시적으로 보내야 사라진다.
        """
        m = Marker()
        m.header.frame_id = self.base_frame
        m.action = Marker.DELETEALL
        self.pub.publish(MarkerArray(markers=[m]))

    def _on_clear(self, request, response):
        """자취를 지운다. 도형을 바꿔가며 실험할 때 이전 선이 겹치지 않게."""
        n = len(self.points)
        self.points.clear()
        self._delete_all()
        self.get_logger().info(f"자취 삭제 ({n} 점)")
        return response

    # ─────────────────────────────────────────────────────────────────────────
    def _tick(self):
        # TF 조회. Time() = 0 은 "가장 최근에 받은 것"을 뜻한다.
        # 특정 시각을 지정하면 그 시각의 변환을 보간해 주지만, 여기서는
        # 최신값이면 충분하고 타이밍 문제도 피할 수 있다.
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.tip_frame,
                rclpy.time.Time(), timeout=Duration(seconds=0.05)
            )
        except tf2_ros.TransformException as e:
            # demo.launch.py 가 아직 안 떴거나 프레임 이름이 틀리면 여기로 온다.
            # 매 틱마다 찍으면 로그가 도배되므로 한 번만 경고한다.
            if not self._warned:
                self.get_logger().warn(
                    f"TF 조회 실패 ({self.base_frame} → {self.tip_frame}): {e}. "
                    f"demo.launch.py 가 떠 있는지, 프레임 이름이 맞는지 확인."
                )
                self._warned = True
            return

        if self._warned:
            self.get_logger().info("TF 복구됨")
            self._warned = False

        t = tf.transform.translation
        p = Point(x=t.x, y=t.y, z=t.z)

        # 직전 점에서 충분히 움직였을 때만 추가한다.
        if self.points:
            last = self.points[-1]
            d = math.dist((p.x, p.y, p.z), (last.x, last.y, last.z))
            if d < self.min_dist:
                # 움직이지 않았어도 마커는 계속 발행한다.
                # (RViz 를 나중에 켜도 자취가 보이게)
                self._publish(p)
                return

        self.points.append(p)
        if len(self.points) > self.max_points:
            # 앞쪽(오래된 것)부터 버린다.
            del self.points[: len(self.points) - self.max_points]

        self._publish(p)

    # ─────────────────────────────────────────────────────────────────────────
    def _publish(self, current: Point):
        now = self.get_clock().now().to_msg()
        arr = MarkerArray()

        # ① 자취 — LINE_STRIP: points 를 순서대로 이어 선을 그린다
        trail = Marker()
        trail.header.frame_id = self.base_frame
        trail.header.stamp = now
        trail.ns = "pen_trail"
        trail.id = 0
        trail.type = Marker.LINE_STRIP
        trail.action = Marker.ADD
        # ★ pose 를 안 채우면 RViz 가 "uninitialized quaternion" 경고를 낸다.
        #   LINE_STRIP 은 points 가 이미 절대좌표라 pose 는 항등이어야 한다.
        trail.pose.orientation.w = 1.0
        # LINE_STRIP 은 scale.x 만 쓴다 (선 굵기). y·z 는 무시된다.
        trail.scale.x = self.line_width
        (trail.color.r, trail.color.g, trail.color.b, trail.color.a) = self.trail_color
        trail.points = self.points
        arr.markers.append(trail)

        # ② 현재 펜 끝 — SPHERE: 지금 어디 있는지 한눈에
        tip = Marker()
        tip.header.frame_id = self.base_frame
        tip.header.stamp = now
        tip.ns = "pen_tip"
        tip.id = 1
        tip.type = Marker.SPHERE
        tip.action = Marker.ADD
        tip.pose.position = current
        tip.pose.orientation.w = 1.0
        # SPHERE 는 x·y·z 가 각 축 지름이다.
        tip.scale.x = tip.scale.y = tip.scale.z = self.tip_size
        (tip.color.r, tip.color.g, tip.color.b, tip.color.a) = self.tip_color
        arr.markers.append(tip)

        self.pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = PenTrail()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        # 이미 shutdown 된 뒤 또 호출하면 예외가 나므로 확인 후 호출한다.
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
