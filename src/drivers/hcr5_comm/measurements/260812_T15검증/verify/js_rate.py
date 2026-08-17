import rclpy, time, json
from rclpy.node import Node
from sensor_msgs.msg import JointState
class N(Node):
    def __init__(s):
        super().__init__("js_rate")
        s.rows=[]; s.create_subscription(JointState,"/joint_states",s.cb,50); s.t0=time.time()
    def cb(s,m): s.rows.append([time.time(), list(m.position)])
rclpy.init(); n=N()
while time.time()-n.t0 < 30: rclpy.spin_once(n, timeout_sec=0.1)
r=n.rows
dt=r[-1][0]-r[0][0]
chg=sum(1 for i in range(1,len(r)) if r[i][1]!=r[i-1][1])
print(f"표본 {len(r)}  창 {dt:.2f}s")
print(f"발행률(토픽)   {(len(r)-1)/dt:8.3f} Hz")
print(f"값 갱신(ROS)   {chg/dt:8.3f} Hz   (값이 실제로 바뀐 횟수 {chg})")
json.dump({"win_s":dt,"n":len(r),"changes":chg}, open("/tmp/js_rate.json","w"))
rclpy.shutdown()
