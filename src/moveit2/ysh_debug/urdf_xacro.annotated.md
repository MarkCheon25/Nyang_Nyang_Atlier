# URDF / xacro 구조 — 한 줄씩 해설본

> ⚠️ **읽기용 문서.** 실제 파일은 `ws_moveit2/src/hcr5_description/urdf/` 에 있다.
> 원본을 고치면 이 문서도 같이 고쳐야 한다 (자동 동기화 없음).

---

## 0. 전체 구조 — 파일이 왜 나뉘어 있나

```
hcr5.urdf.xacro          ★ 최상위. 이것 하나만 xacro 에 넘기면 완성된 로봇이 나온다
 ├─ hcr5_materials.xacro    색상 정의
 ├─ hcr5_arm.xacro          링크 7개 + 관절 6개 (CAD 산출물)
 ├─ hcr5_tool.xacro       ★ 우리가 신설한 부분 — tool0 · pen_tip
 └─ hcr5.ros2_control.xacro ★ sim / 실기가 갈리는 지점
```

**왜 한 파일에 몰아넣지 않나** — 바뀌는 주기가 다르기 때문이다.

| 파일 | 언제 바뀌나 |
|---|---|
| `hcr5_arm.xacro` | 거의 안 바뀜 (CAD 치수) |
| `hcr5_tool.xacro` | 펜홀더 설계가 확정되면 |
| `hcr5.ros2_control.xacro` | sim ↔ 실기 전환할 때 |
| `hcr5.urdf.xacro` | 인자를 추가할 때만 |

**xacro 란** — URDF 는 순수 XML 이라 변수도 조건문도 없다. 6개 관절을 쓰려면
비슷한 블록을 6번 복사해야 한다. xacro 는 그 위에 매크로·변수·조건을 얹은
전처리기다. 최종적으로는 평평한 URDF XML 로 펼쳐진다(`xacro` 명령).

---

## 1. `hcr5.urdf.xacro` — 최상위

```xml
<?xml version="1.0"?>
<robot name="hcr5" xmlns:xacro="http://www.ros.org/wiki/xacro">
```
- `name="hcr5"` — **최상위 파일의 이름이 최종 로봇 이름**이 된다.
  include 되는 파일들의 `<robot name=...>` 은 무시된다.
- `xmlns:xacro` — 이걸 선언해야 `<xacro:...>` 태그를 쓸 수 있다. 빠뜨리면
  xacro 태그가 그냥 알 수 없는 XML 로 남아 파싱 에러가 난다.

### 인자 3개

```xml
<xacro:arg name="ros2_control"    default="true"/>
<xacro:arg name="hardware_plugin" default="mock_components/GenericSystem"/>
<xacro:arg name="pen_length"      default="0.15"/>
```

`<xacro:arg>` 는 **커맨드라인에서 받는 인자**다. `<xacro:property>` 와 다르다:

| | 어디서 값이 오나 | 참조 |
|---|---|---|
| `<xacro:arg>` | 커맨드라인 `xacro f.xacro name:=value` | `$(arg name)` |
| `<xacro:property>` | 파일 안에서 정의 | `${name}` |

**세 인자의 뜻**

- **`hardware_plugin`** ★ — 이 프로젝트의 **하드웨어 추상화 경계**다.
  ```bash
  xacro hcr5.urdf.xacro                                                    # mock
  xacro hcr5.urdf.xacro hardware_plugin:=mujoco_ros2_control/MujocoSystemInterface  # sim
  xacro hcr5.urdf.xacro hardware_plugin:=<우리드라이버>/HcrSystem            # 실기
  ```
  **같은 URDF 로 플러그인만 갈아끼운다.** 4단계 실기 연결이 이 한 줄로 끝나도록 설계했다.

- **`ros2_control`** — `false` 면 ros2_control 블록을 아예 안 만든다.
  감싸는 쪽이 자체 블록을 정의해야 할 때 쓴다(시뮬레이션이 `<param>` 을
  넣어야 하는 경우). 이 갈래가 없으면 mock 블록이 **항상** 들어가서
  다른 하드웨어와 같은 관절을 두고 충돌한다.

- **`pen_length`** — 펜 끝까지 거리 [m]. 펜홀더 설계 확정 시 숫자만 교체.

### include 와 조립

```xml
<xacro:include filename="$(find hcr5_description)/urdf/hcr5_arm.xacro"/>
```
- `$(find 패키지)` — 그 패키지의 **share 디렉터리** 경로로 치환된다
  (`install/hcr5_description/share/hcr5_description`).
- 그래서 `CMakeLists.txt` 의 `install(DIRECTORY urdf meshes config ...)` 가
  **반드시 있어야 한다.** 없으면 share 에 아무것도 안 깔려 `$(find ...)` 가
  찾지 못한다.
- include 는 그 파일의 내용을 **여기에 펼쳐 넣는다.** `hcr5_arm.xacro` 의
  `<link>`·`<joint>` 들이 이 문서에 합쳐진다.

```xml
<xacro:hcr5_tool parent="link6_1" pen_length="$(arg pen_length)"/>
```
- 매크로 **호출**. include 만으로는 아무 링크도 안 생긴다 —
  `hcr5_tool.xacro` 는 매크로 **정의**만 담고 있기 때문이다.
- `parent="link6_1"` — 펜홀더를 어느 링크에 붙일지. 플랜지에 붙인다.
- `$(arg pen_length)` 로 커맨드라인 값을 매크로에 전달.

```xml
<xacro:if value="$(arg ros2_control)">
  <xacro:hcr5_ros2_control plugin="$(arg hardware_plugin)"/>
</xacro:if>
```
- `<xacro:if>` — 조건부 포함. `value` 가 `true` 일 때만 안쪽이 살아남는다.
- 검증: `xacro hcr5.urdf.xacro ros2_control:=false | grep -c "<ros2_control"` → `0`

---

## 2. `hcr5_tool.xacro` — 우리가 신설한 핵심

### 왜 필요했나

물려받은 URDF 의 체인 끝은 `link6_1` = **플랜지**였다. SRDF 에 엔드이펙터
정의도 없었다. 그리기는 **펜 끝** 기준이어야 하므로 프레임을 새로 만들어야 했다.

```
link6_1 ──fixed──> tool0 ──fixed──> pen_tip
(플랜지)          (ROS-I 관례)      (실제 TCP)
```

### 매크로 정의

```xml
<xacro:macro name="hcr5_tool"
             params="parent
                     pen_length:=0.15
                     flange_r:=0 flange_p:=-1.5707963 flange_y:=0
                     pen_x:=0 pen_y:=0">
```
- `params` 에서 `이름:=기본값` 은 **선택 인자**, 그냥 `이름` 은 **필수 인자**.
  즉 `parent` 는 반드시 줘야 하고 나머지는 생략 가능하다.
- 오프셋을 전부 인자로 뺀 이유: **펜홀더 설계가 아직 없기 때문**이다.
  설계가 나오면 호출 쪽 숫자만 바꾸면 되고, 매크로는 안 건드린다.

### `tool0` — 왜 중간에 하나 더 두나

```xml
<link name="tool0"/>
<joint name="flange_to_tool0" type="fixed">
  <parent link="${parent}"/>
  <child  link="tool0"/>
  <origin xyz="0 0 0" rpy="${flange_r} ${flange_p} ${flange_y}"/>
</joint>
```

- `<link name="tool0"/>` — **비어 있는 링크**다. `<visual>`·`<collision>`·
  `<inertial>` 이 없다. 순수하게 **좌표계(프레임)** 역할만 한다.
  RViz 에 아무것도 그려지지 않는다.
- `type="fixed"` — 움직이지 않는 관절. **자유도에 포함되지 않는다.**
  그래서 `joint_1`~`joint_6` 6자유도가 그대로 유지된다.
- `xyz="0 0 0"` — 위치는 플랜지와 같다. **회전만** 준다.

**`tool0` 을 왜 두나** — ROS-Industrial 관례로 `tool0` 은 **+Z 가 공구 진출
방향**인 프레임이다. 이 규약을 따르면:
- 나중에 다른 공구로 바꿔도 `tool0` 기준으로 붙이면 된다
- 다른 ROS-I 도구·예제와 호환된다
- 펜을 `+Z` 로 뻗는 것으로 단순화된다

### 회전값 `flange_p = -π/2` 는 어떻게 정했나

**단서 1** — 이 URDF 는 모든 관절의 `rpy` 가 `0` 이다.
→ 모든 링크 프레임이 `base_link` 와 **축 정렬**되어 있다.

**단서 2** — `joint_6` 의 축이 `(-1, 0, 0)` 이다.
→ 6축 팔에서 마지막 관절의 회전축 = 공구 진출축.
→ **플랜지 법선 = `link6_1` 의 -X**

**결론** — `tool0` 의 `+Z` 를 `link6_1` 의 `-X` 에 맞춰야 한다.

Y축 회전 θ 는 `Z=(0,0,1)` 을 `(sinθ, 0, cosθ)` 로 보낸다.
θ = −90° → `(−1, 0, 0)` = **−X** ✓ 그래서 `flange_p = -1.5707963`.

**FK 로 검증한 결과** (실측):
```
link6_1 → pen_tip  translation = [-0.1500  0.0000  0.0000]
tool0 의 +Z 방향   = [-1. 0. 0.]   (link6_1 기준)
```
손목이 −X 로 밀려나는 방향(link4 −0.109 → link5 −0.171 → link6 −0.303)의
연장선으로 펜이 −0.453 까지 나간다. **팔 바깥을 향한다.**

> 부호가 반대였다면 펜이 손목 안쪽을 파고들어 기하적으로 불합리했을 것이다.

**✅ 2026-08-09 — 실기 실측으로 확정됐다.**
홈 자세에서 `pen_tip` 의 TF 자세가 **RPY (180°, 0, 0)** 으로 나오는데, 펜던트가 같은 자세에서
보고한 flange **`rx = −180°`** 와 일치한다. 즉 여기서 기하 추론만으로 유도한 `tool0` 프레임이
**실기 flange 프레임 규약과 같다.** 3단계에서 데카르트 자세를 실기로 넘길 때 미지의 회전
보정이 끼지 않는다는 뜻이다.

> ⚠️ **위 링크 좌표는 08-09 기구학 정본 교정 후 값이다.** 교정 전에는 link6 이 −0.260 이었고
> (43mm 짧았다), 그래도 **펜 축 방향 결론 자체는 바뀌지 않았다** — 교정은 순수 병진이라
> 회전 규약에 영향이 없기 때문이다. 교정 경위는
> [`../docs/ysh_HCR5_ros2_control_구축_상세.md`](../docs/ysh_HCR5_ros2_control_구축_상세.md) §5.5.

### `pen_tip` — 실제 TCP

```xml
<link name="pen_tip"/>
<joint name="tool0_to_pen_tip" type="fixed">
  <parent link="tool0"/>
  <child  link="pen_tip"/>
  <origin xyz="${pen_x} ${pen_y} ${pen_length}" rpy="0 0 0"/>
</joint>
```
- `tool0` 의 `+Z` 로 `pen_length` 만큼 나간 지점. `tool0` 이 이미 회전을
  흡수했으므로 여기서는 **회전이 필요 없다** (`rpy="0 0 0"`).
- 이 링크가 SRDF 체인의 `tip_link` 가 되고, 그래서
  **IK·`getCurrentPose()`·데카르트 경로가 전부 펜 끝 기준**이 된다.

---

## 3. `hcr5_arm.xacro` — 링크·관절 (물려받은 것 + 한계 교체)

### 물려받은 것

STL 메쉬 7개와 관절 원점·축은 **CAD 변환 산출물**이라 원본 CAD 없이 재생성이
불가능하다. 그래서 이것만 가져왔다.

```xml
<mesh filename="package://hcr5_description/meshes/base_link.stl"
      scale="0.001 0.001 0.001"/>
```
- `package://패키지/경로` — ROS 의 메쉬 경로 표기. `$(find)` 와 달리
  **런타임에 RViz·MoveIt 이 해석**한다.
- `scale="0.001 …"` — STL 이 **mm 단위**로 만들어져 있어 m 로 줄인다.
  이게 없으면 로봇이 1000배 크게 나온다.

### 교체한 것 — `<limit>`

```xml
<joint name="joint_1" type="revolute">
  <origin xyz="0.0 0.0 0.03" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child  link="link1_1"/>
  <axis xyz="0.0 -0.0 1.0"/>
  <limit upper="6.283185" lower="-6.283185" effort="100" velocity="3.1416"/>
</joint>
```

| 속성 | 뜻 | 단위 |
|---|---|---|
| `upper` / `lower` | 관절 가동범위 | rad |
| `effort` | 최대 토크 | N·m |
| `velocity` | 최대 속도 | rad/s |

**⚠️ `<limit>` 에는 가속도 항목이 없다.** URDF 규격 자체에 없다.
그래서 가속도는 `joint_limits.yaml` 로만 줄 수 있고, 안 주면
시간 매개변수화가 실패한다 (상세본 §7.1).

**교체 내역**

| 관절 | 기존 | 교체 | 이유 |
|---|---|---|---|
| joint_1·6 | `lower="0.0"` | **`-6.283185`** | 음수 방향이 막혀 있었다 ↓ |
| joint_3 | ±130° | **±165°** | 실기 공식값 |
| 전 관절 | `velocity="100"` | **`3.1416`** | 실기 180°/s 의 32배였다 |

**joint_1·6 음수 불가가 왜 치명적이었나**

`revolute` 는 범위 밖으로 감기지 못하므로 플래너가 범위 **안쪽**을 통과해야 한다.

```
+0.1 rad → -0.1 rad 로 가고 싶다      (실제 이동량 0.2 rad = 11°)
  ↓ -0.1 은 범위 밖 → 등가값 6.183 rad 로 가야 함
계획되는 이동: 0.1 → 6.183 rad = 348° 대회전
```

**11° 움직이면 될 것을 348° 돌린다.** 그리기 중 base 가 한 바퀴 도는 셈이다.
`lower` 를 `-6.283185` 로 바꿔 **0을 범위 중앙에** 두어 해결했다.

> `type="continuous"` 로 바꾸면 무한 회전이 되지만, 실기는 ±360° 로
> 제한되어 있으므로 `revolute` 가 맞다.

---

## 4. `hcr5.ros2_control.xacro` — sim / 실기 갈림길

```xml
<xacro:macro name="hcr5_ros2_control"
             params="name:=hcr5_system plugin:=mock_components/GenericSystem">
  <ros2_control name="${name}" type="system">
    <hardware>
      <plugin>${plugin}</plugin>
    </hardware>
    <joint name="joint_1">
      <command_interface name="position"/>
      <state_interface name="position">
        <param name="initial_value">0</param>
      </state_interface>
      <state_interface name="velocity"/>
    </joint>
    ...
  </ros2_control>
</xacro:macro>
```

**`<command_interface>` / `<state_interface>` 의 뜻**

| | 방향 | 의미 |
|---|---|---|
| `command_interface` | 컨트롤러 → 하드웨어 | **쓰기.** "이 관절을 이 위치로" |
| `state_interface` | 하드웨어 → 컨트롤러 | **읽기.** "지금 이 위치·속도다" |

★ **이 선언이 `ros2_controllers.yaml` 과 정확히 일치해야 한다.**
```yaml
command_interfaces: [position]              # <command_interface name="position"/>
state_interfaces:   [position, velocity]    # <state_interface> 둘
```
어긋나거나 비어 있으면 컨트롤러가 configure 에 실패하고,
`Action client not connected to action server` 로 나타난다 (상세본 §7.2).

**`initial_value`** — mock 하드웨어가 시작할 때의 관절값.
`joint_3=1.5669, joint_4=-1.59, joint_5=-1.4695` 는 팔이 접힌 자세다.
실기 플러그인에서는 이 값이 무시된다(진짜 로봇의 실제 값을 읽으므로).

**`name="hcr5_system"`** — `ros2 control list_hardware_components` 에 표시되는
이름. **지금 무엇에 연결되어 있는지 판별하는 가장 빠른 방법**이 이 명령이다:
```bash
ros2 control list_hardware_components
#   name: hcr5_system
#   plugin name: mock_components/GenericSystem   ← mock 인지 실기인지 여기서 확인
```

---

## 5. 검증 명령 모음

```bash
# xacro 가 파싱되는가
xacro $(ros2 pkg prefix hcr5_description)/share/hcr5_description/urdf/hcr5.urdf.xacro > /tmp/hcr5.urdf

# 링크 트리가 맞는가  (끝이 pen_tip 이어야 한다)
check_urdf /tmp/hcr5.urdf

# 인자 교체가 먹는가
xacro ... hardware_plugin:=mujoco_ros2_control/MujocoSystemInterface | grep "<plugin>"
xacro ... ros2_control:=false | grep -c "<ros2_control"     # 0
xacro ... pen_length:=0.182 | grep -A3 tool0_to_pen_tip

# 설정 전반 검사
python3 src/moveit2/tools/ysh_check_moveit_config.py
```

**기대되는 링크 트리**
```
base_link → link1_1 → link2_1 → link3_1 → link4_1 → link5_1 → link6_1 → tool0 → pen_tip
```

---

관련 문서: [`docs/ysh_HCR5_ros2_control_구축_상세.md`](../docs/ysh_HCR5_ros2_control_구축_상세.md) §5
