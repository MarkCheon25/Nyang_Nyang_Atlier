# RUNTIME — 명령이 어디서 도는가

> **짝 문서**: [`운전절차.md`](운전절차.md) 가 *무엇을 치나*를 든다면, 이 문서는 ***어디서 치나***를 든다.
> 로봇 앞에서 여는 것은 저쪽이고, 이쪽은 **한 번 세우면 몇 주 안 여는** 문서다.
>
> 가른 이유 — `운전절차.md` §5 함정 ①② 가 **둘 다 호스트↔컨테이너 경계에서 난 사고**다.
> 경계를 문서로 세우면 그 사고가 준다 (Mark 판단 2026-08-15).

**이 문서는 03pc(markch03 / MSI Vector 16) 기준이다.** PC 로컬 값은 §5 에 모았다.

---

## 1. 계층 — 명령은 두 자리에서 돈다

| 계층 | 명령 | 도는 자리 | 예 |
|---|---|---|---|
| **인프라** | `run_container.sh` · `docker exec` · `xhost` · `nmcli` | **호스트** | 컨테이너 진입·재생성, X11 허용, 실기 랜 프로필 |
| **로봇 제어** | `ros2 launch` · `ros2 run` · `python3 tools/…` · `colcon build` | **컨테이너 안** | 기동·지령·정지·빌드 |

**두 계층을 한 줄에 섞는 순간이 사고 지점이다** — §4 가 그 경계를 다룬다.

---

## 2. 컨테이너에 들어가기

```bash
cd ~/WorkSpace_260814/Nyang_Nyang_Atlier/src/moveit2 && ./run_container.sh shell
```

`run_container.sh shell` 은 `docker exec -it moveit2_dev bash` 다 — **새로 만들지 않고 이미 뜬 컨테이너에 붙는다.**
`docker compose` 를 직접 부르지 않는 이유는 UID/GID·렌더 GID·`XAUTHORITY` 가 안 맞을 수 있어서다(프로젝트 CLAUDE.md).

**들어가면 `source` 가 필요 없다.** 컨테이너 `.bashrc:118-119` 가 둘 다 물어준다:

```bash
source /opt/ros/jazzy/setup.bash
[ -f ~/ws_moveit2/install/setup.bash ] && source ~/ws_moveit2/install/setup.bash
```

⚠️ **이 편의는 대화형 셸에서만 산다.** 호스트에서 `docker exec … bash -lc "ros2 …"` 로 쏘면 `.bashrc` 를
안 읽어 `ros2: command not found` 가 난다 — §4 함정 ①.

---

## 3. 컨테이너가 보는 파일 — 볼륨 매핑

`src/moveit2/compose.yml` 이 세 곳을 잇는다. **호스트 배치는 그대로 두면서** 컨테이너 안에서만
colcon 워크스페이스 모양으로 보이게 하는 것이 요점이다.

| compose.yml | 호스트 | 컨테이너 |
|---|---|---|
| `:61` | `src/moveit2/ws_moveit2` | `~/ws_moveit2` |
| `:66` | `hanwha_robot_arm/` (리포 최상위) | `~/ws_moveit2/src/hanwha_robot_arm` |
| **`:72`** | **`src/drivers/`** | **`~/ws_moveit2/src/drivers`** |

`:72` 가 이 프로젝트에서 제일 많이 걸리는 줄이다:

- **`src/drivers/` 아래는 어떻게 재배치해도 컨테이너에서 그대로 보인다.** `hcr5_bridge` 를 두 번 옮기고도
  `compose.yml` 을 안 고친 이유다(2026-08-15, 르누아르·앵그르 실측)
- `package.xml` 이 없는 디렉터리(`hcr5_mqtt`·`hcr5_measurements`·`ros2_control_hw_interface`)는
  colcon 이 **자동으로 건너뛴다** — 빌드에 안 섞인다
- `hcr5_mqtt/tools/` 가 컨테이너에서 보이므로 **`docker cp` 우회가 필요 없다**

⚠️ **`compose.yml` 을 고치면 컨테이너를 재생성해야 붙는다.** 재시작만으로는 마운트가 안 바뀐다:
```bash
./run_container.sh down && ./run_container.sh shell
```

### GPU — `compose.override.yml`

모듈별 GPU 설정은 **PC 로컬 자산이라 git 에 안 올린다**. 공용 `compose.yml` 에는 GPU 설정을 넣지 않는다
(팀 컨벤션, `src/moveit2/README.md` §2). override 없이 띄우면 RViz2 3D 가 소프트웨어 렌더링으로 느려진다 —
동작은 하므로 환경 확인용으로는 쓸 수 있다.

---

## 4. 경계 함정 — 호스트에서 컨테이너 명령을 쏠 때 ⚠️

`운전절차.md` §5 함정 ①② 의 원본이다. **둘 다 여기서 난다.**

### ① `bash -lc` 는 ROS 환경을 안 물고 온다

```bash
docker exec moveit2_dev bash -lc "ros2 launch …"     # ✗ ros2: command not found
docker exec moveit2_dev bash -lc "colcon build …"    # ✗ ament_cmake 를 못 찾는다
```

`.bashrc` 는 **대화형 셸에서만** 읽힌다. 호스트에서 쏠 때는 `source` 를 명시한다:

```bash
docker exec moveit2_dev bash -lc "source /opt/ros/jazzy/setup.bash && source ~/ws_moveit2/install/setup.bash && ros2 …"
```

빌드는 워크스페이스 setup 이 아직 없을 수 있으므로 `/opt/ros` 만 물어도 된다.
**대화형(`docker exec -it … bash`)과 섞으면 되던 것이 안 된다** — 두 형태를 한 문서에서 오갈 때 특히 걸린다.

### ② 여러 줄을 통째로 붙여넣으면 깨진다

블록 첫 줄이 `run_container.sh shell` 이나 `docker exec -it … bash` 면 **새 셸이 열리고 뒤따르는 줄들이
그 셸의 입력으로 빨려 들어간다.** 진입 줄과 다음 명령은 **따로** 붙여넣는다.

같은 이유로 이 프로젝트 문서의 명령은 **한 줄로 완결**되고 셸을 바꾸지 않는다.
함수 정의(`GO() { … }`)도 쓰지 않는다 — 그 셸에서만 살아서 창을 새로 열 때마다 다시 붙여넣어야 한다.

---

## 5. 빌드

```bash
# 컨테이너 안 (진입했으면 source 불필요)
cd ~/ws_moveit2 && flock /tmp/atlier_colcon.lock colcon build --packages-select hcr5_bridge
```

- **`flock`** — 두 세션이 동시에 빌드하면 깨진다. 이 프로젝트는 병렬 세션이 흔해서 락을 건다
- **완전 재빌드**는 `build/hcr5_bridge` `install/hcr5_bridge` 를 지우고 한다. 소스 경로가 바뀌면
  `CMakeCache.txt` 가 stale 이 되어 **반드시** 필요하다 (실측: 단일 패키지 5~8초)
- `build/` `install/` `log/` 는 git 에 안 올린다 — 지워도 재생성된다

⚠️ **실기 스택이 떠 있는 중에 재빌드하지 않는다.** 플러그인 `.so` 를 갈아끼우는 것이라 위험하다.

### 빌드 결과 확인

```bash
ros2 pkg executables hcr5_bridge          # movej · movel · servo · state_bridge_node
ros2 control list_hardware_components     # 스택이 떠 있을 때만
```

---

## 6. GUI (RViz2)

컨테이너가 호스트 X 서버에 그린다.

| 항목 | 값 | 확인 |
|---|---|---|
| 호스트 `DISPLAY` | `:1` | `echo $DISPLAY` |
| 컨테이너 `DISPLAY` | `:1` (compose 가 물려줌) | `docker exec moveit2_dev printenv DISPLAY` |
| X11 소켓 | `/tmp/.X11-unix` 마운트 | `docker exec moveit2_dev ls /tmp/.X11-unix` → `X1` |
| 접근 허용 | `LOCAL:` 항목 필요 | `xhost` |

안 뜨면 호스트에서 한 번:
```bash
xhost +local:root
```

⚠️ **컨테이너의 `DISPLAY` 는 생성 시점 값이 박힌다.** 호스트 `DISPLAY` 가 바뀌었으면(재로그인 등)
컨테이너를 재생성해야 한다.

---

## 7. 이 PC 의 값 (03pc)

| 항목 | 값 |
|---|---|
| 컨테이너 이름 | **`moveit2_dev`** (04pc 는 `markch_moveit2_dev`) |
| 이미지 | `moveit2_dev:jazzy` |
| 리포 | `~/WorkSpace_260814/Nyang_Nyang_Atlier` |
| PC IP (실기 랜) | `192.168.0.100/24` · NIC `enp131s0` |
| 실기 컨트롤러 | `192.168.0.20` · MQTT `1883` |
| GPU | RTX 5080 Laptop (NVIDIA 스니펫 override) |

**실기 랜 프로필** — 컨트롤러를 재부팅하면 링크가 끊겨 프로필이 내려간다(`autoconnect no`):
```bash
nmcli connection up hcr5 && ping -c2 192.168.0.20
```

---

## 8. 뒷정리

```bash
docker exec moveit2_dev ps -ef | grep -E "move_group|ros2_control_node|robot_state_pub|static_transform|rviz2" | grep -v grep
```

**비어 있어야 정상이다.** 종료 시 `move_group` 이 exit −11(Segfault)로 죽는 것은 알려진 정상 거동이지만
(`MoveItCpp` 소멸자), 그때 **고아 프로세스가 남을 수 있다** — `robot_state_publisher` ·
`static_transform_publisher` 가 남은 사례가 있다. 남았으면:

```bash
docker exec moveit2_dev pkill -INT -f "ros2 launch"
```

⚠️ 고아를 둔 채 재기동하면 **낡은 `/robot_description`(transient_local QoS)이 새 스택을 오염시킨다** —
검증 S1 실패 원인이 이것이었다.

---

## 9. 아직 없는 것

- **`drivers/` 전용 컨테이너가 없다.** Dockerfile·compose 는 `src/moveit2/` 에만 있고, `hcr5_bridge` 는
  그 컨테이너를 빌려 쓴다. 실행 의존은 MoveIt 에서 독립했지만(`bringup.launch.py` 는 `move_group` 을 안 띄운다)
  **컨테이너는 아직 독립하지 않았다.** 독립 컨테이너를 세우려면 `hanwha_robot_arm/` 매핑(§3 `:66`)이
  따라와야 한다 — `hcr_robot_description` 이 URDF 원본이기 때문이다.
  → 협업일지 **E9**, 최종결과물 '남은 것'
