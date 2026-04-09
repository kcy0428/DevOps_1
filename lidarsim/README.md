
# lidarsim - LiDAR 기반 장애물 회피 노드

ROS 2 (Jazzy) 패키지로, LiDAR 센서 데이터를 활용해 장애물을 감지하고 Dynamixel 모터로 자율 회피 주행하는 노드입니다.

---

## 패키지 구조

```
lidarsim/
├── CMakeLists.txt          # 빌드 설정
├── include/
│   └── lidarsim/
│       └── lidarsim.hpp    # ObstacleAvoidance 클래스 선언
└── src/
    ├── lidarsim.cpp        # 핵심 로직 구현
    └── main.cpp            # 노드 진입점
```

---

## 동작 흐름

```
[LiDAR /scan]           [Camera image/topic]
      │                         │
      ▼                         ▼
 scan_callback()         image_callback()
      │                         │
      │  (LaserScan 분석)        │  (CompressedImage 디코딩)
      │                         │
      ▼                         ▼
 장애물 감지 & 오차 계산     OpenCV로 카메라 영상 표시
      │
      ▼
 비례 제어 (P-Control)
      │
      ▼
 [vel_cmd/topic] → Dynamixel 모터 속도 명령 발행
```

---

## 핵심 알고리즘 상세

### 1. LiDAR 스캔 처리 (`scan_callback`)

#### 좌표계 보정
LiDAR의 물리적 정면(180°)을 화면 12시(0°) 방향으로 맞추기 위해 **+π(180°) 보정**을 적용하고, 각도를 `[-π, π]` 범위로 정규화합니다.

```
보정 후 각도 기준:
         0° (전방)
          │
270°──────┼──────90°
          │
        180°
```

#### 탐색 영역 분할

| 영역 | 각도 범위 | 변수 |
|------|-----------|------|
| 좌측 | 0° ~ 120° | `L_dist`, `L_angle_deg` |
| 우측 | -120° ~ 0° | `R_dist`, `R_angle_deg` |

- 2m 이내의 장애물만 대상으로 처리
- 각 영역에서 **최단거리 장애물 하나**만 추출

> 전방 180°가 아닌 **±120° 범위**를 사용하는 이유:
> 라이다가 살짝 기울어졌을 때 벽의 수직 최단거리 지점을 놓치는 편향을 방지하기 위해 여유 버퍼를 둡니다.

---

### 2. 오차(Error) 계산

좌우 장애물의 각도 평균을 기반으로 조향 오차를 계산합니다.

| 상태 | 계산식 | 의미 |
|------|--------|------|
| 양쪽 모두 감지 | `error = -(L_angle + R_angle) / 2` | 두 장애물의 중간으로 주행 |
| 좌측만 감지 | `error = (90 - L_angle) / 2` | 우측이 뚫려있으므로 우회전 |
| 우측만 감지 | `error = -(90 + R_angle) / 2` | 좌측이 뚫려있으므로 좌회전 |
| 장애물 없음 | `error = 0` | 직진 |

- `error > 0` → 우회전 (좌측 장애물이 더 가까움)
- `error < 0` → 좌회전 (우측 장애물이 더 가까움)

---

### 3. 비례 제어 (P-Control)

```
vel.x =  50 + k * error    ← 좌측 Dynamixel 속도
vel.y = -(50 - k * error)  ← 우측 Dynamixel 속도 (부호 반전: 모터 방향)
```

- 기본 속도: `50` (직진)
- 비례 상수: `k = 0.1`
- 차동 구동(Differential Drive) 방식으로 좌우 바퀴 속도 차이로 회전

---

### 4. 키보드 제어

터미널에서 논블로킹 방식으로 키 입력을 감지합니다.

| 키 | 동작 |
|----|------|
| `s` | 자율 주행 시작 (`mode = true`) |
| `q` | 정지 (`mode = false`) |

---

### 5. OpenCV 시각화

500×500 픽셀 창에 LiDAR 스캔 결과를 실시간으로 표시합니다.

| 표시 요소 | 색상 | 설명 |
|-----------|------|------|
| 로봇 위치 | 파란 점 | 화면 중앙 (250, 250) |
| 스캔 포인트 | 검정 점 | 감지된 모든 장애물 |
| 좌측 최단 장애물 | 녹색 원 + "L" | 좌측에서 가장 가까운 지점 |
| 우측 최단 장애물 | 빨간 원 + "R" | 우측에서 가장 가까운 지점 |
| 조향 방향 화살표 | 파란 화살표 | 계산된 목표 주행 방향 |
| 2m 기준 원 | 회색 실선 | 탐지 반경 기준선 |
| 1m 보조 원 | 연회색 실선 | 거리 참고선 |

> 스케일: 1m = 125픽셀, 2m = 250픽셀

카메라 영상은 별도의 `"Camera View"` 창에 표시됩니다.

---

## ROS 2 토픽

| 토픽 | 방향 | 타입 | 설명 |
|------|------|------|------|
| `/scan` | Subscribe | `sensor_msgs/LaserScan` | LiDAR 스캔 데이터 |
| `image/topic` | Subscribe | `sensor_msgs/CompressedImage` | 카메라 압축 이미지 |
| `vel_cmd/topic` | Publish | `geometry_msgs/Vector3` | 모터 속도 명령 (`x`: 좌, `y`: 우) |

QoS: `KeepLast(10)`

---

## 빌드 및 실행

```bash
# 패키지 빌드
cd ~/ros2_ws
colcon build --packages-select lidarsim

# 환경 설정
source install/setup.bash

# 노드 실행
ros2 run lidarsim lidarsim_node
```

---

## 의존성

- ROS 2 Jazzy
- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `std_msgs`
- `OpenCV`
