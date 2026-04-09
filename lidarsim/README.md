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

---

## 라인별 코드 설명

### `main.cpp`

```cpp
#include "lidarsim/lidarsim.hpp"          // ObstacleAvoidance 클래스 헤더 포함
```

```cpp
int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);             // ROS 2 런타임 초기화 (argc/argv로 ROS 인자 파싱)
    auto node = make_shared<ObstacleAvoidance>(); // ObstacleAvoidance 노드를 힙에 생성 (shared_ptr)
    rclcpp::spin(node);                   // 노드 실행 루프 시작 - 콜백이 이 안에서 처리됨
    rclcpp::shutdown();                   // spin 종료 후 ROS 2 정리
    return 0;
}
```

---

### `lidarsim.hpp`

```cpp
#ifndef SUB_HPP_
#define SUB_HPP_                          // 헤더 중복 포함 방지 (include guard)

#include "rclcpp/rclcpp.hpp"             // ROS 2 C++ 클라이언트 라이브러리
#include "sensor_msgs/msg/laser_scan.hpp"        // LiDAR 메시지 타입
#include "sensor_msgs/msg/compressed_image.hpp"  // 압축 카메라 이미지 메시지 타입
#include "geometry_msgs/msg/vector3.hpp"         // Vector3 (x, y, z) 메시지 타입
#include "opencv2/opencv.hpp"            // OpenCV 헤더 (영상 처리 및 시각화)
#include <fcntl.h>                       // fcntl() - 파일 디스크립터 제어 (O_NONBLOCK)
#include <termios.h>                     // termios - 터미널 입출력 설정 (Raw 모드)
#define STDIN_FILENO 0                   // 표준 입력 파일 디스크립터 번호 명시적 정의
```

```cpp
class ObstacleAvoidance : public rclcpp::Node { // rclcpp::Node를 상속해 ROS 2 노드로 동작
public:
    ObstacleAvoidance();                 // 생성자: 구독자/발행자 초기화

private:
    void scan_callback(...);             // /scan 토픽 수신 시 호출되는 콜백
    void image_callback(...);            // image/topic 수신 시 호출되는 콜백

    int getch(void);                     // 엔터 없이 한 글자 즉시 읽기
    bool kbhit(void);                    // 키 입력 여부 논블로킹 확인

    rclcpp::Subscription<LaserScan>::SharedPtr scan_sub_;        // LiDAR 구독자
    rclcpp::Subscription<CompressedImage>::SharedPtr img_sub_;   // 카메라 구독자
    rclcpp::Publisher<Vector3>::SharedPtr dxl_pub_;              // 모터 속도 발행자

    bool mode = false;                   // false=정지, true=자율주행 모드
    geometry_msgs::msg::Vector3 vel;     // 발행할 속도 명령 (x=좌모터, y=우모터)
    double k = 0.1;                      // P제어 비례 상수
};
```

---

### `lidarsim.cpp` - 생성자

```cpp
ObstacleAvoidance::ObstacleAvoidance() : Node("lidarsim_node") {
    // 노드 이름을 "lidarsim_node"로 지정하며 Node 초기화
    vel.x = 0; vel.y = 0; vel.z = 0;   // 초기 속도 명령 0으로 설정

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // QoS: 최신 10개 메시지만 버퍼에 유지하는 프로파일

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan",                         // 구독할 토픽 이름
        qos_profile,
        bind(&ObstacleAvoidance::scan_callback, this, placeholders::_1));
    // /scan 토픽에 메시지가 오면 scan_callback을 호출하도록 바인딩

    img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/topic",
        qos_profile,
        bind(&ObstacleAvoidance::image_callback, this, placeholders::_1));
    // image/topic 토픽 구독, image_callback 바인딩

    dxl_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "vel_cmd/topic", qos_profile);
    // vel_cmd/topic 으로 모터 속도 명령을 발행할 발행자 생성
}
```

---

### `lidarsim.cpp` - `getch` / `kbhit`

```cpp
int ObstacleAvoidance::getch(void) {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);      // 현재 터미널 설정을 oldt에 백업
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);   // Raw 모드: 줄 단위 입력(ICANON) 및 에코(ECHO) 비활성화
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 변경된 설정 즉시 적용
    int ch = getchar();                  // 엔터 없이 한 글자 바로 읽기
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 원래 터미널 설정 복원
    return ch;
}

bool ObstacleAvoidance::kbhit(void) {
    struct termios oldt, newt;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);   // Raw 모드 설정
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    int oldf = fcntl(STDIN_FILENO, F_GETFL, 0);         // 현재 파일 플래그 저장
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);    // 논블로킹 모드로 전환
    int ch = getchar();                  // 즉시 반환: 키 없으면 EOF(-1) 반환
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);             // 터미널 설정 복원
    fcntl(STDIN_FILENO, F_SETFL, oldf);                  // 파일 플래그 복원

    if (ch != EOF) {
        ungetc(ch, stdin);               // 읽은 문자를 버퍼에 되돌려 getch()가 다시 읽을 수 있게 함
        return true;                     // 키 입력 있음
    }
    return false;                        // 키 입력 없음
}
```

---

### `lidarsim.cpp` - `scan_callback`

```cpp
void ObstacleAvoidance::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    // ── 시각화 캔버스 초기화 ──────────────────────────────────────
    cv::Mat result(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));
    // 500×500 픽셀, 3채널(BGR), 흰색(255,255,255)으로 초기화

    cv::Point robot_pos(250, 250);
    cv::circle(result, robot_pos, 5, cv::Scalar(255, 0, 0), -1);
    // 로봇 위치를 화면 중앙에 파란 원(반지름 5, 채움)으로 표시

    cv::circle(result, robot_pos, 250, cv::Scalar(200, 200, 200), 1);
    // 2m 탐지 반경을 회색 원으로 표시 (250px = 2m)
    cv::circle(result, robot_pos, 125, cv::Scalar(220, 220, 220), 1);
    // 1m 보조 원 표시 (125px = 1m)

    // ── 좌/우 최단 장애물 추적 변수 초기화 ──────────────────────
    double L_dist = 1000.0;  bool L_found = false; double L_angle_deg = 0.0;
    double R_dist = 1000.0;  bool R_found = false; double R_angle_deg = 0.0;
    // 1000.0은 충분히 큰 초기값 (실질적 무한대)

    // ── 전체 스캔 포인트 순회 ────────────────────────────────────
    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double r = msg->ranges[i];

        if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max)
            continue;
        // 무효값(inf, nan) 또는 센서 유효 범위 밖의 데이터 스킵

        if (r > 2.0) continue;
        // 2m 초과 장애물은 무시 (관심 영역 외)

        double theta = msg->angle_min + i * msg->angle_increment;
        // i번째 스캔의 절대 각도 계산 (라디안)

        theta += M_PI;
        // 라이다 물리적 정면(180°)을 코드상 0°(전방)으로 보정

        while (theta > M_PI)  theta -= 2.0 * M_PI;
        while (theta <= -M_PI) theta += 2.0 * M_PI;
        // 각도를 [-π, π] 범위로 정규화 (wrapping)

        double theta_deg = theta * 180.0 / M_PI;  // 라디안 → 도(°) 변환

        // ── 좌/우 영역별 최단거리 장애물 탐색 ──────────────────
        if (theta_deg >= 0 && theta_deg <= 120) {   // 좌측 120° 영역
            if (r < L_dist) { L_dist = r; L_angle_deg = theta_deg; L_found = true; }
        } else if (theta_deg >= -120 && theta_deg < 0) { // 우측 120° 영역
            if (r < R_dist) { R_dist = r; R_angle_deg = theta_deg; R_found = true; }
        }

        // ── 스캔 포인트 픽셀 좌표 변환 및 시각화 ───────────────
        int r_px  = static_cast<int>(r * 125.0);   // 거리(m) → 픽셀 (1m=125px)
        int img_x = 250 - r_px * sin(theta);        // x축: 좌측 방향이 화면 왼쪽
        int img_y = 250 - r_px * cos(theta);        // y축: 전방이 화면 위쪽 (y 감소)

        if (img_x >= 0 && img_x < 500 && img_y >= 0 && img_y < 500)
            cv::circle(result, cv::Point(img_x, img_y), 2, cv::Scalar(0, 0, 0), -1);
        // 유효 범위 내 포인트만 검정 점(반지름 2)으로 표시
    }

    // ── 조향 오차(error) 계산 ─────────────────────────────────
    int error = 0;
    if (L_found && R_found) {
        error = -static_cast<int>((L_angle_deg + R_angle_deg) / 2.0);
        // 양쪽 장애물 각도의 평균 방향으로 주행 (부호 반전: 오차→조향방향)
    } else if (L_found && !R_found) {
        error = static_cast<int>((90.0 - L_angle_deg) / 2.0);
        // 우측 개방: 가상 우측 장애물을 -90°에 배치해 우회전 유도
    } else if (!L_found && R_found) {
        error = -static_cast<int>((90.0 + R_angle_deg) / 2.0);
        // 좌측 개방: 가상 좌측 장애물을 90°에 배치해 좌회전 유도
    } else {
        error = 0;  // 장애물 없음: 직진
    }

    // ── 키보드 입력으로 모드 전환 ─────────────────────────────
    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false;   // q: 정지 모드
        else if(ch == 's') mode = true; // s: 자율주행 시작
    }

    // ── 모터 속도 계산 및 발행 ────────────────────────────────
    if (mode) {
        vel.x =  50 + k * error;   // 좌측 모터: 기본속도 + 비례보정
        vel.y = -(50 - k * error); // 우측 모터: 부호 반전 (모터 장착 방향 반대)
    } else {
        vel.x = 0; vel.y = 0;      // 정지 모드: 속도 0
    }

    dxl_pub_->publish(vel);
    // vel_cmd/topic 으로 속도 명령 발행

    RCLCPP_INFO(this->get_logger(), "Error: %d, leftvel: %.2f, rightvel: %.2f",
                error, vel.x, vel.y);
    // 터미널에 현재 오차 및 좌우 속도 로그 출력

    // ── 좌측 최단 장애물 시각화 (녹색) ──────────────────────
    if (L_found) {
        int r_px = static_cast<int>(L_dist * 125.0);
        double theta = L_angle_deg * M_PI / 180.0;
        int img_x = 250 - r_px * sin(theta);
        int img_y = 250 - r_px * cos(theta);
        cv::circle(result, cv::Point(img_x, img_y), 6, cv::Scalar(0, 255, 0), -1);
        cv::putText(result, "L", cv::Point(img_x+10, img_y-10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
        // 녹색 원(반지름 6) + "L" 텍스트 표시
    }

    // ── 우측 최단 장애물 시각화 (빨간색) ─────────────────────
    if (R_found) {
        int r_px = static_cast<int>(R_dist * 125.0);
        double theta = R_angle_deg * M_PI / 180.0;
        int img_x = 250 - r_px * sin(theta);
        int img_y = 250 - r_px * cos(theta);
        cv::circle(result, cv::Point(img_x, img_y), 6, cv::Scalar(0, 0, 255), -1);
        cv::putText(result, "R", cv::Point(img_x+10, img_y-10),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);
        // 빨간 원(반지름 6) + "R" 텍스트 표시
    }

    // ── 목표 조향 방향 화살표 시각화 ────────────────────────
    double target_theta = -error * M_PI / 180.0;
    // error를 반전해 실제 주행 방향 각도로 변환
    int t_img_x = 250 - 150 * sin(target_theta);
    int t_img_y = 250 - 150 * cos(target_theta);
    cv::arrowedLine(result, robot_pos, cv::Point(t_img_x, t_img_y),
                    cv::Scalar(255, 0, 0), 2);
    // 로봇 중심에서 목표 방향으로 파란 화살표 그리기 (길이 150px)

    cv::imshow("Lidar Scan View", result);  // 시각화 창 갱신
    cv::waitKey(1);                          // 1ms 대기 (GUI 이벤트 처리)
}
```

---

### `lidarsim.cpp` - `image_callback`

```cpp
void ObstacleAvoidance::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    // msg->data (압축된 JPEG/PNG 바이트 배열)를 BGR 컬러 이미지로 디코딩

    if (!frame.empty()) {
        cv::imshow("Camera View", frame);  // "Camera View" 창에 표시
        cv::waitKey(1);                    // 1ms 대기 (GUI 이벤트 처리)
    }
    // frame.empty()는 디코딩 실패 시 true → 실패한 프레임은 표시하지 않음
}
```
