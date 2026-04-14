# human view
```
https://www.youtube.com/shorts/EGbEtkZMm98
```
# 실행 영상
```
https://github.com/user-attachments/assets/b7a5bc31-e3da-4113-af54-26bf848c2df9
```

# robot view
```
https://youtu.be/oqEtx10QnHs
```
# 블럭도
```
<img width="1621" height="1353" alt="linetracer_2블럭도 drawio" src="https://github.com/user-attachments/assets/8c695ede-3636-4d2a-9990-1d1c5dd5ca21" />
```

## 1. 헤더 선언부: `include/linetracer_real/linetracer_real.hpp`
구현부와 설계를 분리하여 외부에서 이 노드 구조를 참고할 수 있도록 클래스와 인터페이스를 선언한 헤더 파일입니다.

```cpp
#ifndef LINETRACER_REAL_HPP_  // 헤더 가드: 동일한 헤더가 여러 번 포함되어 발생하는 재정의 오류를 방지합니다.
#define LINETRACER_REAL_HPP_

#include "rclcpp/rclcpp.hpp"  // ROS 2 C++ 핵심 클라이언트 라이브러리 포함
#include "sensor_msgs/msg/compressed_image.hpp"  // ROS 2 압축 이미지 메시지 타입 포함
#include "geometry_msgs/msg/vector3.hpp"  // 모터 속도 지시를 주고받기 위한 3차원 벡터(xyz) 메시지 포함
#include "cv_bridge/cv_bridge.hpp"  // ROS 이미지 메시지와 시스템의 OpenCV 이미지 간의 형 변환 유틸리티
#include "opencv2/opencv.hpp"  // 이미지 필터링 및 시각을 처리하는 OpenCV 라이브러리 코어 포함
#include <cmath>      // 거리 계산(유클리디안) 등 수학 함수 사용을 위한 표준 라이브러리
#include <chrono>     // 처리 시간 확인 및 FPS 측정을 위한 시간 라이브러리
#include <termios.h>  // 터미널 I/O 속성을 제어하여 비동기 키보드 입력을 처리하기 위한 헤더
#include <fcntl.h>    // 파일 제어 옵션을 조정하기 위한 라이브러리
#include <thread>     // 비동기 루프를 통해 이미지가 멈추지 않도록 처리하는 멀티스레드 지원
#include <atomic>     // 복수의 스레드 사이에서 데이터를 안전하게 교환하기 위한 원자적 변수 템플릿

// LineTrackerProcessor 클래스 선언 (ROS 2 Node 클래스를 상속받음)
class LineTrackerProcessor : public rclcpp::Node {
public:
    LineTrackerProcessor();   // 생성자 선언: 초기화 및 파라미터 셋업 담당
    ~LineTrackerProcessor();  // 소멸자 선언: 종료 시 쓰레드 해제 및 리소스 반환 담당

private:
    cv::Mat setROI(cv::Mat &frame);  // 관심 영역(ROI)을 잘라내어 시인성을 높이고 이진화 처리하는 전처리 함수
    int findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids);  // 라인 블랍(blob)을 탐색 후 유효한 트레이싱 인덱스를 반환하는 함수
    cv::Mat drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx);  // 이진화 화면 위에 처리 결과(사각형 등)를 그려 반환하는 시각화 함수
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg); // 압축 이미지가 들어올 때마다 호출되는 콜백 함수
    void keyboardLoop();  // 메인 스레드에 엮이지 않고 독립적인 스레드에서 키보드(s:시작, q:정지) 입력을 폴링하는 함수

    double last_line_x_, last_line_y_;  // 이전에 포착된 라인의 최종 X, Y 중심 좌표. 추적의 기준점이 됩니다.
    std::atomic<bool> mode_;            // 라인트레이서의 로봇 주행 모드 플래그(true: 동력인가, false: 정지)
    double k_;                          // 에러 편차에 대응하기 위한 조향 비례 제어(Proportional) 게인 계수 상수
    int base_vel_;                      // 오차가 없을 시 모터에 전달할 100% 직진 주행 기본 속도
    cv::Mat labels_;                    // Connected Components 알고리즘에서 각 픽셀 덩어리를 구별할 라벨 배열
    std::atomic<bool> running_;         // 노드가 아직 강제종료되지 않고 실행중인지 파악하는 상태 플래그
    std::thread key_thread_;            // 키보드 입력 전담용 백그라운드 스레드 핸들
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr raw_sub_; // 라즈베리파이에서 넘어오는 영상 토픽 구독자
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr vel_pub_;          // 분석 완료 후, 로봇 주행 속도 명령 토픽 발행자
};

#endif // LINETRACER_REAL_HPP_  // 헤더 가드 종료
```

---

## 2. 노드 실행을 위한 엔트리 파일: `src/main.cpp`
최소한의 ROS 2 초기화 정보만 포함시켜, 컴파일 시 클래스 부분과 실행 부분을 논리적으로 분리합니다. 

```cpp
#include "rclcpp/rclcpp.hpp"  // 통신 연결을 위해 ROS 2 시스템 라이브러리 포함
#include "linetracer_real/linetracer_real.hpp"  // 분리한 라인트레이서 클래스의 헤더 파일 포함

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);  // 노드가 실행될 때 전달받은 커맨드 명령어를 통해 ROS 2 시스템 전역 초기화
    
    // LineTrackerProcessor 클래스의 인스턴스를 공유 포인터(make_shared)로 생성, 
    // 이를 spin() 함수에 넘겨 노드가 죽지 않도록 블로킹하며 지속적으로 콜백을 처리하게 만듦
    rclcpp::spin(std::make_shared<LineTrackerProcessor>()); 
    
    // Ctrl+C 등으로 종료 신호가 들어오면, 노드 자원을 깔끔하게 해제
    rclcpp::shutdown();  
    
    return 0;  // 프로세스 정상 종료를 알림
}
```

---

## 3. 클래스 본문 구현부: `src/linetracer_real.cpp`
실제 영상 필터링과 알고리즘이 동작하는 코어 구현부입니다. 

```cpp
#include "linetracer_real/linetracer_real.hpp"  // 헤더 파일을 참조하여 선언된 메소드의 형태를 불러옴

// 생성자 구현. ROS 2 "line_tracker_node"라는 이름으로 초기화하고 모드 트리거, 게인 등을 초기 리스트에 세팅 
LineTrackerProcessor::LineTrackerProcessor() : Node("line_tracker_node"), mode_(false), k_(0.13), base_vel_(150), running_(true) {
    this->declare_parameter("k", 0.14);          // 동적 ROS 2 시스템 파라미터로 "k"(비례 게인)를 등록, 초기값 0.14
    this->declare_parameter("base_vel", 120);    // 동적 ROS 2 파라미터로 "base_vel"(직진 베이스속도) 등록, 기본값 120
        
    k_ = this->get_parameter("k").as_double();        // 등록된 "k" 파라미터의 실수값을 읽어 내부 멤버 변수로 동기화
    base_vel_ = this->get_parameter("base_vel").as_int(); // 등록된 기본 "base_vel" 파라미터를 읽어 내부 정수 변수로 동기화
    
    // Subscriber 품질 보증(QoS) 규칙 적용: 최근 10개만 유지하며 통신 유실(best_effort)보다 성능 위주
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    // Publisher 품질 보증(QoS) 규칙 적용: 모터 제어 명령어는 유실되면 안되므로 TCP처럼 데이터 무결을 보장(기본값 Reliable)
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    // "image/topic" 에서 날라오는 CompressedImage 토픽 구독자를 생성. 이미지 수신시 멤버 함수인 image_callback 에 msg 객체를 바인딩
    raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/topic", sub_qos,
        std::bind(&LineTrackerProcessor::image_callback, this, std::placeholders::_1));

    // 계산된 제어 데이터를 기반으로 "vel_cmd/topic" 토픽에 Vector3 속도 정보 퍼블리셔 등록
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd/topic", pub_qos);

    last_line_x_ = 320.0;  // 추적선을 못 찾았을 때를 대비한 최초 기준점 설정(640픽셀의 절반)
    last_line_y_ = 45.0;   // 탐지할 이미지 ROI 내부의 타겟 Y축 기준
    
    // 터미널 디스플레이 화면에 노드 실행 안내문을 남김
    RCLCPP_INFO(this->get_logger(), "분석 및 제어 노드 시작. 's': 주행, 'q': 정지"); 

    // 생성자가 완료될 쯤 keyboardLoop() 메소드를 독립된 스레드로 작동시켜 메인 이벤트 루프 프리징을 막음
    key_thread_ = std::thread(&LineTrackerProcessor::keyboardLoop, this);
}

// 소멸자: 노드 종료/다운 시 호출되는 클린업 영역
LineTrackerProcessor::~LineTrackerProcessor() {
    running_ = false;  // 키보드 입력을 감지하는 스레드의 while 루프의 조건을 깨기 위해 false
    if (key_thread_.joinable()) key_thread_.join(); // 작동중인 키보드 스레드의 잔업이 완전히 끝나 메인 스레드로 병합될 때까지 대기
}

// --- 1. 전처리 함수 ---
cv::Mat LineTrackerProcessor::setROI(cv::Mat &frame) {
    // 프레임 세로 높이의 3/4 지점(아래)부터 1/4 크기로 영역을 분리하는 사각형(ROI) 메타 규칙 설계
    cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
    cv::Mat roi = frame(roi_rect).clone();  // 원본 영상을 보호하며 해당 사각형만 잘라내 새로운 OpenCV 매트릭스로 복제

    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY); // 자른 컬러 영상을 흑백 그레이스케일 영상으로 단일채널화
    // ROI 화면의 평균 밝기(mean)를 구한 뒤, 베이스 밝기인 100이 되도록 화소를 가감해 조명 환경 변화를 상쇄(영상의 밝기 평균화)
    roi += cv::Scalar(100) - cv::mean(roi); 
    // 150 이라는 임계(Threshold) 수치를 이용해 화소값을 150보다 크면 완전히 하얀색(255), 낮으면 검은색(0) 이진 영상으로 반전 적용
    cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);
    
    return roi;  // 노이즈가 제거된 이진화 관심 영상을 리턴
}

// --- 2. 라인 탐색 함수 ---
int LineTrackerProcessor::findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids) {
    // OpenCV에 내장된 알고리즘을 사용하여 픽셀 덩어리들을 군집화하고 면적(stats)과 무게중심(centroids)를 추출
    int n_labels = cv::connectedComponentsWithStats(bin_roi, labels_, stats, centroids);

    int min_index = -1;  // 라인을 발견하지 못했을 때의 기본 반환값(-1)
    double min_dist = static_cast<double>(bin_roi.cols); // 최단 거리를 무한대(영상 너비)로 설정하여 갱신할 수도록 함

    // 0번 라벨은 배경(배경을 칠한 검은 면적)이므로 1번 객체부터 순회 분석
    for (int i = 1; i < n_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA); // i번째 군집화 객체의 픽셀 갯수(면적) 추출
        if (area > 100) { // 자잘한 노이즈로 튀는 오인식을 막기 위해 면적이 100 픽셀 이상인 덩어리만 라인 후보군으로 편입
            double cx = centroids.at<double>(i, 0); // 후보 덩어리의 X 중심점 위치 추출
            double cy = centroids.at<double>(i, 1); // 후보 덩어리의 Y 중심점 위치 추출
            // 현재 후보의 중심점과, 직전 루프에서 마지막으로 잡은 트랙킹 라인(last_line) 사이의 최단거리(피타고라스 수학)를 구함
            double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_));

            // 구해진 거리가 기존의 최소 거리보다 작고, 가이드 이내(150픽셀 이내의 객체)로 점프한 후보일 경우
            if (dist < min_dist && dist <= 150.0) {
                min_dist = dist;     // 가장 신뢰할 만한 짧은 거리로 기록을 단축 갱신
                min_index = i;       // 이번 루프에서 최고 확률로 라인인 객체의 라벨 인덱스 저장 
            }
        }
    }

    // [1단계 위치 갱신] 방금 150픽셀 이내에서 라인을 찾아내었다면, 중심 좌표를 그곳으로 임시 갱신
    if (min_index != -1 && min_dist <= 150.0) {
        last_line_x_ = centroids.at<double>(min_index, 0);
        last_line_y_ = centroids.at<double>(min_index, 1);
    }

    int idx = -1;
    double best = static_cast<double>(bin_roi.cols);
    
    // [2단계 확정 루프] 업데이트된 최신 위치 좌표(last_line_x_)를 기준으로 주변을 다시 검색하여 최종 객체를 고정
    for (int i = 1; i < stats.rows; i++) {
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);
        double d = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(last_line_x_, last_line_y_)); // 갱신된 지점 기준 거리 추출

        if (d < best) { 
            best = d;  // 최적 거리 기록 갱신
            idx = i;   // 최고 적합 인덱스 저장
        }
    }

    // 검증 후 선택된 객체가 업데이트 지점 대비하여 너무 거리가 멀다면 (30픽셀을 초과) 
    // 급격한 점프이므로 잘못 잡은 것으로 인정해 무시처리(-1)
    if (best > 30.0) {
        idx = -1;
    }
    return idx; // 분석이 끝난 최종 타깃 인덱스를 반환
}

// --- 3. 디버그 시각화 함수 ---
cv::Mat LineTrackerProcessor::drawResult(cv::Mat &bin_roi, cv::Mat &stats, int best_idx) {
    cv::Mat display; 
    // 단순히 이분화된 흑백(단일채널) 영상에 사각형을 색상으로 그리기 위해 3채널(BGR)로 전환하는 트릭 (이미지 모양이 바뀌진 않음)
    cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);

    for (int i = 1; i < stats.rows; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA); // 객체 면적 확인
        if (area < 100) continue; // 100 미만인 잡티는 그리지 않고 건너뜀

        // 사각형 바운딩 박스를 그리기 위한 좌표 매핑 (x시작, y시작, 폭, 높이 순)
        int l = stats.at<int>(i, 0), t = stats.at<int>(i, 1), w = stats.at<int>(i, 2), h = stats.at<int>(i, 3);
        
        if (i == best_idx) { // findLine()에서 확립시킨 최적의 타깃 인덱스일 경우 
            // 2 단위 투께의 붉은 색상으로 해당 영역을 빨간색 박스로 표시 (BGR: 0, 0, 255)
            cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(0, 0, 255), 2);
        } else { // 타깃이 아닌 기타 잡티/외곽선 사물일 경우 
            // 1 단위 두께의 파란색 박스로 영역을 표시하여 다른 객체를 구분 (BGR: 255, 0, 0)
            cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(255, 0, 0), 1);
        }
    }
    // 객체 중심이 아닌 최종적으로 추적하고 있는 가상의 포인터(last_line) 자리에 3의 지름의 빨간 점을 채워서(-1) 기름 
    cv::circle(display, cv::Point(static_cast<int>(last_line_x_), static_cast<int>(last_line_y_)), 3, cv::Scalar(0, 0, 255), -1);
    
    return display; // 각종 박스와 점이 그려진 컬러 Mat을 리턴
}

// --- 4. 중심 ROS 콜백 루틴 함수 ---
void LineTrackerProcessor::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    auto startTime = std::chrono::steady_clock::now(); // 전체 프로세싱 처리 시간(latency)을 체크하기 위한 스탑워치 시작
    
    // 동작 중 파라미터가 rqt-reconfigure 등에서 동적으로 바뀌었을 경우를 상정하여 매 프레임마다 파라미터 호출 초기화 
    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();

    // 외부 노드/라즈베리파이 등에서 넘어온 압축 Jpeg 타입 영상을 OpenCV Mat(BGR 구조)로 디코딩
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return; // 파싱 중 프레임이 깨지거나 빈 매트릭스일 경우 이번 틱을 포기하고 함수 건너뜀

    // [로직 1] 전처리: 촬영 영상을 잘라서 이진화(bin_roi) 추출
    cv::Mat bin_roi = setROI(frame);
    cv::Mat stats, centroids;
    // [로직 2] 탐지부: 전처리 영상 내에서 라인 검출하여 타겟 라벨 획득 
    int best_idx = findLine(bin_roi, stats, centroids);
    // [로직 3] 시각화: 결과 바탕 디버그 박스를 그린 컬러 모니터링 출력 이미지 리턴
    cv::Mat display = drawResult(bin_roi, stats, best_idx);

    // --- 차동 구동 모델 제어 로직 ---
    // 오차 추출 식 = ROI 카메라 너비 / 2 (즉 화면 중앙점) 과 라인 X위치 간의 픽셀 간격
    double error = (bin_roi.cols / 2.0) - last_line_x_; 
    geometry_msgs::msg::Vector3 vel_msg; // 모터 속도를 조작하기 위해 전송할 Vector3 메시지 할당

    if (mode_) { // 키보드 스레드에서 's'를 받아 주행상태(mode_ = true)가 켜져 있다면
        // 왼쪽 바퀴: 베이스 속도(base_vel) 기준, 조향오차(error)*게인(k) 만큼 빼서 속도 조절 
        vel_msg.x = base_vel_ - error * k_;
        // 오른쪽 바퀴: 반대 극성 구조거나 역회전을 상정하여 음수로 변환, 조향오차(error)*게인(k) 만큼 더해 속도 조절
        vel_msg.y = -(base_vel_ + error * k_);
    } else { // 키보드에서 'q'를 눌러 정지해 있다면
        vel_msg.x = 0; vel_msg.y = 0; // 좌/우 모터의 속도 명령값을 0으로 줘서 직진/회전 전부 중단
    }
    vel_pub_->publish(vel_msg); // 최종적으로 구축된 속도 메시지 vel_msg 세트를 ROS 환경의 vel_cmd 토픽에 발행

    auto endTime = std::chrono::steady_clock::now(); // 노드 프로세스 마무리 시간 기록
    // 스탑워치 시작/종료를 실수형 ms(밀리초) 단위로 빼서 계산
    float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    // 터미널 디버깅을 위해 에러 크기, 양쪽 인가 속도 값, 로직 소요 시간을 Info 레벨로 출력
    RCLCPP_INFO(this->get_logger(), "err:%.2lf lvel:%.2f rvel:%.2f time:%.2f", error, vel_msg.x, vel_msg.y, totalTime);

    // OpenCV GUI Window 창으로 디버깅 모니터 시각화 (가상 머신, WSL 그래픽 설정 필요)
    cv::imshow("1. Raw Video", frame);           // 첫 번째 창: 원본 비디오 출력
    cv::imshow("2. Binary Debug", display);      // 두 번째 창: 전처리 및 객체 판별이 된 이진 로직 출력
    cv::waitKey(1); // 1ms 동안 대기하여 GUI 창 이벤트 및 시스템 포커스가 정상 갱신되도록 숨고르기 양보
}

// 키보드 입력을 처리하는 전용 스레드 구현
void LineTrackerProcessor::keyboardLoop() {
    struct termios oldt, newt; 
    tcgetattr(STDIN_FILENO, &oldt); // 현재 리눅스 터미널의 입력 환경을 get 해와서 oldt 구조체에 백업
    newt = oldt;
    // 터미널 입력 속성 변경 (ICANON 해제: 매번 엔터를 누르지 않도록 만듦, ECHO 해제: 키보드 친 글씨가 화면에 올라오지 않도록 함)
    newt.c_lflag &= ~(ICANON | ECHO); 
    tcsetattr(STDIN_FILENO, TCSANOW, &newt); // 변경된 키보드 속성을 즉각(TCSANOW) 터미널에 적용해 세팅

    while (running_) { // 애플리케이션 라이프사이클이 끝날 때까지 메인 루프
        fd_set fds;
        FD_ZERO(&fds);             // 파일 디스크립터(File Descriptor) 집합 메모리 비우기
        FD_SET(STDIN_FILENO, &fds);// 감시해야 할 디스크립터 목록에 표준 입력 소켓 (키보드) 추가
        struct timeval tv = {0, 100000}; // select()가 입력을 대기할 최대 타임아웃을 100ms 지정
        
        // select 함수는 키보드 입력을 감지해 값이 있으면 0 초과의 반환값을 나타냄
        // 블로킹 없이 설정시간 내로 값이 들어올 경우에만 동작
        if (select(STDIN_FILENO + 1, &fds, nullptr, nullptr, &tv) > 0) { 
            char ch = getchar(); // 대기열에 쌓인 키를 하나 읽어 문자 객체에 삽입
            
            if (ch == 'q') {  // 'q' 버튼 입력일 경우 정지
                mode_ = false; // 플래그를 false 로 내려 메인 콜백의 속도를 0으로 연계
                RCLCPP_WARN(this->get_logger(), "STOP"); // 터미널 창에 일시정지 문구를 명시 
            }
            else if (ch == 's') { // 's' 버튼 입력일 경우 시작
                mode_ = true;  // 플래그를 true 로 올려 콜백이 오차 대비 속도 연산을 구동하도록 연계
                RCLCPP_INFO(this->get_logger(), "START"); // 터미널 창에 트레이싱 개시를 명시
            }
        }
    }

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt); // 프로세스 종료 직전 터미널 환경을 기존상태로 온전히 롤백하여 복구
}
```
