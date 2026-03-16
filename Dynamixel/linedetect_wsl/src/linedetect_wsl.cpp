#include "linedetect_wsl/sub.hpp"
#include <chrono>

using namespace std;
using std::placeholders::_1;

LineDetector::LineDetector()
: Node("linedetect_wsl"), first_run_(true), tmp_pt_(320, 60), running_(false)
{
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));

    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed",
        qos_profile,
        std::bind(&LineDetector::image_callback, this, _1));

    pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("topic_dxlpub", qos_profile);

    // 키보드 입력 감지 타이머 (100ms 주기)
    key_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        std::bind(&LineDetector::key_timer_callback, this));

    RCLCPP_INFO(this->get_logger(), "LineDetector started. Press 's': start, 'q': stop(0rpm)");
}

bool LineDetector::kbhit()
{
    struct termios oldt, newt;
    int ch, oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF) {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

int LineDetector::getch()
{
    struct termios oldt, newt;
    int ch;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    return ch;
}

void LineDetector::key_timer_callback()
{
    if (!kbhit()) return;
    int key = getch();

    if (key == 's' || key == 'S') {
        running_ = true;
        RCLCPP_INFO(this->get_logger(), "[KEY] 's' pressed -> Speed command START");
    } else if (key == 'q' || key == 'Q') {
        running_ = false;
        auto msg = geometry_msgs::msg::Vector3();
        msg.x = 0.0;
        msg.y = 0.0;
        pub_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "[KEY] 'q' pressed -> STOP (0 rpm published)");
    }
}

void LineDetector::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto startTime = chrono::steady_clock::now();

    // 이미지 디코딩
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame_color.empty()) {
        RCLCPP_ERROR(this->get_logger(), "Frame decode error");
        return;
    }

    // 전처리: Gray -> 밝기 보정 -> 이진화
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);
    cv::Scalar bright_avg = cv::mean(frame_gray);
    frame_gray = frame_gray + (100 - bright_avg[0]);
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);

    // ROI: 하단 1/3 영역
    cv::Mat roi = frame_binary(cv::Rect(0, 240, 640, 120));

    // 연결 성분 분석
    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
    int cnt = stats.rows;
    int target_idx = -1;

    // 이전 위치 기준 가장 가까운 라인 탐색
    int min_idx = -1;
    int min_dist = 10000;
    int search_radius = first_run_ ? roi.cols : 60;
    cv::Point search_center = first_run_ ? cv::Point(320, 60) : tmp_pt_;

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int dist = (int)cv::norm(cv::Point(x, y) - search_center);
            if (dist < min_dist && dist <= search_radius) {
                min_dist = dist;
                min_idx = i;
            }
        }
    }

    if (min_idx != -1) {
        tmp_pt_ = cv::Point(cvRound(centroids.at<double>(min_idx, 0)),
                            cvRound(centroids.at<double>(min_idx, 1)));
        target_idx = min_idx;
        first_run_ = false;
    }

    // 위치 오차 계산 (화면 중앙 320 기준)
    double error = 320.0 - tmp_pt_.x;

    // 속도 명령 계산 (P 제어)
    // error > 0: 라인이 왼쪽 -> 왼쪽으로 회전 -> 우모터 빠르게
    // error < 0: 라인이 오른쪽 -> 오른쪽으로 회전 -> 좌모터 빠르게
    double v_left  = STRAIGHT_RPM - GAIN_K * error;
    double v_right = STRAIGHT_RPM + GAIN_K * error;

    // 속도 명령 퍼블리시 (s키 눌렀을 때만)
    if (running_) {
        auto vel_msg = geometry_msgs::msg::Vector3();
        vel_msg.x = v_left;
        vel_msg.y = v_right;
        pub_->publish(vel_msg);
    }

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    // 터미널 출력: 오차, 좌속도, 우속도, 처리시간
    RCLCPP_INFO(this->get_logger(),
        "[%s] err:%.0f, L:%.1f rpm, R:%.1f rpm, time:%.2f ms",
        running_ ? "RUN" : "IDLE", error, v_left, v_right, totalTime);

    // 시각화
    cv::Mat result_view = roi.clone();
    cv::cvtColor(result_view, result_view, cv::COLOR_GRAY2BGR);

    // 타겟 라인 = 빨강, 나머지 인식된 성분 = 파랑
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int left   = stats.at<int>(i, 0);
            int top    = stats.at<int>(i, 1);
            int width  = stats.at<int>(i, 2);
            int height = stats.at<int>(i, 3);
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            cv::Scalar color = (i == target_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::rectangle(result_view, cv::Rect(left, top, width, height), color, 2);
            cv::circle(result_view, cv::Point(x, y), 5, color, -1);
        }
    }

    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
}
