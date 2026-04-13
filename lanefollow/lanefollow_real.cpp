#include "lanefollow_real/lanefollow_real.hpp"

// kbhit implementation (dxl_wsl)
int getch(void)
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

bool kbhit(void)
{
    struct termios oldt, newt;
    int ch;
    int oldf;
    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &newt);
    oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
    fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
    ch = getchar();
    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
    fcntl(STDIN_FILENO, F_SETFL, oldf);
    if (ch != EOF)
    {
        ungetc(ch, stdin);
        return true;
    }
    return false;
}

LaneFollowProcessor::LaneFollowProcessor() : Node("lanefollow_node"), mode_(false), k_(0.13), base_vel_(150), running_(true) {
    this->declare_parameter("k", 0.14);
    this->declare_parameter("base_vel", 120);
        
    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();
    auto sub_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(10));

    raw_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/topic", sub_qos,
        std::bind(&LaneFollowProcessor::image_callback, this, std::placeholders::_1));

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>("vel_cmd/topic", pub_qos);

    // 초기 영상 너비: 보통 640을 기준으로 함, x좌표 1/4(160), 3/4(480) 지점
    last_left_x_ = 160.0;
    last_right_x_ = 480.0;
    last_line_y_ = 45.0; // ROI 내부 초기 기준
    
    RCLCPP_INFO(this->get_logger(), "이중 라인 추적 노드 시작. 's': 주행, 'q': 정지");

    key_thread_ = std::thread(&LaneFollowProcessor::keyboardLoop, this);
}

LaneFollowProcessor::~LaneFollowProcessor() {
    running_ = false;
    if (key_thread_.joinable()) key_thread_.join();
}

cv::Mat LaneFollowProcessor::setROI(cv::Mat &frame) {
    // 하단 1/4
    cv::Rect roi_rect(0, frame.rows * 3 / 4, frame.cols, frame.rows / 4);
    cv::Mat roi = frame(roi_rect).clone();

    cv::cvtColor(roi, roi, cv::COLOR_BGR2GRAY);
    roi += cv::Scalar(100) - cv::mean(roi); 
    cv::threshold(roi, roi, 150, 255, cv::THRESH_BINARY);
    
    return roi;
}

// sub_roi 기반 라인 탐색 및 결과 상대좌표 리턴, 찾지못하면 기존 target 유지(-1)
int LaneFollowProcessor::findLine(cv::Mat &bin_roi, cv::Mat &stats, cv::Mat &centroids, double &target_x) {
    cv::Mat labels;
    int n_labels = cv::connectedComponentsWithStats(bin_roi, labels, stats, centroids);

    int min_index = -1;
    double min_dist = static_cast<double>(bin_roi.cols * 2.0); 

    for (int i = 1; i < n_labels; i++) {
        int area = stats.at<int>(i, cv::CC_STAT_AREA);
        if (area > 100) {
            double cx = centroids.at<double>(i, 0);
            double cy = centroids.at<double>(i, 1);
            double dist = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(target_x, last_line_y_));

            if (dist < min_dist && dist <= 150.0) {
                min_dist = dist;
                min_index = i;
            }
        }
    }

    if (min_index != -1 && min_dist <= 150.0) {
        target_x = centroids.at<double>(min_index, 0);
    }

    int idx = -1;
    double best = static_cast<double>(bin_roi.cols * 2.0);
    
    for (int i = 1; i < stats.rows; i++) {
        double cx = centroids.at<double>(i, 0);
        double cy = centroids.at<double>(i, 1);
        double d = cv::norm(cv::Point2d(cx, cy) - cv::Point2d(target_x, last_line_y_));

        if (d < best) {
            best = d;
            idx = i;
        }
    }

    // 거리가 30픽셀 밖이면 점프한것으로 간주
    if (best > 30.0) {
        idx = -1;
    }
    return idx;
}

void LaneFollowProcessor::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg) {
    auto startTime = std::chrono::steady_clock::now();
    k_ = this->get_parameter("k").as_double();
    base_vel_ = this->get_parameter("base_vel").as_int();

    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (frame.empty()) return;

    // 1. 전체 관심영역 추출
    cv::Mat bin_roi = setROI(frame);
    int half_w = bin_roi.cols / 2;

    // 2. 좌/우 분할
    cv::Rect left_rect(0, 0, half_w, bin_roi.rows);
    cv::Rect right_rect(half_w, 0, half_w, bin_roi.rows);
    cv::Mat left_roi = bin_roi(left_rect);
    cv::Mat right_roi = bin_roi(right_rect);

    // 3. 좌측 라인 탐색
    cv::Mat l_stats, l_cents;
    double rel_left_x = last_left_x_; 
    int l_idx = findLine(left_roi, l_stats, l_cents, rel_left_x);
    if (l_idx != -1) last_left_x_ = rel_left_x; 

    // 4. 우측 라인 탐색 (기준점을 좌측 좌표계에 맡게 뺌)
    cv::Mat r_stats, r_cents;
    double rel_right_x = last_right_x_ - half_w;
    int r_idx = findLine(right_roi, r_stats, r_cents, rel_right_x);
    // 검출결과(rel_right_x)를 원래 화면비율로 덧셈하여 보정
    if (r_idx != -1) last_right_x_ = rel_right_x + half_w;

    // 5. 시각화 (Bounding Boxes 및 위치점 드로잉)
    cv::Mat display;
    cv::cvtColor(bin_roi, display, cv::COLOR_GRAY2BGR);

    // 좌측 라인 표시(빨간색)
    if (l_idx != -1) {
        int l = l_stats.at<int>(l_idx, 0), t = l_stats.at<int>(l_idx, 1);
        int w = l_stats.at<int>(l_idx, 2), h = l_stats.at<int>(l_idx, 3);
        cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(0, 0, 255), 2);
    }
    // 우측 라인 표시(초록색), offset 추가
    if (r_idx != -1) {
        int l = r_stats.at<int>(r_idx, 0) + half_w, t = r_stats.at<int>(r_idx, 1);
        int w = r_stats.at<int>(r_idx, 2), h = r_stats.at<int>(r_idx, 3);
        cv::rectangle(display, cv::Rect(l, t, w, h), cv::Scalar(0, 255, 0), 2);
    }
    // 절반 분할 선(하늘색) 및 좌/우 포인트 좌표점(빨강, 초록)
    cv::line(display, cv::Point(half_w, 0), cv::Point(half_w, display.rows), cv::Scalar(255, 255, 0), 1);
    cv::circle(display, cv::Point(static_cast<int>(last_left_x_), static_cast<int>(last_line_y_)), 3, cv::Scalar(0, 0, 255), -1);
    cv::circle(display, cv::Point(static_cast<int>(last_right_x_), static_cast<int>(last_line_y_)), 3, cv::Scalar(0, 255, 0), -1);

    // 6. 오차(error) 및 방향명령 생성
    double center_pt = (last_left_x_ + last_right_x_) / 2.0;
    double error = (bin_roi.cols / 2.0) - center_pt; 
    
    geometry_msgs::msg::Vector3 vel_msg;
    if (mode_) {
        vel_msg.x = base_vel_ - error * k_;
        vel_msg.y = -(base_vel_ + error * k_);
    } else { // 'q' 정지상태
        vel_msg.x = 0; vel_msg.y = 0;
    }
    vel_pub_->publish(vel_msg);

    auto endTime = std::chrono::steady_clock::now();
    float totalTime = std::chrono::duration<float, std::milli>(endTime - startTime).count();
    RCLCPP_INFO(this->get_logger(), "Err:%.2lf L(%.0f) R(%.0f) lV:%.1f rV:%.1f ms:%.1f", 
                error, last_left_x_, last_right_x_, vel_msg.x, vel_msg.y, totalTime);

    cv::imshow("1. LaneFollow Raw", frame);
    cv::imshow("2. Dual-Line Debug", display);
    cv::waitKey(1);
}

// dxl_wsl 패키지의 kbhit, getch 사용
void LaneFollowProcessor::keyboardLoop() {
    while (running_) {
        if (kbhit()) {
            char ch = getch();
            if (ch == 'q') { 
                mode_ = false; 
                RCLCPP_WARN(this->get_logger(), "STOP (q pressed)"); 
            }
            else if (ch == 's') { 
                mode_ = true; 
                RCLCPP_INFO(this->get_logger(), "START (s pressed)"); 
            }
        }
        // CPU 부하 방지
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}
