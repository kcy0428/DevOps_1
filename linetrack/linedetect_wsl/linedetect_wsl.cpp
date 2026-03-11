#include "linedetect_wsl/sub.hpp"
#include "opencv2/opencv.hpp" // 추가: cv::Mat, cvtColor 등 사용
#include <memory> 
#include <chrono> 

using namespace std;
using std::placeholders::_1;

// 생성자 (변경 없음)
LineDetector::LineDetector() : Node("campub_video"), tmp_pt_(320, 60), first_run_(true) { 
    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    // 토픽, 콜백 넣음
    sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/compressed", 
        qos_profile, 
        bind(&LineDetector::mysub_callback, this, _1));
}

void LineDetector::mysub_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    auto startTime = chrono::steady_clock::now();
    cv::Mat frame_color = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);//이미지 디코딩
    if(frame_color.empty()) return;
    cv::Mat frame_gray;
    cv::cvtColor(frame_color, frame_gray, cv::COLOR_BGR2GRAY);//bgr->gray
    cv::Scalar bright_avg = cv::mean(frame_gray); //밝기값 평균
    frame_gray = frame_gray + (100 - bright_avg[0]);//밝기값 평균
    cv::Mat frame_binary;
    cv::threshold(frame_gray, frame_binary, 130, 255, cv::THRESH_BINARY);//이진화
    cv::Mat roi = frame_binary(cv::Rect(0, 240, 640, 120));//1/4값 저장

    cv::Mat labels, stats, centroids;
    cv::connectedComponentsWithStats(roi, labels, stats, centroids);
    int cnt = stats.rows;
    int target_idx = -1;
    
    // --- 4. 통합 라인 탐색 로직 ---
    int min_idx = -1;
    int min_dist = 10000;
    
    // 첫 실행/추적 중 상태에 따라 탐색 기준점과 반경을 설정
    int search_radius = first_run_ ? roi.cols : 60; // 첫 실행 시에는 ROI 폭만큼 (사실상 무제한)
    // tmp_pt_의 초기값 (320, 60)을 첫 실행의 기준점으로 사용
    cv::Point search_center = first_run_ ? cv::Point(320, 60) : tmp_pt_; 

    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            
            // 현재 탐색 기준점과의 거리 계산
            int dist = cv::norm(cv::Point(x, y) - search_center);

            // 거리 가깝고 설정된 범위 안이면 선택
            if (dist < min_dist && dist <= search_radius) {
                min_dist = dist;
                min_idx = i;
            }
        }
    }

    // 찾았으면 멤버변수 위치 갱신하고 target_idx 설정 (로직 통합)
    if (min_idx != -1) {
        tmp_pt_ = cv::Point(cvRound(centroids.at<double>(min_idx, 0)), cvRound(centroids.at<double>(min_idx, 1)));
        target_idx = min_idx;
        first_run_ = false; // 첫 실행을 끝냄
    }
    
    // --- 5. 시각화 로직 ---
    cv::Mat result_view = roi.clone();
    cv::cvtColor(result_view, result_view, cv::COLOR_GRAY2BGR);
    
    for (int i = 1; i < cnt; i++) {
        int area = stats.at<int>(i, 4);
        if (area > 100) {
            int x = cvRound(centroids.at<double>(i, 0));
            int y = cvRound(centroids.at<double>(i, 1));
            int left = stats.at<int>(i, 0);
            int top = stats.at<int>(i, 1);
            int width = stats.at<int>(i, 2);
            int height = stats.at<int>(i, 3);

            // 타겟 라인은 빨간색, 나머지는 파란색
            cv::Scalar color = (i == target_idx) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
            cv::rectangle(result_view, cv::Rect(left, top, width, height), color, 1);
            cv::circle(result_view, cv::Point(x, y), 5, color, -1);
        }
    }

    int error = 320 - tmp_pt_.x;

    auto endTime = chrono::steady_clock::now();
    float totalTime = chrono::duration<float, milli>(endTime - startTime).count();

    RCLCPP_INFO(this->get_logger(), "Received Image : err:%d, time:%.2f ms", error, totalTime);


    cv::imshow("frame_color", frame_color);
    cv::imshow("frame_roi", result_view);
    cv::waitKey(1);
}