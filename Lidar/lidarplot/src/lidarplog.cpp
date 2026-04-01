/*
 *  SLLIDAR ROS2 CLIENT
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2022 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <math.h>
#include <opencv2/opencv.hpp>

#define RAD2DEG(x) ((x)*180./M_PI)

cv::VideoWriter video_writer;

static void scanCb(sensor_msgs::msg::LaserScan::SharedPtr scan) {
  int count = scan->scan_time / scan->time_increment;
  
  // 500x500 영상 생성 (검은색 배경)
  cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);
  
  // 중심점 십자가 표시
  int center_x = 250;
  int center_y = 250;
  cv::line(img, cv::Point(center_x - 10, center_y), cv::Point(center_x + 10, center_y), cv::Scalar(0, 255, 0), 2);
  cv::line(img, cv::Point(center_x, center_y - 10), cv::Point(center_x, center_y + 10), cv::Scalar(0, 255, 0), 2);
  
  // 기준 원 그리기 (예: 1m 단위)
  float pixels_per_meter = 500.0 / 10.0; // 50 pixels = 1m
  for (int i = 1; i <= 5; ++i) {
      cv::circle(img, cv::Point(center_x, center_y), i * pixels_per_meter, cv::Scalar(50, 50, 50), 1);
  }

  // 앞서 출력하던 정보 중 일부 로그 유지 또는 생략 가능
  // printf("[SLLIDAR INFO]: I heard a laser scan %s[%d]:\n", scan->header.frame_id.c_str(), count);

  for (int i = 0; i < count; i++) {
    float distance = scan->ranges[i];
    
    // 유효한 거리값인지 확인
    if (std::isinf(distance) || std::isnan(distance) || distance < scan->range_min || distance > scan->range_max) {
        continue;
    }

    float angle = scan->angle_min + scan->angle_increment * i;
    
    // 라이다가 조립된 실제 전면 방향과 화면의 12시(앞) 방향을 일치시키기 위한 각도 오프셋
    // 기본적으로 코드 상 앞(X축)은 화면의 12시에 이미 매핑되어 있습니다.
    // 하지만 라이다 하드웨어의 케이블 위치 등에 따라 180도(M_PI)나 90도(M_PI/2) 회전이 필요할 수 있습니다.
    float angle_offset = M_PI; // 라이다가 180도 뒤집혀 장착된 경우 (필요에 따라 0, M_PI/2, -M_PI/2 등으로 수정)
    angle += angle_offset;
    
    // 삼각함수를 이용한 좌표 변환 (스캔 영상 그리기)
    // ROS 기준: 전방(X), 좌측(Y). 화면 기준: 우측(X), 하단(Y)
    float x_ros = distance * cos(angle);
    float y_ros = distance * sin(angle);
    
    // 화면 좌표계로 변환
    int x_img = center_x - (int)(y_ros * pixels_per_meter);
    int y_img = center_y - (int)(x_ros * pixels_per_meter);

    // 영상 내부에 들어오는지 확인 후 장애물을 빨간색 점으로 표시
    if (x_img >= 0 && x_img < 500 && y_img >= 0 && y_img < 500) {
        cv::circle(img, cv::Point(x_img, y_img), 2, cv::Scalar(0, 0, 255), -1);
    }
  }

  // 모니터에 출력
  cv::imshow("Lidar Scan", img);
  cv::waitKey(1); // 1ms 대기 및 키 입력 처리 (화면 갱신에 필요)

  // 동영상(mp4)으로 저장
  if (video_writer.isOpened()) {
      video_writer.write(img);
  }
}

int main(int argc, char **argv) {
  // VideoWriter 초기화: mp4 코덱 사용, 10 fps (LiDAR 주기에 따라 조정 가능), 500x500
  video_writer.open("lidar_scan.mp4", cv::VideoWriter::fourcc('m', 'p', '4', 'v'), 10.0, cv::Size(500, 500));
  if (!video_writer.isOpened()) {
      printf("[ERROR] Failed to open video writer!\n");
  }

  rclcpp::init(argc, argv);

  auto node = rclcpp::Node::make_shared("sllidar_client");

  // /scan 토픽을 구독하도록 설정
  auto lidar_info_sub = node->create_subscription<sensor_msgs::msg::LaserScan>(
                        "/scan", rclcpp::SensorDataQoS(), scanCb);

  rclcpp::spin(node);

  video_writer.release();
  cv::destroyAllWindows();
  rclcpp::shutdown();

  return 0;
}
