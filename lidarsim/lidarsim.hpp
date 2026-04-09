#ifndef SUB_HPP_
#define SUB_HPP_

#include "rclcpp/rclcpp.hpp" // ROS 2 기본 헤더
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "opencv2/opencv.hpp" // OpenCV 헤더
#include <memory> 
#include <chrono>
#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <utility>
#define STDIN_FILENO 0 

using namespace std;

class ObstacleAvoidance : public rclcpp::Node { // 라이다 장애물 회피 클래스
public:
    ObstacleAvoidance(); // 생성자
private:
    void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg); // 라이다 스캔 콜백
    void image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg); // 카메라 이미지 콜백

    int getch(void); // 엔터키 입력 없이 한 문자만 입력시 바로 리턴
    bool kbhit(void); // 키보드가 눌렸는지 체크해주는 함수

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_; 
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr img_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr dxl_pub_; 

    bool mode = false;
    geometry_msgs::msg::Vector3 vel;
    double k = 0.1; // 비례 상수 (필요시 조정)
};

#endif // SUB_HPP_