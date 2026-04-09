#include "lidarsim/lidarsim.hpp"

ObstacleAvoidance::ObstacleAvoidance() : Node("lidarsim_node") {
    vel.x = 0;
    vel.y = 0;
    vel.z = 0;

    auto qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
    
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 
        qos_profile, 
        bind(&ObstacleAvoidance::scan_callback, this, placeholders::_1));

    img_sub_ = this->create_subscription<sensor_msgs::msg::CompressedImage>(
        "image/topic", 
        qos_profile, 
        bind(&ObstacleAvoidance::image_callback, this, placeholders::_1));

    dxl_pub_ = this->create_publisher<geometry_msgs::msg::Vector3>(
        "vel_cmd/topic",
        qos_profile);
}

int ObstacleAvoidance::getch(void) 
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

bool ObstacleAvoidance::kbhit(void) 
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

void ObstacleAvoidance::scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
{
    cv::Mat result(500, 500, CV_8UC3, cv::Scalar(255, 255, 255));

    // 로봇 위치 (중앙)
    cv::Point robot_pos(250, 250);
    cv::circle(result, robot_pos, 5, cv::Scalar(255, 0, 0), -1);

    // 거리 측정 기준 원 (2m 반경 표시)
    cv::circle(result, robot_pos, 250, cv::Scalar(200, 200, 200), 1);
    // 보조선 (1m 반경 표시)
    cv::circle(result, robot_pos, 125, cv::Scalar(220, 220, 220), 1);
    
    double L_dist = 1000.0;
    double L_angle_deg = 0.0;
    bool L_found = false;

    double R_dist = 1000.0;
    double R_angle_deg = 0.0;
    bool R_found = false;

    for (size_t i = 0; i < msg->ranges.size(); i++) {
        double r = msg->ranges[i];
        if (std::isinf(r) || std::isnan(r) || r < msg->range_min || r > msg->range_max) {
            continue;
        }
        if (r > 2.0) continue; // 2m 반경 내 장애물만 표시 및 처리

        double theta = msg->angle_min + i * msg->angle_increment;
        
        // 라이다 물리적 정면(180도)을 코드 상에서 12시(0도) 방향으로 맞추기 위해 180도(M_PI) 보정
        theta += M_PI; 
        
        // 각도를 [-pi, pi] 범위로 정규화
        while (theta > M_PI) theta -= 2.0 * M_PI;
        while (theta <= -M_PI) theta += 2.0 * M_PI;

        double theta_deg = theta * 180.0 / M_PI;

        // 장애물 영역 체크 (전방 180도 제한시 라이다가 살짝 삐뚤어지면 한쪽 벽의 수직점을 놓쳐 편향 발생)
        // 이를 방지하기 위해 120도까지 여유 버퍼를 두고 스캔하여 정확히 벽의 최단거리 위치를 보장
        if (theta_deg >= 0 && theta_deg <= 120) { // 좌측 영역 
            if (r < L_dist) {
                L_dist = r;
                L_angle_deg = theta_deg;
                L_found = true;
            }
        } else if (theta_deg >= -120 && theta_deg < 0) { // 우측 영역
            if (r < R_dist) {
                R_dist = r;
                R_angle_deg = theta_deg;
                R_found = true;
            }
        }

        // 스캔 영상 좌표 계산 (2m = 250 픽셀, 즉 1m = 125 픽셀)
        // 화면상 전방은 y좌표 감소 (위쪽), 좌측은 x좌표 감소 (왼쪽)
        int r_px = static_cast<int>(r * 125.0);
        int img_x = 250 - r_px * sin(theta);
        int img_y = 250 - r_px * cos(theta);

        if (img_x >= 0 && img_x < 500 && img_y >= 0 && img_y < 500) {
            cv::circle(result, cv::Point(img_x, img_y), 2, cv::Scalar(0, 0, 0), -1);
        }
    }

    int error = 0;
    if (L_found && R_found) {
        // 장해물이 양쪽에 있을 경우
        error = -static_cast<int>((L_angle_deg + R_angle_deg) / 2.0);
    } else if (L_found && !R_found) {
        // 우측이 완전히 뚫려있다고 가정 (가상의 우측 방해물을 -90도 끝단에 배치)
        // 공식: -(L_angle_deg + (-90)) / 2  = (90 - L_angle_deg) / 2
        error = static_cast<int>((90.0 - L_angle_deg) / 2.0);
    } else if (!L_found && R_found) {
        // 좌측이 완전히 뚫려있다고 가정 (가상의 좌측 방해물을 90도 끝단에 배치)
        // 공식: -(90 + R_angle_deg) / 2
        error = -static_cast<int>((90.0 + R_angle_deg) / 2.0);
    } else {
        // 전방 180도 영역에 장애물이 없는 경우
        error = 0;
    }

    // 키보드 입력을 통한 모드 제어
    if(kbhit()) {
        int ch = getch();
        if(ch == 'q') mode = false;
        else if(ch == 's') mode = true;
    }

    if (mode) {
        // 에러가 양수면 좌측 다이내믹셀(x) 속도 증가, 우측(y) 속도 감소로 우회전 구현
        vel.x = 50 + k * error;
        vel.y = -(50 - k * error);
    } 
    else {
        vel.x = 0;
        vel.y = 0;
    }

    dxl_pub_->publish(vel);

    RCLCPP_INFO(this->get_logger(), "Error: %d, leftvel: %.2f, rightvel: %.2f", error, vel.x, vel.y);

    // 시각화: 좌측 최단거리 장애물
    if (L_found) {
        int r_px = static_cast<int>(L_dist * 125.0);
        double theta = L_angle_deg * M_PI / 180.0;
        int img_x = 250 - r_px * sin(theta);
        int img_y = 250 - r_px * cos(theta);
        cv::circle(result, cv::Point(img_x, img_y), 6, cv::Scalar(0, 255, 0), -1); // 녹색 원
        cv::putText(result, "L", cv::Point(img_x+10, img_y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,255,0), 2);
    }

    // 시각화: 우측 최단거리 장애물
    if (R_found) {
        int r_px = static_cast<int>(R_dist * 125.0);
        double theta = R_angle_deg * M_PI / 180.0;
        int img_x = 250 - r_px * sin(theta);
        int img_y = 250 - r_px * cos(theta);
        cv::circle(result, cv::Point(img_x, img_y), 6, cv::Scalar(0, 0, 255), -1); // 적색 원
        cv::putText(result, "R", cv::Point(img_x+10, img_y-10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,0,255), 2);
    }

    // 목표 조향 방향 시각화 (방향각 theta = -error * M_PI/180)
    double target_theta = -error * M_PI / 180.0;
    int t_img_x = 250 - 150 * sin(target_theta);
    int t_img_y = 250 - 150 * cos(target_theta);
    cv::arrowedLine(result, robot_pos, cv::Point(t_img_x, t_img_y), cv::Scalar(255, 0, 0), 2);

    cv::imshow("Lidar Scan View", result);
    cv::waitKey(1);
}

void ObstacleAvoidance::image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr msg)
{
    cv::Mat frame = cv::imdecode(cv::Mat(msg->data), cv::IMREAD_COLOR);
    if (!frame.empty()) {
        cv::imshow("Camera View", frame);
        cv::waitKey(1);
    }
}