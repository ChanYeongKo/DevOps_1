/*******************************************************************************
* Dynamixel control wrapper for dxl_rapi5
* Based on dxl_nano by ROBOTIS CO., LTD.
*******************************************************************************/

#ifndef _DXL_HPP_
#define _DXL_HPP_

#include <fcntl.h>
#include <termios.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include "dynamixel_sdk.h"

#define STDIN_FILENO 0

// 사용할 모델 선택 (하나만 활성화)
#define MX12W       0
#define XC430W150   1
#define XL430W250   2

#define DXL_MODEL   MX12W
//#define DXL_MODEL   XC430W150
//#define DXL_MODEL   XL430W250

// MX-12W 제어 테이블
#define ADDR_MX_TORQUE_ENABLE   24
#define ADDR_MX_MOVING_SPEED    32
#define LEN_MX_MOVING_SPEED     2
#define MX_PROTOCOL_VERSION     1.0
#define MX_BAUDRATE             2000000

// XL430 / XC430 제어 테이블
#define ADDR_XL_TORQUE_ENABLE   64
#define ADDR_XL_OPERATING_MODE  11
#define ADDR_XL_GOAL_VELOCITY   104
#define LEN_XL_GOAL_VELOCITY    4
#define XL_PROTOCOL_VERSION     2.0
#define XL_BAUDRATE             4000000

// 공통
#define DXL1_ID         1               // 왼쪽 바퀴
#define DXL2_ID         2               // 오른쪽 바퀴
#define DEVICENAME      "/dev/ttyUSB0"
#define TORQUE_ENABLE   1
#define TORQUE_DISABLE  0
#define OPMODE_VELOCITY 1

class Dxl
{
private:
    int dxl_comm_result;
    uint8_t dxl_addparam_result;
    uint8_t dxl_error;
    dynamixel::PortHandler*   portHandler;
    dynamixel::PacketHandler* packetHandler;

public:
    Dxl(void);
    bool open(void);
    void close(void);
    bool setVelocity(int goal_rpm1, int goal_rpm2);
    unsigned int velConvert(int speed);
    static int  getch(void);
    static bool kbhit(void);
};

#endif // _DXL_HPP_
