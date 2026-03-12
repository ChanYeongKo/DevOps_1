/*******************************************************************************
* Dynamixel control implementation for dxl_rapi5
* Based on dxl_nano by ROBOTIS CO., LTD.
*******************************************************************************/

#include "dxl_rapi5/dxl.hpp"

// ─────────────────────────────────────────────
// 키보드 유틸
// ─────────────────────────────────────────────
int Dxl::getch(void)
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

bool Dxl::kbhit(void)
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

// ─────────────────────────────────────────────
// MX-12W 구현 (Protocol 1.0, Wheel mode)
// ─────────────────────────────────────────────
#if DXL_MODEL == MX12W

Dxl::Dxl(void)
{
    dxl_comm_result      = COMM_TX_FAIL;
    dxl_addparam_result  = false;
    dxl_error            = 0;
    portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(MX_PROTOCOL_VERSION);
}

bool Dxl::open(void)
{
    if (!portHandler->openPort()) {
        printf("Failed to open port!\n");
        return false;
    }
    printf("Port opened.\n");

    if (!portHandler->setBaudRate(MX_BAUDRATE)) {
        printf("Failed to set baudrate!\n");
        return false;
    }
    printf("Baudrate set to %d.\n", MX_BAUDRATE);

    // 토크 ON (DXL1, DXL2)
    for (int id : {DXL1_ID, DXL2_ID}) {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) {
            printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
            return false;
        } else if (dxl_error != 0) {
            printf("%s\n", packetHandler->getRxPacketError(dxl_error));
            return false;
        }
        printf("DXL ID:%d connected.\n", id);
    }
    return true;
}

void Dxl::close(void)
{
    setVelocity(0, 0);

    for (int id : {DXL1_ID, DXL2_ID}) {
        packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_MX_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    }
    portHandler->closePort();
    printf("Port closed.\n");
}

// goal_rpm1,2 : -470 ~ 470 rpm
bool Dxl::setVelocity(int goal_rpm1, int goal_rpm2)
{
    int v1 = (int)(goal_rpm1 / 0.916);  // 1 unit = 0.916 rpm
    int v2 = (int)(goal_rpm2 / 0.916);

    v1 = std::max(-470, std::min(470, v1));
    v2 = std::max(-470, std::min(470, v2));

    dynamixel::GroupSyncWrite gsw(portHandler, packetHandler,
                                  ADDR_MX_MOVING_SPEED, LEN_MX_MOVING_SPEED);
    uint8_t param[2];

    param[0] = DXL_LOBYTE(velConvert(v1));
    param[1] = DXL_HIBYTE(velConvert(v1));
    if (!gsw.addParam(DXL1_ID, param)) {
        fprintf(stderr, "[ID:%03d] addParam failed\n", DXL1_ID);
        return false;
    }

    param[0] = DXL_LOBYTE(velConvert(v2));
    param[1] = DXL_HIBYTE(velConvert(v2));
    if (!gsw.addParam(DXL2_ID, param)) {
        fprintf(stderr, "[ID:%03d] addParam failed\n", DXL2_ID);
        return false;
    }

    dxl_comm_result = gsw.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) {
        printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    gsw.clearParam();
    return true;
}

// MX-12W: +CCW(0~1023), -CW(1024~2047)
unsigned int Dxl::velConvert(int speed)
{
    if (speed >  1023) speed =  1023;
    if (speed < -1023) speed = -1023;
    return (speed >= 0) ? (unsigned int)speed : (unsigned int)(-speed + 1023);
}

// ─────────────────────────────────────────────
// XC430-W150 구현 (Protocol 2.0, Velocity mode)
// ─────────────────────────────────────────────
#elif DXL_MODEL == XC430W150

Dxl::Dxl(void)
{
    dxl_comm_result      = COMM_TX_FAIL;
    dxl_addparam_result  = false;
    dxl_error            = 0;
    portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(XL_PROTOCOL_VERSION);
}

bool Dxl::open(void)
{
    if (!portHandler->openPort()) { printf("Failed to open port!\n"); return false; }
    printf("Port opened.\n");
    if (!portHandler->setBaudRate(XL_BAUDRATE)) { printf("Failed to set baudrate!\n"); return false; }
    printf("Baudrate set to %d.\n", XL_BAUDRATE);

    for (int id : {DXL1_ID, DXL2_ID}) {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }
        else if (dxl_error != 0)             { printf("%s\n", packetHandler->getRxPacketError(dxl_error)); return false; }
        printf("DXL ID:%d connected.\n", id);
    }
    return true;
}

void Dxl::close(void)
{
    setVelocity(0, 0);
    for (int id : {DXL1_ID, DXL2_ID})
        packetHandler->write1ByteTxRx(portHandler, id, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    portHandler->closePort();
    printf("Port closed.\n");
}

// XC430-W150: -105 ~ 105 rpm (460 * 0.229 rpm)
bool Dxl::setVelocity(int goal_rpm1, int goal_rpm2)
{
    int v1 = (int)(goal_rpm1 / 0.229);
    int v2 = (int)(goal_rpm2 / 0.229);
    v1 = std::max(-460, std::min(460, v1));
    v2 = std::max(-460, std::min(460, v2));

    dynamixel::GroupSyncWrite gsw(portHandler, packetHandler,
                                  ADDR_XL_GOAL_VELOCITY, LEN_XL_GOAL_VELOCITY);
    uint8_t param[4];

    auto fill = [&](int vel) {
        param[0] = DXL_LOBYTE(DXL_LOWORD(vel));
        param[1] = DXL_HIBYTE(DXL_LOWORD(vel));
        param[2] = DXL_LOBYTE(DXL_HIWORD(vel));
        param[3] = DXL_HIBYTE(DXL_HIWORD(vel));
    };

    fill(v1); gsw.addParam(DXL1_ID, param);
    fill(v2); gsw.addParam(DXL2_ID, param);

    dxl_comm_result = gsw.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }
    gsw.clearParam();
    return true;
}

unsigned int Dxl::velConvert(int speed) { return 0; } // XC430는 미사용

// ─────────────────────────────────────────────
// XL430-W250 구현 (Protocol 2.0, Velocity mode)
// ─────────────────────────────────────────────
#elif DXL_MODEL == XL430W250

Dxl::Dxl(void)
{
    dxl_comm_result      = COMM_TX_FAIL;
    dxl_addparam_result  = false;
    dxl_error            = 0;
    portHandler   = dynamixel::PortHandler::getPortHandler(DEVICENAME);
    packetHandler = dynamixel::PacketHandler::getPacketHandler(XL_PROTOCOL_VERSION);
}

bool Dxl::open(void)
{
    if (!portHandler->openPort()) { printf("Failed to open port!\n"); return false; }
    printf("Port opened.\n");
    if (!portHandler->setBaudRate(XL_BAUDRATE)) { printf("Failed to set baudrate!\n"); return false; }
    printf("Baudrate set to %d.\n", XL_BAUDRATE);

    for (int id : {DXL1_ID, DXL2_ID}) {
        dxl_comm_result = packetHandler->write1ByteTxRx(
            portHandler, id, ADDR_XL_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
        if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }
        else if (dxl_error != 0)             { printf("%s\n", packetHandler->getRxPacketError(dxl_error)); return false; }
        printf("DXL ID:%d connected.\n", id);
    }
    return true;
}

void Dxl::close(void)
{
    setVelocity(0, 0);
    for (int id : {DXL1_ID, DXL2_ID})
        packetHandler->write1ByteTxRx(portHandler, id, ADDR_XL_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    portHandler->closePort();
    printf("Port closed.\n");
}

// XL430-W250: -60 ~ 60 rpm (265 * 0.229 rpm)
bool Dxl::setVelocity(int goal_rpm1, int goal_rpm2)
{
    int v1 = (int)(goal_rpm1 / 0.229);
    int v2 = (int)(goal_rpm2 / 0.229);
    v1 = std::max(-265, std::min(265, v1));
    v2 = std::max(-265, std::min(265, v2));

    dynamixel::GroupSyncWrite gsw(portHandler, packetHandler,
                                  ADDR_XL_GOAL_VELOCITY, LEN_XL_GOAL_VELOCITY);
    uint8_t param[4];

    auto fill = [&](int vel) {
        param[0] = DXL_LOBYTE(DXL_LOWORD(vel));
        param[1] = DXL_HIBYTE(DXL_LOWORD(vel));
        param[2] = DXL_LOBYTE(DXL_HIWORD(vel));
        param[3] = DXL_HIBYTE(DXL_HIWORD(vel));
    };

    fill(v1); gsw.addParam(DXL1_ID, param);
    fill(v2); gsw.addParam(DXL2_ID, param);

    dxl_comm_result = gsw.txPacket();
    if (dxl_comm_result != COMM_SUCCESS) { printf("%s\n", packetHandler->getTxRxResult(dxl_comm_result)); return false; }
    gsw.clearParam();
    return true;
}

unsigned int Dxl::velConvert(int speed) { return 0; } // XL430는 미사용

#endif
