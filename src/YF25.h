#pragma once
#include "CRC16.h"
#include "RS485.h"

/*
注意事项：
温度传感器：这项为选装，如果没有配备，则温度传感器读数无效
刹车抱闸：这项为选装，如果没有配备，则刹车抱闸无效
*/

#define YF25Mgr_WaitHead    0x00
#define YF25Mgr_WaitID      0x01
#define YF25Mgr_WaitLength  0x02
#define YF25Mgr_WaitData    0x03
#define YF25Mgr_WaitCRC1    0x04
#define YF25Mgr_WaitCRC2    0x05

template<typename T, int size>
class YF25FIFO {
private:
    T arr[size];
    int head;
    int tail;
    int count;
public:
    YF25FIFO() : head(0), tail(0), count(0) {}
    // 入队
    bool enqueue(T element) {
        if (count >= size) return false; // 队列已满
        arr[tail] = element;
        tail = (tail + 1) % size;
        count++;
        return true;
    }

    // 出队
    bool dequeue(T &element) {
        if (count <= 0) return false; // 队列为空
        element = arr[head];
        head = (head + 1) % size;
        count--;
        return true;
    }

    // 出队
    bool dequeue() {
        if (count <= 0) return false; // 队列为空
        head = (head + 1) % size;
        count--;
        return true;
    }

    // 查看队首元素
    bool peek(T &element) {
        if (count <= 0) return false; // 队列为空
        element = arr[head];
        return true;
    }

    // 队列是否为空
    bool isEmpty() const {
        return count == 0;
    }

    // 队列是否已满
    bool isFull() const {
        return count == size;
    }
};


enum class YF25Cmd : uint8_t {
    FunctionControl = 0x20,                   // 功能控制指令（0x20）
    ReadPIArgs = 0x30,                        // 读取PI参数命令（0x30）
    WritePIArgsToRAM = 0x31,                  // 写入PI参数到 RAM 命令（0x31）
    WritePIArgsToROM = 0x32,                  // 写入PI参数到 ROM 命令（0x32）
    ReadAcceleration = 0x42,                  // 读取加速度命令（0x42）
    WriteAccelerationToRAMAndROM = 0x43,      // 写入加速度到 RAM 和 ROM 命令（0x43） 掉电保存
    ReadMultiTurnPosition = 0x60,             // 读取多圈编码器位置数据命令（0x60）
    ReadMultiTurnPositionRaw = 0x61,          // 读取多圈编码器原始位置数据命令（0x61）
    ReadMultiTurnZeroPoint = 0x62,            // 读取多圈编码器零偏数据命令（0x62）
    WriteMultiTurnZeroPointToROM = 0x63,      // 写入编码器多圈值到 ROM 作为电机零点命令（0x63）
    WriteCurrentPosAsMultiTurnZeroPointToROM = 0x64, // 写入编码器当前多圈位置到 ROM 作为电机零点命令（0x64）
    ReadSingleTurnPosition = 0x90,            // 读取单圈编码器命令（0x90） 仅适用于直驱系列
    ReadMultiTurnAngle = 0x92,                // 读取多圈角度命令（0x92）
    ReadSingleTurnAngle = 0x94,               // 读取单圈角度命令（0x94） 仅适用于直驱系列
    ReadMotorState1 = 0x9A,                   // 读取电机状态 1 和错误标志命令（0x9A）
    ReadMotorState2 = 0x9C,                   // 读取电机状态 2（0x9C）
    ReadMotorState3 = 0x9D,                   // 读取电机状态 3（0x9D）
    ShutDown = 0x80,                          // 电机关闭命令（0x80）
    Stop = 0x81,                              // 电机停止命令（0x81）
    SetTorqueCurrentControl = 0xA1,           // 转矩闭环控制命令（0xA1）
    SetSpeedControl = 0xA2,                   // 速度闭环控制命令（0xA2）
    SetAbsolutePositionControl = 0xA4,        // 绝对位置闭环控制命令（0xA4）
    SetSingleTurnAbsolutePositionControl = 0xA6,     // 单圈位置控制命令（0xA6） 仅适用于直驱系列
    SetIncrementalPositionControl = 0xA8,     // 增量位置闭环控制命令（0xA8）
    GetSystemMode = 0x70,                     // 系统运行模式获取（0x70）
    GetSystemPower = 0x71,                    // 电机功率获取（0x71）
    Reset = 0x76,                             // 系统复位指令（0x76）
    ReleaseBrake = 0x77,                      // 系统抱闸释放指令（0x77）
    HoldBrake = 0x78,                         // 系统抱闸锁死指令（0x78）
    Set485ID = 0x79,                          // RS485-ID 设置指令（0x79）
    GetSystemTick = 0xB1,                     // 系统运行时间读取指令（0xB1）
    GetSystemSoftwareVersionDate = 0xB2,      // 系统软件版本日期读取指令（0xB2）
    SetCommBrokenProtection = 0xB3,           // 通讯中断保护时间设置指令（0xB3）
    SetBaudRate = 0xB4,                       // 通讯波特率设置指令（0xB4）
    GetModel = 0x85                           // 电机型号读取指令（0x85）
};

#pragma pack(push, 1)

class YF25PackData {
public:
    YF25Cmd command;
    uint8_t data[7];
};

class YF25PackData_ReadPIArgs {
public:
    // PI参数
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;    // 命令字节
            uint8_t unused1;    // 未使用的字节
            uint8_t currentKP;  // 电流环 KP 参数
            uint8_t currentKI;  // 电流环 KI 参数
            uint8_t speedKP;    // 速度环 KP 参数
            uint8_t speedKI;    // 速度环 KI 参数
            uint8_t positionKP; // 位置环 KP 参数
            uint8_t positionKI; // 位置环 KI 参数
        };
    };

    // 构造函数
    YF25PackData_ReadPIArgs() 
        : command(YF25Cmd::ReadPIArgs), unused1(0), 
          currentKP(0), currentKI(0), 
          speedKP(0), speedKI(0), 
          positionKP(0), positionKI(0) {}
};

class YF25PackData_WritePIArgsToRAM {
public:
    // 写入PI参数到RAM的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;    // 命令字节
            uint8_t unused1;    // 未使用的字节
            uint8_t currentKP;  // 电流环 KP 参数
            uint8_t currentKI;  // 电流环 KI 参数
            uint8_t speedKP;    // 速度环 KP 参数
            uint8_t speedKI;    // 速度环 KI 参数
            uint8_t positionKP; // 位置环 KP 参数
            uint8_t positionKI; // 位置环 KI 参数
        };
    };

    // 构造函数
    YF25PackData_WritePIArgsToRAM(uint8_t currKP, uint8_t spdKP, uint8_t posKP, uint8_t currKI = 0, uint8_t spdKI = 0, uint8_t posKI = 0)
        : command(YF25Cmd::WritePIArgsToRAM), unused1(0), 
          currentKP(currKP), currentKI(currKI), 
          speedKP(spdKP), speedKI(spdKI), 
          positionKP(posKP), positionKI(posKI) {}
};

class YF25PackData_WritePIArgsToROM {
public:
    // 写入PI参数到ROM的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;    // 命令字节
            uint8_t unused1;    // 未使用的字节
            uint8_t currentKP;  // 电流环 KP 参数
            uint8_t currentKI;  // 电流环 KI 参数
            uint8_t speedKP;    // 速度环 KP 参数
            uint8_t speedKI;    // 速度环 KI 参数
            uint8_t positionKP; // 位置环 KP 参数
            uint8_t positionKI; // 位置环 KI 参数
        };
    };

    // 构造函数
    YF25PackData_WritePIArgsToROM(uint8_t currKP, uint8_t spdKP, uint8_t posKP, uint8_t currKI = 0, uint8_t spdKI = 0, uint8_t posKI = 0)
        : command(YF25Cmd::WritePIArgsToROM), unused1(0), 
          currentKP(currKP), currentKI(currKI), 
          speedKP(spdKP), speedKI(spdKI), 
          positionKP(posKP), positionKI(posKI) {}
};

class YF25PackData_ReadAcceleration {
public:
    // 加速度数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // 未使用的字节
            uint8_t unused2;        // 未使用的字节
            uint8_t unused3;        // 未使用的字节
            int32_t acceleration;   // 加速度数据(角度/秒^2)
        };
    };

    // 构造函数
    YF25PackData_ReadAcceleration() 
        : command(YF25Cmd::ReadAcceleration), unused1(0), 
          unused2(0), unused3(0), acceleration(0) {}
};

enum class YF25AccelerationFunctionIndex : uint8_t {
    PositionPlanningAcceleration = 0x00,    // 位置规划加速度，位置规划中从初始速度到最大速度的加速度
    PositionPlanningDeceleration = 0x01,    // 位置规划减速度，位置规划中从最大速度到停止的减速度
    SpeedPlanningAcceleration = 0x02,       // 速度规划加速度，从当前速度加速到目标速度的加速度，包括正反方向加速度
    SpeedPlanningDeceleration = 0x03        // 速度规划减速度，在相同方向上，从当前速度减速到目标速度的减速度值
};
class YF25PackData_WriteAccelerationToRAMAndROM {
public:
    // 写入加减速度到 RAM 和 ROM 的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t index;          // 功能索引
            uint8_t unused1;        // 未使用的字节
            uint8_t unused2;        // 未使用的字节
            int32_t acceleration;   // 加速度数据(角度/秒^2)
        };
    };

    // 构造函数
    YF25PackData_WriteAccelerationToRAMAndROM(uint8_t idx, int32_t accel) 
        : command(YF25Cmd::WriteAccelerationToRAMAndROM), index(idx), 
          unused1(0), unused2(0), acceleration(accel) {}
};

class YF25PackData_ReadMultiTurnPosition {
public:
    // 读取多圈编码器位置的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;   // 命令字节
            uint8_t unused1;   // NULL
            uint8_t unused2;   // NULL
            uint8_t unused3;   // NULL
            int32_t encoder;   // 编码器位置（32位整数）
        };
    };

    // 构造函数
    YF25PackData_ReadMultiTurnPosition() 
        : command(YF25Cmd::ReadMultiTurnPosition), unused1(0), 
          unused2(0), unused3(0), encoder(0) {}
};

class YF25PackData_ReadMultiTurnPositionRaw {
public:
    // 读取多圈编码器原始位置的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;   // 命令字节
            uint8_t unused1;   // NULL
            uint8_t unused2;   // NULL
            uint8_t unused3;   // NULL
            int32_t encoderRaw;  // 编码器位置（32位整数）
        };
    };

    // 构造函数
    YF25PackData_ReadMultiTurnPositionRaw() 
        : command(YF25Cmd::ReadMultiTurnPositionRaw), unused1(0), 
          unused2(0), unused3(0), encoderRaw(0) {}
};

class YF25PackData_ReadMultiTurnZeroPoint {
public:
    // 读取多圈编码器原始位置的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;   // 命令字节
            uint8_t unused1;   // NULL
            uint8_t unused2;   // NULL
            uint8_t unused3;   // NULL
            int32_t encoderZeroPoint;  // 编码器位置（32位整数）
        };
    };

    // 构造函数
    YF25PackData_ReadMultiTurnZeroPoint() 
        : command(YF25Cmd::ReadMultiTurnZeroPoint), unused1(0), 
          unused2(0), unused3(0), encoderZeroPoint(0) {}
};

class YF25PackData_WriteMultiTurnZeroPointToROM {
public:
    // 写入编码器多圈值到 ROM 的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;       // 命令字节
            uint8_t unused1;       // NULL
            uint8_t unused2;       // NULL
            uint8_t unused3;       // NULL
            int32_t encoderOffset; // 编码器多圈零偏值
        };
    };

    // 构造函数
    YF25PackData_WriteMultiTurnZeroPointToROM(int32_t offset) 
        : command(YF25Cmd::WriteMultiTurnZeroPointToROM), unused1(0), unused2(0), unused3(0), encoderOffset(offset) {}
};

class YF25PackData_WriteCurrentPosAsMultiTurnZeroPointToROM  {
public:
    // 写入编码器当前多圈位置到 ROM 作为电机零点的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;       // 命令字节
            uint8_t unused1;       // NULL
            uint8_t unused2;       // NULL
            uint8_t unused3;       // NULL
            int32_t encoderOffset; // 编码器多圈零偏值
        };
    };

    // 构造函数
    YF25PackData_WriteCurrentPosAsMultiTurnZeroPointToROM(int32_t offset) 
        : command(YF25Cmd::WriteCurrentPosAsMultiTurnZeroPointToROM), unused1(0), unused2(0), unused3(0), encoderOffset(offset) {}
};

class YF25PackData_ReadSingleTurnPosition {
public:
    // 读取单圈编码器位置的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;       // 命令字节
            uint8_t unused1;       // NULL
            int16_t encoder;       // 编码器位置
            int16_t encoderRaw;    // 编码器原始位置
            int16_t encoderOffset; // 编码器零偏
        };
    };

    // 构造函数
    YF25PackData_ReadSingleTurnPosition() 
        : command(YF25Cmd::ReadSingleTurnPosition), unused1(0), 
          encoder(0), encoderRaw(0), encoderOffset(0) {}
};

class YF25PackData_ReadMultiTurnAngle {
public:
    // 读取多圈角度的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            int32_t angle;          // 电机输出轴角度（0.01°）
        };
    };

    // 构造函数
    YF25PackData_ReadMultiTurnAngle() 
        : command(YF25Cmd::ReadMultiTurnAngle), unused1(0), unused2(0), unused3(0), angle(0) {}
};

class YF25PackData_ReadSingleTurnAngle {
public:
    // 读取单圈角度的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;         // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint8_t unused4;        // NULL
            uint8_t unused5;        // NULL
            int16_t motorSingleTurnAngle; // 电机单圈角度
        };
    };

    // 构造函数
    YF25PackData_ReadSingleTurnAngle() 
        : command(YF25Cmd::ReadSingleTurnAngle), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), motorSingleTurnAngle(0) {}
};

enum class YF25SystemErrorState : uint16_t {
    Normal = 0x0000,            // 正常
    OverCurrent = 0x0002,       // 过流保护
    OverVoltage = 0x0004,       // 过压保护
    UnderVoltage = 0x0008,      // 欠压保护
    OverTemperature = 0x0010,   // 过温保护
    EncoderError = 0x0040,      // 动态切割保护
    Overload = 0x0080,          // 动态切割错误
    DriverFault = 0x0100,       // 驱动故障
    MotorStalled = 0x1000,      // 电机堵转保护
    FeedbackMismatch = 0x2000,  // 编码器反馈错误
};
class YF25PackData_ReadMotorState1 {
public:
    // 读取电机状态 1 和错误标志的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;          // 命令字节
            int8_t temperature;       // 电机温度（℃）
            uint8_t unused1;           // NULL
            uint8_t brakeRelease;     // 抱闸释放指令
            uint16_t voltage;         // 电压（0.1V）
            YF25SystemErrorState errorState;      // 错误状态
        };
    };

    // 构造函数
    YF25PackData_ReadMotorState1() 
        : command(YF25Cmd::ReadMotorState1), temperature(0), unused1(0), brakeRelease(0), voltage(0), errorState(YF25SystemErrorState::Normal) {}
};

class YF25PackData_ReadMotorState2 {
public:
    // 读取电机状态 2 的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;         // 命令字节
            int8_t temperature;      // 电机温度（℃）
            int16_t torqueCurrent;   // 转矩电流（0.01A）
            int16_t speed;           // 速度（角度/秒）
            int16_t angle;           // 角度（角度）
        };
    };

    // 构造函数
    YF25PackData_ReadMotorState2() 
        : command(YF25Cmd::ReadMotorState2), temperature(0), torqueCurrent(0), speed(0), angle(0) {}
};

class YF25PackData_ReadMotorState3 {
public:
    // 读取电机状态 3 的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;     // 命令字节
            int8_t temperature;  // 电机温度（℃）
            int16_t currentA;    // A相电流（0.01A）
            int16_t currentB;    // B相电流（0.01A）
            int16_t currentC;    // C相电流（0.01A）
        };
    };

    // 构造函数
    YF25PackData_ReadMotorState3() 
        : command(YF25Cmd::ReadMotorState3), temperature(0), currentA(0), currentB(0), currentC(0) {}
};

class YF25PackData_ShutDown {
public:
    // 电机关闭命令的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint8_t unused4;        // NULL
            uint8_t unused5;        // NULL
            uint8_t unused6;        // NULL
            uint8_t unused7;        // NULL
        };
    };

    // 构造函数
    YF25PackData_ShutDown() 
        : command(YF25Cmd::ShutDown), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0), unused7(0) {}
};

class YF25PackData_Stop {
public:
    // 电机停止命令的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint8_t unused4;        // NULL
            uint8_t unused5;        // NULL
            uint8_t unused6;        // NULL
            uint8_t unused7;        // NULL
        };
    };

    // 构造函数
    YF25PackData_Stop() 
        : command(YF25Cmd::Stop), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0), unused7(0) {}
};

class YF25PackData_SetTorqueCurrentControl {
public:
    // 转矩闭环控制命令的数据，返回数据为 电机状态2 数据包
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;          // 命令字节
            union {
                struct{    //发送数据
                    uint8_t unused1;          // NULL
                    uint8_t unused2;          // NULL
                    uint8_t unused3;          // NULL
                    int16_t torqueCurrentControl;    // 电流控制值
                    uint8_t unused4;          // NULL
                    uint8_t unused5;          // NULL
                };
                struct{    //接收数据
                    int8_t temperature;      // 电机温度（℃）
                    int16_t torqueCurrent;   // 转矩电流（0.01A）
                    int16_t speed;           // 速度（角度/秒）
                    int16_t angle;           // 角度（角度）
                };
            };
        };
    };

    // 构造函数
    YF25PackData_SetTorqueCurrentControl(int16_t iq) 
        : command(YF25Cmd::SetTorqueCurrentControl), unused1(0), unused2(0), unused3(0), torqueCurrentControl(iq), unused4(0), unused5(0) {}
};

class YF25PackData_SetSpeedControl {
public:
    // 速度闭环控制命令的数据，返回数据为 电机状态2 数据包
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;       // 命令字节
            union {
                struct {    //发送数据
                    uint8_t unused1;       // NULL
                    uint8_t unused2;       // NULL
                    uint8_t unused3;       // NULL
                    int32_t speedControl;  // 速度控制值
                };
                struct {    //接收数据
                    int8_t temperature;      // 电机温度（℃）
                    int16_t torqueCurrent;   // 转矩电流（0.01A）
                    int16_t speed;           // 速度（角度/秒）
                    int16_t angle;           // 角度（角度）
                };
            };
        };
    };

    // 构造函数
    YF25PackData_SetSpeedControl(int32_t s = 0) 
        : command(YF25Cmd::SetSpeedControl), unused1(0), unused2(0), unused3(0), speedControl(s) {}
};


class YF25PackData_SetAbsolutePositionControl {
public:
    // 绝对位置闭环控制命令的数据，返回数据为 电机状态2 数据包
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;            // 命令字节
            union {
                struct {    //发送数据
                    uint8_t unused1;            // NULL
                    int16_t maxSpeed;           // 最大速度
                    int32_t positionControl;    // 位置控制
                };
                struct {    //接收数据
                    int8_t temperature;      // 电机温度（℃）
                    int16_t torqueCurrent;   // 转矩电流（0.01A）
                    int16_t speed;           // 速度（角度/秒）
                    int16_t angle;           // 角度（角度）
                };
            };
        };
    };

    // 构造函数
    YF25PackData_SetAbsolutePositionControl(int32_t pos, int16_t mSpeed) 
        : command(YF25Cmd::SetAbsolutePositionControl), unused1(0), maxSpeed(mSpeed), positionControl(pos) {}
};


class YF25PackData_SetSingleTurnAbsolutePositionControl {
public:
    // 单圈位置控制命令的数据
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;            // 命令字节
            union {
                struct {    //发送数据
                    uint8_t direction;          // 旋转方向
                    uint16_t maxSpeed;          // 最大速度
                    uint16_t positionControl;   // 角度控制
                    uint8_t unused1;            // NULL
                    uint8_t unused2;            // NULL
                };
                struct {    //接收数据
                    int8_t temperature;      // 电机温度（℃）
                    int16_t torqueCurrent;   // 转矩电流（0.01A）
                    int16_t speed;           // 速度（角度/秒）
                    int16_t encoder;         // 编码器的数值，范围由编码器位数决定
                };
            };
        };
    };

    // 构造函数
    YF25PackData_SetSingleTurnAbsolutePositionControl(uint8_t dir, uint16_t mSpeed, uint16_t pos) 
        : command(YF25Cmd::SetSingleTurnAbsolutePositionControl), direction(dir), maxSpeed(mSpeed), positionControl(pos), unused1(0), unused2(0) {}
};

class YF25PackData_SetIncrementalPositionControl {
public:
    // 增量位置闭环控制命令的数据结构，返回数据为 电机状态2 数据包
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;            // 命令字节
            union {
                struct {    //发送数据
                    uint8_t unused1;             // 未使用，填充为 NULL
                    uint16_t maxSpeed;           // 最大速度控制值
                    int32_t incPositionControl;  // 增量位置控制值
                };
                struct {    //接收数据
                    int8_t temperature;      // 电机温度（℃）
                    int16_t torqueCurrent;   // 转矩电流（0.01A）
                    int16_t speed;           // 速度（角度/秒）
                    int16_t angle;           // 角度（角度）
                };
            };
        };
    };

    // 构造函数，用于初始化命令、最大速度和角度控制值
    YF25PackData_SetIncrementalPositionControl(uint16_t speed, int32_t incPosition) 
        : command(YF25Cmd::SetIncrementalPositionControl), unused1(0), maxSpeed(speed), incPositionControl(incPosition) {}
};

enum class YF25SystemRunMode : uint8_t {
    CurrentLoop  = 0x01, // 电流环模式
    SpeedLoop    = 0x02, // 速度环模式
    PositionLoop = 0x03  // 位置环模式
};
class YF25PackData_GetSystemMode {
public:
    // 系统运行模式获取命令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;    // 命令字节，对应系统运行模式获取命令
            uint8_t unused1;    // NULL
            uint8_t unused2;    // NULL
            uint8_t unused3;    // NULL
            uint8_t unused4;    // NULL
            uint8_t unused5;    // NULL
            uint8_t unused6;    // NULL
            YF25SystemRunMode runMode;    // 运行模式
        };
    };

    // 构造函数
    YF25PackData_GetSystemMode() 
        : command(YF25Cmd::GetSystemMode), unused1(0), unused2(0), unused3(0),
          unused4(0), unused5(0), unused6(0), runMode((YF25SystemRunMode)0) {}
};

class YF25PackData_GetSystemPower {
public:
    // 系统运行模式获取命令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;    // 命令字节，对应系统运行模式获取命令
            uint8_t unused1;    // NULL
            uint8_t unused2;    // NULL
            uint8_t unused3;    // NULL
            uint8_t unused4;    // NULL
            uint8_t unused5;    // NULL
            uint16_t power;     // 系统功率（0.1W）
        };
    };

    // 构造函数
    YF25PackData_GetSystemPower() 
        : command(YF25Cmd::GetSystemPower), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), power(0) {}
};

class YF25PackData_Reset {
public:
    // 系统复位指令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint8_t unused4;        // NULL
            uint8_t unused5;        // NULL
            uint8_t unused6;        // NULL
            uint8_t unused7;        // NULL
        };
    };

    // 构造函数
    YF25PackData_Reset() : command(YF25Cmd::Reset), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0), unused7(0) {}
};

class YF25PackData_ReleaseBrake {
public:
    // 系统抱闸释放指令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint8_t unused4;        // NULL
            uint8_t unused5;        // NULL
            uint8_t unused6;        // NULL
            uint8_t unused7;        // NULL
        };
    };

    // 构造函数
    YF25PackData_ReleaseBrake() : command(YF25Cmd::ReleaseBrake), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0), unused7(0) {}
};

class YF25PackData_HoldBrake {
public:
    // 系统抱闸锁死指令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint8_t unused4;        // NULL
            uint8_t unused5;        // NULL
            uint8_t unused6;        // NULL
            uint8_t unused7;        // NULL
        };
    };

    // 构造函数
    YF25PackData_HoldBrake() : command(YF25Cmd::HoldBrake), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0), unused7(0) {}
};


class YF25PackData_GetSystemTick {
public:
    // 系统运行时间读取命令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;        // 命令字节
            uint8_t unused1;        // NULL
            uint8_t unused2;        // NULL
            uint8_t unused3;        // NULL
            uint32_t systemTick;    // 系统运行时间（ms）
        };
    };

    // 构造函数
    YF25PackData_GetSystemTick() 
        : command(YF25Cmd::GetSystemTick), unused1(0), unused2(0), unused3(0), systemTick(0) {}
};

class YF25PackData_GetSystemSoftwareVersionDate {
public:
    // 系统软件版本日期读取命令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;            // 命令字节
            uint8_t unused1;            // NULL
            uint8_t unused2;            // NULL
            uint8_t unused3;            // NULL
            uint32_t versionDate;       // 软件版本日期
        };
    };

    // 构造函数
    YF25PackData_GetSystemSoftwareVersionDate() 
        : command(YF25Cmd::GetSystemSoftwareVersionDate), unused1(0), unused2(0), unused3(0), versionDate(0) {}
};

class YF25PackData_SetCommBrokenProtection {
public:
    // 通讯中断保护时间设置指令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;            // 命令字节
            uint8_t unused1;            // NULL
            uint8_t unused2;            // NULL
            uint8_t unused3;            // NULL
            uint32_t commBreakTime;     // 通讯中断时间
        };
    };

    // 构造函数
    YF25PackData_SetCommBrokenProtection(uint32_t commBreak) 
        : command(YF25Cmd::SetCommBrokenProtection), unused1(0), unused2(0), unused3(0), commBreakTime(commBreak) {}
};

enum class YF25RS485Baudrate : uint8_t {
    Baudrate_115200 = 0,   // 115200bps
    Baudrate_500000 = 1,   // 500kbps
    Baudrate_1000000 = 2,  // 1Mbps
    Baudrate_1500000 = 3,  // 1.5Mbps
    Baudrate_2500000 = 4   // 2.5Mbps
};
enum class YF25CANBaudrate : uint8_t {
    Baudrate_500000 = 0,   // 500kbps
    Baudrate_1000000 = 1   // 1Mbps
};
class YF25PackData_SetBaudRate {
public:
    // 通讯波特率设置指令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;      // 命令字节，对应通讯波特率设置指令
            uint8_t unused1;      // NULL
            uint8_t unused2;      // NULL
            uint8_t unused3;      // NULL
            uint8_t unused4;      // NULL
            uint8_t unused5;      // NULL
            uint8_t unused6;      // NULL
            YF25RS485Baudrate baudrate;     // 波特率
        };
    };

    // 构造函数
    YF25PackData_SetBaudRate(YF25RS485Baudrate baud) 
        : command(YF25Cmd::SetBaudRate), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0), baudrate(baud) {}
    YF25PackData_SetBaudRate(uint32_t baud) 
        : command(YF25Cmd::SetBaudRate), unused1(0), unused2(0), unused3(0), unused4(0), unused5(0), unused6(0) {
            if(baud>=2500000){
                baudrate = YF25RS485Baudrate::Baudrate_2500000;
            }else if(baud>=1500000){
                baudrate = YF25RS485Baudrate::Baudrate_1500000;
            }else if (baud >= 1000000){
                baudrate = YF25RS485Baudrate::Baudrate_1000000;
            }else if(baud >= 500000){
                baudrate = YF25RS485Baudrate::Baudrate_500000;
            }else{
                baudrate = YF25RS485Baudrate::Baudrate_115200;
            }
        }
};

class YF25PackData_GetModel {
public:
    // 电机型号读取命令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;      // 命令字节，对应电机型号读取指令
            char model[7];        // 电机型号，ASCII字符
        };
    };

    // 构造函数
    YF25PackData_GetModel() 
        : command(YF25Cmd::GetModel) {
            std::fill(std::begin(model), std::end(model), 0x00);
        }
};

enum class YF25FunctionIndex : uint8_t {
    ControlMode = 0x01,    // 控制模式设置
    CANIDSetting = 0x02,   // CAN ID 设置
    CommunicationSetting = 0x03, // 通讯状态设置
    ErrorReset = 0x04      // 故障复位的保存方式设置
};
class YF25PackData_FunctionControl {
public:
    // 功能控制指令的数据结构
    union {
        YF25PackData raw;
        struct {
            YF25Cmd command;      // 命令字节
            uint8_t index;        // 功能索引
            uint8_t unused1;      // NULL
            uint8_t unused2;      // NULL
            uint32_t value;       // 功能值
        };
    };

    // 构造函数
    YF25PackData_FunctionControl(uint8_t idx, uint32_t val) 
        : command(YF25Cmd::FunctionControl), index(idx), unused1(0), unused2(0), value(val) {}
};

class YF25Pack{
public:
    YF25Pack();
    uint8_t head;
    uint8_t id;
    uint8_t length;
    YF25PackData packData;
    uint16_t CRC;
    bool verify();
    void calcCRC();
};
#pragma pack(pop)

class YF25Manager;
class YF25 {
public:
    YF25(uint8_t mID = 0xCD);
    void addSync(YF25Cmd code); //添加轮询同步指令
    bool sync(YF25Cmd code);    //主动同步
    void update(YF25Manager *YF25Mgr);
    void onReceived(YF25PackData *pData);

    void stop();
    void setPIArguments(uint8_t currentKP, uint8_t speedKP, uint8_t positionKP, uint8_t currentKI = 100, uint8_t speedKI = 5, uint8_t positionKI = 5, bool save = false);
    void setSpeedControl(float speedControl);
    void setTorqueCurrentControl(float currentControl);
    void setAbsolutePositionControl(float positionControl, float maxSpeed);
    void releaseBrake();
    void holdBrake();

    uint8_t getCurrentKP();
    uint8_t getCurrentKI();
    uint8_t getSpeedKP();
    uint8_t getSpeedKI();
    uint8_t getPositionKP();
    uint8_t getPositionKI();
    float getSystemPower();
    float getEncoderPosition();
    float getVoltage();
    YF25SystemErrorState getErrorFlag();
    float getTorqueCurrent();
    float getSpeed();
    double getHighResolutionMultiTurnAngle();
    float getLowResolutionMultiTurnAngle();
    float getAPhaseCurrent();
    float getBPhaseCurrent();
    float getCPhaseCurrent();
    float getTemperature();
    bool isBrakeReleased();
    uint8_t getMotorID();
    void setBaudRate(YF25RS485Baudrate baudRate);
    uint32_t getSyncLoop();
    bool isOnline();
    bool isOverCurrentDetected();
    bool online;
private:
    uint32_t syncLoopCounter;
    uint8_t motorID;
    uint32_t transferTickUs;
    YF25FIFO<YF25PackData,16> txQueue;   //缓存16个指令
    YF25Cmd syncCode[32];
    uint8_t syncCodeLength;
    uint8_t syncStep;
    //反馈值
    int32_t encoderPosition;            // 编码器位置
    uint32_t voltage;                   // 电压（0.1V）
    YF25SystemErrorState errorState;    // 错误状态
    int16_t torqueCurrent;              // 转矩电流（0.01A）
    int16_t speed;                      // 速度（角度/秒）
    int16_t lrAngle;                    // 电机轴角度，低分辨率（角度）
    int32_t hrAngle;                    // 电机轴角度，高分辨率（角度）
    int16_t currentA;                   // A相电流（0.01A）
    int16_t currentB;                   // B相电流（0.01A）
    int16_t currentC;                   // C相电流（0.01A）
    int8_t temperature;                 // 电机温度（℃）
    bool brakeRelease;                  // 抱闸释放
    float power;                        // 系统功率
    //参数
    uint8_t currentKP;  // 电流环 KP 参数（力矩刚度）
    uint8_t currentKI;  // 电流环 KI 参数
    uint8_t speedKP;    // 速度环 KP 参数（速度刚度）
    uint8_t speedKI;    // 速度环 KI 参数
    uint8_t positionKP; // 位置环 KP 参数（位置刚度）
    uint8_t positionKI; // 位置环 KI 参数
public:
    //保护
    bool overcurrentDetected;

    double maxCurrentThreshold;             //上位机执行的电流保护 最大值
    double highCurrentThreshold;            //上位机执行的电流保护 过流
    uint64_t highCurrentMaxDuration;        //上位机执行的电流保护 过流最大持续时间
    uint64_t highCurrentDuration;           //上位机执行的电流保护 过流持续时间
    uint32_t lastTime;
    double weightedCurrentSum;
    double weightedCurrentAverage;
    double timeSum;
};

class YF25Manager : public RS485{
public:
    YF25Manager(int uart_nr);
    YF25Manager(const HardwareSerial& serial);
    YF25 *motorList[32]; //电机列表
    uint8_t motorPriorityList[32];  //优先级
    uint8_t motorOnlineList[32];    //电机在线列表
    uint32_t failedPacks[32];
    bool registerMotor(YF25 *motor, uint8_t priority = 1);
    bool registerMotor(YF25 &motor, uint8_t priority = 1);
    void update();
    void sendToMotor(YF25 *motor, YF25PackData *data);
    void sendToAll(YF25PackData *data);
    void startSync();
    void stopSync();
    bool isBatchTransferComplete();
    void listAllMotors();
    uint8_t currentMotorID;

    bool sync;
    //发送
    YF25Pack txPack;
    //接收器
    YF25Pack rxPack;
    uint8_t receiveStatus;
    uint8_t receiveLength;
    uint32_t receiveTimedoutTick;;
    uint32_t responseWaitTick;
private:
    bool onBatchTransferComplete;
    bool isTransferingBatch;  // 正在传输，批量
    bool isTransfering;       // 正在传输
    uint8_t motorPriorityRunning;  //当前电机优先级

};