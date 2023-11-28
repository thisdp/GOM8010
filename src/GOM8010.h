#pragma once
#include "HardwareSerial.h"
#include "CRC16.h"
#include "RS485.h"
#include "math.h"
#define clamp(x,minValue,maxValue) (x<minValue?minValue:x)>maxValue?maxValue:x

#define MERROR_Normal 0
#define MERROR_OverHeat 1
#define MERROR_OverCurrent 2
#define MERROR_OverVoltage 3
#define MERROR_EncoderBroken 4

#pragma pack(push,1)
class M8010TXPack{
public:
  uint8_t headA;
  uint8_t headB;
  uint8_t ID:4;
  uint8_t status:3;
  uint8_t reserved1:1;

  int16_t expectedTorque;
  int16_t expectedSpeed;
  int32_t expectedPosition;
  uint16_t kPosition;
  uint16_t kSpeed;
  uint16_t CRC;
  M8010TXPack();
  void calcCRC();
};

class M8010RXPack{
public:
  uint8_t headA;
  uint8_t headB;
  uint8_t ID:4;
  uint8_t status:3;
  uint8_t reserved1:1;

  int16_t realTorque;
  int16_t realSpeed;
  int32_t realPosition;
  uint8_t temperature;
  uint16_t errorFlag:3;
  uint16_t force:12;
  uint16_t reversed2:1;
  uint16_t CRC;
  M8010RXPack();
  bool verify();
};
#pragma pack(pop)

#define M8010Mgr_Idle 0
#define M8010Mgr_WaitHeadFD 1
#define M8010Mgr_WaitHeadEE 2
#define M8010Mgr_ReadData 3

typedef void(*M8010ReceiveCallBack)();

class M8010{
public:
  M8010TXPack txPack;
  M8010RXPack rxPack;
  M8010(uint8_t motorID, uint8_t mode = 1);
  uint8_t getMotorID();
  void setMotorID(uint8_t id);
  //includeReduction为是否将减速机构的效果考虑进目标参数内
  void setExpectedTorque(float eTorque, bool includeReduction = true);   //期望力矩(牛*米): 输出轴:±810N*m 电机:±127N*m
  void setExpectedTorqueByOffset(float eTorque, bool includeReduction = true);
  float getExpectedTorque(bool includeReduction = true);
  void setExpectedSpeed(float eSpeed, bool includeReduction = true);     //期望速度(转/秒): 输出轴:±20RPS 电机:±127RPS 
  void setExpectedSpeedByOffset(float eSpeed, bool includeReduction = true);
  float getExpectedSpeed(bool includeReduction = true);
  void setExpectedPosition(float position, bool includeReduction = true);//期望位置(圈): 输出轴:±10402圈 电机:±65535圈
  void setExpectedPositionByOffset(float position, bool includeReduction = true);
  float getExpectedPosition(bool includeReduction = true);
  void setPositionStiffness(float kP);//位置刚度 (比例系数 Kp)  可影响力矩、速度模式下的位置
  void setPositionStiffnessByOffset(float kP);
  float getPositionStiffness();
  void setSpeedStiffness(float kW);   //速度刚度 (微分系数 Kd)  可影响力矩、位置模式下的速度
  void setSpeedStiffnessByOffset(float kW);
  float getSpeedStiffness();
  float getRealTorque(bool includeReduction = true);  //期望力矩(牛*米): 输出轴:±810N*m 电机:±127N*m
  float getRealSpeed(bool includeReduction = true);   //期望速度(转/秒): 输出轴:±20RPS 电机:±127RPS 
  float getRealPosition(bool includeReduction = true);//期望位置(圈): 输出轴:±10402圈 电机:±65535圈
  bool isOnline();
  uint8_t getTemperature(); //温度
  uint8_t getErrorFlag();
  void stop();  //停止电机
  uint8_t* serialize();
  M8010ReceiveCallBack onReceived;
  bool online;
private:
  uint8_t ID;
  uint8_t status;
  float expectedTorque;   //期望力矩=τff*256  |τff|≤127.99N*m 
  float expectedSpeed;    //期望速度=256*ωdes/(2Pi)   |ωdes|≤804.0 rad/s
  float expectedPosition; //期望位置=32768*θdes/(2Pi) |θdes|≤411774 rad(≈65535圈)
  float kPosition;        //位置刚度系数=Kp * 1280     0≤Kp≤25.599
  float kSpeed;           //速度刚度系数=Ks * 1280     0≤Kp≤25.599
};

class M8010Manager : public RS485{
public:
  M8010Manager(int uart_nr);
  M8010Manager(const HardwareSerial& serial);
  M8010 *motorList[15]; //电机列表
  uint32_t failedPacks[15];
  bool registerMotor(M8010 &motor);
  void update();
  uint8_t nextMotorID;
  uint8_t currentMotorID;
  //接收器
  M8010RXPack rxPackReceive;
  uint8_t receiveStatus;
  uint8_t receiveLength;
  uint32_t lastReceiveTick;
};