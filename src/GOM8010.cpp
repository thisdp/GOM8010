#include "GOM8010.h"

CRC16 GOM8010CRC(CRC16CCITT);

//Pack
M8010TXPack::M8010TXPack(){
  headA = 0xFE;
  headB = 0xEE;
}

void M8010TXPack::calcCRC(){
  GOM8010CRC.clear();
  GOM8010CRC.update((uint8_t *)this,sizeof(M8010TXPack)-2);
  CRC = GOM8010CRC.get();
}

M8010RXPack::M8010RXPack(){
  headA = 0xFD;
  headB = 0xEE;
}

bool M8010RXPack::verify(){
  GOM8010CRC.clear();
  GOM8010CRC.update((uint8_t *)this,sizeof(M8010RXPack)-2);
  return CRC == GOM8010CRC.get();
}

//M8010 Motor
M8010::M8010(uint8_t motorID, uint8_t mode){
  ID = motorID;
  status = mode;
  online = false;
}

void M8010::setMotorID(uint8_t id){
  ID = id;
}

uint8_t M8010::getMotorID(){
  return ID;
}

void M8010::setExpectedTorque(float eTorque, bool includeReduction){
  if(includeReduction) eTorque /= 6.33;
  expectedTorque = clamp(eTorque,-127.99,127.99);
}
void M8010::setExpectedSpeed(float eSpeed, bool includeReduction){
  if(includeReduction) eSpeed *= 6.33;
  expectedSpeed = clamp(eSpeed,-804.0/(2*PI),804.0/(2*PI));
}

void M8010::setExpectedPosition(float position, bool includeReduction){
  if(includeReduction) position *= 6.33;
  expectedPosition = clamp(position,-411774.0/(2*PI),411774.0/(2*PI));
}

void M8010::setPositionStiffness(float kP){
  kPosition = clamp(kP,0,25.599);
}

void M8010::setSpeedStiffness(float kW){
  kSpeed = clamp(kW,0,25.599);
}

float M8010::getExpectedTorque(bool includeReduction){
  if(includeReduction) return expectedTorque*6.33;
  return expectedTorque;
}
float M8010::getExpectedSpeed(bool includeReduction){
  if(includeReduction) return expectedSpeed/6.33;
  return expectedSpeed;
}

float M8010::getExpectedPosition(bool includeReduction){
  if(includeReduction) return expectedPosition/6.33;
  return expectedPosition;
}

float M8010::getPositionStiffness(){
  return kPosition;
}

float M8010::getSpeedStiffness(){
  return kSpeed;
}

void M8010::setExpectedTorqueByOffset(float eTorque, bool includeReduction){
  if(includeReduction) eTorque /= 6.33;
  expectedTorque = clamp(expectedTorque+eTorque,-127.99,127.99);
}
void M8010::setExpectedSpeedByOffset(float eSpeed, bool includeReduction){
  if(includeReduction) eSpeed *= 6.33;
  expectedSpeed = clamp(expectedSpeed+eSpeed,-804.0/(2*PI),804.0/(2*PI));
}

void M8010::setExpectedPositionByOffset(float position, bool includeReduction){
  if(includeReduction) position *= 6.33;
  expectedPosition = clamp(expectedPosition+position,-411774.0/(2*PI),411774.0/(2*PI));
}

void M8010::setPositionStiffnessByOffset(float kP){
  kPosition = clamp(kPosition+kP,0,25.599);
}

void M8010::setSpeedStiffnessByOffset(float kW){
  kSpeed = clamp(kSpeed+kW,0,25.599);
}
/*Utilities*/
void M8010::stop(){
  setExpectedTorque(0.0);
  setExpectedSpeed(0.0);
  setExpectedPosition(0);
  setPositionStiffness(0.0);
  setSpeedStiffness(0.0);
}
/*Utilities*/

bool M8010::isOnline(){
  return online;
}

uint8_t* M8010::serialize(){
  txPack.ID = ID&0x0F;
  txPack.status = status&0b00000111;
  txPack.expectedTorque = 256.0*expectedTorque;
  txPack.expectedSpeed = 256.0*expectedSpeed;
  txPack.expectedPosition = 32768.0*expectedPosition;
  txPack.kPosition = kPosition*1280.0;
  txPack.kSpeed = kSpeed*1280.0;
  txPack.calcCRC();
  return (uint8_t*)&txPack;
}

float M8010::getRealTorque(bool includeReduction){
  if(includeReduction)
    return rxPack.realTorque/256.0*6.33;
  else
    return rxPack.realTorque/256.0;
}

float M8010::getRealSpeed(bool includeReduction){
  if(includeReduction)
    return rxPack.realSpeed/256.0/6.33;
  else
    return rxPack.realSpeed/256.0;
}

float M8010::getRealPosition(bool includeReduction){
  if(includeReduction)
    return rxPack.realPosition/32768.0/6.33;
  else
    return rxPack.realPosition/32768.0;
}

uint8_t M8010::getTemperature(){
  return rxPack.temperature;
}

uint8_t M8010::getErrorFlag(){
  return rxPack.errorFlag;
}

//M8010 Manager
M8010Manager::M8010Manager(int uart_nr):RS485(uart_nr){
  receiveStatus = M8010Mgr_Idle;
  receiveLength = 0;
  nextMotorID = 0;
  currentMotorID = 0;
  for(uint8_t i=0;i<15;i++){
    motorList[i] = 0;
    failedPacks[i] = 0;
  }
}

M8010Manager::M8010Manager(const HardwareSerial& serial):RS485(serial){
  receiveStatus = M8010Mgr_Idle;
  receiveLength = 0;
  nextMotorID = 0;
  currentMotorID = 0;
  for(uint8_t i=0;i<15;i++){
    motorList[i] = 0;
    failedPacks[i] = 0;
  }
}

bool M8010Manager::registerMotor(M8010 &motor){
  if(motorList[motor.getMotorID()] != 0) return false;
  motorList[motor.getMotorID()] = &motor;
  return true;
}

void M8010Manager::update(){
  //Serial.println(available());
  while(available()){ //先读取
    uint8_t d = read();
    lastReceiveTick = micros();
    switch(receiveStatus){
      case M8010Mgr_WaitHeadFD:
        if(d == 0xFD)
          receiveStatus = M8010Mgr_WaitHeadEE;
      break;
      case M8010Mgr_WaitHeadEE:
        if(d == 0xEE){
          receiveStatus = M8010Mgr_ReadData;
          receiveLength = 2;
        }
      break;
      case M8010Mgr_ReadData:
        uint8_t *pRxPack = (uint8_t*)&rxPackReceive;
        pRxPack[receiveLength] = d;
        receiveLength++;
        if(receiveLength == 16){
          if(rxPackReceive.verify()){
            if(motorList[currentMotorID]){
              motorList[currentMotorID]->rxPack = rxPackReceive;
              motorList[currentMotorID]->online = true;
              if(motorList[currentMotorID]->onReceived) motorList[currentMotorID]->onReceived();
            }
          }else{
            failedPacks[currentMotorID] ++;
          }
          receiveStatus = M8010Mgr_Idle;
        }
      break;
    }
  }
  if(receiveStatus != M8010Mgr_Idle && micros()-lastReceiveTick>=100){  //如果在非空闲状态下没有读取到数据，并且超时
    receiveStatus = M8010Mgr_Idle;
    failedPacks[currentMotorID] ++; //当前丢包
  }
  if(receiveStatus == M8010Mgr_Idle){
    for(uint8_t i=nextMotorID;i<15;i++){ //从nextMotorID开始寻找已注册电机
      if(motorList[i]){ //如果找到
        send(motorList[i]->serialize(),sizeof(M8010TXPack)); //发送同步数据包
        currentMotorID = i;
        nextMotorID = i;
        receiveStatus = M8010Mgr_WaitHeadFD;
        break;  //跳出循环
      }
    }
    nextMotorID ++;
    nextMotorID %= 15; //对15取模
  }
}