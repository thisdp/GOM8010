#include "YF25.h"
#define lerp(a,b,s) (b+(a-b)*s)


CRC16 YF25CRC(CRC16MODBUS);

YF25Pack::YF25Pack(){
  head = 0x3E;
  length = 8;
}

void YF25Pack::calcCRC(){
  YF25CRC.clear();
  YF25CRC.update((uint8_t *)this,sizeof(YF25Pack)-2);
  CRC = YF25CRC.get();
}

bool YF25Pack::verify(){
  YF25CRC.clear();
  YF25CRC.update((uint8_t *)this,sizeof(YF25Pack)-2);
  return CRC == YF25CRC.get();
}

YF25::YF25(uint8_t mID) : currentKP(0), currentKI(0), speedKP(0), speedKI(0), positionKP(0), positionKI(0){ //初始化PI参数为0
  motorID = mID;
  syncCodeLength = 0;
  syncStep = 0;
  online = false;
  syncLoopCounter = 0;

  //电流保护
  overcurrentDetected = false;
  maxCurrentThreshold = 15;             //上位机执行的电流保护 最大值
  highCurrentThreshold = 10;            //上位机执行的电流保护 过流
  highCurrentMaxDuration = 4000000;     //上位机执行的电流保护 过流最大持续时间
  highCurrentDuration = 0;              //上位机执行的电流保护 过流持续时间
  lastTime = micros();
  weightedCurrentSum = 0;
  weightedCurrentAverage = 0;
  timeSum = 0;
  sync(YF25Cmd::ReadPIArgs);  //读取PI参数
}

void YF25::addSync(YF25Cmd code){
  syncCode[syncCodeLength] = (YF25Cmd)code;
  syncCodeLength++;
}

bool YF25::sync(YF25Cmd cmd){
  if(txQueue.isFull()) return false;
  switch(cmd){
    case YF25Cmd::ReadPIArgs:{
      YF25PackData_ReadPIArgs tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
    case YF25Cmd::ReadMultiTurnPosition:{
      YF25PackData_ReadMultiTurnPosition tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
    case YF25Cmd::ReadMotorState1:{
      YF25PackData_ReadMotorState1 tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
    case YF25Cmd::ReadMotorState2:{
      YF25PackData_ReadMotorState2 tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
    case YF25Cmd::ReadMotorState3:{
      YF25PackData_ReadMotorState3 tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
    case YF25Cmd::GetSystemPower:{
      YF25PackData_GetSystemPower tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
    case YF25Cmd::ReadMultiTurnAngle:{
      YF25PackData_ReadMultiTurnAngle tp;
      txQueue.enqueue(*((YF25PackData *)&tp));
      break;
    }
  }
  return true;
}

void YF25::update(YF25Manager *YF25Mgr){
  if((micros()-transferTickUs) < 500) return;  //如果正在传输并且未超时，则跳过
  bool pendingToTransfer = false;
  if(!txQueue.isEmpty()){ //优先传输txQueue，如果有
    pendingToTransfer = true;
  }else{  //如果没有待传输的任务
    if(syncCodeLength == 0) return;
    if(sync(syncCode[syncStep])){
      pendingToTransfer = true;
      syncStep++;
      syncStep %= syncCodeLength;
      if(syncStep == 0) syncLoopCounter++;
    }
  }
  if(pendingToTransfer){
    YF25PackData pData;
    txQueue.peek(pData);
    YF25Mgr->sendToMotor(this,&pData);
  }
}

void YF25::onReceived(YF25PackData *rcv){
  txQueue.dequeue();
  switch(rcv->command){
    case YF25Cmd::ReadMotorState1:{  //读取电机状态1
      YF25PackData_ReadMotorState1 *data = (YF25PackData_ReadMotorState1 *)rcv;
      temperature = data->temperature;
      brakeRelease = data->brakeRelease;
      voltage = data->voltage;
      errorState = data->errorState;
      break;
    }
    case YF25Cmd::ReadMotorState3:{  //读取电机状态3
      YF25PackData_ReadMotorState3 *data = (YF25PackData_ReadMotorState3 *)rcv;
      temperature = data->temperature;
      currentA = data->currentA;
      currentB = data->currentB;
      currentC = data->currentC;
      double currentAf = currentA;
      double currentBf = currentB;
      double currentCf = currentC;
      double newCurrent = sqrt(currentAf*currentAf + currentBf*currentBf + currentCf*currentCf)/100.0;
      uint32_t currentTime = micros();
      uint32_t deltaTime = currentTime-lastTime;
      // 应用衰减，用于减少旧数据的影响
      weightedCurrentSum *= 0.9;
      timeSum *= 0.9;
      // 更新加权和与时间总和
      weightedCurrentSum += newCurrent * deltaTime;
      timeSum += deltaTime;
      weightedCurrentAverage = timeSum == 0 ? 0 : weightedCurrentSum / timeSum;
      lastTime = currentTime;
      if(weightedCurrentAverage >= highCurrentThreshold){  //电流过大
        highCurrentDuration += deltaTime*(weightedCurrentAverage/highCurrentThreshold);   //反时限过流保护
        if(highCurrentDuration >= highCurrentMaxDuration){  //如果到达过流时间
          if(highCurrentDuration >= highCurrentMaxDuration*2) //限幅
            highCurrentDuration = highCurrentMaxDuration*2;
          overcurrentDetected = true; //判定为过流
        }
      }else{  //电流正常
        if(highCurrentDuration <= deltaTime){ //如果时间小于delta时间
          highCurrentDuration = 0;  //置0
          overcurrentDetected = false; //冷却完毕
        }else
          highCurrentDuration -= deltaTime; //否则减去delta时间
      }
      break;
    }
    case YF25Cmd::ReadMotorState2:                 // 读取电机状态2
    case YF25Cmd::SetTorqueCurrentControl:         // 该数据包返回格式与 读取电机状态2 一样
    case YF25Cmd::SetSpeedControl:                 // 该数据包返回格式与 读取电机状态2 一样
    case YF25Cmd::SetAbsolutePositionControl:      // 该数据包返回格式与 读取电机状态2 一样
    case YF25Cmd::SetIncrementalPositionControl:{  // 该数据包返回格式与 读取电机状态2 一样
      YF25PackData_ReadMotorState2 *data = (YF25PackData_ReadMotorState2 *)rcv;
      torqueCurrent = data->torqueCurrent;  // 力矩电流
      temperature = data->temperature;      // 温度
      speed = data->speed;                  // 转速
      lrAngle = data->angle;                // 角度
      break;
    }
    case YF25Cmd::ReadMultiTurnPosition:{  // 读取多圈编码器
      YF25PackData_ReadMultiTurnPosition *data = (YF25PackData_ReadMultiTurnPosition *)rcv;
      encoderPosition = data->encoder;
      break;
    }
    case YF25Cmd::ReadMultiTurnAngle:{
      YF25PackData_ReadMultiTurnAngle *data = (YF25PackData_ReadMultiTurnAngle *)rcv;
      hrAngle = data->angle;
      break;
    }
    case YF25Cmd::WritePIArgsToRAM:  // 写PI参数
    case YF25Cmd::WritePIArgsToROM:  // 写PI参数
    case YF25Cmd::ReadPIArgs:{  // 读取PI参数
      YF25PackData_ReadPIArgs *data = (YF25PackData_ReadPIArgs *)rcv;
      currentKP = data->currentKP;    // 电流环 KP 参数（力矩刚度）
      currentKI = data->currentKI;    // 电流环 KI 参数
      speedKP = data->speedKP;        // 速度环 KP 参数（速度刚度）
      speedKI = data->speedKI;        // 速度环 KI 参数
      positionKP = data->positionKP;  // 位置环 KP 参数（位置刚度）
      positionKI = data->positionKI;  // 位置环 KI 参数
      break;
    }
    case YF25Cmd::GetSystemPower:{
      YF25PackData_GetSystemPower *data = (YF25PackData_GetSystemPower *)rcv;
      power = data->power;
      break;
    }
  }
}

/*设置*/
void YF25::stop(){
  YF25PackData_Stop tp;
  txQueue.enqueue(*((YF25PackData *)&tp));
}

void YF25::setPIArguments(uint8_t currentKP, uint8_t speedKP, uint8_t positionKP, uint8_t currentKI, uint8_t speedKI, uint8_t positionKI, bool save){
  if(save){
    YF25PackData_WritePIArgsToROM tp(currentKP, speedKP, positionKP,currentKI, speedKI, positionKI);
    txQueue.enqueue(*((YF25PackData *)&tp));
  }else{
    YF25PackData_WritePIArgsToRAM tp(currentKP, speedKP, positionKP,currentKI, speedKI, positionKI);
    txQueue.enqueue(*((YF25PackData *)&tp));
  }
}

void YF25::setSpeedControl(float speed){
  YF25PackData_SetSpeedControl tp(speed*100);
  txQueue.enqueue(*((YF25PackData *)&tp));
}

void YF25::setTorqueCurrentControl(float current){
  YF25PackData_SetTorqueCurrentControl tp(current*100); //0.01A
  txQueue.enqueue(*((YF25PackData *)&tp));
}

void YF25::setAbsolutePositionControl(float position, float maxSpeed){
  YF25PackData_SetAbsolutePositionControl tp(position*100, maxSpeed);
  txQueue.enqueue(*((YF25PackData *)&tp));
}

void YF25::releaseBrake(){
  YF25PackData_ReleaseBrake tp;
  txQueue.enqueue(*((YF25PackData *)&tp));
}

void YF25::holdBrake(){
  YF25PackData_HoldBrake tp;
  txQueue.enqueue(*((YF25PackData *)&tp));
}

void YF25::setBaudRate(YF25RS485Baudrate baud){
  YF25PackData_SetBaudRate tp(baud);
  txQueue.enqueue(*((YF25PackData *)&tp));
}

/*获取*/
bool YF25::isOverCurrentDetected(){
  return overcurrentDetected;
}

bool YF25::isOnline(){
  return online;
}

float YF25::getSystemPower(){
  return power*10.0;
}

uint8_t YF25::getCurrentKP(){
  return currentKP;
}

uint8_t YF25::getCurrentKI(){
  return currentKI;
}

uint8_t YF25::getSpeedKP(){
  return speedKP;
}

uint8_t YF25::getSpeedKI(){
  return speedKI;
}

uint8_t YF25::getPositionKP(){
  return positionKP;
}

uint8_t YF25::getPositionKI(){
  return positionKI;
}

uint8_t YF25::getMotorID(){
  return motorID;
}

float YF25::getEncoderPosition(){
  return encoderPosition;
}

float YF25::getVoltage(){
  return voltage*10.0;
}

float YF25::getTemperature(){
  return temperature;
}

float YF25::getTorqueCurrent(){
  return torqueCurrent/100.0;
}

float YF25::getSpeed(){
  return speed;
}

double YF25::getHighResolutionMultiTurnAngle(){
  double hrfAngle = hrAngle;
  return hrfAngle/100.0;
}

float YF25::getLowResolutionMultiTurnAngle(){
  return lrAngle;
}

bool YF25::isBrakeReleased(){
  return brakeRelease;
}

YF25SystemErrorState YF25::getErrorFlag(){
  return errorState;
}

float YF25::getAPhaseCurrent(){
  return currentA/100.0;
}

float YF25::getBPhaseCurrent(){
  return currentB/100.0;
}

float YF25::getCPhaseCurrent(){
  return currentC/100.0;
}

uint32_t YF25::getSyncLoop(){
  return syncLoopCounter;
}

//YF25 Manager
YF25Manager::YF25Manager(int uart_nr):RS485(uart_nr){
  receiveStatus = YF25Mgr_WaitHead;
  receiveLength = 0;
  currentMotorID = 0;
  motorPriorityRunning = 0;
  onBatchTransferComplete = false;
  sync = true;
  for(uint8_t i=0;i<32;i++){
    motorList[i] = 0;
    failedPacks[i] = 0;
  }
}

YF25Manager::YF25Manager(const HardwareSerial& serial):RS485(serial){
  receiveStatus = YF25Mgr_WaitHead;
  receiveLength = 0;
  currentMotorID = 0;
  motorPriorityRunning = 0;
  onBatchTransferComplete = false;
  sync = true;
  for(uint8_t i=0;i<32;i++){
    motorList[i] = 0;
    failedPacks[i] = 0;
  }
}

void YF25Manager::startSync(){
  sync = true;
}

void YF25Manager::stopSync(){
  sync = false;
}

bool YF25Manager::registerMotor(YF25 *motor, uint8_t priority){
  uint8_t mID = motor->getMotorID();
  if(motorList[mID] != 0) return false;
  motorList[mID] = motor;
  motorPriorityList[mID] = priority;
  return true;
}

bool YF25Manager::registerMotor(YF25 &motor, uint8_t priority){
  return registerMotor(&motor, priority);
}

void YF25Manager::sendToMotor(YF25 *motor, YF25PackData *pData){
  isTransfering = true;
  txPack.id = motor->getMotorID();
  txPack.packData = *pData;
  txPack.calcCRC();
  /*Serial.print("Send To ");
  Serial.println(pData->command);*/
  send((uint8_t *)&txPack,sizeof(txPack));
  responseWaitTick = micros();
}

void YF25Manager::sendToAll(YF25PackData *pData){
  isTransferingBatch = true;
  txPack.id = 0xCD;
  txPack.packData = *pData;
  txPack.calcCRC();
  //Serial.println("Send To All");
  send((uint8_t *)&txPack,sizeof(txPack));
  onBatchTransferComplete = false;
}

void YF25Manager::listAllMotors(){
  for(uint8_t i=0;i<32;i++)
    motorOnlineList[i] = false;
  YF25PackData_GetSystemMode tp;
  sendToAll((YF25PackData *)&tp);
}

bool YF25Manager::isBatchTransferComplete(){
  if(onBatchTransferComplete){
    onBatchTransferComplete = false;
    return true;
  }
  return false;
}

void YF25Manager::update(){
  if(isTransferingBatch) isTransfering = true;
  while(available()){ //先读取
    uint8_t d = read();
    //Serial.println(d);
    receiveTimedoutTick = micros();
    switch(receiveStatus){
      case YF25Mgr_WaitHead:  //读头
        if(d == 0x3E){
          rxPack.head = d;
          receiveStatus = YF25Mgr_WaitID;
        }
      break;
      case YF25Mgr_WaitID:  //读ID
        receiveStatus = YF25Mgr_WaitLength;
        rxPack.id = d;
      break;
      case YF25Mgr_WaitLength:  //读长度
        if(d != 8){ //一般长度都是8
          receiveStatus = YF25Mgr_WaitHead;
          break;
        }
        receiveStatus = YF25Mgr_WaitData;
        rxPack.length = d;
        receiveLength = 0;
      break;
      case YF25Mgr_WaitData:
        ((uint8_t *)&(rxPack.packData))[receiveLength] = d;
        receiveLength++;
        if(receiveLength == rxPack.length){
          receiveStatus = YF25Mgr_WaitCRC1;
        }
      break;
      case YF25Mgr_WaitCRC1:
        rxPack.CRC = d;
        receiveStatus = YF25Mgr_WaitCRC2;
      break;
      case YF25Mgr_WaitCRC2:
        rxPack.CRC += (d<<8);
        if(rxPack.verify()){
          isTransfering = false;
          motorOnlineList[rxPack.id] = true;
          if(motorList[rxPack.id]){
            motorList[rxPack.id]->online = true;
            motorList[rxPack.id]->onReceived(&(rxPack.packData));
          }
        }else{
          failedPacks[rxPack.id] ++;
        }
        receiveStatus = YF25Mgr_WaitHead;
      break;
    }
  }
  if(isTransfering){  //正在传输
    bool waitPackHeadTimedout = (receiveStatus == YF25Mgr_WaitHead && micros()-responseWaitTick>=5000);
    bool waitPackDataTimedout = (receiveStatus != YF25Mgr_WaitHead && micros()-receiveTimedoutTick>=1000);
    if( waitPackHeadTimedout || waitPackDataTimedout ){  //如果 等待数据包头超时 或 接收数据包超时
      if(receiveStatus > YF25Mgr_WaitID) failedPacks[rxPack.id] ++; //当前丢包
      receiveStatus = YF25Mgr_WaitHead;
      isTransfering = false;
      if(waitPackHeadTimedout){
        isTransferingBatch = false; //如果是等待数据包头超时，则关闭批量传输
        onBatchTransferComplete = true;
      }
    }   
  }
  if(receiveStatus == YF25Mgr_WaitHead && !isTransfering && sync) {  //如果正在空闲
    for (int i = currentMotorID; i < 32; i++) {
      if (motorList[i]) { // 如果电机有效
        if (motorPriorityRunning == 0) motorPriorityRunning = motorPriorityList[i]; // 检查优先级，如果为0则重置
        motorList[i]->update(this); // 更新电机
        motorPriorityRunning--; // 自减优先级
        if(motorPriorityRunning == 0) {  //优先级已经使用完毕
          currentMotorID = i+1; //选择下一个电机
          currentMotorID %= 32;
        }
        return; // 完成更新后退出循环
      }
    }
    currentMotorID = 0; //重置
    motorPriorityRunning = 0;
  }
}
