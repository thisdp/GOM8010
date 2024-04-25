#include <GOM8010.h>
M8010Manager motorMgr(Serial1);
M8010 motor[15] = {
  M8010(0),   M8010(1),   M8010(2),   M8010(3),   M8010(4),
  M8010(5),   M8010(6),   M8010(7),   M8010(8),   M8010(9),
  M8010(10),  M8010(11),  M8010(12),  M8010(13),  M8010(14),
};

RS485Config P1Config = {
  .baudrate = 4000000,
  .config = SERIAL_8N1,
  .pinRX = 35,
  .pinTX = 33,
  .pinDE = 32,
  .pinRE = -1,
  .readBack = true,
};

void setup(){
  Serial.begin(115200);
  delay(500);
  Serial.println("");
  delay(500);
  for(uint8_t i=0;i<15;i++){
    motorMgr.registerMotor(motor[i]);
  }
  motorMgr.begin(P1Config);
}

uint32_t lastTick = 0;
void loop(){
  motorMgr.update();
  if(millis()-lastTick>=1000){
    Serial.println("Online: ");
    for(uint8_t i=0;i<15;i++){
      if(motor[i].isOnline()){
        Serial.println(i);
      }
    }
    lastTick = millis();
  }
}