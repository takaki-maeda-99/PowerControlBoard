#include <Arduino.h>
#include <ps5Controller.h>

enum SystemState {
  READY = 0,
  OVERCURRENT = 1,
  UNDERVOLTAGE = 2,
  NOT_CONNECTED = 3,
  E_STOP = 4
};

const int8_t CAN_RX = 19;
const int8_t CAN_TX = 18;
const int8_t UART_RX = 22;
const int8_t UART_TX = 23;
const int8_t BUZZ = 17;
const int8_t POWER_ON = 16;
const int8_t V_SENSE = 35;
const int8_t I_SENSE = 34;

void buzzPattern(int count, int on_time, int off_time) {
  for(int i = 0; i < count; i++) {
    analogWrite(BUZZ, 250);
    delay(on_time);
    analogWrite(BUZZ, 0);
    delay(off_time);
  }
}

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
  pinMode(V_SENSE, INPUT);
  pinMode(I_SENSE, INPUT);
  pinMode(POWER_ON, OUTPUT);
  digitalWrite(POWER_ON, HIGH);
  ps5.begin("4c:b9:9b:8a:c3:07");

  buzzPattern(3, 100, 100);
}


void loop() {
  float i_sense = analogRead(I_SENSE);//*3.3/(4095.0*0.001*20.0);
  float v_sense = analogRead(V_SENSE)*3.3*11.9/4095.0;
  SystemState state = NOT_CONNECTED;
  

  if(i_sense > 100){
    state = OVERCURRENT;
    digitalWrite(POWER_ON, LOW);
    while(true) {
      buzzPattern(1, 100, 100);
    }
  } 
  else if(v_sense < 22.0){
    state = UNDERVOLTAGE;
    digitalWrite(POWER_ON, LOW);
    while(true) {
      buzzPattern(1, 500, 500);
    }
  }
  if(!ps5.isConnected()) {
    state = NOT_CONNECTED;
    digitalWrite(POWER_ON, LOW);
    buzzPattern(2, 100, 500);
  }
  else if(digitalRead(POWER_ON) == LOW) {
    state = E_STOP;
  }
  else {
    state = READY;
  }
  
  uint16_t buttons = 0;
  int16_t lx = 0, ly = 0, rx = 0, ry = 0;
  uint8_t l2 = 0, r2 = 0;
  
  if (ps5.isConnected()) {
    buttons |= ps5.Square() << 0;
    buttons |= ps5.Cross() << 1;
    buttons |= ps5.Circle() << 2;
    buttons |= ps5.Triangle() << 3;
    buttons |= ps5.L1() << 4;
    buttons |= ps5.R1() << 5;
    buttons |= ps5.L3() << 6;
    buttons |= ps5.R3() << 7;
    buttons |= ps5.Share() << 8;
    buttons |= ps5.Options() << 9;
    buttons |= ps5.PSButton() << 10;
    buttons |= ps5.Touchpad() << 11;
    buttons |= ps5.Up() << 12;
    buttons |= ps5.Down() << 13;
    buttons |= ps5.Left() << 14;
    buttons |= ps5.Right() << 15;
    
    lx = ps5.LStickX();
    ly = ps5.LStickY();
    rx = ps5.RStickX();
    ry = ps5.RStickY();
    l2 = ps5.L2Value();
    r2 = ps5.R2Value();
    
    if (ps5.Touchpad()) {
      digitalWrite(POWER_ON, !digitalRead(POWER_ON));
      buzzPattern(1, 500, 0);
    }
  }
  
  Serial2.printf("%d,%u,%d,%d,%d,%d,%u,%u\n", 
                 state, buttons, lx, ly, rx, ry, l2, r2);

  Serial.printf("I_Sense: %.2f A, V_Sense: %.2f V", i_sense, v_sense);

  Serial.printf("%d,%u,%d,%d,%d,%d,%u,%u\n", 
                state, buttons, lx, ly, rx, ry, l2, r2);
  
  // 受信側マイコンでの処理例:
  // if(Serial.available()) {
  //   String data = Serial.readStringUntil('\n');
  //   int values[8];
  //   int index = 0;
  //   char* token = strtok(data.c_str(), ",");
  //   while(token && index < 8) {
  //     values[index++] = atoi(token);
  //     token = strtok(NULL, ",");
  //   }
  //   // values[0]: state (0=READY, 1=OVERCURRENT, 2=UNDERVOLTAGE, 3=NOT_CONNECTED, 4=E_STOP)
  //   // values[1]: buttons ビット配置
  //   // values[2]: lx (-128~127), values[3]: ly (-128~127)
  //   // values[4]: rx (-128~127), values[5]: ry (-128~127)
  //   // values[6]: l2 (0~255), values[7]: r2 (0~255)
  //   
  //   if(values[1] & (1<<0)) Serial.println("Square pressed");
  //   if(values[1] & (1<<1)) Serial.println("Cross pressed");
  //   if(values[1] & (1<<2)) Serial.println("Circle pressed");
  //   if(values[1] & (1<<3)) Serial.println("Triangle pressed");
  //   if(values[1] & (1<<4)) Serial.println("L1 pressed");
  //   if(values[1] & (1<<5)) Serial.println("R1 pressed");
  //   if(values[1] & (1<<6)) Serial.println("L3 pressed");
  //   if(values[1] & (1<<7)) Serial.println("R3 pressed");
  //   if(values[1] & (1<<8)) Serial.println("Share pressed");
  //   if(values[1] & (1<<9)) Serial.println("Options pressed");
  //   if(values[1] & (1<<10)) Serial.println("PS pressed");
  //   if(values[1] & (1<<11)) Serial.println("Touchpad pressed");
  //   if(values[1] & (1<<12)) Serial.println("Up pressed");
  //   if(values[1] & (1<<13)) Serial.println("Down pressed");
  //   if(values[1] & (1<<14)) Serial.println("Left pressed");
  //   if(values[1] & (1<<15)) Serial.println("Right pressed");
  // }
  
  delay(50);

}
