
#include<stdio.h>
#include<stdbool.h>
#include<time.h>
#include <Wire.h>
#include <VL53L0X.h>
VL53L0X sensor;  // 宣告VL53L0X類型物件
int I2C_SDA = 21;
int I2C_SCL = 26;
int distance[7] = {
  -1,
  -1,
  -1,
  -1,
  -1,
  -1,
  -1
};
int current_distance;
int sensorAngle = 0;
int trueAngle = 0;
int sensorVal = 0;
int _alarm = 0;
char buf[256];

int X0 = 1, X1 = 0, X10 = 1, X11 = 0, X12 = 0, X2 = 0,reset=0,count=0;
bool input_control_parameter=true,A_one_Finished=false,distance_detection_start=true;
int servoPin = 18;
int beePin = 6;
int pos = 0;
int noSound = 0;
int echo = 200;
int threwhole = 20;
char* _time;
// Fill-in information from your Blynk Template here
#define BLYNK_TEMPLATE_ID "TMPL8HjmPoEO"
#define BLYNK_DEVICE_NAME "LED"
#define BLYNK_FIRMWARE_VERSION        "0.1.1"
#define BLYNK_PRINT Serial
#define LED_BUILTIN 2
#define DELAY 500
#define ABS(x) ({ \
    int _x = x; \
    ((_x > 0)?_x:(-1*_x)); \
})
//#define BLYNK_DEBUG
#define APP_DEBUG
#include "BlynkEdgent.h"
char* timestamp()
{
  time_t rawtime;
  struct tm * timeinfo;

  time ( &rawtime );
  timeinfo = localtime ( &rawtime );
  printf ( "The current date/time is: %s", asctime (timeinfo) );
  return  asctime (timeinfo);
}
BLYNK_WRITE(V0)
{
  int pinValue = param.asInt();//assiging incoming value from V0 to a variable
  digitalWrite(LED_BUILTIN, pinValue);
}

BlynkTimer timer;

void myTimer()
{
  // This function describes what will happen with each timer tick
  // e.g. writing sensor value to datastream V5
  if (sensor.timeoutOccurred())
  {
    Serial.print("TIMEOUT");
  }

  Blynk.virtualWrite(V1, sensorVal);
  Blynk.virtualWrite(V2, sensorAngle);
  Blynk.virtualWrite(V3, trueAngle);
  Blynk.virtualWrite(V0, _alarm);
  Blynk.virtualWrite(V4, _time);
  Serial.println(sensorVal);
}
void setup() {
  timer.setInterval(500L, myTimer);
  printf("X0 = %d,X1 = %d,X2 = %d\n",X0 ,X1 ,X2 );
  pinMode(servoPin, OUTPUT);
  pinMode(beePin, OUTPUT); 

  distance_detected_setup();
  
  delay(100);
  pinMode(LED_BUILTIN, OUTPUT);
  BlynkEdgent.begin();
}

void distance_detected_setup(){
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL);  // 啟動I2C通訊
  Serial.println("Setting timeout...");
  sensor.setTimeout(500);  // 設定感測器超時時間
  // 若無法初始化感測器（如：硬體沒有接好），則顯示錯誤訊息。
  Serial.println("Initial sensor...");
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {Serial.println("Failed to detect and initialize sensor!");}
  }
}

// the loop function runs over and over again forever
void loop() {

  datapath0();
  grafcet0();
  printf("X0 = %d,X1 = %d,X2 = %d\n",X0 ,X1 ,X2 );                       // wait for a second

}

void grafcet0()
{

  if((X0 == 1) && (input_control_parameter))
  {
    X0 = 0;
    X1 = 1;
    return;
  }

  if((X1 == 1) && (A_one_Finished))
  {
    X1 = 0;
    X2 = 1;
    return;
  }

  if(X2 == 1)
  {
    if(reset=1)
    {
      X2 = 0;
      X0 = 1;
    }
    else if( reset=0)
    {
      X2 = 0;
      X1 = 1;
    }
    return;
  }

}
void grafcet1()
{

  if((X10 == 1) && (input_control_parameter))
  {
    X10 = 0;
    X11 = 1;
    return;
  }

  if((X11 == 1) && (1))
  {
    X11 = 0;
    X12 = 1;
    return;
  }

  if((X12 == 1) && (distance_detection_start))
  {
    X12 = 0;
    X10 = 1;
    return;
  }

}
void datapath0()
{
  if(X0 == 1)
    action();
  if(X1 == 1)
    A_one();
  if(X2 == 1)
    A_two();
}
void datapath1()
{
  if(X10 == 1)
    A_one0_action();
  if(X11 == 1)
    A_one1_turbo_rotate_angle();
  if(X12 == 1)
    A_one2_distance_detected();

}
void action()
{
  printf("action activate !!\n");
}
void A_one()
{
  printf("A_one activate !!\n");
  datapath1();
  grafcet1();
  printf("X10 = %d,X11 = %d,X12 = %d\n",X10 ,X11 ,X12 );
}
void A_two()
{
  printf("A_two activate !!\n");
  BlynkEdgent.run();
  timer.run(); 

  // distance not change
  if(ABS(current_distance-distance[count])<threwhole&&distance[count]!=-1){
    _alarm = 1;
    analogWrite(beePin,noSound); 
  }else{ 
    // distance has changed
    sensorVal = sensor.readRangeSingleMillimeters();

    switch(count) {
    case 0:
      sensorAngle = 270;
      trueAngle = -90;
      break;
    case 1:
      sensorAngle = 300;
      trueAngle = -60;
      break;
    case 2: 
      sensorAngle = 0;
      trueAngle = 0;
      break;
    case 3:
      sensorAngle = 60;
      trueAngle = 60;
      break;
    case 4:
      sensorAngle = 90;
      trueAngle = 90;
      break;
    default:
      break;
  }

    analogWrite(beePin,echo); 
    delay(50);
    analogWrite(beePin,noSound);
    _time=timestamp();
    _alarm = 0;
  }
  distance[count]=current_distance;
}

void A_one1_turbo_rotate_angle()
{
  switch(count) {
  case 0:
    ServoWrite(servoPin, -90 , DELAY); //-90
    count++;
    break;
  case 1:
    ServoWrite(servoPin, -60 , DELAY);  //0
    count++;
    break;
  case 2: 
    ServoWrite(servoPin, 0 , DELAY);  //0
    count++;
    break;
  case 3:
    ServoWrite(servoPin, 60 , DELAY); //90
    count++;
    break;
  case 4:
    ServoWrite(servoPin, 90 , DELAY); //90
    count = 0;
    break;
  default:
    break;
  }
  printf("Angle: %d\n", sensorAngle);
  printf("A_one1_turbo_rotate_angle+1 activate !!\n");
}
void A_one2_distance_detected()
{

  printf("A_one2_distance_detected activate !!\n");
  
  // 在序列埠監控視窗顯示測距值
  Serial.print(sensor.readRangeSingleMillimeters());
  // 若發生超時（感測器沒有回應），則顯示“TIMEOUT”。
  if (sensor.timeoutOccurred())
  {
    Serial.print("TIMEOUT");
  }
  delay(500);
  Serial.println();
  current_distance=sensor.readRangeSingleMillimeters();
  
  A_one_Finished=true;
}

void A_one0_action(){
  A_one_Finished=false;
}

double mapf(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
void pwmWrite(int pin, double period, double duty_cycle) //servo、led pin number,pwm period(ms),duty cycle(ms)
{
  int period_us = period * 1000;
  int duty_cycle_us = duty_cycle * 1000;
  digitalWrite(pin, HIGH);
  delayMicroseconds(duty_cycle_us);
  digitalWrite(pin, LOW);
  delayMicroseconds(period_us - duty_cycle_us);
  //Serial.println(String(duty_cycle_us));
}
void ServoWrite(int servoPin, double degree, double delay_ms) //degree -90 ~ 90
{
  double value = mapf(degree, -90.0, 90.0, 0.5, 2.5);
  for (int i = 0; i < delay_ms/30; i++)
  {
    pwmWrite(servoPin, 20, value); //sg90's period is 20 ms
  }
  printf("angle : %lf \n",value);
}
