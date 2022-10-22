#include <LiquidCrystal.h>

#include "Wire.h"
#include "SPI.h"  
#include "I2Cdev.h"
#include "MPU6050.h"

LiquidCrystal lcd(13,12,11,10,9,8);

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define woltomierz A0

#define Gry_offset 0
#define Gyr_Gain 0.00763358  
#define Angle_offset 1
#define RMotor_offset 0 
#define LMotor_offset 5
#define pi 3.14159 

long data;
int x, y;
float kp, ki, kd; 
float r_angle, f_angle, omega;
float offset_kat = 3.25 ;
float L_Speed = -50;
float R_Speed = -50;
float LOutput,ROutput;
unsigned long preTime = 0;
float SampleTime = 0.08;
unsigned long lastTime;
float Input, Output;
float errSum, dErr, error, lastErr;
int timeChange; 

float voltage =0;
float procent;

byte stopien[8]= { 
  B00110,
  B01001,
  B01001,
  B00110,
  B00000,
  B00000,
  B00000,
  B00000,
};

int blue=0;

int IN1=2;
int IN2=7;
int ENA=5;

int IN3=3;
int IN4=4;
int ENB=6;

void setup() {
  lcd.createChar(0,stopien);
  lcd.begin(16,2);
  lcd.print("angle:");

  Wire.begin();
  accelgyro.initialize();
  pinMode(IN1,OUTPUT);
  pinMode(IN2,OUTPUT);
  pinMode(IN3,OUTPUT);
  pinMode(IN4,OUTPUT);
  pinMode(ENA,OUTPUT);
  pinMode(ENB,OUTPUT);
  pinMode(woltomierz, INPUT);
  Serial.begin(115200);
}

void loop() {
  accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
  r_angle = (atan2(ay, az) * 180 / pi + Angle_offset + offset_kat);
  omega =  Gyr_Gain * (gx +  Gry_offset);  

  bluetooth();
  akumulator();
  info();

  if (abs(r_angle)<45)
  {
    PID();
    PWMControl();
    // Status();
    delay(3);
  }
  else{
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  }
}

void PID(){
  kp = 15;  
  kd = 900;  
  ki = 0.3;

  unsigned long now = millis();
  float dt = (now - preTime) / 1000.0;
  preTime = now;
  float K = 0.98;
  float A = K / (K + dt);
  f_angle = A * (f_angle + omega * dt) + (1 - A) * r_angle;  

  timeChange = (now - lastTime);
  if(timeChange >= SampleTime){
    Input = f_angle;
    error = Input;
    errSum += error * timeChange;
    dErr = (error - lastErr) / timeChange;
    Output = kp * error + ki * errSum + kd * dErr;
    LOutput = Output + L_Speed ;      
    ROutput = Output + R_Speed ;     
    lastErr = error;
    lastTime = now;
  }
}

void PWMControl(){
  if(LOutput > 0){
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
  }
  else if(LOutput < 0){
    digitalWrite(IN1, HIGH);
    digitalWrite(IN2, LOW);
  }
  else{
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, LOW);
  }
  if(ROutput > 0){
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
  }
  else if(ROutput < 0){   
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
  }
  else{
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, LOW);
  }
  analogWrite(ENA, min(255, abs(LOutput) + LMotor_offset));
  analogWrite(ENB, min(255, abs(ROutput) + RMotor_offset));
}
void akumulator()
{
  voltage = 5.0 * ((float) analogRead(woltomierz)) / 1024.0;
  procent = (voltage - 3.01)*93.46;
}
void info()
{
  lcd.setCursor(6,0);
  lcd.print("    ");
  lcd.setCursor(14,1);
  lcd.print("  ");

  lcd.setCursor(6,0);
  lcd.print(f_angle, 0);
  lcd.write(byte(0));

  lcd.setCursor(4,1);
  lcd.print("battery:");
  lcd.print(procent, 0); 
  lcd.print("%");
}
void Status()
{
  Serial.print("omega=");
  Serial.print(omega); 
  Serial.print("\t");

  Serial.print("angle=");
  Serial.print(f_angle);
  Serial.print("\t");

  Serial.print("LOutput=");
  Serial.print(LOutput);
  Serial.print("\t");

  Serial.print("ROutput=");
  Serial.println(ROutput);
  Serial.print("\t");

  Serial.print("kp=");
  Serial.print(kp);
  Serial.print("\t");

  Serial.print("kd=");
  Serial.print(kd);
  Serial.print("\t");

  Serial.print("ki=");
  Serial.print(ki);
  Serial.print("\t");

  Serial.print("napiecie=");
  Serial.print(voltage);
  Serial.print("\t");

  Serial.print("bateria=");
  Serial.print(procent);
  Serial.print("\t");
}

void bluetooth()
{
  if(Serial.available() > 0)
  {
    blue=Serial.read();

    if(blue == 'a')
    {
      offset_kat = 5;      
    }

    if(blue == 'b')
    {
      L_Speed =-50;
      R_Speed =0;      
    }

    if(blue == 'c')
    {
      offset_kat = 1;  
    }

    if(blue == 'd')
    {
      L_Speed =0;
      R_Speed =-50;
    }

    if (blue == 'x')
    {
      L_Speed =-50;
      R_Speed =-50;
      offset_kat= 3.25;
    }
  }
} 


