#include <LiquidCrystal.h>
#include <Wire.h>
int16_t Acc_rawX, Acc_rawY, Acc_rawZ,Gyr_rawX, Gyr_rawY, Gyr_rawZ;
 

float Acceleration_angle[2];
float Gyro_angle[2];
float Total_angle[2];


float elapsedTime, time, timePrev;
int i;
float rad_to_deg = 180/3.141592654;

float PID, pwmLeft, pwmRight, error, previous_error;
float pid_p=0;
float pid_i=0;
float pid_d=0;
float pid;

double kp=30.00;//30.00
double ki=2.30; //2.20
double kd=0.80; // 0.80
int q;
int x;
int state=0;
bool state2=0;
char b;

float desired_angle = -6.45;
double output=0;
int output2=0;

const int rs = 8, en = 9, d4 = 4, d5 = 5, d6 = 6, d7 = 7;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);

void setup() {
  lcd.begin(16,2);
  pinMode(0,OUTPUT);
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(12,OUTPUT);
  Wire.begin(); 
  Wire.beginTransmission(0x68);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  time = millis(); 
  
}

void loop() {
    timePrev = time;  
    time = millis(); 
    elapsedTime = (time - timePrev) / 1000; 
   
     Wire.beginTransmission(0x68);
     Wire.write(0x3B); 
     Wire.endTransmission(false);
     Wire.requestFrom(0x68,6,true); 
    
     Acc_rawX=Wire.read()<<8|Wire.read(); //each value needs two registres
     Acc_rawY=Wire.read()<<8|Wire.read();
     Acc_rawZ=Wire.read()<<8|Wire.read();


     /*---X---*/
     Acceleration_angle[0] = atan((Acc_rawY/16384.0)/sqrt(pow((Acc_rawX/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     /*---Y---*/
     Acceleration_angle[1] = atan(-1*(Acc_rawX/16384.0)/sqrt(pow((Acc_rawY/16384.0),2) + pow((Acc_rawZ/16384.0),2)))*rad_to_deg;
     
    
   Wire.beginTransmission(0x68);
   Wire.write(0x43); //Gyro data first adress
   Wire.endTransmission(false);
   Wire.requestFrom(0x68,4,true); //Just 4 registers
   
   Gyr_rawX=Wire.read()<<8|Wire.read(); //Once again we shif and sum
   Gyr_rawY=Wire.read()<<8|Wire.read();
 

   Gyro_angle[0] = Gyr_rawX/131.0; 
   Gyro_angle[1] = Gyr_rawY/131.0;

   Total_angle[0] = 0.98 *(Total_angle[0] + Gyro_angle[0]*elapsedTime) + 0.02*Acceleration_angle[0];
  
   Total_angle[1] = 0.98 *(Total_angle[1] + Gyro_angle[1]*elapsedTime) + 0.02*Acceleration_angle[1];
   
  lcd.setCursor(0,0);
  lcd.print(Total_angle[0]);
  
error = Total_angle[0] - desired_angle;
lcd.setCursor(9,0);
lcd.print(error);
lcd.setCursor(0,1);
lcd.print(kp);
lcd.print(" ");
Serial2.print(error);
Serial2.print(" ");
pid_p = kp*error;

 pid_i = pid_i+(ki*error); 
 if(pid_i>255)
 {
  pid_i=255; 
 }
  if(pid_i<-255)
 {
  pid_i=-255; 
 }

pid_d = kd*((error - previous_error)/elapsedTime);

PID = pid_p + pid_i + pid_d;
pid=PID/25;
Serial2.println(pid);
output=abs(PID);
if(output>255)
{
  output=255;
}
lcd.setCursor(9,1);
lcd.print(output);
if((Total_angle[0]>-6.45)&& (state==0))
{
  forward();
}
if((Total_angle[0]<-6.45)&&(state==0))
{
  backward();
}

previous_error = error;
state=0;
}


void forward()
{ 
  digitalWrite(0,LOW);
  digitalWrite(1,HIGH);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  analogWrite(11,output);
  analogWrite(12,output);
}
void backward()
{ 

  digitalWrite(0,HIGH);
  digitalWrite(1,LOW);
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  analogWrite(11,output);
  analogWrite(12,output);
}
void right()
{
    digitalWrite(0,HIGH);
  digitalWrite(1,LOW);
  digitalWrite(2,HIGH);
  digitalWrite(3,LOW);
  analogWrite(11,output2);
  analogWrite(12,output2);
}
void left()
{
  digitalWrite(0,LOW);
  digitalWrite(1,HIGH);
  digitalWrite(2,LOW);
  digitalWrite(3,HIGH);
  analogWrite(11,output2);
  analogWrite(12,output2);
}
