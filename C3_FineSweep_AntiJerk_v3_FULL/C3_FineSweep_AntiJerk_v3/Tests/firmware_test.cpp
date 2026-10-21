#include <stdint.h>
#include <cassert>
#include <iostream>
#include <fstream>
uint32_t clockMs=2000,clockUs=2000000;bool testButton=false;
#include "../Arduino/C3_FineSweep_AntiJerk_v3/C3_FineSweep_AntiJerk_v3.ino"
void sensor(float a=0,float g=0,bool newFrame=true){imu.angle[1]=lroundf(a*32768/180);imu.gyro[1]=lroundf(g*32768/2000);angleHz=gyroHz=100;rateAt=clockMs;if(newFrame){lastAngleAt=lastGyroAt=clockMs;angleSequence++;anglePeriodUs=gyroPeriodUs=10000;}}
void tick(float a=0,float g=0){clockUs+=5000;clockMs=clockUs/1000;heartbeatAt=clockMs;sensor(a,g,clockMs%10==0);if(state==RUNNING)for(int w=0;w<2;w++)if(actualPWM[w]&&clockMs%10==0){encoder[w]+=actualPWM[w]>0?1:-1;absEncoder[w]++;}loop();}
void idle(){stopMotors();state=IDLE;faultCode=0;connected=true;heartbeatAt=clockMs;rowCount=0;sensor();}
int main(int argc,char**argv){
 setup();assert(pwmReady[0]&&pwmReady[1]);assert(sizeof(traceRows)==96048);assert(state==WAIT_SENSOR);
 // Positive forward speed must request a positive braking angle, making the inner PD command reverse.
 Lab::BalancePD speedBrakeCheck;speedBrakeCheck.config.kp=10.5f;speedBrakeCheck.config.kd=0;
 assert(speedBrakeCheck.step(0,0,0.005f,0.5f)<0);
 int32_t savedEncoder[2]={encoder[0],encoder[1]},savedSpeedCount[2]={speedCount[0],speedCount[1]};
 float savedFiltered[2]={speedFiltered[0],speedFiltered[1]};
 float savedAverage=speedAverage,savedIntegral=speedIntegral,savedRef=speedRef,savedApplied=speedApplied;
 uint32_t savedSpeedAt=speedAtUs,savedArmedAt=armedAt;
 speedAtUs=clockUs;armedAt=clockMs;speedCount[0]=encoder[0];speedCount[1]=encoder[1];
 speedFiltered[0]=speedFiltered[1]=speedAverage=speedIntegral=speedRef=speedApplied=0;
 encoder[0]+=20;encoder[1]+=20;
 updateSpeed(clockUs+SPEED_US,clockMs+50);
 assert(speedAverage>0&&speedRef>0);
 assert(speedBrakeCheck.step(0,0,0.005f,pd.config.target+speedRef)<0);
 encoder[0]=savedEncoder[0];encoder[1]=savedEncoder[1];
 speedCount[0]=savedSpeedCount[0];speedCount[1]=savedSpeedCount[1];
 speedFiltered[0]=savedFiltered[0];speedFiltered[1]=savedFiltered[1];
 speedAverage=savedAverage;speedIntegral=savedIntegral;speedRef=savedRef;speedApplied=savedApplied;
 speedAtUs=savedSpeedAt;armedAt=savedArmedAt;
 connected=true;heartbeatAt=clockMs;sensor();loop();assert(state==IDLE&&!actualPWM[0]);
 // Smooth drive: raw model can demand 120+, applied PWM must rise gradually.
 stopMotors();uint32_t when=clockUs;
 int m=smoothCommandPwm(0,120,false,when);assert(m==SLEW_RISE_NEAR);writeMotor(0,m);
 m=smoothCommandPwm(0,120,false,when+2500);assert(m==2*SLEW_RISE_NEAR);writeMotor(0,m);
 // A reversal must first go through neutral; cannot instantly apply negative duty.
 m=smoothCommandPwm(0,-130,true,when+5000);assert(m>=0);writeMotor(0,m);
 m=smoothCommandPwm(0,-130,true,when+7500);assert(m==0);writeMotor(0,m);
 m=smoothCommandPwm(0,-130,true,when+10000);assert(m<0);writeMotor(0,m);
 stopMotors();assert(!actualPWM[0]&&!actualPWM[1]);

 command("P5.5",clockMs);command("D0.3",clockMs);command("L255",clockMs);command("T4.9",clockMs);assert(pd.config.kp==5.5f&&pd.config.maxPwm==255&&controlPeriodUs==4900);
 command("T4.95",clockMs);assert(faultCode==22&&controlPeriodUs==4900);command("T5",clockMs);command("P10.5",clockMs);command("D0.25",clockMs);command("L220",clockMs);
 command("B",clockMs);assert(faultCode==23&&state==IDLE);
 command("Z",clockMs);assert(state==CALIBRATING);for(int i=0;i<400&&state==CALIBRATING;i++)tick();assert(state==IDLE&&zeroValid);
 // Monitor: every 5ms sample saved, motor stays zero, sensor frames100Hz.
 beginTrace(1,clockMs);for(int i=0;i<2002&&state==RUNNING;i++){tick((i%200)*.005f);assert(!actualPWM[0]&&!actualPWM[1]);}
 assert(state==IDLE&&rowCount>=895&&rowCount<=905);
 for(int i=0;i<rowCount;i++){assert((traceRows[i][32]|traceRows[i][33])==0);assert(Trace::crc(traceRows[i],48)!=0);}
 if(argc>1){capturedPackets().clear();sendStatus();for(unsigned i=0;i<rowCount;i++){char cmd[20];sprintf(cmd,"R%u,%u",traceSession,i);command(cmd,clockMs);}std::ofstream out(argv[1],std::ios::binary);for(auto&p:capturedPackets())out.write(p.data(),p.size());}
 // Editing/rezero with unsaved buffer is refused until explicit E.
 command("P7",clockMs);assert(faultCode==24&&pd.config.kp==10.5f);command("Z",clockMs);assert(faultCode==24&&state==IDLE);
 command("E",clockMs);command("L255",clockMs);beginTrace(2,clockMs);assert(state==PREPARING);for(int i=0;i<600;i++)tick();assert(state==RUNNING);
 for(int i=0;i<20;i++)tick(4);assert(actualPWM[0]>0&&actualPWM[1]>0);
 command("K",clockMs);tick(-4);assert(releaseMarked&&(traceRows[rowCount-1][42]&64));
 int n=rowCount;command("R0,0",clockMs);assert(rowCount==n);command("X",clockMs);assert(state==FAULTED&&!actualPWM[0]&&!actualPWM[1]&&traceRows[rowCount-1][43]==10);
 command("E",clockMs);command("T10.0",clockMs);beginTrace(1,clockMs);int before=rowCount;for(int i=0;i<4;i++)tick();assert(rowCount==before+2);
 connected=false;tick();assert(state==FAULTED&&faultCode==20&&!actualPWM[0]);
 idle();controlPeriodUs=5000;beginTrace(2,clockMs);armMotors(clockMs);clockUs+=30000;clockMs=clockUs/1000;heartbeatAt=clockMs;sensor();loop();assert(state==FAULTED&&faultCode==6&&!actualPWM[0]);
 std::cout<<"PASS: speed-loop braking sign, fractional Ts arm/zero, config validation, no writes over recorded data, 2000 full-rate monitor samples at5ms with100Hz sensor, prepared PD, cap255, marker, STOP/fault rows,20ms cadence, BLE and timing stops\n";
}
