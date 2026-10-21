package vn.phenikaa.responselab;
import java.util.*;
public final class TraceData {
 public static final int SIZE=48,MAX=2001;
 public int state,fault,count,session,angleHz,gyroHz,mode,configSession=-1,cap=220,zeroSamples,protocol=1; public double period=5.0;public boolean zero,marked,valid;
 public double kp=14.5,kd=.25,target,zeroAngle,bias;
 public Row live;public Row[] rows=new Row[0];public int downloadSession=-1,wanted,crcErrors,duplicates;public boolean downloading;
 private int assemblyIndex=-1,assemblySession=-1,assemblyCrc=-1,mask;private boolean assemblyLive;
 private final byte[] buffer=new byte[48];
 public static int u16(byte[]b,int o){return (b[o]&255)|((b[o+1]&255)<<8);}
 public static int s16(byte[]b,int o){return (short)u16(b,o);}
 public static long u32(byte[]b,int o){return (u16(b,o)&65535L)|((long)u16(b,o+2)<<16);}
 public static int crc(byte[]b){int c=65535;for(byte x:b){c^=(x&255)<<8;for(int i=0;i<8;i++)c=((c&32768)!=0?(c<<1)^0x1021:c<<1)&65535;}return c;}
 public void beginDownload(){if(state==3||state==4||state==2||count<1||count>MAX||configSession!=session)throw new IllegalStateException("Bản ghi chưa sẵn sàng");rows=new Row[count];wanted=0;downloadSession=session;downloading=true;mask=0;crcErrors=duplicates=0;}
 public boolean complete(){return rows.length>0&&wanted==rows.length&&!downloading;}
 public int accept(byte[]p){
  if(p==null||p.length!=20||p[1]!=1)return 0;int kind=p[0]&255;
  if(kind==0x90){int n=u16(p,4);if(n>MAX||(p[2]&255)>5||(p[18]!=1&&p[18]!=2))return 0;state=p[2]&255;fault=p[3]&255;count=n;session=u16(p,6);protocol=p[18]&255;period=(p[8]&255)/(protocol>=2?10.0:1.0);zero=p[9]==1;angleHz=u16(p,10);gyroHz=u16(p,12);mode=p[14]&255;marked=p[15]==1;zeroSamples=u16(p,16);valid=true;return 1;}
  if(kind==0x91){configSession=u16(p,2);kp=u16(p,4)/100.;kd=u16(p,6)/1000.;target=s16(p,8)/100.;cap=u16(p,10);period=u16(p,12)/(protocol>=2?10.0:1.0);mode=p[14]&255;zeroAngle=s16(p,16)/100.;bias=s16(p,18)/1000.;return 2;}
  boolean isLive=kind>=0xb0&&kind<=0xb3,isRow=kind>=0xa0&&kind<=0xa3;if(!isLive&&!isRow)return 0;
  int sid=u16(p,2),index=u16(p,4),check=u16(p,6),part=kind&3;
  if(isLive){if(sid!=session||index!=65535)return 0;}
  else if(!downloading||sid!=downloadSession||index!=wanted){if(index<wanted)duplicates++;return 0;}
  if(index!=assemblyIndex||sid!=assemblySession||check!=assemblyCrc||isLive!=assemblyLive){mask=0;assemblyIndex=index;assemblySession=sid;assemblyCrc=check;assemblyLive=isLive;}
  System.arraycopy(p,8,buffer,part*12,12);mask|=1<<part;
  if(mask!=15)return 0;mask=0;if(crc(buffer)!=check){crcErrors++;return 0;}
  Row row=new Row(buffer);
  if(isLive){live=row;return 3;}
  rows[wanted++]=row;if(wanted==rows.length)downloading=false;return 4;
 }
 public static class Row {
  public long us;public int dt,angleAge,gyroAge,anglePeriod,gyroPeriod,pwm1,pwm2,req1,req2,n1,n2,flags,fault,frame;
  public double theta,feedback,target,rate,filteredRate,p,d,raw,u;
  public Row(byte[]b){us=u32(b,0);dt=u16(b,4);angleAge=u16(b,6);gyroAge=u16(b,8);anglePeriod=u16(b,10);theta=s16(b,12)/100.;feedback=s16(b,14)/100.;target=s16(b,16)/100.;rate=s16(b,18)/10.;filteredRate=s16(b,20)/10.;p=s16(b,22)/100.;d=s16(b,24)/100.;raw=s16(b,26)/100.;u=s16(b,28)/100.;req1=s16(b,30);pwm1=s16(b,32);req2=s16(b,34);pwm2=s16(b,36);n1=s16(b,38);n2=s16(b,40);flags=b[42]&255;fault=b[43]&255;frame=u16(b,44);gyroPeriod=u16(b,46);}
  public boolean saturated(){return (flags&7)!=0;}
 }
 public static String faultText(int f){switch(f){case 0:return "Không lỗi";case 1:return "Không khởi tạo PWM / bộ nhớ BLE";case 3:return "Chưa lấy được mốc đứng yên";case 5:return "Sensor mất/chậm dữ liệu";case 6:return "Trễ vòng điều khiển";case 7:return "Dữ liệu số không hợp lệ";case 8:return "Xe nghiêng quá ngưỡng ngã 35°";case 10:return "Đã dừng BOOT / app";case 20:return "Mất heartbeat BLE";case 21:return "Bộ đệm đầy";case 22:return "Thông số không hợp lệ";case 23:return "Chưa lấy mốc";case 24:return "Tải bản ghi hoặc xóa trước khi sửa/lấy mốc";default:return "Lỗi "+f;}}
 public static class Quality {
  public int n, motorSamples, markedSamples, overrun, saturation, reused, pwmJumps80, pwmStarts, pwmReversals;
  public double rmse,mae,maxAbs,meanBias,meanDtUs,p95DtUs,elapsedS;
  public boolean validForRanking;
 }
 public Quality quality(){
  Quality q=new Quality(); if(!complete())return q;
  ArrayList<Integer>dts=new ArrayList<>();double s2=0,sAbs=0,sBias=0,sumdt=0;
  boolean anyMark=false;for(Row r:rows)if((r.flags&64)!=0&&(r.flags&8)!=0)anyMark=true;
  long first=0,last=0;
  for(int i=0;i<rows.length;i++){
   Row r=rows[i];
   if(i>0){
     Row prev=rows[i-1];
     for(int ch=0;ch<2;ch++){
       int before=ch==0?prev.pwm1:prev.pwm2;
       int after=ch==0?r.pwm1:r.pwm2;
       if(Math.abs(after-before)>=80)q.pwmJumps80++;
       if(before==0 && after!=0)q.pwmStarts++;
       if(before!=0 && after!=0 && (before>0)!=(after>0))q.pwmReversals++;
     }
   }
   if(r.dt>0){sumdt+=r.dt;dts.add(r.dt);}if((r.flags&32)!=0)q.overrun++;if(r.saturated())q.saturation++;
   if(i>0&&r.frame==rows[i-1].frame)q.reused++;
   if((r.flags&8)!=0)q.motorSamples++;
   if((r.flags&64)!=0)q.markedSamples++;
   if((r.flags&8)==0||mode==1||r.fault!=0)continue;
   // For physically relevant comparison, use samples AFTER marked release if available;
   // otherwise use the entire motor-enabled trace and warn that it includes hand support.
   if(anyMark&&(r.flags&64)==0)continue;
   if(first==0)first=r.us;last=r.us;
   double e=r.feedback-r.target;
   s2+=e*e;sAbs+=Math.abs(e);sBias+=e;q.maxAbs=Math.max(q.maxAbs,Math.abs(e));q.n++;
  }
  Collections.sort(dts);q.meanDtUs=dts.isEmpty()?0:sumdt/dts.size();q.p95DtUs=dts.isEmpty()?0:dts.get(Math.min(dts.size()-1,(int)Math.ceil(dts.size()*.95)-1));
  if(q.n>0){q.rmse=Math.sqrt(s2/q.n);q.mae=sAbs/q.n;q.meanBias=sBias/q.n;q.elapsedS=((last-first)&0xffffffffL)/1e6;}
  q.validForRanking=mode==3&&anyMark&&q.n>=100&&q.motorSamples>0;
  return q;
 }
 public String summary(){
  if(!complete())return "Chưa tải đủ bản ghi; không kết luận từ dữ liệu xem nhanh.";
  Quality q=quality();
  String base=String.format(Locale.US,"Đủ %d mẫu; Ts đặt %.1f ms, Ts thực TB %.3f ms, P95 %.3f ms\nDùng lại góc %d lần; trễ >120%% Ts %d; chạm biên %d mẫu\nMotor hoạt động: %d mẫu. %s",
     rows.length,period,q.meanDtUs/1000.,q.p95DtUs/1000.,q.reused,q.overrun,q.saturation,q.motorSamples,faultText(fault));
  base+=String.format(Locale.US,"\nPWM: %d lần bật từ 0, %d thay đổi ≥80, %d đảo chiều trực tiếp (tổng 2 bánh)",q.pwmStarts,q.pwmJumps80,q.pwmReversals);
  if(q.n==0)return base+"\nKhông có mẫu PD đang chạy hợp lệ để tính sai số bám góc.";
  String metrics=String.format(Locale.US,"\nMAE |e|: %.3f°; RMSE: %.3f°; lệch trung bình %+.3f°; |e| lớn nhất %.3f°",
     q.mae,q.rmse,q.meanBias,q.maxAbs);
  return base+metrics+(q.validForRanking?"\nCó đánh dấu thả tay; đủ điều kiện xếp hạng sơ bộ (cần điều kiện thử giống nhau).":"\nChưa đủ điều kiện xếp hạng: cần chế độ TRÊN SÀN, bấm đánh dấu thả tay, ghi >=100 mẫu PD sau dấu.");
 }
 public static final String HEADER="session,index,esp_us,dt_us,Ts_set_ms,angle_age_ms,gyro_age_ms,angle_period_us,gyro_period_us,angle_frame_id,theta_sensor_deg,theta_feedback_deg,theta_target_deg,gyro_debiased_deg_s,gyro_filtered_deg_s,P_cm_eq_s,D_cm_eq_s,u_raw_cm_eq_s,u_clipped_cm_eq_s,pwm1_before_cap,pwm1_actual,pwm2_before_cap,pwm2_actual,encoder1_delta,encoder2_delta,v1_cm_s,v2_cm_s,v0_cm_s,u_saturated,pwm1_saturated,pwm2_saturated,motor_enabled,zero_valid,deadline_over_120pct,release_mark,numeric_field_clipped,fault,kp,kd,pwm_cap,mode,zero_angle_deg,gyro_bias_deg_s\n";
 public String csv(){if(!complete())throw new IllegalStateException("Thiếu mẫu");StringBuilder s=new StringBuilder(HEADER);for(int i=0;i<rows.length;i++){Row r=rows[i];double v1=r.dt>0?r.n1*Math.PI*6.74*1e6/(495.1*r.dt):Double.NaN,v2=r.dt>0?r.n2*Math.PI*6.74*1e6/(496.4*r.dt):Double.NaN;
  s.append(String.format(Locale.US,"%d,%d,%d,%d,%.1f,%d,%d,%d,%d,%d,%.2f,%.2f,%.2f,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%s,%s,%s,%d,%d,%d,%d,%d,%d,%d,%d,%d,%.2f,%.3f,%d,%d,%.2f,%.3f\n",downloadSession,i,r.us,r.dt,period,r.angleAge,r.gyroAge,r.anglePeriod,r.gyroPeriod,r.frame,r.theta,r.feedback,r.target,r.rate,r.filteredRate,r.p,r.d,r.raw,r.u,r.req1,r.pwm1,r.req2,r.pwm2,r.n1,r.n2,num(v1),num(v2),num((v1+v2)/2),r.flags&1,(r.flags>>1)&1,(r.flags>>2)&1,(r.flags>>3)&1,(r.flags>>4)&1,(r.flags>>5)&1,(r.flags>>6)&1,(r.flags>>7)&1,r.fault,kp,kd,cap,mode,zeroAngle,bias));}return s.toString();}
 static String num(double d){return Double.isFinite(d)?String.format(Locale.US,"%.6f",d):"";}
}
