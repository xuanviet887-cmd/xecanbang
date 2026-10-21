package vn.phenikaa.responselab;
import java.io.*;
public class TraceDataTest {
 static void check(boolean b){if(!b)throw new AssertionError();}
 static void put(byte[]p,int o,int v){p[o]=(byte)v;p[o+1]=(byte)(v>>8);}
 static byte[] chunk(byte[]row,int index,int part){byte[]p=new byte[20];p[0]=(byte)(0xa0+part);p[1]=1;put(p,2,7);put(p,4,index);put(p,6,TraceData.crc(row));System.arraycopy(row,part*12,p,8,12);return p;}
 public static void main(String[]args)throws Exception{
  TraceData d=new TraceData();byte[]s=new byte[20];s[0]=(byte)0x90;s[1]=1;s[2]=1;put(s,4,2);put(s,6,7);s[8]=5;s[18]=1;check(d.accept(s)==1);byte[]c=new byte[20];c[0]=(byte)0x91;c[1]=1;put(c,2,7);put(c,4,1450);put(c,6,250);put(c,10,220);put(c,12,5);check(d.accept(c)==2);d.beginDownload();
  byte[]row=new byte[48];put(row,4,5000);put(row,12,-123);put(row,30,270);put(row,32,220);row[42]=3;
  byte[]bad=chunk(row,0,3);bad[19]^=1;
  for(int part:new int[]{2,0,1})check(d.accept(chunk(row,0,part))==0);check(d.accept(bad)==0&&d.crcErrors==1&&d.wanted==0);
  for(int part:new int[]{3,1,0,2})d.accept(chunk(row,0,part));check(d.wanted==1&&d.rows[0].theta==-1.23&&d.rows[0].saturated());
  d.accept(chunk(row,0,0));check(d.wanted==1&&d.duplicates==1);
  byte[]wrong=chunk(row,1,0);put(wrong,2,8);check(d.accept(wrong)==0);
  for(int part=0;part<4;part++)d.accept(chunk(row,1,part));check(d.complete());String[]lines=d.csv().trim().split("\n");check(lines.length==3);for(int i=1;i<lines.length;i++)check(lines[i].split(",",-1).length==TraceData.HEADER.trim().split(",",-1).length);
  TraceData real=new TraceData();boolean begun=false;int loaded=0;
  try(InputStream in=new FileInputStream(args[0])){byte[]p=new byte[20];for(;;){int n=in.read(p);if(n<0)break;check(n==20);int result=real.accept(p);if(result==2&&!begun&&real.count>0){real.beginDownload();begun=true;}if(result==4)loaded++;}}
  check(real.complete()&&loaded>850&&loaded<950);int repeats=0;for(int i=0;i<real.rows.length;i++){TraceData.Row r=real.rows[i];check(r.dt==5000&&r.pwm1==0&&r.pwm2==0&&r.anglePeriod==10000);if(i>0&&r.frame==real.rows[i-1].frame)repeats++;}check(repeats>420);check(real.summary().contains("5.000 ms"));
  try(Writer out=new FileWriter(args[1])){out.write(real.csv());}
  System.out.println("PASS: CRC error/retry, reordered chunks, duplicate/session rejection, signed fields/CSV width; actual native firmware -> app decoder -> ~900 CSV rows, 5ms control/10ms sensor/reused frames");
 }
}
