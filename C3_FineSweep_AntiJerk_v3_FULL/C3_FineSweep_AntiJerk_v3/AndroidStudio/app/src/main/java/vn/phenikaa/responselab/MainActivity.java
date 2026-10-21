package vn.phenikaa.responselab;
import android.Manifest;
import android.app.*;
import android.bluetooth.*;
import android.bluetooth.le.*;
import android.content.*;
import android.content.pm.PackageManager;
import android.graphics.*;
import android.graphics.drawable.GradientDrawable;
import android.location.LocationManager;
import android.net.Uri;
import android.os.*;
import android.provider.Settings;
import android.view.*;
import android.widget.*;
import java.io.*;
import java.nio.charset.StandardCharsets;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.zip.*;
public class MainActivity extends Activity {
 static final UUID SERVICE=UUID.fromString("6e400001-b5a3-f393-e0a9-e50e24dcca9e"),TX_UUID=UUID.fromString("6e400003-b5a3-f393-e0a9-e50e24dcca9e"),RX_UUID=UUID.fromString("6e400002-b5a3-f393-e0a9-e50e24dcca9e"),CCCD=UUID.fromString("00002902-0000-1000-8000-00805f9b34fb");
 final Handler ui=new Handler(Looper.getMainLooper());BluetoothAdapter adapter;BluetoothGatt gatt;BluetoothGattCharacteristic rx;
 TraceData data=new TraceData();boolean ready,scanning,writing,destroyed,foreground=true;
 long lastRx,lastHeartbeat,writeStarted,requestAt;int retries,requested=-1,savedSession=-1,seenSession=-1;
 final ArrayDeque<String>writes=new ArrayDeque<>();
 boolean waitingConfigAck=false, configVerified=false;
 long configRequestAt=0;
 double desiredKp, desiredKd, desiredTarget, desiredPeriod;
 int desiredCap;
 boolean matchesActualSettings(){
   return data.valid && data.configSession==data.session &&
     Math.abs(data.kp-number(kp))<0.011 &&
     Math.abs(data.kd-number(kd))<0.0011 &&
     Math.abs(data.target-number(trim))<0.011 &&
     Math.abs(data.cap-number(cap))<0.01 &&
     Math.abs(data.period-(25+period.getSelectedItemPosition())/10.0)<0.051;
 }

 TextView status,actual,live,record,analysis;Button connect,zero,apply,monitor,bench,floor,stop,download,erase,mark,export;
 EditText kp,kd,trim,cap;Spinner period;Button periodMinus,periodPlus,compare;Plot angles,pwms,timing;
 final ArrayList<TraceData.Row>liveRows=new ArrayList<>();File exportFile;
 AlertDialog scanDialog;ArrayAdapter<String>scanAdapter;final ArrayList<BluetoothDevice>devices=new ArrayList<>();final ArrayList<String>names=new ArrayList<>();
 final Runnable scanTimeout=()->stopScan(),connectTimeout=()->{if(gatt!=null&&!ready)fail("Kết nối quá hạn");};
 @Override public void onCreate(Bundle state){super.onCreate(state);getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);BluetoothManager m=(BluetoothManager)getSystemService(BLUETOOTH_SERVICE);adapter=m==null?null:m.getAdapter();buildUi();ui.post(tick);}
 int dp(int v){return (int)(v*getResources().getDisplayMetrics().density+.5f);}
 TextView text(String s,int size,int color){TextView v=new TextView(this);v.setText(s);v.setTextSize(size);v.setTextColor(color);v.setPadding(0,dp(5),0,dp(5));return v;}
 LinearLayout card(LinearLayout root){LinearLayout p=new LinearLayout(this);p.setOrientation(1);p.setPadding(dp(14),dp(10),dp(14),dp(12));GradientDrawable bg=new GradientDrawable();bg.setColor(0xff132238);bg.setCornerRadius(dp(14));p.setBackground(bg);LinearLayout.LayoutParams lp=new LinearLayout.LayoutParams(-1,-2);lp.bottomMargin=dp(12);root.addView(p,lp);return p;}
 Button button(LinearLayout p,String title,Runnable fn){Button b=new Button(this);b.setText(title);b.setAllCaps(false);p.addView(b,new LinearLayout.LayoutParams(-1,dp(54)));b.setOnClickListener(v->fn.run());return b;}
 EditText input(LinearLayout p,String title,String value){p.addView(text(title,13,0xffb2c4db));EditText e=new EditText(this);e.setTextColor(Color.WHITE);e.setInputType(0x3002);e.setSingleLine(true);e.setText(value);p.addView(e);return e;}
 void buildUi(){
  ScrollView scroll=new ScrollView(this);scroll.setBackgroundColor(0xff091321);LinearLayout root=new LinearLayout(this);root.setOrientation(1);root.setPadding(dp(16),dp(18),dp(16),dp(24));scroll.addView(root);setContentView(scroll);
  root.addView(text("C3 FINE SWEEP · CHỐNG GIẬT v3",25,Color.WHITE));root.addView(text("Chỉnh bước 0,1ms · đo bám góc · so sánh RMSE",15,0xff55e3c2));
  LinearLayout c=card(root);status=text("Chưa kết nối",15,0xff55e3c2);c.addView(status);connect=button(c,"Kết nối C3_RESPONSE_LAB",()->{if(gatt!=null)disconnect();else beginScan();});stop=button(c,"DỪNG MOTOR NGAY",()->send("X"));stop.setTextColor(0xffb00020);
  LinearLayout now=card(root);actual=text("Chờ firmware Response Lab1.0",14,0xffa9c0d9);now.addView(actual);live=text("Chưa có dữ liệu",17,Color.WHITE);now.addView(live);record=text("Bản ghi nằm trên ESP32; tải sau khi dừng.",14,0xffffce69);now.addView(record);
  LinearLayout settings=card(root);settings.addView(text("1 · THÔNG SỐ THỰC NGHIỆM",18,Color.WHITE));kp=input(settings,"Kp • chỉ số quy đổi / độ","10.5");kd=input(settings,"Kd • chỉ số quy đổi / (độ/s)","0.25");trim=input(settings,"Góc đích so với mốc (−3…+3°)","0");cap=input(settings,"Giới hạn PWM (110…255)","220");settings.addView(text("Chu kỳ PD đặt; đây không phải chu kỳ JY61",13,0xffa9c0d9));period=new Spinner(this);
  ArrayList<String> levels=new ArrayList<>();
  for(int deci=25;deci<=100;deci++)levels.add(String.format(Locale.US,"%.1f ms  ·  %.1f Hz",deci/10.,10000.0/deci));
  ArrayAdapter<String>a=new ArrayAdapter<>(this,android.R.layout.simple_spinner_item,levels);
  a.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);period.setAdapter(a);period.setSelection(25);
  LinearLayout fineRow=new LinearLayout(this);fineRow.setOrientation(LinearLayout.HORIZONTAL);
  periodMinus=new Button(this);periodMinus.setText("−0,1 ms");periodMinus.setAllCaps(false);
  periodPlus=new Button(this);periodPlus.setText("+0,1 ms");periodPlus.setAllCaps(false);
  periodMinus.setOnClickListener(v->period.setSelection(Math.max(0,period.getSelectedItemPosition()-1)));
  periodPlus.setOnClickListener(v->period.setSelection(Math.min(75,period.getSelectedItemPosition()+1)));
  fineRow.addView(periodMinus,new LinearLayout.LayoutParams(0,dp(48),1));
  fineRow.addView(period,new LinearLayout.LayoutParams(0,dp(48),2));
  fineRow.addView(periodPlus,new LinearLayout.LayoutParams(0,dp(48),1));settings.addView(fineRow);
  settings.addView(text("CHỈ được thử motor sau khi bấm Áp dụng và ESP32 xác nhận đủ Kp, Kd, góc đích, PWM và Ts. Chu kỳ từ 2,5 đến 10 ms, bước 0,1 ms.",13,0xffa9c0d9));
  apply=button(settings,"Áp dụng khi motor dừng",this::applySettings);
  zero=button(settings,"Lấy mốc góc + bias • giữ xe đứng yên",()->{if(prepareNew())send("Z");});settings.addView(text("Gain giữ từ code gần nhất; chưa xác nhận tối ưu. PI tốc độ và bù hướng đang tắt. Tự cắt PWM khi góc >35°, mất BLE hoặc nhấn BOOT.",13,0xffa9c0d9));
  LinearLayout run=card(root);run.addView(text("2 · GHI MỖI NHỊP TRÊN ESP32",18,Color.WHITE));monitor=button(run,"Ghi góc 4,5 giây • MOTOR TẮT",()->start("M"));bench=button(run,"Thử PD 4,5 giây • KÊ HAI BÁNH",()->start("B"));floor=button(run,"Thử PD 4,5 giây • TRÊN SÀN",()->start("C"));mark=button(run,"Đánh dấu lúc thả tay",()->send("K"));download=button(run,"Tải đủ bản ghi từ ESP32",this::beginDownload);compare=button(run,"Bảng so sánh các chu kỳ đã đo",this::showLeaderboard);erase=button(run,"Bỏ bản ghi trên ESP32",()->new AlertDialog.Builder(this).setMessage("Xóa bản ghi trên ESP32? Bản ZIP đã lưu trên điện thoại vẫn còn.").setPositiveButton("Xóa",(d,w)->{send("E");data.downloading=false;}).setNegativeButton("Giữ lại",null).show());
  run.addView(text("Giữ thẳng trong 3 giây chuẩn bị. Đo 4,5 giây cho mọi chu kỳ. Muốn so sánh cân bằng, bấm Đánh dấu khi thả tay; chỉ xếp hạng số liệu motor chạy trên sàn. Dừng bằng app/BOOT nếu bất thường.",13,0xffa9c0d9));
  LinearLayout plots=card(root);plots.addView(text("3 · ĐỒ THỊ THEO THỜI GIAN ESP32",18,Color.WHITE));angles=new Plot(0);pwms=new Plot(1);timing=new Plot(2);for(Plot p:new Plot[]{angles,pwms,timing})plots.addView(p,new LinearLayout.LayoutParams(-1,dp(190)));analysis=text("Chưa có thống kê đủ mẫu.",14,Color.WHITE);plots.addView(analysis);export=button(plots,"Xuất SESSION.zip / CSV đã lưu",this::chooseExport);
 }
 double number(EditText e){try{return Double.parseDouble(e.getText().toString().trim().replace(',','.'));}catch(Exception x){return Double.NaN;}}
 boolean pending(){return data.count>0&&savedSession!=data.session;}
 boolean prepareNew(){if(data.downloading||data.state==2||data.state==3||data.state==4)return false;if(pending()){toast("Tải bản ghi trước, hoặc bấm Bỏ bản ghi nếu không cần giữ.");return false;}if(data.count>0)send("E");return true;}
 void applySettings(){double p=number(kp),d=number(kd),o=number(trim),m=number(cap);if(!Double.isFinite(p)||p<0||p>20||!Double.isFinite(d)||d<0||d>2||!Double.isFinite(o)||Math.abs(o)>3||!Double.isFinite(m)||m<110||m>255||m!=Math.rint(m)){toast("Kiểm tra Kp0–20, Kd0–2, góc±3°, PWM nguyên110–255.");return;}if(!prepareNew())return;
  desiredKp=p; desiredKd=d; desiredTarget=o; desiredCap=(int)m;
  desiredPeriod=(25+period.getSelectedItemPosition())/10.0;
  waitingConfigAck=true;configVerified=false;configRequestAt=SystemClock.elapsedRealtime();
  status.setText("Đang gửi cấu hình; chờ ESP32 xác nhận…");
  send(String.format(Locale.US,"P%.2f",p));send(String.format(Locale.US,"D%.3f",d));
  send(String.format(Locale.US,"O%.2f",o));send("L"+(int)m);
  send(String.format(Locale.US,"T%.1f",desiredPeriod));send("Q");
}
 void start(String command){
  if(!configVerified || waitingConfigAck || !matchesActualSettings()){
    toast("Chưa xác nhận cấu hình: dừng motor, bấm Áp dụng và chờ Ts ESP32 trùng Ts chọn.");
    return;
  }
  if(!prepareNew())return;String msg=command.equals("M")?"Motor tắt. Giữ yên rồi nghiêng trước/sau nhẹ trong 4,5 giây để đo chu kỳ sensor.":command.equals("B")?"Kê chắc hai bánh. Giữ thẳng 3 giây chuẩn bị rồi nghiêng trước/sau nhẹ. Tự dừng sau 4,5 giây.":"Hai bánh trên sàn. Giữ thẳng 3 giây chuẩn bị, luôn sẵn sàng đỡ xe. Bấm Đánh dấu ngay lúc thả tay. Tự dừng sau 4,5 giây.";new AlertDialog.Builder(this).setTitle("Bắt đầu lượt đo — chỉ sau xác nhận Ts").setMessage(msg).setPositiveButton("Bắt đầu",(d,w)->{liveRows.clear();send("H");send(command);}).setNegativeButton("Đóng",null).show();}
 void beginDownload(){try{data.beginDownload();requested=-1;retries=0;requestRow();updateButtons();}catch(Exception e){toast(e.getMessage());}}
 void requestRow(){if(!data.downloading)return;requested=data.wanted;requestAt=SystemClock.elapsedRealtime();send("R"+data.downloadSession+","+data.wanted);}
 void receive(byte[]p){int result=data.accept(p);if(result==0)return;lastRx=SystemClock.elapsedRealtime();
  if(result==2 && waitingConfigAck){
    boolean ok=data.configSession==data.session &&
      Math.abs(data.kp-desiredKp)<0.011 && Math.abs(data.kd-desiredKd)<0.0011 &&
      Math.abs(data.target-desiredTarget)<0.011 && data.cap==desiredCap &&
      Math.abs(data.period-desiredPeriod)<0.051 && data.fault==0;
    if(ok){waitingConfigAck=false;configVerified=true;
      status.setText(String.format(Locale.US,"ESP32 đã xác nhận Ts %.1f ms, Kp %.2f, Kd %.3f",data.period,data.kp,data.kd));
      toast("Cấu hình đã áp dụng trên ESP32.");}
  }

  if(result==1&&seenSession!=data.session){if(data.downloading&&data.downloadSession!=data.session){data.downloading=false;toast("Phiên ESP32 đã đổi, dừng tải.");}seenSession=data.session;liveRows.clear();data.live=null;if(!data.downloading){data.rows=new TraceData.Row[0];data.wanted=0;}analysis.setText("Chưa tải đủ dữ liệu của phiên này.");}
  if(result==3){liveRows.add(data.live);while(liveRows.size()>150)liveRows.remove(0);}
  if(result==4){retries=0;if(data.downloading)requestRow();else saveSession();}
  actual.setText(String.format(Locale.US,"FW CHỐNG GIẬT v3 • %s\nESP32 xác nhận Kp%.2f / Kd%.3f / đích%+.2f° / PWM≤%d\nPD đặt %.1f ms • JY góc%dHz / gyro%dHz\nMốc: %s • bias%+.3f°/s • %d mẫu lấy mốc",new String[]{"CHỜ SENSOR","ĐANG DỪNG","LẤY MỐC","CHUẨN BỊ3s","ĐANG GHI","ĐÃ DỪNG"}[Math.min(5,data.state)],data.kp,data.kd,data.target,data.cap,data.period,data.angleHz,data.gyroHz,data.zero?"ĐÃ CÓ":"CHƯA CÓ",data.bias,data.zeroSamples));
  actual.append(String.format(Locale.US,"\nApp chọn %.1f ms • %s",
      (25+period.getSelectedItemPosition())/10.0,
      waitingConfigAck?"CHỜ ESP XÁC NHẬN":(configVerified&&matchesActualSettings()?"KHỚP CẤU HÌNH":"CHƯA KHỚP / BẤM ÁP DỤNG")));
  if(data.live!=null){TraceData.Row r=data.live;live.setText(String.format(Locale.US,"Mẫu xem nhanh gần nhất\nθ sensor %+.2f° • phản hồi %+.2f°\nθ đích %+.2f° • dt %s\nPWM %+d / %+d • %s",r.theta,r.feedback,r.target,r.dt==0?"— (đang dừng)":String.format(Locale.US,"%.3fms",r.dt/1000.),r.pwm1,r.pwm2,r.saturated()?"CHẠM BIÊN":"chưa chạm biên"));}
  record.setText((data.downloading?"Đang tải "+data.wanted+" / "+data.rows.length+" mẫu":"ESP32: "+data.count+" mẫu • "+TraceData.faultText(data.fault))+(data.complete()?"\nĐồ thị: đầy đủ từng nhịp":"\nĐồ thị: xem nhanh BLE, không đủ để kết luận"));
  if(result>=3||result==1){angles.invalidate();pwms.invalidate();timing.invalidate();}updateButtons();
 }
 void showLeaderboard(){
  File file=new File(getFilesDir(),"FINE_SWEEP_COMPARISON.csv");
  if(!file.exists()){new AlertDialog.Builder(this).setMessage("Chưa có lượt thử TRÊN SÀN hợp lệ. Hãy bấm Đánh dấu lúc thả tay và tải đầy đủ dữ liệu. Mỗi mức nên đo ít nhất 3 lượt.").setPositiveButton("Đóng",null).show();return;}
  HashMap<String,double[]> grouped=new HashMap<>();
  try(BufferedReader br=new BufferedReader(new FileReader(file))){String line;while((line=br.readLine())!=null){String[]v=line.split(",",-1);if(v.length!=8||!v[0].matches("[0-9.]+"))continue;
      String key=v[0]+" ms | Kp "+v[1]+" Kd "+v[2]+" target "+v[3];
      double[]g=grouped.get(key);if(g==null){g=new double[3];grouped.put(key,g);}
      g[0]+=Double.parseDouble(v[4]);g[1]+=Double.parseDouble(v[5]);g[2]+=1;
  }}catch(Exception e){toast("Đọc so sánh lỗi: "+e.getMessage());return;}
  ArrayList<Map.Entry<String,double[]>> entries=new ArrayList<>(grouped.entrySet());
  Collections.sort(entries,(x,y)->Double.compare(x.getValue()[0]/x.getValue()[2],y.getValue()[0]/y.getValue()[2]));
  StringBuilder s=new StringBuilder("RMSE TRUNG BÌNH THẤP NHẤT (°)\nChỉ so sánh cùng Kp/Kd, cùng cách thả tay và cùng mặt sàn.\n\n");
  for(int i=0;i<Math.min(40,entries.size());i++){
    Map.Entry<String,double[]>ent=entries.get(i);double[]v=ent.getValue();
    s.append(String.format(Locale.US,"%d. %s\n   RMSE %.3f° | MAE %.3f° | %d lượt %s\n",i+1,ent.getKey(),v[0]/v[2],v[1]/v[2],(int)v[2],v[2]<3?"(chưa đủ 3 lượt)":""));
  }
  if(entries.isEmpty())s.append("Chưa có lượt đo hợp lệ.");
  new AlertDialog.Builder(this).setTitle("Bảng so sánh chu kỳ").setMessage(s.toString()).setPositiveButton("Đóng",null).show();
 }
 void appendComparison(String stamp){
   TraceData.Quality q=data.quality();if(!q.validForRanking)return;
   File f=new File(getFilesDir(),"FINE_SWEEP_COMPARISON.csv");
   try(FileWriter w=new FileWriter(f,true)){
     if(f.length()==0)w.write("Ts_ms,Kp,Kd,Target,RMSE_deg,MAE_deg,NumSamples,SessionName\n");
     w.write(String.format(Locale.US,"%.1f,%.2f,%.3f,%.2f,%.5f,%.5f,%d,%s\n",data.period,data.kp,data.kd,data.target,q.rmse,q.mae,q.n,stamp));
   }catch(Exception e){toast("Không lưu bảng so sánh: "+e.getMessage());}
 }
 void saveSession(){try{String stamp=new SimpleDateFormat("yyyyMMdd_HHmmss_SSS",Locale.US).format(new Date());String stem="RESPONSE_"+stamp;File csv=new File(getFilesDir(),stem+".csv"),txt=new File(getFilesDir(),stem+"_SUMMARY.txt");String summary=data.summary()+String.format(Locale.US,"\nKp%.2f Kd%.3f target%.2f cap%d; mốc%.2f bias%.3f; mode%d; trụcY; mô hình từ005829.\nDữ liệu ghi trên ESP32, tải đủ từng mẫu. CRC lỗi khi tải%d. Không đo độ trễ cơ học hoặc tự nhận dạng gain.\n",data.kp,data.kd,data.target,data.cap,data.zeroAngle,data.bias,data.mode,data.crcErrors);write(csv,data.csv());write(txt,summary);File zip=new File(getFilesDir(),stem+"_SESSION.zip");try(ZipOutputStream z=new ZipOutputStream(new FileOutputStream(zip))){for(File f:new File[]{csv,txt}){z.putNextEntry(new ZipEntry(f.getName()));try(InputStream in=new FileInputStream(f)){byte[]b=new byte[8192];int n;while((n=in.read(b))>0)z.write(b,0,n);}z.closeEntry();}}savedSession=data.downloadSession;appendComparison(stem);analysis.setText(data.summary());toast("Đã lưu đủ bản ghi. Có thể xuất SESSION.zip.");}catch(Exception e){toast("Lưu lỗi: "+e.getMessage());}}
 void write(File f,String content)throws IOException{try(Writer w=new OutputStreamWriter(new FileOutputStream(f),StandardCharsets.UTF_8)){w.write(content);}}
 void updateButtons(){if(monitor==null)return;boolean link=ready&&data.valid&&lastRx!=0&&SystemClock.elapsedRealtime()-lastRx<1500;boolean idle=link&&(data.state==1||data.state==5)&&!data.downloading;boolean clean=idle&&!pending();zero.setEnabled(clean&&data.angleHz>=50&&data.gyroHz>=50);apply.setEnabled(clean);monitor.setEnabled(clean&&data.angleHz>=50&&data.gyroHz>=50);bench.setEnabled(clean&&configVerified&&!waitingConfigAck&&matchesActualSettings()&&data.zero&&data.angleHz>=50&&data.gyroHz>=50);floor.setEnabled(clean&&configVerified&&!waitingConfigAck&&matchesActualSettings()&&data.zero&&data.angleHz>=50&&data.gyroHz>=50);stop.setEnabled(ready);mark.setEnabled(link&&data.state==4);download.setEnabled(idle&&data.count>0);erase.setEnabled(idle&&data.count>0);for(EditText e:new EditText[]{kp,kd,trim,cap})e.setEnabled(clean);period.setEnabled(clean);periodMinus.setEnabled(clean);periodPlus.setEnabled(clean);}
 final Runnable tick=new Runnable(){public void run(){if(destroyed)return;long now=SystemClock.elapsedRealtime();if(ready&&foreground&&now-lastHeartbeat>=250){lastHeartbeat=now;send("H");}if(writing&&now-writeStarted>2500)fail("BLE không xác nhận lệnh; motor sẽ dừng.");if(waitingConfigAck && now-configRequestAt>6000){
   waitingConfigAck=false;configVerified=false;
   status.setText("Chưa xác nhận Ts/Kp/Kd — bấm Áp dụng lại khi motor dừng.");
   toast("ESP32 chưa xác nhận cấu hình; không cho bật motor.");
  }
  if(data.downloading&&now-requestAt>1500){if(++retries>4){data.downloading=false;toast("Tải bị gián đoạn. Bản ghi vẫn trên ESP32, bấm Tải để thử lại.");}else requestRow();}updateButtons();ui.postDelayed(this,100);}};

    boolean permissions(){
        String[] p=Build.VERSION.SDK_INT>=31?new String[]{Manifest.permission.BLUETOOTH_SCAN,Manifest.permission.BLUETOOTH_CONNECT}:new String[]{Manifest.permission.ACCESS_FINE_LOCATION};
        for(String v:p)if(checkSelfPermission(v)!=PackageManager.PERMISSION_GRANTED){requestPermissions(p,10);return false;}return true;
    }
    @Override public void onRequestPermissionsResult(int req,String[] p,int[] grants){super.onRequestPermissionsResult(req,p,grants);if(req==10&&grants.length>0){for(int g:grants)if(g!=0){toast("Cần quyền Bluetooth / vị trí để quét BLE.");return;}beginScan();}}
    void beginScan(){
        if(adapter==null){toast("Thiết bị không hỗ trợ Bluetooth.");return;}if(!permissions())return;
        if(!adapter.isEnabled()){startActivityForResult(new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE),11);return;}
        if(Build.VERSION.SDK_INT<31){LocationManager lm=(LocationManager)getSystemService(LOCATION_SERVICE);boolean on=Build.VERSION.SDK_INT>=28?lm!=null&&lm.isLocationEnabled():Settings.Secure.getInt(getContentResolver(),Settings.Secure.LOCATION_MODE,0)!=0;if(!on){new AlertDialog.Builder(this).setMessage("Bật Vị trí để Android quét BLE.").setPositiveButton("Mở cài đặt",(d,w)->startActivity(new Intent(Settings.ACTION_LOCATION_SOURCE_SETTINGS))).setNegativeButton("Đóng",null).show();return;}}
        stopScan();devices.clear();names.clear();scanAdapter=new ArrayAdapter<>(this,android.R.layout.simple_list_item_1,names);
        scanDialog=new AlertDialog.Builder(this).setTitle("Tìm C3_RESPONSE_LAB…").setAdapter(scanAdapter,(d,i)->{BluetoothDevice device=devices.get(i);stopScan();connectDevice(device);}).setNegativeButton("Dừng",(d,w)->stopScan()).create();scanDialog.setOnCancelListener(d->stopScan());scanDialog.show();
        try{scanning=true;adapter.getBluetoothLeScanner().startScan(null,new ScanSettings.Builder().setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY).build(),scanCallback);ui.postDelayed(scanTimeout,12000);}catch(Exception e){stopScan();toast("Không quét được: "+e.getMessage());}
    }
    final ScanCallback scanCallback=new ScanCallback(){
        @Override public void onScanResult(int type,ScanResult result){ui.post(()->{if(!scanning)return;String name=result.getScanRecord()==null?null:result.getScanRecord().getDeviceName();if(!"C3_RESPONSE_LAB".equals(name))return;for(BluetoothDevice d:devices)if(d.getAddress().equals(result.getDevice().getAddress()))return;devices.add(result.getDevice());names.add(name+"\n"+result.getDevice().getAddress()+" • "+result.getRssi()+" dBm");scanAdapter.notifyDataSetChanged();});}
        @Override public void onScanFailed(int error){ui.post(()->{stopScan();toast("Quét BLE lỗi "+error);});}
    };
    void stopScan(){ui.removeCallbacks(scanTimeout);if(scanning){try{adapter.getBluetoothLeScanner().stopScan(scanCallback);}catch(Exception ignored){}scanning=false;}}
    void connectDevice(BluetoothDevice d){disconnect();data=new TraceData();savedSession=-1;seenSession=-1;liveRows.clear();lastRx=0;status.setText("Đang kết nối…");connect.setText("Ngắt kết nối");try{gatt=d.connectGatt(this,false,callbacks,BluetoothDevice.TRANSPORT_LE);ui.postDelayed(connectTimeout,15000);}catch(Exception e){fail("Kết nối lỗi: "+e.getMessage());}}
    void fail(String s){disconnect();status.setText(s);}
    void disconnect(){stopScan();waitingConfigAck=false;configVerified=false;ready=false;rx=null;writing=false;writes.clear();ui.removeCallbacks(connectTimeout);data.downloading=false;BluetoothGatt old=gatt;gatt=null;if(old!=null){try{old.disconnect();old.close();}catch(Exception ignored){}}lastRx=0;if(connect!=null){connect.setText("Kết nối C3_RESPONSE_LAB");status.setText("Chưa kết nối");updateButtons();}}
    final BluetoothGattCallback callbacks=new BluetoothGattCallback(){
        @Override public void onConnectionStateChange(BluetoothGatt g,int code,int state){ui.post(()->{if(g!=gatt)return;if(code!=0){fail("BLE lỗi "+code);return;}if(state==BluetoothProfile.STATE_CONNECTED){status.setText("Đang tìm dịch vụ…");if(!g.discoverServices())fail("Không tìm được dịch vụ.");}else if(state==BluetoothProfile.STATE_DISCONNECTED)fail("BLE đã ngắt; motor sẽ dừng.");});}
        @Override public void onServicesDiscovered(BluetoothGatt g,int code){ui.post(()->{if(g!=gatt)return;BluetoothGattService service=g.getService(SERVICE);if(code!=0||service==null){fail("Sai firmware: cần C3_Response_Lab.");return;}BluetoothGattCharacteristic tx=service.getCharacteristic(TX_UUID);rx=service.getCharacteristic(RX_UUID);if(tx==null||rx==null||!g.setCharacteristicNotification(tx,true)){fail("Không bật được dữ liệu BLE.");return;}BluetoothGattDescriptor d=tx.getDescriptor(CCCD);if(d==null){fail("Thiếu CCCD.");return;}d.setValue(BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE);if(!g.writeDescriptor(d))fail("Không đăng ký được thông báo.");});}
        @Override public void onDescriptorWrite(BluetoothGatt g,BluetoothGattDescriptor d,int code){ui.post(()->{if(g!=gatt)return;if(code!=0){fail("Đăng ký BLE lỗi "+code);return;}ready=true;ui.removeCallbacks(connectTimeout);status.setText("BLE sẵn sàng • C3_RESPONSE_LAB");send("H");send("Q");});}
        @Override public void onCharacteristicChanged(BluetoothGatt g,BluetoothGattCharacteristic c){byte[] b=c.getValue();if(b!=null){byte[] copy=b.clone();ui.post(()->{if(g==gatt&&!destroyed)receive(copy);});}}
        @Override public void onCharacteristicWrite(BluetoothGatt g,BluetoothGattCharacteristic c,int code){ui.post(()->{if(g!=gatt)return;writing=false;if(code!=0){fail("Gửi lệnh BLE lỗi "+code);return;}pump();});}
    };
    void send(String command){if(!ready||gatt==null||rx==null)return;if(command.equals("X")){writes.clear();writes.addFirst(command);}else if(!writes.contains(command))writes.addLast(command);pump();}
    void pump(){if(writing||writes.isEmpty()||!ready)return;String c=writes.removeFirst();try{rx.setWriteType(BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT);rx.setValue(c.getBytes(StandardCharsets.US_ASCII));writing=true;writeStarted=SystemClock.elapsedRealtime();if(!gatt.writeCharacteristic(rx)){writing=false;fail("BLE đang bận. Kết nối lại.");}}catch(Exception e){writing=false;fail("Gửi BLE lỗi: "+e.getMessage());}}


 class Plot extends View {
  final int kind;final Paint paint=new Paint(3);
  Plot(int k){super(MainActivity.this);kind=k;}
  ArrayList<TraceData.Row> values(){ArrayList<TraceData.Row>r=new ArrayList<>();if(data.complete())Collections.addAll(r,data.rows);else r.addAll(liveRows);return r;}
  double value(TraceData.Row r,int series){if(kind==0)return series==0?r.theta:series==1?r.feedback:r.target;if(kind==1)return series==0?r.pwm1:series==1?r.pwm2:series==2?r.req1:r.req2;return series==0?r.dt/1000.:series==1?data.period:r.anglePeriod/1000.;}
  @Override protected void onDraw(Canvas c){super.onDraw(c);float w=getWidth(),h=getHeight(),top=dp(38),bottom=h-dp(22),left=dp(40);ArrayList<TraceData.Row>rows=values();int series=kind==1?4:3;double bound=kind==0?2:kind==1?Math.max(110,data.cap):Math.max(10,data.period);for(TraceData.Row r:rows)for(int j=0;j<series;j++)bound=Math.max(bound,Math.abs(value(r,j))*1.1);
   paint.setTextSize(dp(11));paint.setStrokeWidth(dp(1));paint.setColor(0xffb2c4db);c.drawText(kind==0?"θ sensor xanh / phản hồi vàng / đích tím":kind==1?"PWM12 xanh /34 vàng; trước cap: tím/hồng":"dt thực xanh / Ts đặt vàng / chu kỳ góc tím",0,dp(14),paint);c.drawText(data.complete()?"Đủ mẫu ESP32":"Xem nhanh 5Hz",0,dp(29),paint);
   double low=kind==2?0:-bound,high=bound;
   for(int j=0;j<3;j++){double v=low+(high-low)*j/2;float y=(float)(bottom-(v-low)/(high-low)*(bottom-top));paint.setColor(0xff2c4059);c.drawLine(left,y,w,y,paint);paint.setColor(0xffb2c4db);c.drawText(String.format(Locale.US,"%.1f",v),0,y,paint);}
   if(kind==1){paint.setColor(0xffed7186);for(int sign:new int[]{-1,1}){float y=(float)(bottom-(sign*data.cap-low)/(high-low)*(bottom-top));c.drawLine(left,y,w,y,paint);}}
   if(rows.size()<2)return;long first=rows.get(0).us;double duration=(rows.get(rows.size()-1).us-first)&0xffffffffL;if(duration<=0)return;int[]colors={0xff55e3c2,0xffffce69,0xffc093ff,0xfff5a7bd};
   for(int j=0;j<series;j++){paint.setColor(colors[j]);paint.setStrokeWidth(dp(j<2?2:1));for(int i=1;i<rows.size();i++){TraceData.Row a=rows.get(i-1),b=rows.get(i);float x1=(float)(left+(((a.us-first)&0xffffffffL)/duration)*(w-left)),x2=(float)(left+(((b.us-first)&0xffffffffL)/duration)*(w-left));float y1=(float)(bottom-(value(a,j)-low)/(high-low)*(bottom-top)),y2=(float)(bottom-(value(b,j)-low)/(high-low)*(bottom-top));c.drawLine(x1,y1,x2,y2,paint);}}
   paint.setColor(0xffb2c4db);c.drawText(String.format(Locale.US,"0 → %.2f s",duration/1e6),left,h-dp(4),paint);
  }
 }
 void chooseExport(){File[]files=getFilesDir().listFiles((d,n)->(n.startsWith("RESPONSE_")||n.equals("FINE_SWEEP_COMPARISON.csv"))&&(n.endsWith(".zip")||n.endsWith(".csv")||n.endsWith(".txt")));if(files==null||files.length==0){toast("Tải đủ một bản ghi trước khi xuất.");return;}Arrays.sort(files,(a,b)->Long.compare(b.lastModified(),a.lastModified()));String[]labels=new String[files.length];for(int i=0;i<files.length;i++)labels[i]=files[i].getName();new AlertDialog.Builder(this).setTitle("Lưu kết quả").setItems(labels,(d,i)->{exportFile=files[i];Intent save=new Intent(Intent.ACTION_CREATE_DOCUMENT);save.setType(exportFile.getName().endsWith(".zip")?"application/zip":exportFile.getName().endsWith(".csv")?"text/csv":"text/plain");save.addCategory(Intent.CATEGORY_OPENABLE);save.putExtra(Intent.EXTRA_TITLE,exportFile.getName());startActivityForResult(save,20);}).setNegativeButton("Đóng",null).show();}
 @Override public void onActivityResult(int req,int res,Intent result){super.onActivityResult(req,res,result);if(req==11&&res==RESULT_OK){beginScan();return;}if(req==20&&res==RESULT_OK&&result!=null&&result.getData()!=null&&exportFile!=null){Uri uri=result.getData();File source=exportFile;new Thread(()->{try(InputStream in=new FileInputStream(source);OutputStream out=getContentResolver().openOutputStream(uri)){if(out==null)throw new IOException("Không mở được nơi lưu");byte[]b=new byte[8192];int n;while((n=in.read(b))>0)out.write(b,0,n);ui.post(()->toast("Đã xuất file"));}catch(Exception e){ui.post(()->toast("Xuất lỗi: "+e.getMessage()));}}).start();}}
 void toast(String s){Toast.makeText(this,s,Toast.LENGTH_LONG).show();}
 @Override protected void onResume(){super.onResume();foreground=true;}
 @Override protected void onPause(){foreground=false;if(ready)send("X");super.onPause();}
 @Override protected void onDestroy(){destroyed=true;ui.removeCallbacksAndMessages(null);disconnect();if(scanDialog!=null)scanDialog.dismiss();super.onDestroy();}
}
