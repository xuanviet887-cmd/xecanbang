#pragma once
#include "Arduino.h"
#include <vector>
inline std::vector<std::string>& capturedPackets(){static std::vector<std::string> p;return p;}
class BLEServer;class BLECharacteristic;
struct BLEServerCallbacks{virtual void onConnect(BLEServer*){}virtual void onDisconnect(BLEServer*){}};
struct BLECharacteristicCallbacks{virtual void onWrite(BLECharacteristic*){}};
struct BLECharacteristic{enum {PROPERTY_NOTIFY=1,PROPERTY_WRITE=2};std::string data;void setValue(uint8_t*p,int n){data.assign((char*)p,n);}void notify(){capturedPackets().push_back(data);}std::string getValue(){return data;}void setCallbacks(BLECharacteristicCallbacks*){}template<class T>void addDescriptor(T*){}};
struct BLEService{BLECharacteristic*createCharacteristic(const char*,int){return new BLECharacteristic();}void start(){}};
struct BLEServer{void setCallbacks(BLEServerCallbacks*){}BLEService*createService(const char*){return new BLEService();}};
struct BLEAdvertising{void addServiceUUID(const char*){}void setScanResponse(bool){}void start(){}};
struct BLEDevice{static void init(const char*){}static BLEServer*createServer(){return new BLEServer();}static BLEAdvertising*getAdvertising(){static BLEAdvertising a;return &a;}static void startAdvertising(){}};
