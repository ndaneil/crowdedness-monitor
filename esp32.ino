/* Author: Daniel Nemeth
 * Used sources:
 * https://www.hackster.io/p99will/esp32-wifi-mac-scanner-sniffer-promiscuous-4c12f4
 * https://github.com/dollop80/ESP32-BLE-Scanner/blob/master/ESP32_BLE_Scanner.ino
*/

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <WiFi.h>
#include "esp_wifi.h"


SET_LOOP_TASK_STACK_SIZE(32 * 1024)

const wifi_promiscuous_filter_t filt={
    .filter_mask=WIFI_PROMIS_FILTER_MASK_MGMT|WIFI_PROMIS_FILTER_MASK_DATA
};

typedef struct {
  uint8_t mac[6];
} __attribute__((packed)) MacAddr;

typedef struct {
  int16_t fctl;
  int16_t duration;
  MacAddr da;
  MacAddr sa;
  MacAddr bssid;
  int16_t seqctl;
  unsigned char payload[];
} __attribute__((packed)) WifiMgmtHdr;

  
#define MAX_WIFI_CHANNEL 13 //max channel for scanning -> US = 11, EU = 13, Japan = 14

#define DEVICE_BUFFER_SIZE 1024
#define DEVICE_TTL_MS 360000 // 6 minutes

typedef struct {
  uint8_t mac[6];
  long lastms = 0;
  bool alive = false;
} Device;

Device wifiDevices[DEVICE_BUFFER_SIZE];
Device bleDevices[DEVICE_BUFFER_SIZE];


#define RXD2 22
#define TXD2 19
#define WAKEUP_PIN GPIO_NUM_4 // GPIO 4 - RTC_GPIO10

long scanRoundCount = 0;


void registerDevice(uint8_t mac[6], bool isWiFi){
  for(int i = 0; i < DEVICE_BUFFER_SIZE; i++){
    if(isWiFi && wifiDevices[i].alive && memcmp(mac, wifiDevices[i].mac,6) == 0){
      // Existing device, increase ttl
      wifiDevices[i].lastms = millis();
      wifiDevices[i].alive = true;
      return;
    }else if(!isWiFi && bleDevices[i].alive && memcmp(mac, bleDevices[i].mac,6) == 0){
      // Existing device, increase ttl
      bleDevices[i].lastms = millis();
      bleDevices[i].alive = true;
      return;
    }
  }
  // New device
  for(int i = 0; i < DEVICE_BUFFER_SIZE; i++){
    if(isWiFi && !wifiDevices[i].alive){
      memcpy(wifiDevices[i].mac, mac, 6);
      wifiDevices[i].lastms = millis();
      wifiDevices[i].alive = true;
      Serial.printf("[WIFI] NEW MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return;
    } else if(!isWiFi && !bleDevices[i].alive){
      memcpy(bleDevices[i].mac, mac, 6);
      bleDevices[i].lastms = millis();
      bleDevices[i].alive = true;
      Serial.printf("[BLE ] NEW MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
      return;
    }
  }
  Serial.printf("Buffer is full, no place for MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void cleanupDevices(){
  for(int i = 0; i < DEVICE_BUFFER_SIZE; i++){
    if(wifiDevices[i].alive && millis() - wifiDevices[i].lastms > DEVICE_TTL_MS){
      Serial.printf("[WIFI] REMOVED MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", wifiDevices[i].mac[0], wifiDevices[i].mac[1], wifiDevices[i].mac[2], wifiDevices[i].mac[3], wifiDevices[i].mac[4], wifiDevices[i].mac[5]);
      wifiDevices[i].alive = false;
    }
    if(bleDevices[i].alive && millis() - bleDevices[i].lastms > DEVICE_TTL_MS){
      Serial.printf("[BLE ] REMOVED MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", bleDevices[i].mac[0], bleDevices[i].mac[1], bleDevices[i].mac[2], bleDevices[i].mac[3], bleDevices[i].mac[4], bleDevices[i].mac[5]);
      bleDevices[i].alive = false;
    }
  }
}

int countWiFiDevices(){
  int cnt = 0;
  for(int i = 0; i < DEVICE_BUFFER_SIZE; i++){
    if(wifiDevices[i].alive) cnt++;
  }
  return cnt;
}

int countBleDevices(){
  int cnt = 0;
  for(int i = 0; i < DEVICE_BUFFER_SIZE; i++){
    if(bleDevices[i].alive) cnt++;
  }
  return cnt;
}

void wifiSniffer(void* buf, wifi_promiscuous_pkt_type_t type) {
  wifi_promiscuous_pkt_t *p = (wifi_promiscuous_pkt_t*)buf;
  int len = p->rx_ctrl.sig_len;
  WifiMgmtHdr *wh = (WifiMgmtHdr*)p->payload;
  len -= sizeof(WifiMgmtHdr);
  if (len < 0){
    Serial.println("Received 0");
    return;
  }
  uint8_t mac[6];
  for(int i=4;i<10;i++){
    mac[i-4] = p->payload[i];
  }
  registerDevice(mac, true);
}


class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks{
    void onResult(BLEAdvertisedDevice advertisedDevice){
      registerDevice(advertisedDevice.getAddress().getNative()[0], false);
    }
};

MyAdvertisedDeviceCallbacks* cb;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  WiFi.persistent(false);
  Serial.printf("Startup complete. Free stack: %d\n",uxTaskGetStackHighWaterMark(NULL));
  cb = new MyAdvertisedDeviceCallbacks();
}

void performWifiScan(){
  Serial.println("WiFi scan start");
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
  esp_wifi_init(&cfg);
  esp_wifi_set_storage(WIFI_STORAGE_RAM);
  esp_wifi_set_mode(WIFI_MODE_NULL);
  esp_wifi_set_promiscuous(true);
  esp_wifi_set_promiscuous_filter(&filt);
  esp_wifi_set_promiscuous_rx_cb(&wifiSniffer);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_wifi_start();
  for(int channel = 1; channel <= MAX_WIFI_CHANNEL; channel++){
    esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
    vTaskDelay(2000 / portTICK_PERIOD_MS); // 2s delay
  }
  esp_wifi_stop();
  esp_wifi_deinit();
  Serial.println("WiFi scan end");
}

void performBleScan() {
  Serial.println("BLE scan start");
  BLEDevice::init("");
  BLEScan *pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(cb);
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(0x100);
  pBLEScan->setWindow(0x100);
  pBLEScan->start(6, nullptr, false); // scan for 6 seconds, ignoring return value
  vTaskDelay(5000 / portTICK_PERIOD_MS); // 5s delay
  pBLEScan->stop();
  Serial.printf("BLE scan end. Free stack: %d heap: %d\n", uxTaskGetStackHighWaterMark(NULL), ESP.getFreeHeap());
  pBLEScan->clearResults();
  BLEDevice::deinit(false);
}

void loop() {
  performWifiScan();
  performBleScan();
  Serial.printf("WIFI Device count: %d BLE: %d Free heap: %d Free stack: %d\n", countWiFiDevices(), countBleDevices(), ESP.getFreeHeap(), uxTaskGetStackHighWaterMark(NULL));
  scanRoundCount++;
  if(scanRoundCount >= 3){
    // Device list has stabilized over multiple rounds of scans
    Serial.println("Sending data to AVR IOT CELLULAR MINI...");
    Serial2.printf("%d,%d,%d\n", countWiFiDevices(), countBleDevices(), ESP.getFreeHeap());
  }
  cleanupDevices();
  delay(100);
  if(Serial2.available()){
    String command = Serial2.readStringUntil('\n');
    if(command.startsWith("SLEEP")){
      Serial.println("Going to sleep...");
      esp_sleep_enable_ext0_wakeup(WAKEUP_PIN, HIGH);
    }else{
      Serial.printf("Message received: %s\n", command);
    }
  }
}
