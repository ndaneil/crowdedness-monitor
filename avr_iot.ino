/* 
 * Author: Daniel Nemeth
 */

#include <Arduino.h>
#include <http_client.h>
#include <led_ctrl.h>
#include <log.h>
#include <lte.h>
#include <low_power.h>


#define INFLUX_TOKEN "<<<ENTER_INFLUX_TOKEN_HERE>>>"
#define INFLUX_ORG "<<<ENTER_INFLUX_ORG_ID_HERE>>>"

#define INFLUX_HEADER "Authorization: Token " INFLUX_TOKEN

#define INFLUX_URL "eu-central-1-1.aws.cloud2.influxdata.com"
#define INFLUX_ENDPOINT "/api/v2/write?org=" INFLUX_ORG "&bucket=sensordata&precision=s"
#define DEVICE_TAG "AVR_IOT"


#define ESP_WAKEUP_PIN PIN_PE2 // D5
#define MIC_PIN PIN_PD6 // A0

#define DATA_SEND_INTERVAL 180000 // 3 minutes

long lastSentTime = 0; // The last time data was sent to the cloud

#define NOISE_BUFFER_SIZE 64 // buffer for storing history of noise levels:
int noiseValues[] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int noiseWriteIdx = 0; // index for the circular buffer above

// Storing serial messages from ESP32:
int lastWiFiDeviceCount = 0;
int lastBleDeviceCount = 0;
long lastEspHeapInfo = 0;
long lastDataUpdateFromESP = 0;

#include <SoftwareSerial.h> // For some reason, the hardware serial did not work
SoftwareSerial mySerial(PIN_PF5, PIN_PF4); // RX TX


void sendData(){
    int noisemax = noiseValues[0];
    int noisemin = noiseValues[0];
    double noisemean = noiseValues[0];
    for(int i = 1; i < NOISE_BUFFER_SIZE; i++) {
      noisemax = max(noisemax, noiseValues[i]);
      noisemin = min(noisemin, noiseValues[i]);
      noisemean += noiseValues[i];
    }
    noisemean /= NOISE_BUFFER_SIZE;
    char data[256];
    if(millis()-lastDataUpdateFromESP < DATA_SEND_INTERVAL) {
      // New data from ESP32
      sprintf(data, "noise,device=%s noise-mean=%.1f,noise-max=%di,noise-min=%di\npower,device=%s voltage=%.2f\nwireless,device=%s ble-devices=%di,wifi-devices=%di,esp-free-heap=%ldi", 
      DEVICE_TAG, noisemean, noisemax, noisemin, DEVICE_TAG, (double) LowPower.getSupplyVoltage(), DEVICE_TAG, lastBleDeviceCount, lastWiFiDeviceCount, lastEspHeapInfo);
    }else{
      Log.infof(F("Skipping sending ESP data as there is no recent one available!\n"));
      sprintf(data, "noise,device=%s noise-mean=%.1f,noise-max=%di,noise-min=%di\npower,device=%s voltage=%.2f", DEVICE_TAG, noisemean, noisemax, noisemin,DEVICE_TAG, (double) LowPower.getSupplyVoltage());
    }
    
    Log.infof(F("Sending data:\n%s\n"),data);
    HttpResponse response = HttpClient.post(INFLUX_ENDPOINT, data, INFLUX_HEADER);

    Log.infof(F("POST - HTTP status code: %u, data size: %u\n"),
              response.status_code,
              response.data_size);
}

int getSoundLevel(){
  int l_min;
  int l_max;
  int val = l_min = l_max = analogRead(MIC_PIN);
  for(unsigned int i = 0; i < 50000; i++){
    val = analogRead(MIC_PIN);
    l_min = min(l_min,val);
    l_max = max(l_max, val);
  }
  // Log.infof(F("Sound max: %d min: %d\n"), l_max, l_min);
  return l_max-l_min;
}


void setup() {
    LedCtrl.begin();
    LedCtrl.startupCycle();
    pinConfigure(ESP_WAKEUP_PIN, PIN_DIR_OUTPUT);
    digitalWrite(ESP_WAKEUP_PIN, HIGH);
    mySerial.begin(115200);
    mySerial.println("Hello!");

    Log.begin(115200);
    Log.info(F("Starting HTTPS with header example"));

    // Start LTE modem and connect to the operator
    if (!Lte.begin()) {
        Log.error(F("Failed to connect to the operator"));
        return;
    }

    Log.infof(F("Connected to operator: %s\r\n"), Lte.getOperator().c_str());

    if (!HttpClient.configure(INFLUX_URL, 443, true)) {
        Log.info(F("Failed to configure https client\r\n"));
        return;
    }

    analogRead(MIC_PIN); //A0
    digitalWrite(ESP_WAKEUP_PIN, LOW);
}

void loop() {
  float voltage = LowPower.getSupplyVoltage();
  int soundLevel = getSoundLevel();
  noiseValues[noiseWriteIdx] = soundLevel;
  noiseWriteIdx = (noiseWriteIdx+1)%NOISE_BUFFER_SIZE;
  
  if(noiseWriteIdx == 0) {
    Log.infof(F("   Voltage: %f noise level: %d\n"), voltage, soundLevel);
  }

  if(mySerial.available()){
    String data = mySerial.readStringUntil('\n');
    // Format: <num_wifi_devices>,<num_ble_devies>,<esp_free_heap>
    int firstComma = data.indexOf(",");
    int secondComma = data.indexOf(",",firstComma+1);
    int numWifiDevices = data.substring(0,firstComma).toInt();
    int numBleDevices = data.substring(firstComma+1, secondComma).toInt();
    long espFreeHeap = data.substring(secondComma+1).toInt();
    Log.infof(F("    Data from ESP32 #WIFI: %d #BLE: %d free heap: %ld\n"), numWifiDevices, numBleDevices, espFreeHeap);
    lastWiFiDeviceCount = numWifiDevices;
    lastBleDeviceCount = numBleDevices;
    lastEspHeapInfo = espFreeHeap;
    lastDataUpdateFromESP = millis();

  }
  
  if(millis()-lastSentTime > DATA_SEND_INTERVAL){
    sendData();
    lastSentTime = millis();
  }

  delay(100);
}
