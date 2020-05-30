#include <M5StickC.h>

#include "DHT12.h"
#include <Wire.h>
#include "Adafruit_Sensor.h"
#include <Adafruit_BMP280.h>

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <esp_wifi.h>
#include <esp_gatts_api.h>

DHT12 dht12;
Adafruit_BMP280 bme;

uint16_t temp  = 0;
uint16_t humid = 0;
uint16_t press = 0;
uint8_t  seq   = 0;

BLECharacteristic *pCharacteristic = NULL;
bool deviceConnected = false;
uint8_t buf[7];

// https://www.uuidgenerator.net/
//Device UUID  = D9DFFD12-62B9-1EF7-33C2-C5A5E1D44D10
#define SERVICE_UUID        "3352"
#define CHARACTERISTIC_UUID "3353"

//Value  = 0x + sequence +  temp  + humid + press
//Value0 = 0x +    00    + 9a-0b  + 0c-0d + 51-27
//Value1 = 0x +    01    + 9a-0b  + 0c-0d + 51-27
//Value2 = 0x +    02    + 9a-0b  + 0c-0d + 51-27
//16-base                   0b9a     0d0c    2751
//10-base                   2970     3340   10065
//

//-------------------------------------------------------------------------------------------------------------
void bdaDump(esp_bd_addr_t bd) {
    for (int i = 0; i < ESP_BD_ADDR_LEN; i++) {
        Serial.printf("%02x", bd[i]);
        if (i < ESP_BD_ADDR_LEN - 1) {
            Serial.print(":");
        }
    }
};

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer, esp_ble_gatts_cb_param_t* param) {
        Serial.print("connected from: ");
        bdaDump(param->connect.remote_bda);
        Serial.println("");

        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;     // timeout = 400*10ms = 4000ms
        esp_ble_gap_update_conn_params(&conn_params);
        deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
        Serial.println("disconnected");
        deviceConnected = false;
    }
};

//-------------------------------------------------------------------------------------------------------------
class dataCb: public BLECharacteristicCallbacks {
  void onRead(BLECharacteristic *pCharacteristic){
    memset(buf, 0, sizeof buf);               // バッファーを0クリア
    buf[0] = seq++;                           // シーケンス番号をバッファーにセット
    buf[1] = (uint8_t)(temp & 0xff);          // 温度の下位バイトをセット
    buf[2] = (uint8_t)((temp >> 8) & 0xff);   // 温度の上位バイトをセット
    buf[3] = (uint8_t)(humid & 0xff);         // 湿度の下位バイトをセット
    buf[4] = (uint8_t)((humid >> 8) & 0xff);  // 湿度の上位バイトをセット
    buf[5] = (uint8_t)(press & 0xff);         // 気圧の下位バイトをセット
    buf[6] = (uint8_t)((press >> 8) & 0xff);  // 気圧の上位バイトをセット
    pCharacteristic->setValue(buf, sizeof buf);         // データーを書き込み
  }
};

//-------------------------------------------------------------------------------------------------------------
void setup() {
    M5.begin();
    
    M5.Axp.ScreenBreath(10);    // 画面の輝度を下げる
    M5.Lcd.setRotation(3);      // 左を上にする
    M5.Lcd.fillScreen(BLACK);   // 背景を黒にする
    M5.Lcd.setCursor(0, 0, 2);
    M5.Lcd.printf("M5Stick-C_Env");

    Wire.begin(0, 26);               // I2Cを初期化する
    if (!bme.begin(0x76)) {  // BMP280を初期化する
        M5.Lcd.println("BMP280 Fail");
        while (1);
    }

    temp  = dht12.readTemperature() * 100;
    humid = dht12.readHumidity() * 100;
    press = bme.readPressure() / 100 * 10;

    Serial.println("Starting BLE work!");
    BLEDevice::init("M5Stick-C_Env");                  // デバイスを初期化
    BLEServer *pServer = BLEDevice::createServer();    // サーバーを生成
    pServer->setCallbacks(new MyServerCallbacks());    // コールバック関数を設定

    BLEService *pService = pServer->createService(SERVICE_UUID);  // サービスを生成
   
    pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_WRITE
                                       );
    pCharacteristic->setCallbacks(new dataCb());

    pService->start();                                 // サービスを起動
    pServer->getAdvertising()->start();                // アドバタイズを起動
    Serial.println("Characteristic defined! Now you can read it in your phone!");
}

void loop() {
  M5.Lcd.setCursor(0, 20, 2);
  M5.Lcd.printf("temp: %2.1f'C\r\n", (float)temp / 100);
  M5.Lcd.setCursor(0, 40, 2);
  M5.Lcd.printf("humid:%2.0f%%\r\n", (float)humid / 100);
  M5.Lcd.setCursor(0, 60, 2);
  M5.Lcd.printf("press:%2.1fhPa\r\n", (float)press / 10);

  if (deviceConnected) {
    if(M5.BtnA.wasPressed()) {
      pCharacteristic->setValue(buf, sizeof buf);
      pCharacteristic->notify();
    }
  }
  M5.update();
}
