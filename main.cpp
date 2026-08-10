#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include "esp_sleep.h"
#include <Adafruit_NeoPixel.h>
#include <Preferences.h>

Preferences preferences;

// --- Piny pre MPU6500 ---
#define I2C_SDA 6
#define I2C_SCL 7
const uint8_t SENSOR_ADDR = 0x68;

#define INT_PIN 5 

// --- Indikačné LED ---
#define LED_PIN 15          // Stavová dióda napájania
#define RGB_LED_PIN 8       // Adresovateľná RGB dióda (WS2812)
#define NUM_LEDS 1
Adafruit_NeoPixel strip(NUM_LEDS, RGB_LED_PIN, NEO_GRB + NEO_KHZ800);

#define SLEEP_TIMEOUT_MS 900000 
unsigned long last_active_time = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;

float gx = 0.0f;
float gy = 0.0f;
float gz = 0.0f; 
bool request_calibration = false;
bool request_accel_calibration = false;
unsigned long last_time = 0;

float gyro_offset_x = 0.0f;
float gyro_offset_y = 0.0f;
float gyro_offset_z = 0.0f;

float accel_offset_x = 0.0f;
float accel_offset_y = 0.0f;
float accel_offset_z = 0.0f;
float accel_scale_x = 1.0f;
float accel_scale_y = 1.0f;
float accel_scale_z = 1.0f;

void load_accel_calibration() {
  preferences.begin("accel_cal", true); 
  accel_offset_x = preferences.getFloat("off_x", 0.0f);
  accel_offset_y = preferences.getFloat("off_y", 0.0f);
  accel_offset_z = preferences.getFloat("off_z", 0.0f);
  accel_scale_x = preferences.getFloat("scale_x", 1.0f);
  accel_scale_y = preferences.getFloat("scale_y", 1.0f);
  accel_scale_z = preferences.getFloat("scale_z", 1.0f);
  preferences.end();
  Serial.printf("Accel Calib: Offsets(%.3f, %.3f, %.3f), Scales(%.3f, %.3f, %.3f)\n", 
                accel_offset_x, accel_offset_y, accel_offset_z, 
                accel_scale_x, accel_scale_y, accel_scale_z);
}

void writeRegister(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

void goToDeepSleep() {
  Serial.println("Pripravujem sa na spanok...");
  writeRegister(0x6B, 0x00); 
  delay(10);
  writeRegister(0x6C, 0x07); 
  writeRegister(0x37, 0x20); 
  writeRegister(0x1D, 0x09); 
  writeRegister(0x38, 0x40); 
  writeRegister(0x69, 0xC0); 
  writeRegister(0x1F, 0x20); 
  writeRegister(0x1E, 0x04); 
  writeRegister(0x6B, 0x20); 
  delay(100); 
  
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x3A); 
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, (uint8_t)1);
  if(Wire.available()) Wire.read();

  // Zhasnutie oboch LED pred spánkom
  digitalWrite(LED_PIN, LOW);
  strip.clear();
  strip.show();

  Serial.println("Zariadenie spi. Zobudis ma zatrasenim...");
  delay(100);
  
  esp_deep_sleep_enable_gpio_wakeup((1ULL << INT_PIN), ESP_GPIO_WAKEUP_GPIO_HIGH);
  esp_deep_sleep_start();
}

void calibrate_sensor() {
  Serial.println("Zacina kalibracia gyroskopu so striktnou kontrolou pokoja...");
  
  // --- BURN FÁZA (Warm-up senzora) ---
  // Zahodíme prvých 50 hodnôt, aby sa stabilizovali vnútorné zosilňovače 
  // a digitálne filtre (DLPF) v MPU6500 po zobudení alebo dlhšej nečinnosti.
  for (int i = 0; i < 50; i++) {
    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(SENSOR_ADDR, (uint8_t)6);
    if (Wire.available() >= 6) {
      for(int j=0; j<6; j++) Wire.read();
    }
    delay(2);
  }
  
  long sum_gx, sum_gy, sum_gz;
  int gyro_samples = 300;
  bool is_stable = false;
  int tolerance = 60; // Tolerancia rozptylu surových dát (cca 0.5 stupňa za sekundu)

  unsigned long last_blink = millis();
  bool led_state = false;

  // 1. Gyroskop - Strict Variance Check
  while (!is_stable) {
    sum_gx = 0; sum_gy = 0; sum_gz = 0;
    int16_t min_gx = 32767, max_gx = -32768;
    int16_t min_gy = 32767, max_gy = -32768;
    int16_t min_gz = 32767, max_gz = -32768;
    is_stable = true;

    for (int i = 0; i < gyro_samples; i++) {
      
      // --- Rýchle blikanie RGB LED na červeno každých 150 ms ---
      if (millis() - last_blink > 150) {
        last_blink = millis();
        led_state = !led_state;
        if (led_state) {
            strip.setPixelColor(0, strip.Color(255, 0, 0)); // Červená
        } else {
            strip.clear(); // Vypnutá
        }
        strip.show();
      }

      Wire.beginTransmission(SENSOR_ADDR);
      Wire.write(0x43); 
      Wire.endTransmission(false);
      Wire.requestFrom(SENSOR_ADDR, (uint8_t)6);
      
      if (Wire.available() >= 6) {
        int16_t rgx = (Wire.read() << 8) | Wire.read();
        int16_t rgy = (Wire.read() << 8) | Wire.read();
        int16_t rgz = (Wire.read() << 8) | Wire.read();
        
        if (rgx > max_gx) max_gx = rgx; if (rgx < min_gx) min_gx = rgx;
        if (rgy > max_gy) max_gy = rgy; if (rgy < min_gy) min_gy = rgy;
        if (rgz > max_gz) max_gz = rgz; if (rgz < min_gz) min_gz = rgz;

        sum_gx += rgx;
        sum_gy += rgy;
        sum_gz += rgz;
      }
      
      // Ak zistíme mikrootras, kalibrácia padne a začne zbierať 300 vzoriek odznova
      if ((max_gx - min_gx > tolerance) || (max_gy - min_gy > tolerance) || (max_gz - min_gz > tolerance)) {
        is_stable = false;
        Serial.println("Zaznamenany pohyb (mikrootras)! Restartujem kalibraciu gyroskopu...");
        delay(100); 
        break;
      }
      delay(2); 
    }
  }
  
  gyro_offset_x = (float)sum_gx / gyro_samples;
  gyro_offset_y = (float)sum_gy / gyro_samples;
  gyro_offset_z = (float)sum_gz / gyro_samples;

  Serial.println("Gyroskop stabilne skalibrovany. Mierim zameriavac na gravitaciu...");

  // 2. Akcelerometer - Vylepseny zber (500 vzoriek)
  float sum_ax = 0, sum_ay = 0, sum_az = 0;
  int accel_samples = 500;
  
  for (int i = 0; i < accel_samples; i++) {
      
    // --- Pokračovanie červeného blikania aj počas merania gravitácie ---
    if (millis() - last_blink > 150) {
      last_blink = millis();
      led_state = !led_state;
      if (led_state) {
          strip.setPixelColor(0, strip.Color(255, 0, 0)); // Červená
      } else {
          strip.clear();
      }
      strip.show();
    }

    Wire.beginTransmission(SENSOR_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(SENSOR_ADDR, (uint8_t)6);
    
    if (Wire.available() >= 6) {
      int16_t rax = (Wire.read() << 8) | Wire.read();
      int16_t ray = (Wire.read() << 8) | Wire.read();
      int16_t raz = (Wire.read() << 8) | Wire.read();
      
      sum_ax += (float)rax / 8192.0f;
      sum_ay += (float)ray / 8192.0f;
      sum_az += (float)raz / 8192.0f;
    }
    delay(2); 
  }
  
  gx = sum_ax / (float)accel_samples;
  gy = sum_ay / (float)accel_samples;
  gz = sum_az / (float)accel_samples;
  
  float g_norm = sqrt(gx*gx + gy*gy + gz*gz);
  if (g_norm > 0.0f) {
    gx /= g_norm;
    gy /= g_norm;
    gz /= g_norm;
  } else {
    gz = 1.0f; 
  }
  
  // 3. Koniec kalibrácie -> RGB LED svieti trvalo na ZELENO
  strip.setPixelColor(0, strip.Color(0, 255, 0));
  strip.show();
  
  if (deviceConnected && pCharacteristic) {
      pCharacteristic->setValue("CALIB_DONE");
      pCharacteristic->notify();
      delay(100); 
  }
  Serial.println("CALIB_DONE");
}

void calibrate_accelerometer_6point() {
  Serial.println("Startuje 6-bodova kalibracia akcelerometra...");
  
  float x_max = -100.0, x_min = 100.0;
  float y_max = -100.0, y_min = 100.0;
  float z_max = -100.0, z_min = 100.0;
  
  int sides_captured = 0;
  bool side_captured[6] = {false, false, false, false, false, false}; // X+, X-, Y+, Y-, Z+, Z-
  
  strip.setPixelColor(0, strip.Color(128, 0, 128)); // Fialova
  strip.show();
  delay(1000);

  unsigned long last_blink = millis();
  bool led_state = false;
  
  while (sides_captured < 6) {
    if (millis() - last_blink > 300) {
      last_blink = millis();
      led_state = !led_state;
      if (led_state) strip.setPixelColor(0, strip.Color(128, 0, 128));
      else strip.clear();
      strip.show();
    }
    
    int accel_samples = 200;
    float sum_ax = 0, sum_ay = 0, sum_az = 0;
    float min_ax = 100, max_ax = -100;
    float min_ay = 100, max_ay = -100;
    float min_az = 100, max_az = -100;
    bool stable = true;
    
    for (int i = 0; i < accel_samples; i++) {
      Wire.beginTransmission(SENSOR_ADDR);
      Wire.write(0x3B);
      Wire.endTransmission(false);
      Wire.requestFrom(SENSOR_ADDR, (uint8_t)6);
      
      if (Wire.available() >= 6) {
        float ax = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
        float ay = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
        float az = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
        
        sum_ax += ax; sum_ay += ay; sum_az += az;
        if(ax < min_ax) min_ax = ax; if(ax > max_ax) max_ax = ax;
        if(ay < min_ay) min_ay = ay; if(ay > max_ay) max_ay = ay;
        if(az < min_az) min_az = az; if(az > max_az) max_az = az;
      }
      delay(5);
    }
    
    if ((max_ax - min_ax > 0.05) || (max_ay - min_ay > 0.05) || (max_az - min_az > 0.05)) {
      stable = false;
    }
    
    if (stable) {
      float avg_x = sum_ax / accel_samples;
      float avg_y = sum_ay / accel_samples;
      float avg_z = sum_az / accel_samples;
      
      int side_idx = -1;
      if (avg_x > 0.7 && !side_captured[0]) { x_max = avg_x; side_idx = 0; }
      else if (avg_x < -0.7 && !side_captured[1]) { x_min = avg_x; side_idx = 1; }
      else if (avg_y > 0.7 && !side_captured[2]) { y_max = avg_y; side_idx = 2; }
      else if (avg_y < -0.7 && !side_captured[3]) { y_min = avg_y; side_idx = 3; }
      else if (avg_z > 0.7 && !side_captured[4]) { z_max = avg_z; side_idx = 4; }
      else if (avg_z < -0.7 && !side_captured[5]) { z_min = avg_z; side_idx = 5; }
      
      if (side_idx != -1) {
        side_captured[side_idx] = true;
        sides_captured++;
        
        Serial.printf("Strana %d zachytena!\n", side_idx);
        
        strip.setPixelColor(0, strip.Color(0, 255, 0)); // Zelena
        strip.show();
        delay(1500);
        
        if (sides_captured < 6) {
          strip.setPixelColor(0, strip.Color(0, 0, 255)); // Modra
          strip.show();
          
          bool moved = false;
          while (!moved) {
            Wire.beginTransmission(SENSOR_ADDR);
            Wire.write(0x3B);
            Wire.endTransmission(false);
            Wire.requestFrom(SENSOR_ADDR, (uint8_t)6);
            if (Wire.available() >= 6) {
              float c_ax = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
              float c_ay = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
              float c_az = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
              if (abs(c_ax - avg_x) > 0.2 || abs(c_ay - avg_y) > 0.2 || abs(c_az - avg_z) > 0.2) {
                moved = true;
              }
            }
            delay(50);
          }
        }
      }
    }
  }
  
  accel_offset_x = (x_max + x_min) / 2.0f;
  accel_scale_x  = (x_max - x_min) / 2.0f;
  accel_offset_y = (y_max + y_min) / 2.0f;
  accel_scale_y  = (y_max - y_min) / 2.0f;
  accel_offset_z = (z_max + z_min) / 2.0f;
  accel_scale_z  = (z_max - z_min) / 2.0f;
  
  preferences.begin("accel_cal", false); 
  preferences.putFloat("off_x", accel_offset_x);
  preferences.putFloat("off_y", accel_offset_y);
  preferences.putFloat("off_z", accel_offset_z);
  preferences.putFloat("scale_x", accel_scale_x);
  preferences.putFloat("scale_y", accel_scale_y);
  preferences.putFloat("scale_z", accel_scale_z);
  preferences.end();
  
  Serial.println("6-bodova kalibracia akcelerometra uspesna!");
  
  if (deviceConnected && pCharacteristic) {
      pCharacteristic->setValue("ACCEL_CALIB_DONE");
      pCharacteristic->notify();
      delay(100); 
  }
  
  for(int i=0; i<3; i++) {
    strip.setPixelColor(0, strip.Color(0, 255, 0));
    strip.show();
    delay(200);
    strip.clear();
    strip.show();
    delay(200);
  }
}

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      Serial.println("Bluetooth pripojené!");
    };
    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
      Serial.println("Bluetooth odpojené!");
      BLEDevice::startAdvertising();
      // Pri strate spojenia zhasneme RGB ledku
      strip.clear();
      strip.show();
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      String rxValue = pCharacteristic->getValue();
      if (rxValue.length() > 0) {
        if (rxValue[0] == 'C') {
            request_calibration = true;
        } else if (rxValue[0] == 'A') {
            request_accel_calibration = true;
        }
      }
    }
};

void setup() {
  Serial.begin(115200);
  delay(1000); 
  
  // Inicializácia stavovej LED (Pin 15)
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  
  // Inicializácia RGB LED (Pin 8)
  strip.begin();
  strip.setBrightness(50); // Bezpečný jas (aby neoslepila)
  strip.setPixelColor(0, strip.Color(0, 0, 255)); // Modrá = prebieha štart / hľadá BLE
  strip.show();

  pinMode(INT_PIN, INPUT_PULLDOWN);
  
  Wire.begin(I2C_SDA, I2C_SCL);
  Wire.setClock(400000);

  Wire.beginTransmission(SENSOR_ADDR);
  if (Wire.endTransmission() != 0) {
    Serial.println("CHYBA: Senzor neodpoveda!");
    strip.setPixelColor(0, strip.Color(255, 165, 0)); // Oranžová = hardvérová chyba senzora
    strip.show();
    while(1) { delay(100); } 
  }

  writeRegister(0x6B, 0x00); 
  delay(50);
  writeRegister(0x1C, 0x08); 
  writeRegister(0x1B, 0x08); 
  writeRegister(0x1A, 0x03);
  writeRegister(0x37, 0x00); 
  
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x3A); 
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, (uint8_t)1);
  if(Wire.available()) Wire.read();

  BLEDevice::init("Smart_Collar_ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_NOTIFY | BLECharacteristic::PROPERTY_WRITE
                    );
                    
  pCharacteristic->setCallbacks(new MyCallbacks());
  pService->start();
  
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  
  pAdvertising->setMinPreferred(0x06);  
  pAdvertising->setMaxPreferred(0x12); 
  
  BLEDevice::startAdvertising();
  Serial.println("Čakám na BLE pripojenie...");

  load_accel_calibration();

  // Spustenie prvotnej kalibrácie priamo pri boote
  calibrate_sensor();
  last_time = micros();
  last_active_time = millis();
}

void loop() {
  // --- 1. SPRACOVANIE POŽIADAVKY NA KALIBRÁCIU Z PYTHONU ---
  if (request_calibration) {
      request_calibration = false;
      calibrate_sensor();
      
      last_time = micros();
      last_active_time = millis();
      
      return; 
  }
  if (request_accel_calibration) {
      request_accel_calibration = false;
      calibrate_accelerometer_6point();
      last_time = micros();
      last_active_time = millis();
      return; 
  }
  // -----------------------------------------------------------

  // --- 2. KONTROLA DEEP SLEEP ---
  if (deviceConnected) {
    last_active_time = millis(); 
  } else {
    if (millis() - last_active_time > SLEEP_TIMEOUT_MS) {
      goToDeepSleep();
    }
  }

  // --- 3. VÝPOČET ČASOVÉHO KROKU (dt) ---
  unsigned long current_time = micros();
  float dt = (current_time - last_time) / 1000000.0f;
  last_time = current_time;

  // Bezpečnostný limit pre dt
  if (dt > 0.1f) dt = 0.01f;

  // --- 4. ZBER DÁT ZO SENZORA ---
  Wire.beginTransmission(SENSOR_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(SENSOR_ADDR, (uint8_t)14);
  
  if (Wire.available() >= 14) {
    float ax = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
    float ay = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
    float az = (float)((int16_t)((Wire.read() << 8) | Wire.read())) / 8192.0f;
    
    // Aplikacia 6-bodovej kalibracie
    ax = (ax - accel_offset_x) / accel_scale_x;
    ay = (ay - accel_offset_y) / accel_scale_y;
    az = (az - accel_offset_z) / accel_scale_z;
    
    Wire.read(); Wire.read(); // Teplotu preskočíme
    
    float wx = (((int16_t)((Wire.read() << 8) | Wire.read()) - gyro_offset_x) / 65.5f) * (PI / 180.0f);
    float wy = (((int16_t)((Wire.read() << 8) | Wire.read()) - gyro_offset_y) / 65.5f) * (PI / 180.0f);
    float wz = (((int16_t)((Wire.read() << 8) | Wire.read()) - gyro_offset_z) / 65.5f) * (PI / 180.0f);

    // Integrácia gyroskopu
    gx = gx + (gy * wz - gz * wy) * dt;
    gy = gy + (gz * wx - gx * wz) * dt;
    gz = gz + (gx * wy - gy * wx) * dt;

    float g_norm = sqrt(gx*gx + gy*gy + gz*gz);
    if (g_norm > 0.0f) {
        gx /= g_norm;
        gy /= g_norm;
        gz /= g_norm;
    }

    // Výpočet lineárneho zrýchlenia
    float lin_ax = ax - gx; 
    float lin_ay = ay - gy; 
    float lin_az = az - gz;

    // --- Adaptívny ZUPT Filter ---
    float w_norm = sqrt(wx*wx + wy*wy + wz*wz);
    
    float a_norm = sqrt(ax*ax + ay*ay + az*az);
    if (w_norm < 0.08f && a_norm > 0.9f && a_norm < 1.1f) {
        float alpha = 0.98f;
            gx = gx * alpha + (ax / a_norm) * (1.0f - alpha);
            gy = gy * alpha + (ay / a_norm) * (1.0f - alpha);
            gz = gz * alpha + (az / a_norm) * (1.0f - alpha);
            
            g_norm = sqrt(gx*gx + gy*gy + gz*gz);
            if (g_norm > 0.0f) {
                gx /= g_norm;
                gy /= g_norm;
                gz /= g_norm;
            }
            
            lin_ax = ax - gx; 
            lin_ay = ay - gy; 
            lin_az = az - gz;
        }

    // Finálna projekcia do globálnych osí
    float global_az = -(lin_ax * gx + lin_ay * gy + lin_az * gz);

    float norm_y = sqrt(gz * gz + gy * gy);
    float global_ay = 0.0f;
    
    if (norm_y > 0.01f) { 
        float fwd_y = gz / norm_y;
        float fwd_z = -gy / norm_y;
        global_ay = (lin_ay * fwd_y + lin_az * fwd_z);
    }

    float final_ay = global_ay * 9.81f;
    float final_az = global_az * 9.81f;

    // 5. Odoslanie paketu do Pythonu
    if (deviceConnected) {
        char buffer[48];
        snprintf(buffer, sizeof(buffer), "%.2f,%.2f,%.3f", final_ay, final_az, dt);
        pCharacteristic->setValue((uint8_t*)buffer, strlen(buffer));
        pCharacteristic->notify();
    }
  }

  // --- 6. STABILIZÁCIA SLUČKY NA 100 Hz ---
  unsigned long loop_duration = micros() - current_time;
  if (loop_duration < 10000) {
      delayMicroseconds(10000 - loop_duration);
  }
}