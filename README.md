## Low Memory Solution

```#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_ADXL345_U.h>
#include <DHT.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define DHTPIN 15
#define DHTTYPE DHT22
DHT dht(DHTPIN, DHTTYPE);

#define MPU_SDA 8
#define MPU_SCL 9
Adafruit_MPU6050 mpu;

#define ADXL_SDA 8
#define ADXL_SCL 9
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

#define STRAIN_PIN 34

const char *ssid = "DTEO-VOKASI";
const char *password = "TEO123456";

void setup() {
  Wire.begin(MPU_SDA, MPU_SCL);
  Serial.begin(115200);

  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1);
  }

  if (!adxl.begin()) {
    Serial.println("Failed to find ADXL345 chip");
    while (1);
  }

  dht.begin();

  if (!display.begin(SSD1306_I2C_ADDRESS, OLED_RESET)) {
    Serial.println("SSD1306 allocation failed");
    while (1);
  }
  display.clearDisplay();

  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
}

void loop() {
  static unsigned long lastSensorUpdate = 0;
  static unsigned long lastDHTUpdate = 0;
  static unsigned long lastDisplayUpdate = 0;

  if (millis() - lastSensorUpdate >= 200) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float accelX = a.acceleration.x;
    float accelY = a.acceleration.y;
    float accelZ = a.acceleration.z;
    float gyroX = g.gyro.x;
    float gyroY = g.gyro.y;
    float gyroZ = g.gyro.z;

    sensors_event_t adxlEvent;
    adxl.getEvent(&adxlEvent);
    float adxlAccelX = adxlEvent.acceleration.x;
    float adxlAccelY = adxlEvent.acceleration.y;
    float adxlAccelZ = adxlEvent.acceleration.z;

    int strainValue = analogRead(STRAIN_PIN);

    Serial.print("MPU6050 Accel X: "); Serial.print(accelX); Serial.println(" m/s^2");
    Serial.print("MPU6050 Accel Y: "); Serial.print(accelY); Serial.println(" m/s^2");
    Serial.print("MPU6050 Accel Z: "); Serial.print(accelZ); Serial.println(" m/s^2");
    Serial.print("MPU6050 Gyro X: "); Serial.print(gyroX); Serial.println(" rad/s");
    Serial.print("MPU6050 Gyro Y: "); Serial.print(gyroY); Serial.println(" rad/s");
    Serial.print("MPU6050 Gyro Z: "); Serial.print(gyroZ); Serial.println(" rad/s");

    Serial.print("ADXL345 Accel X: "); Serial.print(adxlAccelX); Serial.println(" m/s^2");
    Serial.print("ADXL345 Accel Y: "); Serial.print(adxlAccelY); Serial.println(" m/s^2");
    Serial.print("ADXL345 Accel Z: "); Serial.print(adxlAccelZ); Serial.println(" m/s^2");

    Serial.print("Strain Value: "); Serial.print(strainValue); Serial.println(" units");

    lastSensorUpdate = millis();
  }

  if (millis() - lastDHTUpdate >= 2000) {
    float temperature = dht.readTemperature();
    float humidity = dht.readHumidity();
    if (isnan(temperature) || isnan(humidity)) {
      temperature = 0;
      humidity = 0;
    }

    Serial.print("Temperature: "); Serial.print(temperature); Serial.println(" °C");
    Serial.print("Humidity: "); Serial.print(humidity); Serial.println(" %");

    lastDHTUpdate = millis();
  }

  if (millis() - lastDisplayUpdate >= 2000) {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);

    display.print("MPU6050 Accel X: "); display.println(a.acceleration.x);
    display.print("MPU6050 Accel Y: "); display.println(a.acceleration.y);
    display.print("MPU6050 Accel Z: "); display.println(a.acceleration.z);
    display.print("MPU6050 Gyro X: "); display.println(g.gyro.x);
    display.print("MPU6050 Gyro Y: "); display.println(g.gyro.y);
    display.print("MPU6050 Gyro Z: "); display.println(g.gyro.z);

    display.print("ADXL345 Accel X: "); display.println(adxlAccelX);
    display.print("ADXL345 Accel Y: "); display.println(adxlAccelY);
    display.print("ADXL345 Accel Z: "); display.println(adxlAccelZ);

    display.print("Strain: "); display.println(analogRead(STRAIN_PIN));
    display.display();

    lastDisplayUpdate = millis();
  }
}

```

## Multitask optimization

```

#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_ADXL345_U.h>
#include <DHT.h>

#define DHTPIN 6
#define DHTTYPE DHT22
#define MPU6050_ADDRESS 0x68
#define ADXL345_ADDRESS 0x53
#define STRAIN_GAUGE_PIN 10
#define SDA_PIN 8
#define SCL_PIN 9
#define RESET_BUTTON 2
#define OLED_RESET 7

DHT dht(DHTPIN, DHTTYPE);
Adafruit_MPU6050 mpu;
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified(12345);

TaskHandle_t sensorTaskHandle;
TaskHandle_t wifiTaskHandle;

char ssid[] = "DTEO-VOKASI";
char password[] = "TEO123456";
unsigned long lastDHT22 = 0;
unsigned long lastSensorRead = 0;

float gyroX, gyroY, gyroZ;
float accelX, accelY, accelZ;
int strainValue;
int temperature, humidity;

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.begin(115200);
  
  pinMode(RESET_BUTTON, INPUT_PULLUP);

  if (!mpu.begin(MPU6050_ADDRESS)) {
    Serial.println("MPU6050 failed to initialize!");
    while (1);
  }

  if (!adxl.begin()) {
    Serial.println("ADXL345 failed to initialize!");
    while (1);
  }

  dht.begin();
  connectToWiFi();

  xTaskCreatePinnedToCore(sensorTask, "SensorTask", 2000, NULL, 1, &sensorTaskHandle, 1);
  xTaskCreatePinnedToCore(wifiTask, "WiFiTask", 2000, NULL, 1, &wifiTaskHandle, 0);
}

void loop() {
  if (digitalRead(RESET_BUTTON) == LOW) {
    resetSensors();
  }
  delay(100);
}

void sensorTask(void *parameter) {
  while (1) {
    if (millis() - lastSensorRead >= 200) {
      readMPU6050();
      readADXL345();
      readStrainGauge();
      lastSensorRead = millis();
    }
    
    if (millis() - lastDHT22 >= 2000) {
      readDHT22();
      lastDHT22 = millis();
    }

    printToSerial();
    vTaskDelay(200 / portTICK_PERIOD_MS);
  }
}

void wifiTask(void *parameter) {
  while (1) {
    sendDataToServer();
    vTaskDelay(5000 / portTICK_PERIOD_MS);
  }
}

void connectToWiFi() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
  }
  Serial.println("WiFi connected!");
}

void readMPU6050() {
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroX = g.gyro.x * 100;  // converting to int for lighter data processing
  gyroY = g.gyro.y * 100;
  gyroZ = g.gyro.z * 100;
}

void readADXL345() {
  sensors_event_t event;
  adxl.getEvent(&event);
  accelX = event.acceleration.x * 100;
  accelY = event.acceleration.y * 100;
  accelZ = event.acceleration.z * 100;
}

void readStrainGauge() {
  strainValue = analogRead(STRAIN_GAUGE_PIN);
}

void readDHT22() {
  temperature = dht.readTemperature() * 100;  // Converting to int for lighter data
  humidity = dht.readHumidity() * 100;
}

void sendDataToServer() {
  if (WiFi.status() != WL_CONNECTED) {
    connectToWiFi();
  }
  
  HTTPClient http;
  http.begin("http://10.17.38.92/WebsiteMonitoring/SHMS");
  http.addHeader("Content-Type", "application/json");

  char jsonData[256];
  sprintf(jsonData, "{\"gyroX\":%d,\"gyroY\":%d,\"gyroZ\":%d,\"accelX\":%d,\"accelY\":%d,\"accelZ\":%d,\"strain\":%d,\"temperature\":%d,\"humidity\":%d}",
          (int)gyroX, (int)gyroY, (int)gyroZ, (int)accelX, (int)accelY, (int)accelZ, strainValue, temperature, humidity);
  
  int httpResponseCode = http.POST(jsonData);
  if (httpResponseCode > 0) {
    Serial.printf("Data sent successfully: %s\n", jsonData);
  } else {
    Serial.printf("Failed to send data, code: %d\n", httpResponseCode);
  }
  http.end();
}

void printToSerial() {
  Serial.print("Gyro X: ");
  Serial.print(gyroX / 100.0);
  Serial.println(" rad/s");
  Serial.print("Gyro Y: ");
  Serial.print(gyroY / 100.0);
  Serial.println(" rad/s");
  Serial.print("Gyro Z: ");
  Serial.print(gyroZ / 100.0);
  Serial.println(" rad/s");

  Serial.print("Accel X: ");
  Serial.print(accelX / 100.0);
  Serial.println(" m/s^2");
  Serial.print("Accel Y: ");
  Serial.print(accelY / 100.0);
  Serial.println(" m/s^2");
  Serial.print("Accel Z: ");
  Serial.print(accelZ / 100.0);
  Serial.println(" m/s^2");

  Serial.print("Strain: ");
  Serial.println(strainValue);

  Serial.print("Temp: ");
  Serial.print(temperature / 100.0);
  Serial.println(" C");

  Serial.print("Humidity: ");
  Serial.print(humidity / 100.0);
  Serial.println(" %");
}

void resetSensors() {
  ESP.restart();
}

```