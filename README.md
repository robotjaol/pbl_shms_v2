## Low Memory Solution

```
#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_SSD1306.h>

// Define PIN DHT22
#define DHTPIN 1
#define DHTTYPE DHT22

// Define MPU and ADXL
#define MPU6050_ADDRESS 0x68
#define ADXL345_ADDRESS 0x53
#define SDA 8
#define SCL 9

// Define LCD output
#define OLED_RESET 7 // RESET LCD output
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SSD1306_I2C_ADDRESS 0x3C

// Define Strain Gauge and Reset
#define STRAIN_GAUGE_PIN 20
#define RESET_BUTTON_PIN 2 // RESET All Instruments

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified();
Adafruit_MPU6050 mpu;
DHT dht(DHTPIN, DHTTYPE);

unsigned long lastSensorReadTime = 0;
unsigned long lastDHTReadTime = 0;
unsigned long lastDisplayUpdateTime = 0;
const unsigned long sensorReadInterval = 200;     // Define 5hz interval
const unsigned long dhtReadInterval = 2000;       // Define 0.5hz interval
const unsigned long displayUpdateInterval = 2000; // LCD Display Update Interval

int displayIndex = 0;

void kirimDataKeServer(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, float strainValue, float temperature, float humidity);


void connectToWiFi()
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  Serial.println("Connecting to DTEO-VOKASI ... ");
  display.println("Connecting to");
  display.println("\nDTEO-VOKASI ...");
  display.display();
  delay(1500);

  WiFi.begin("DTEO-VOKASI", "TEO123456");

  unsigned long startTime = millis();
  while (WiFi.status() != WL_CONNECTED)
  {
    if (millis() - startTime > 10000) // Re-connect 10s
    {
      Serial.println("Failed connect WiFi");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("WiFi Fail To Connect !!!");
      display.display();
      return;
    }
    delay(500);
  }

  Serial.println("WiFi Connected !!!");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());

  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("WiFi Connected !!!");
  display.print("DTEO-VOKASI");
  display.display();
  delay(1500);
}

void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  pinMode(RESET_BUTTON_PIN, INPUT_PULLUP);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 Gagal"));
    for (;;)
      ;
  }
  adxl.begin(ADXL345_ADDRESS);
  mpu.begin(MPU6050_ADDRESS);
  dht.begin();
  if (!mpu.begin() && !adxl.begin())
  {
    Serial.println("MPU6050 Not Connected");
    Serial.println("ADXL345 Not Connected");
  }
  else
  {
    Serial.println("MPU6050 Connected");
    Serial.println("ADXL Connected");
  }

  connectToWiFi();
}

void readSensors(float &gyroX, float &gyroY, float &gyroZ, float &accelX, float &accelY, float &accelZ, float &strainValue)
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  int rawValue = analogRead(STRAIN_GAUGE_PIN);
  strainValue = rawValue * (100.0 / 1023.0);
}

void readDHT(float &temperature, float &humidity)
{
  temperature = dht.readTemperature();
  humidity = dht.readHumidity();
}

void kirimDataKeServer(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, float strainValue, float temperature, float humidity)
{
  // Print sensor values to Serial
  Serial.println("Data yang dikirim ke server:");
  Serial.print("Gyro X: "); Serial.println(gyroX);
  Serial.print("Gyro Y: "); Serial.println(gyroY);
  Serial.print("Gyro Z: "); Serial.println(gyroZ);
  Serial.print("Accel X: "); Serial.println(accelX);
  Serial.print("Accel Y: "); Serial.println(accelY);
  Serial.print("Accel Z: "); Serial.println(accelZ);
  Serial.print("Strain Value: "); Serial.println(strainValue);
  Serial.print("Temperature: "); Serial.println(temperature);
  Serial.print("Humidity: "); Serial.println(humidity);
  Serial.println();

  HTTPClient http;
  WiFiClient client;
  String postData;
  
  // Construct POST data string
  postData = "humidity=" + String(humidity) +
             "&temperature=" + String(temperature) +
             "&accelX=" + String(accelX) +
             "&accelY=" + String(accelY) +
             "&accelZ=" + String(accelZ) +
             "&gyroX=" + String(gyroX) +
             "&gyroY=" + String(gyroY) +
             "&gyroZ=" + String(gyroZ) +
             "&strainValue=" + String(strainValue);



  http.begin(client, "http://10.17.38.92/shmsv2/sensor.php");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(postData); // Send the request
  String payload = http.getString();  // Get the response payload
  
  Serial.println("HTTP Response code: " + String(httpCode));
  Serial.println("Server response: " + payload);
  
  http.end();
}

void updateDisplay(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, float strainValue, float temperature, float humidity)
{
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);

  switch (displayIndex)
  {
  case 0:
    display.printf("Gyro X: %.2f deg/s", gyroX);
    display.printf("\nGyro Y: %.2f deg/s", gyroY);
    display.printf("\nGyro Z: %.2f deg/s", gyroZ);
    break;
  case 1:
    display.printf("Accel X: %.2f m/s^2", accelX);
    display.printf("\nAccel Y: %.2f m/s^2", accelY);
    display.printf("\nAccel Z: %.2f m/s^2", accelZ);
    break;
  case 2:
    display.printf("Strain  : %.2f N", strainValue);
    display.printf("\nTemp    : %.2f C", temperature);
    display.printf("\nHumidity: %.2f %%", humidity);
    break;
  }

  display.display();
  displayIndex = (displayIndex + 1) % 3;
}

void resetSensors(float &gyroX, float &gyroY, float &gyroZ, float &accelX, float &accelY, float &accelZ, float &strainValue, float &temperature, float &humidity)
{
  gyroX = gyroY = gyroZ = accelX = accelY = accelZ = strainValue = temperature = humidity = 0;
  display.clearDisplay();
  display.display();
}

void loop()
{
  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Reconnect WiFi ...");
    connectToWiFi();
  }

  static float gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperature, humidity;
  unsigned long currentMillis = millis();
  if (digitalRead(RESET_BUTTON_PIN) == LOW)
  {
    resetSensors(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperature, humidity);
  }
  else
  {
    // Update sensor ADXL, MPU, dan strain gauge (5 Hz)
    if (currentMillis - lastSensorReadTime >= sensorReadInterval)
    {
      lastSensorReadTime = currentMillis;
      readSensors(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue);

      // Serial Monitor
      Serial.print("Gyro X: ");
      Serial.print(gyroX);
      Serial.print("\t Gyro Y: ");
      Serial.print(gyroY);
      Serial.print("\t Gyro Z: ");
      Serial.println(gyroZ);
      Serial.print("Accel X: ");
      Serial.print(accelX);
      Serial.print("\t Accel Y: ");
      Serial.print(accelY);
      Serial.print("\t Accel Z: ");
      Serial.println(accelZ);
      Serial.print("Strain Value: ");
      Serial.println(strainValue);
    }

    // Update DHT Sensor (0.5 Hz)
    if (currentMillis - lastDHTReadTime >= dhtReadInterval)
    {
      lastDHTReadTime = currentMillis;
      readDHT(temperature, humidity);

      // Serial Monitor
      Serial.print("Temperature: ");
      Serial.print(temperature);
      Serial.print("\t Humidity: ");
      Serial.println(humidity);

      // Kirim data ke server
      kirimDataKeServer(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperature, humidity);
    }

    // Update OLED Display (2 Hz)
    if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval)
    {
      lastDisplayUpdateTime = currentMillis;
      updateDisplay(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperature, humidity);
    }
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

#define DHTPIN 1
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