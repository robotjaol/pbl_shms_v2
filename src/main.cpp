#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_SSD1306.h>

// Define Floor and indicator
#define FLOOR_1 37
#define FLOOR_2 36
#define FLOOR_3 35
#define FLOOR_IND_1 38
#define FLOOR_IND_2 39
#define FLOOR_IND_3 40

// Additional constants for DB access
int lastLantai = 0;                                                 // Lantai terakhir ke db
bool stateBtn1 = LOW, stateBtn2 = LOW, stateBtn3 = LOW;             // Status tombol
bool lastStateBtn1 = LOW, lastStateBtn2 = LOW, lastStateBtn3 = LOW; // Status tombol sebelumnya

float temperature = 0, humidity = 0;
float temperatureState = temperature; // Handler last value
float humidityState = humidity;       // Handler last value

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
const unsigned long sensorReadInterval = 200;     // Define 5hz interval 200ms
const unsigned long dhtReadInterval = 2000;       // Define 0.5hz interval 2s
const unsigned long displayUpdateInterval = 2000; // LCD Display Update Interval

int displayIndex = 0;
int updateCount = 0;

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
    if (millis() - startTime > 2000) // Re-connect 2s
    {
      //* DEBUG */
      // Serial.println("Failed connect WiFi");
      display.clearDisplay();
      display.setCursor(0, 0);
      display.println("WiFi Fail To Connect !!!");
      display.display();
      return;
    }
    delay(500);
  }

  //* DEBUG */
  // Serial.println("WiFi Connected !!!");
  // Serial.print("IP Address: ");
  // Serial.println(WiFi.localIP());

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

  // Reset Pin Configuration
  pinMode(RESET_BUTTON_PIN, INPUT_PULLDOWN);

  // Floor Button Configuration
  pinMode(FLOOR_1, INPUT);
  pinMode(FLOOR_2, INPUT);
  pinMode(FLOOR_3, INPUT);
  // Floor Button Indicator
  pinMode(FLOOR_IND_1, OUTPUT);
  pinMode(FLOOR_IND_2, OUTPUT);
  pinMode(FLOOR_IND_3, OUTPUT);
  digitalWrite(FLOOR_IND_1, LOW); // Normal mode Mati
  digitalWrite(FLOOR_IND_2, LOW);
  digitalWrite(FLOOR_IND_3, LOW);

  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C))
  {
    Serial.println(F("SSD1306 Gagal"));
    for (;;)
      ;
  }

  //** ERROR HANDLER */
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

  //** ERROR HANDLER */
  if (isnan(temperature) || isnan(humidity))
  {
    Serial.println("DHT Sensor Error: NaN value");
  }
  else
  {
    temperatureState = temperature;
    humidityState = humidity;
  }
}

void kirimDataKeServer(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, float strainValue, float temperature, float humidity) // int current Floor saya hapus
{
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
  //  "&lantai=" + String(lastLantai);

  // http.begin(client, "http://10.17.38.28/shmsv2_2/sensor.php"); // MODUL LAMA
  http.begin(client, "http://10.17.38.28/shmsv2_2/sensor2.php"); // MODUL BARU
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(postData); // request
  String payload = http.getString();  // payload

  Serial.println("HTTP Response code: " + String(httpCode));
  Serial.println("Server response: " + payload);

  http.end();
}

void updateDisplay(float gyroX, float gyroY, float gyroZ, float accelX, float accelY, float accelZ, float strainValue, float temperature, float humidity)
{
  if (updateCount < 3)
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
      digitalWrite(FLOOR_IND_1, HIGH);
      digitalWrite(FLOOR_IND_2, LOW);
      digitalWrite(FLOOR_IND_3, LOW);
      break;
    case 1:
      display.printf("Accel X: %.2f m/s^2", accelX);
      display.printf("\nAccel Y: %.2f m/s^2", accelY);
      display.printf("\nAccel Z: %.2f m/s^2", accelZ);
      digitalWrite(FLOOR_IND_1, LOW);
      digitalWrite(FLOOR_IND_2, HIGH);
      digitalWrite(FLOOR_IND_3, LOW);
      break;
    case 2:
      display.printf("Strain  : %.2f N", strainValue);
      display.printf("\nTemp    : %.2f C", temperature);
      display.printf("\nHumidity: %.2f %%", humidity);
      digitalWrite(FLOOR_IND_1, LOW);
      digitalWrite(FLOOR_IND_2, LOW);
      digitalWrite(FLOOR_IND_3, HIGH);
      break;
      // case 3:
      //   break;
    }

    display.display();
    displayIndex = (displayIndex + 1) % 3;
  }
}

void resetSensors(float &gyroX, float &gyroY, float &gyroZ, float &accelX, float &accelY, float &accelZ, float &strainValue, float &temperature, float &humidity)
{
  gyroX = gyroY = gyroZ = accelX = accelY = accelZ = strainValue = temperature = humidity = 0;

  display.clearDisplay();
  // display.setTextSize(1);
  // display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.printf("Reset Success");
  Serial.println("Reset Success");

  static unsigned long lastBlinkTime = 0;
  static int blinkState = LOW;
  for (int i = 0; i < 4; i++)
  {
    if (millis() - lastBlinkTime >= 500)
    {
      lastBlinkTime = millis();
      blinkState = !blinkState;
      digitalWrite(FLOOR_IND_1, blinkState);
    }
  }
}

void loop()
{
  static float gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue;
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Reconnect WiFi ...");
    connectToWiFi();
  }

  // RESET mu gurung kenek jancok
  if (digitalRead(RESET_BUTTON_PIN) == HIGH)
  {
    resetSensors(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperature, humidity);
  }

  else
  {
    if (currentMillis - lastSensorReadTime >= 200) // saya ganti, sebelumnya lastDHTReadTime
    {
      lastDHTReadTime = currentMillis;
      readDHT(temperature, humidity);
    }

    // (5 Hz)
    if (currentMillis - lastSensorReadTime >= 200)
    {
      lastSensorReadTime = currentMillis;
      readSensors(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue);
      kirimDataKeServer(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperatureState, humidityState); // lastLantai saya hapus
    }

    // * LCD Display *//
    if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval)
    {
      lastDisplayUpdateTime = currentMillis;
      updateDisplay(gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue, temperature, humidity); // int currentFloor saya hapus
    }
  }

  // stateBtn1 = digitalRead(FLOOR_1);
  // stateBtn2 = digitalRead(FLOOR_2);
  // stateBtn3 = digitalRead(FLOOR_3);

  // lastStateBtn1 = stateBtn1;
  // lastStateBtn2 = stateBtn2;
  // lastStateBtn3 = stateBtn3;
}
