// Add Kalman Filter Reading MPU6050
#include <Arduino.h>
#include <Adafruit_ADXL345_U.h>
#include <Adafruit_MPU6050.h>
#include <DHT.h>
#include <Wire.h>
#include <WiFi.h>
#include <HTTPClient.h>
#include <Adafruit_SSD1306.h>

#define DHTPIN 1
#define DHTTYPE DHT22
#define MPU6050_ADDRESS 0x68
#define ADXL345_ADDRESS 0x53
#define SDA 8
#define SCL 9
#define OLED_RESET 7
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 32
#define SSD1306_I2C_ADDRESS 0x3C
#define STRAIN_GAUGE_PIN 20

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified();
Adafruit_MPU6050 mpu;
DHT dht(DHTPIN, DHTTYPE);

float temperature = 0, humidity = 0;
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue;
unsigned long lastSensorReadTime = 0;
unsigned long lastDHTReadTime = 0;
unsigned long lastDisplayUpdateTime = 0;
const unsigned long sensorReadInterval = 200;
const unsigned long dhtReadInterval = 2000;
const unsigned long displayUpdateInterval = 2000;

// Display 
int displayIndex = 0;

// Fuzzy Variable
String vibrationStatus = "Unknown";

//Kalman variabel
float kalmanAccelX = 0, kalmanAccelY = 0, kalmanAccelZ = 0;
float kalmanGain = 0.5; // Simpla Gain
float processNoise = 0.1, measurementNoise = 1.0; // Variabel noise
float estimatedError = 1.0; // Variabel estimasi error

//WIFI
const char* ssid = "DTEO-VOKASI";
const char* password = "TEO123456";
// Fungsi untuk menghubungkan ke WiFi
void connectToWiFi()
{
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (millis() - startTime > 2000) // Timeout 10 detik
    {
      Serial.println("WiFi gagal terhubung. Restarting...");
      ESP.restart();
    }
  }
  Serial.println("WiFi Terhubung!");
}

// Himpunan fuzzy untuk akselerasi
float membershipLow(float value) {
  return (value <= 1.0) ? 1.0 : (value >= 3.0 ? 0.0 : (3.0 - value) / 2.0);
}

float membershipMedium(float value) {
  return (value <= 2.0 || value >= 6.0) ? 0.0 : (value <= 4.0 ? (value - 2.0) / 2.0 : (6.0 - value) / 2.0);
}

float membershipHigh(float value) {
  return (value <= 4.0) ? 0.0 : (value >= 6.0 ? 1.0 : (value - 4.0) / 2.0);
}

// Fungsi parameter float inferensi fuzzy
float fuzzyInference(float accelValue) {

  float low = membershipLow(accelValue);
  float medium = membershipMedium(accelValue);
  float high = membershipHigh(accelValue);

  // fuzzy rule
  float noVibration = low;
  float slightVibration = medium;
  float strongVibration = high;

  // Defuzzifikasi menggunakan metode rata-rata berbobot
  return (noVibration * 0.0 + slightVibration * 50.0 + strongVibration * 100.0) /
         (noVibration + slightVibration + strongVibration);
}

// Decision-making getaran
String detectVibration(float accelValue) {
  float severity = fuzzyInference(accelValue);
  if (severity < 25.0) {
    return "No Vibration";
  } else if (severity < 75.0) {
    return "Slight Vibration";
  } else {
    return "Strong Vibration";
  }
}

float kalmanFilter(float measurement, float &estimatedValue, float &errorEstimate, float processNoise, float measurementNoise) {
  // Perhitungan gain Kalman
  kalmanGain = errorEstimate / (errorEstimate + measurementNoise);
  
  // Update estimasi nilai
  estimatedValue = estimatedValue + kalmanGain * (measurement - estimatedValue);

  // Update error estimasi
  errorEstimate = (1 - kalmanGain) * errorEstimate + fabs(estimatedValue) * processNoise;
  
  return estimatedValue;
}


// Fungsi membaca sensor DHT22
void readDHT()
{
  float temp = dht.readTemperature();
  float hum = dht.readHumidity();

  if (isnan(temp) || isnan(hum))
  {
    Serial.println("DHT Error: Menggunakan nilai terakhir.");
  }
  else
  {
    temperature = temp;
    humidity = hum;
  }
}

// Fungsi membaca data sensor MPU6050, ADXL345, dan strain gauge
void readSensors()
{
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  // Library Read param m/s2
  // accelX = a.acceleration.x;
  // accelY = a.acceleration.y;
  // accelZ = a.acceleration.z;

  // Kalman Param Filter
  accelX = kalmanFilter(a.acceleration.x, kalmanAccelX, estimatedError, processNoise, measurementNoise);
  accelY = kalmanFilter(a.acceleration.y, kalmanAccelY, estimatedError, processNoise, measurementNoise);
  accelZ = kalmanFilter(a.acceleration.z, kalmanAccelZ, estimatedError, processNoise, measurementNoise);

  int rawValue = analogRead(STRAIN_GAUGE_PIN);
  strainValue = rawValue * (100.0 / 1023.0);
  
  vibrationStatus = detectVibration(abs(accelZ)); // Deteksi getaran berdasarkan accelZ
  vibrationStatus = detectVibration(abs(accelZ)); // Deteksi getaran berdasarkan accelZ

  Serial.println("Vibration Status: " + vibrationStatus); // Tampilkan status getaran di Serial
//Display.print(vibrationStatus)
}


// Update tampilan OLED
void updateDisplay()
{
  display.clearDisplay(); // Clear Displat update
  display.setTextSize(1); // Ukuran teks standar
  display.setTextColor(SSD1306_WHITE); // Warna teks putih
  display.setCursor(0, 0); // Mulai dari pojok kiri atas

  switch (displayIndex)
  {
  case 0: // Data Gyroscope
    display.printf("Gyro X: %.2f deg/s\nGyro Y: %.2f deg/s\nGyro Z: %.2f deg/s", gyroX, gyroY, gyroZ);
    break;

  case 1: // Data Accelerometer
    display.printf("Accel X: %.2f m/s^2\nAccel Y: %.2f m/s^2\nAccel Z: %.2f m/s^2", accelX, accelY, accelZ);
    break;

  case 2: // Data Strain Gauge, Temperatur, dan Kelembaban
    display.printf("Strain: %.2f N\nTemp: %.2f C\nHumidity: %.2f %%", strainValue, temperature, humidity);
    break;

  case 3: // Status Getaran
    display.printf("Vibration Status:\n%s", vibrationStatus.c_str());
    break;

  }

  display.display(); 
  displayIndex = (displayIndex + 1) % 4; // Ganti layar berikutnya
}

// Kirim data ke server
void kirimDataKeServer()
{
  HTTPClient http;
  char postData[256];
  snprintf(postData, sizeof(postData),
           "humidity=%.2f&temperature=%.2f&accelX=%.2f&accelY=%.2f&accelZ=%.2f&gyroX=%.2f&gyroY=%.2f&gyroZ=%.2f&strainValue=%.2f",
           humidity, temperature, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, strainValue);

  http.begin("http://192.168.54.36/shmsv2_2/sensor.php");
  http.addHeader("Content-Type", "application/x-www-form-urlencoded");

  int httpCode = http.POST(postData);
  if (httpCode > 0)
  {
    Serial.printf("HTTP Response code: %d\n", httpCode);
    String payload = http.getString();
    Serial.println(payload);
  }
  else
  {
    Serial.printf("HTTP request gagal: %s\n", http.errorToString(httpCode).c_str());
  }
  http.end();
}

// Setup program
void setup()
{
  Serial.begin(115200);
  Wire.begin(SDA, SCL);

  if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS))
  {
    Serial.println("SSD1306 Gagal");
    while (true)
      ;
  }

  if (!mpu.begin())
    Serial.println("MPU6050 Tidak Terhubung");

  if (!adxl.begin())
    Serial.println("ADXL345 Tidak Terhubung");

  dht.begin();
  connectToWiFi();

  // Tampilkan pesan awal
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.println("Connected !!");
  display.print("SSID: ");
  display.println(ssid);
  display.println("Inisialisasi Selesai");
  display.display();
  delay(2000);
}

// Loop utama
void loop()
{
  unsigned long currentMillis = millis();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("Reconnect WiFi ...");
    connectToWiFi();
  }

  if (currentMillis - lastDHTReadTime >= dhtReadInterval)
  {
    lastDHTReadTime = currentMillis;
    readDHT();
  }

  if (currentMillis - lastSensorReadTime >= sensorReadInterval)
  {
    lastSensorReadTime = currentMillis;
    readSensors();
    kirimDataKeServer();
  }

  if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval)
  {
    lastDisplayUpdateTime = currentMillis;
    updateDisplay();
  }
}

// #include <Arduino.h>
// #include <Adafruit_ADXL345_U.h>
// #include <Adafruit_MPU6050.h>
// #include <DHT.h>
// #include <Wire.h>
// #include <WiFi.h>
// #include <HTTPClient.h>
// #include <Adafruit_SSD1306.h>

// #define DHTPIN 1
// #define DHTTYPE DHT22
// #define MPU6050_ADDRESS 0x68
// #define ADXL345_ADDRESS 0x53
// #define SDA 8
// #define SCL 9
// #define OLED_RESET 7
// #define SCREEN_WIDTH 128
// #define SCREEN_HEIGHT 32
// #define SSD1306_I2C_ADDRESS 0x3C
// #define STRAIN_GAUGE_PIN 20

// Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
// Adafruit_ADXL345_Unified adxl = Adafruit_ADXL345_Unified();
// Adafruit_MPU6050 mpu;
// DHT dht(DHTPIN, DHTTYPE);

// String vibrationStatus = "Unknown";

// float temperature = 0, humidity = 0;
// float gyroX, gyroY, gyroZ, accelX, accelY, accelZ, strainValue;
// unsigned long lastSensorReadTime = 0;
// unsigned long lastDHTReadTime = 0;
// unsigned long lastDisplayUpdateTime = 0;
// const unsigned long sensorReadInterval = 200;
// const unsigned long dhtReadInterval = 2000;
// const unsigned long displayUpdateInterval = 2000;
// int displayIndex = 0;

// //WIFI
// const char* ssid = "DTEO-VOKASI";
// const char* password = "TEO123456";
// // Fungsi untuk menghubungkan ke WiFi
// void connectToWiFi()
// {
//   WiFi.begin(ssid, password);
//   unsigned long startTime = millis();

//   while (WiFi.status() != WL_CONNECTED)
//   {
//     delay(500);
//     if (millis() - startTime > 2000) // Timeout 10 detik
//     {
//       Serial.println("WiFi gagal terhubung. Restarting...");
//       ESP.restart();
//     }
//   }
//   Serial.println("WiFi Terhubung!");
// }

// // Himpunan fuzzy untuk akselerasi
// float membershipLow(float value) {
//   return (value <= 1.0) ? 1.0 : (value >= 3.0 ? 0.0 : (3.0 - value) / 2.0);
// }

// float membershipMedium(float value) {
//   return (value <= 2.0 || value >= 6.0) ? 0.0 : (value <= 4.0 ? (value - 2.0) / 2.0 : (6.0 - value) / 2.0);
// }

// float membershipHigh(float value) {
//   return (value <= 4.0) ? 0.0 : (value >= 6.0 ? 1.0 : (value - 4.0) / 2.0);
// }

// // inferensi fuzzy
// float fuzzyInference(float accelValue) {

//   float low = membershipLow(accelValue);
//   float medium = membershipMedium(accelValue);
//   float high = membershipHigh(accelValue);

//   // fuzzy rule
//   float noVibration = low;
//   float slightVibration = medium;
//   float strongVibration = high;

//   // Defuzzifikasi menggunakan metode rata-rata berbobot
//   return (noVibration * 0.0 + slightVibration * 50.0 + strongVibration * 100.0) /
//          (noVibration + slightVibration + strongVibration);
// }

// // Decision-making getaran
// String detectVibration(float accelValue) {
//   float severity = fuzzyInference(accelValue);
//   if (severity < 25.0) {
//     return "No Vibration";
//   } else if (severity < 75.0) {
//     return "Slight Vibration";
//   } else {
//     return "Strong Vibration";
//   }
// }

// // Fungsi membaca sensor DHT22
// void readDHT()
// {
//   float temp = dht.readTemperature();
//   float hum = dht.readHumidity();

//   if (isnan(temp) || isnan(hum))
//   {
//     Serial.println("DHT Error: Menggunakan nilai terakhir.");
//   }
//   else
//   {
//     temperature = temp;
//     humidity = hum;
//   }
// }

// // Fungsi membaca data sensor MPU6050, ADXL345, dan strain gauge
// void readSensors()
// {
//   sensors_event_t a, g, temp;
//   mpu.getEvent(&a, &g, &temp);

//   gyroX = g.gyro.x;
//   gyroY = g.gyro.y;
//   gyroZ = g.gyro.z;
//   accelX = a.acceleration.x;
//   accelY = a.acceleration.y;
//   accelZ = a.acceleration.z;

//   int rawValue = analogRead(STRAIN_GAUGE_PIN);
//   strainValue = rawValue * (100.0 / 1023.0);
  
//   vibrationStatus = detectVibration(abs(accelZ)); // Deteksi getaran berdasarkan accelZ
//   vibrationStatus = detectVibration(abs(accelZ)); // Deteksi getaran berdasarkan accelZ

//   Serial.println("Vibration Status: " + vibrationStatus); // Tampilkan status getaran di Serial
// //Display.print(vibrationStatus)
// }



// // Update tampilan OLED
// void updateDisplay()
// {
//   display.clearDisplay(); // Clear Displat update
//   display.setTextSize(1); // Ukuran teks standar
//   display.setTextColor(SSD1306_WHITE); // Warna teks putih
//   display.setCursor(0, 0); // Mulai dari pojok kiri atas

//   switch (displayIndex)
//   {
//   case 0: // Data Gyroscope
//     display.printf("Gyro X: %.2f deg/s\nGyro Y: %.2f deg/s\nGyro Z: %.2f deg/s", gyroX, gyroY, gyroZ);
//     break;

//   case 1: // Data Accelerometer
//     display.printf("Accel X: %.2f m/s^2\nAccel Y: %.2f m/s^2\nAccel Z: %.2f m/s^2", accelX, accelY, accelZ);
//     break;

//   case 2: // Data Strain Gauge, Temperatur, dan Kelembaban
//     display.printf("Strain: %.2f N\nTemp: %.2f C\nHumidity: %.2f %%", strainValue, temperature, humidity);
//     break;

//   case 3: // Status Getaran
//     display.printf("Vibration Status:\n%s", vibrationStatus.c_str());
//     break;

//   }

//   display.display(); 
//   displayIndex = (displayIndex + 1) % 4; // Ganti layar berikutnya
// }

// // Kirim data ke server
// void kirimDataKeServer()
// {
//   HTTPClient http;
//   char postData[256];
//   snprintf(postData, sizeof(postData),
//            "humidity=%.2f&temperature=%.2f&accelX=%.2f&accelY=%.2f&accelZ=%.2f&gyroX=%.2f&gyroY=%.2f&gyroZ=%.2f&strainValue=%.2f",
//            humidity, temperature, accelX, accelY, accelZ, gyroX, gyroY, gyroZ, strainValue);

//   http.begin("http://192.168.54.36/shmsv2_2/sensor.php");
//   http.addHeader("Content-Type", "application/x-www-form-urlencoded");

//   int httpCode = http.POST(postData);
//   if (httpCode > 0)
//   {
//     Serial.printf("HTTP Response code: %d\n", httpCode);
//     String payload = http.getString();
//     Serial.println(payload);
//   }
//   else
//   {
//     Serial.printf("HTTP request gagal: %s\n", http.errorToString(httpCode).c_str());
//   }
//   http.end();
// }

// // Setup program
// void setup()
// {
//   Serial.begin(115200);
//   Wire.begin(SDA, SCL);

//   if (!display.begin(SSD1306_SWITCHCAPVCC, SSD1306_I2C_ADDRESS))
//   {
//     Serial.println("SSD1306 Gagal");
//     while (true)
//       ;
//   }

//   if (!mpu.begin())
//     Serial.println("MPU6050 Tidak Terhubung");

//   if (!adxl.begin())
//     Serial.println("ADXL345 Tidak Terhubung");

//   dht.begin();
//   connectToWiFi();

//   // Tampilkan pesan awal
//   display.clearDisplay();
//   display.setTextSize(1);
//   display.setCursor(0, 0);
//   display.println("Connected !!");
//   display.print("SSID: ");
//   display.println(ssid);
//   display.println("Inisialisasi Selesai");
//   display.display();
//   delay(2000);
// }

// // Loop utama
// void loop()
// {
//   unsigned long currentMillis = millis();

//   if (WiFi.status() != WL_CONNECTED)
//   {
//     Serial.println("Reconnect WiFi ...");
//     connectToWiFi();
//   }

//   if (currentMillis - lastDHTReadTime >= dhtReadInterval)
//   {
//     lastDHTReadTime = currentMillis;
//     readDHT();
//   }

//   if (currentMillis - lastSensorReadTime >= sensorReadInterval)
//   {
//     lastSensorReadTime = currentMillis;
//     readSensors();
//     kirimDataKeServer();
//   }

//   if (currentMillis - lastDisplayUpdateTime >= displayUpdateInterval)
//   {
//     lastDisplayUpdateTime = currentMillis;
//     updateDisplay();
//   }
// }