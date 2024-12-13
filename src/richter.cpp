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
// Adafruit_ADXL345_Unified accelADXL = Adafruit_ADXL345_Unified(12345);

float temperature = 0, humidity = 0;
float strainValue;
unsigned long lastSensorReadTime = 0;
unsigned long lastDHTReadTime = 0;
unsigned long lastDisplayUpdateTime = 0;

// Interval
#define sensorReadInterval 200
#define dhtReadInterval 2000
#define displayUpdateInterval  2000

//Convertion
// Variabel untuk menyimpan hasil kalkulasi
float richterMagnitude = 0.0;
float tiltAngle = 0.0;
float angleInDegrees = 0.0;
const float distanceToEpicenter = 500.0; // Jarak ke episenter (pusat)

//WIFI
const char* ssid = "DTEO-VOKASI";
const char* password = "TEO123456";

// Display 
int displayIndex = 0;

// Fuzzy Variable
String vibrationStatus = "Unknown";

//Kalman variabel
float gyroX, gyroY, gyroZ, accelX, accelY, accelZ;
float ADaccelX, ADaccelY, ADaccelZ;
float dt = 0.01; // read 10ms Kalman 

typedef struct {
    float Q_angle;    // Varians proses untuk sudut
    float Q_bias;     // Varians proses untuk bias
    float R_measure;  // Varians pengukuran
    float angle;      // Sudut yang dikalkulasi
    float bias;       // Bias kalkulasi
    float rate;       // Laju rotasi
    float P[2][2];    // Matriks error estimasi
} KalmanFilter;

// Inisialisasi Kalman Filter
KalmanFilter kalmanX = {0.001f, 0.003f, 0.03f, 0, 0, 0, {{0, 0}, {0, 0}}};
KalmanFilter kalmanY = {0.001f, 0.003f, 0.03f, 0, 0, 0, {{0, 0}, {0, 0}}};

// Get Angle KF
float getKalmanAngle(KalmanFilter *kalman, float newAngle, float newRate, float dt) {
    // Prediksi
    kalman->rate = newRate - kalman->bias;
    kalman->angle += dt * kalman->rate;

    kalman->P[0][0] += dt * (dt * kalman->P[1][1] - kalman->P[0][1] - kalman->P[1][0] + kalman->Q_angle);
    kalman->P[0][1] -= dt * kalman->P[1][1];
    kalman->P[1][0] -= dt * kalman->P[1][1];
    kalman->P[1][1] += kalman->Q_bias * dt;

    // Update
    float S = kalman->P[0][0] + kalman->R_measure;
    float K[2] = {kalman->P[0][0] / S, kalman->P[1][0] / S};

    float y = newAngle - kalman->angle;
    kalman->angle += K[0] * y;
    kalman->bias += K[1] * y;

    float P00_temp = kalman->P[0][0];
    float P01_temp = kalman->P[0][1];

    kalman->P[0][0] -= K[0] * P00_temp;
    kalman->P[0][1] -= K[0] * P01_temp;
    kalman->P[1][0] -= K[1] * P00_temp;
    kalman->P[1][1] -= K[1] * P01_temp;

    angleInDegrees = kalman->angle * 180.0 / PI;
    return angleInDegrees;
}

// Fungsi untuk menghubungkan ke WiFi
void connectToWiFi()
{
  WiFi.begin(ssid, password);
  unsigned long startTime = millis();

  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
    if (millis() - startTime > 8000) // Timeout 8 detik
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
    return "No";
  } else if (severity < 75.0) {
    return "Slight";
  } else {
    return "Strong";
  }
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
  
  //  gyro  MPU6050
  gyroX = g.gyro.x;
  gyroY = g.gyro.y;
  gyroZ = g.gyro.z;

  // MPU accelerometer
  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  // Baca akselerometer dari ADXL345
  sensors_event_t event;
  adxl.getEvent(&event);

  // ADXL accelerometer
  ADaccelX = event.acceleration.x;
  ADaccelY = event.acceleration.y;
  ADaccelZ = event.acceleration.z;

 // Hitung sudut dari akselerometer
  float accelAngleX = atan2(accelY, accelZ) * 180 / PI;
  float accelAngleY = atan2(-accelX, sqrt(accelY * accelY + accelZ * accelZ)) * 180 / PI;

 // Terapkan Kalman Filter
  float kalmanAngleX = getKalmanAngle(&kalmanX, accelAngleX, gyroX, dt);
  float kalmanAngleY = getKalmanAngle(&kalmanY, accelAngleY, gyroY, dt);

 // Debug
  // Serial.print("Kalman Angle X: "); Serial.println(kalmanAngleX);
  // Serial.print("Kalman Angle Y: "); Serial.println(kalmanAngleY);

  int rawValue = analogRead(STRAIN_GAUGE_PIN);
  strainValue = rawValue * (100.0 / 4095.0);

  vibrationStatus = detectVibration(abs(ADaccelZ)); // Deteksi getaran berdasarkan accelZ
  // Serial.println("Vibration Status: " + vibrationStatus);  // Debugging
  // Serial.println("Angle Kalman : ", angleInDegrees);
  richterMagnitude = calculateRichterMagnitude(ADaccelX, ADaccelY, ADaccelZ, distanceToEpicenter);

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
    display.printf("Richter Mag: %.2f\nTilt: %.2f deg\nVibration: %s", 
                   richterMagnitude, angleInDegrees, vibrationStatus.c_str());
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
           "humidity=%.2f&temperature=%.2f&gyroX=%.2f&gyroY=%.2f&gyroZ=%.2f&accelX=%.2f&accelY=%.2f&accelZ=%.2f&strainValue=%.2f",
           humidity, temperature, ADaccelX, ADaccelY, ADaccelZ, gyroX, gyroY, gyroZ, strainValue); 

  //http.begin("http://192.168.54.36/shmsv2_2/sensor.php"); -->
  http.begin("http://10.17.39.83/shmsv2_2/sensor.php"); //modul 2
//   // http.begin("http://10.17.39.83/shmsv2_2/sensor.php"); // modul 1
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

// Function Magnitudo Richter (estimation < 500 Meter Center)
float calculateRichterMagnitude(float accelX, float accelY, float accelZ, float distance) {
  float totalAcceleration = sqrt(pow(accelX, 2) + pow(accelY, 2) + pow(accelZ, 2));
  float groundMotion = totalAcceleration * 980.0; // Konversi dari m/s² ke gal
  return log10(groundMotion) + 3.0 * log10(distance / 100.0) - 2.92;
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