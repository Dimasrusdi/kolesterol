#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "SimpleKalmanFilter.h" // Tambahkan pustaka SimpleKalmanFilter
#include <LiquidCrystal_I2C.h> // Tambahkan pustaka LiquidCrystal_I2C
#include <ESP8266WiFi.h> // Pustaka WiFi untuk ESP8266
#include <FirebaseESP8266.h> // Pustaka Firebase untuk ESP8266

#define WIFI_SSID "POCO X3 NFC"
#define WIFI_PASSWORD "dimasrusdi"

#define FIREBASE_HOST "monitoring-kolesterol-default-rtdb.asia-southeast1.firebasedatabase.app"
#define FIREBASE_AUTH "o7aouX4Q0my67S5CVZHD4KhFYeWxNKyNoW8IJHB7"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;
void printResult(FirebaseData &data);

MAX30105 particleSensor;

const byte RATE_SIZE = 4; //Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; //Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; //Time at which the last beat occurred

float beatsPerMinute;
int beatAvg;

// Inisialisasi objek Kalman Filter
SimpleKalmanFilter kalmanFilterIR(2, 2, 0.01); // Q, R, F (Q and R are the process and measurement noise covariance)
SimpleKalmanFilter kalmanFilterRed(2, 2, 0.01); // Tambahkan filter untuk nilai Red jika diperlukan

// Inisialisasi objek LCD dengan alamat I2C 0x27 (sesuaikan dengan alamat I2C LCD Anda)
LiquidCrystal_I2C lcd(0x27, 20, 4);

void setup() {
  Serial.begin(115200);
  Serial.println("Initializing...");

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) {
    Serial.println("MAX30102 was not found. Please check wiring/power.");
    while (1);
  }
  Serial.println("Place your index finger on the sensor with steady pressure.");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green LED

  // Inisialisasi LCD
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("Initializing...");

  // Inisialisasi WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println("Connected to Wi-Fi");

  // Inisialisasi Firebase
  config.host = FIREBASE_HOST;
  config.signer.tokens.legacy_token = FIREBASE_AUTH;

  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
}

void loop() {
  long irValue = particleSensor.getIR();
  int redValue = particleSensor.getRed();
  
  // Terapkan filter Kalman pada nilai IR dan Red
  int filteredIRValue = kalmanFilterIR.updateEstimate(irValue);
  int filteredRedValue = kalmanFilterRed.updateEstimate(redValue);
  
  // Hitung nilai kolesterol berdasarkan nilai IR yang telah difilter
  int Kolesterol = (filteredIRValue * 0.0052) - 392.63;

  // Tampilkan nilai pada LCD
  if (filteredIRValue < 50000) {
    Serial.println("No finger?");
    lcd.clear();
    lcd.setCursor(2, 0);
    lcd.print("TIDAK TERDETEKSI");
    delay(5000);
  } else {
    Serial.print("R=");
    Serial.print(filteredRedValue);
    Serial.print(", IR=");
    Serial.print(filteredIRValue);
    Serial.print(", Kolesterol=");
    Serial.print(Kolesterol);
    Serial.println();

    // Mengirim data kolesterol ke Firebase
    if (Firebase.ready()) {
      String path = "/kolesterol";
      Serial.print("Mengirim data ke path: ");
      Serial.println(path);
      if (Firebase.setInt(fbdo, path.c_str(), Kolesterol)) {
        Serial.println("Data kolesterol berhasil dikirim");
      } else {
        Serial.println("Gagal mengirim data kolesterol");
        Serial.println(fbdo.errorReason());
      }
    }

    if (Kolesterol < 200) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("IR  : ");
      lcd.print(filteredIRValue);
      lcd.setCursor(0, 1);
      lcd.print("Chol: ");
      lcd.print(Kolesterol);
      lcd.print(" mg/dL");
      lcd.setCursor(6, 2);
      lcd.print("Normal");
    } else if (Kolesterol > 200 && Kolesterol < 240) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("IR  : ");
      lcd.print(filteredIRValue);
      lcd.setCursor(0, 1);
      lcd.print("Chol: ");
      lcd.print(Kolesterol);
      lcd.print(" mg/dL");
      lcd.setCursor(6, 2);
      lcd.print("Tinggi");
    } else if (Kolesterol > 240) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print("IR  : ");
      lcd.print(filteredIRValue);
      lcd.setCursor(0, 1);
      lcd.print("Chol: ");
      lcd.print(Kolesterol);
      lcd.print(" mg/dL");
      lcd.setCursor(6, 2);
      lcd.print("Sangat Tinggi");
    }
    delay(5000);
  }
}
