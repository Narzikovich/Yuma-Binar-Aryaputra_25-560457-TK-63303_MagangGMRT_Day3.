/*
Nama: Yuma Binar Aryaputra
NIM: 25/560457/TK/63303

Program ini mensimulasikan sistem dengan 5 servo motor yang dikendalikan oleh ESP32, 
menggunakan sensor MPU6050 untuk mendeteksi gerakan roll, pitch, dan yaw serta PIR
sensor untuk mendeteksi gerakan eksternal.
- Servo 1 dan 2 berputar melawan arah roll (positif roll -> servo ke arah negatif).
- Servo 3 dan 4 berputar searah dengan pitch (positif pitch -> servo ke arah positf).
- Servo 5 berputar mengikuti arah yaw. 
- Jika PIR mendeteksi gerakan, semua servo ke posisi 45 derajat, lalu kembali ke initial.

Karena Wokwi tidak mendukung servo fisik penuh, program ini menggunakan output serial untuk simulasi virtual
(Posisi servo dicetak ke Serial Monitor sebagai pengganti indikasi gerakan fisik)
Jika ingin menggunakan servo fisik, ganti Serial output dengan fungsi writeServo() dan attach pin.

Komponen:
- ESP32
- MPU6050
- PIR Sensor
- 5 buah servo

Cara Kerja:
1. Setup: Inisialisasi Serial, I2C, MPU6050, dan PIR
2. Loop: Baca data MPU6050, hitung roll, pitch dan yaw, baca PIR, lalu cetak posisi sesuai dengan ketentuan
3. Output: Serial Monitor menampilkan nilai sensor dan posisi servo
*/

#include <Wire.h>           // Library untuk komunikasi I2C dengan MPU6050
#include <MPU6050.h>        // Library untuk sensor MPU6050 (akses data akselerometer dan giroskop)

MPU6050 mpu;                 // Objek MPU6050 untuk mengakses sensor
#define PIR_PIN 4            // Pin GPIO ESP32 untuk PIR sensor (OUT)

float roll, pitch, yaw;      // Variabel untuk menyimpan nilai rotasi (derajat)
bool pirDetected = false;    // Status deteksi PIR (true jika ada gerakan)

void setup() {
  Serial.begin(115200)       // Mulai Serial untuk output (baud rate 115200)
  Wire.begin();              // Inisialisasi I2C
  mpu.initialized();         // Inisialisasi MPU6050 (cek koneksi dan setup dasar)
  pinMode(PIR_PIN, INPUT);   // Set pin PIR sebagai input digital
  Serial.printIn("Sistem siap.");
}

void loop() {
  // Baca data mentah dari MPU6050 (akselerometer dan giroskop)
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  // Hitung roll, pitch, yaw dari data sensor menggunakan trigonometri
  roll = atan2(ay, az) * 180 / PI;
  pitch = atan2(-ax, sqrt(ay*ay + az*az)) * 180 / PI;
  yaw = atan2(ax, ay) * 180 / PI;

  // Baca status pir
  pirDetected = digitalRead(PIR_PIN);

  if(pirDetected) {
    // Jika PIR mendeteksi gerakan: Simulasi semua servo ke 45 derajat, lalu kembali ke 90
    Serial.print("PIR: Gerakan terdeteksi - Semua servo ke 45 derajat, lalu kembali ke 90");
    delay(1000);
  } else {
    // Jika tidak ada gerakan: Hitung posisi servo berdasarkan roll, pitch, yaw
    int servo1_2_pos = map(roll, -90, 90, 180, 0);
    int servo3_4_pos = map(pitch, -90, 90, 0, 180);
    int servo5_pos = map(yaw, -90, 90, 0, 180);

    // Cetak output ke monitor
    Serial.print("Roll: "); Serial.print(roll); Serial.print("-> Servo1-2: "); Serial.print(servo1_2_pos);
    Serial.print("Pitch: "); Serial.print(pitch); Serial.print("-> Servo3-4: "); Serial.print(servo3_4_pos);
    Serial.print("Yaw: "); Serial.print(yaw); Serial.print("-> Servo5: "); Serial.print(servo5_pos);
  }
  delay(100);
}
