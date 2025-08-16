# Basic-Rocket-control-centre-
A basic rocket control centre made by using ESP32 and is also connected to blynk , note that ESP32 Doesn't have a long range so it'll be your wish to modify the code or just use a sd card to log data

## Hardware used
1x ESP32 DEV V1
1x MPU6050 3 Axis 
1x SSD1306 White 0.96" OLED
1x BMP180
1x DHT11
Other Miscellaneous 

## Code
   #define BLYNK_TEMPLATE_ID  "YOUR       TEMPLATE ID"
   #define BLYNK_TEMPLATE_NAME "YOUR      NAME "
   #define BLYNK_AUTH_TOKEN  "YOUR        AUTH"

   #define BLYNK_PRINT Serial

   // ===== Includes =====
   #include <WiFi.h>
   #include <WiFiClient.h>
   #include <BlynkSimpleEsp32.h>
   #include <Wire.h>
   #include <Adafruit_GFX.h>
   #include <Adafruit_SSD1306.h>
   #include <Adafruit_BMP085.h>   //      BMP180
   #include <DHT.h>
   #include <MPU6050.h>          //       Electronic Cats / i2cdevlib style

   // ===== Wi-Fi =====
   char ssid[] = "Your";
   char pass[] = "Password";

   // ===== Pins =====
   #define SDA_PIN 21
   #define SCL_PIN 22
   #define DHTPIN  4
   #define DHTTYPE DHT11

   // ===== Devices =====
   DHT dht(DHTPIN, DHTTYPE);
   Adafruit_BMP085 bmp;  // BMP180        uses this driver

   // OLED 128x64 I2C
   #define SCREEN_WIDTH 128
   #define SCREEN_HEIGHT 64
   #define OLED_ADDR 0x3C
   Adafruit_SSD1306                       display(SCREEN_WIDTH,                  SCREEN_HEIGHT, &Wire, -1);

// MPU6050 (I2C address 0x68 by default)
MPU6050 mpu;

// ===== Blynk Virtual Pins =====
#define VP_TEMP_DHT  V0
#define VP_HUM_DHT   V1
#define VP_PRESS     V2
#define VP_ALT       V3
#define VP_AX        V4
#define VP_AY        V5
#define VP_AZ        V6
#define VP_GX        V7
#define VP_GY        V8
#define VP_GZ        V9

BlynkTimer timer;

// Set full-scale ranges for flight (avoid clipping)
static const uint8_t GYRO_FS   = MPU6050_GYRO_FS_2000;  // ±2000 °/s → 16.4 LSB/(°/s)
static const uint8_t ACCEL_FS  = MPU6050_ACCEL_FS_16;   // ±16 g    → 2048 LSB/g
static const float   GYRO_SENS = 16.4f;                 // for 2000 dps
static const float   ACC_SENS  = 2048.0f;               // for ±16 g

void sendSensorData() {
  // --- DHT11 ---
  float t_dht = dht.readTemperature();
  float h_dht = dht.readHumidity();

  // --- BMP180 ---
  float p_hPa = NAN, alt_m = NAN;
  int32_t p_Pa = bmp.readPressure();           // Pa
  if (p_Pa > 0) {
    p_hPa = p_Pa / 100.0f;                     // hPa
    alt_m = bmp.readAltitude(101325);          // sea-level Pa
  }

  // --- MPU6050 raw -> scaled ---
  int16_t ax, ay, az, gx, gy, gz;
  mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

  float ax_g = ax / ACC_SENS;
  float ay_g = ay / ACC_SENS;
  float az_g = az / ACC_SENS;
  float gx_dps = gx / GYRO_SENS;
  float gy_dps = gy / GYRO_SENS;
  float gz_dps = gz / GYRO_SENS;

  // --- Blynk ---
  if (!isnan(t_dht)) Blynk.virtualWrite(VP_TEMP_DHT, t_dht);
  if (!isnan(h_dht)) Blynk.virtualWrite(VP_HUM_DHT,  h_dht);
  if (!isnan(p_hPa)) Blynk.virtualWrite(VP_PRESS,    p_hPa);
  if (!isnan(alt_m)) Blynk.virtualWrite(VP_ALT,      alt_m);

  Blynk.virtualWrite(VP_AX, ax_g);
  Blynk.virtualWrite(VP_AY, ay_g);
  Blynk.virtualWrite(VP_AZ, az_g);
  Blynk.virtualWrite(VP_GX, gx_dps);
  Blynk.virtualWrite(VP_GY, gy_dps);
  Blynk.virtualWrite(VP_GZ, gz_dps);

  // --- OLED ---
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  display.setCursor(0, 0);
  display.print("T:"); display.print(t_dht, 1); display.print("C  ");
  display.print("H:"); display.print(h_dht, 0); display.println("%");

  display.print("P:"); display.print(p_hPa, 1); display.print("hPa  ");
  display.print("Alt:"); display.print(alt_m, 1); display.println("m");

  display.print("AX:"); display.print(ax_g, 2); display.print(" ");
  display.print("AY:"); display.print(ay_g, 2); display.print(" ");
  display.print("AZ:"); display.print(az_g, 2); display.println();

  display.print("GX:"); display.print(gx_dps, 0); display.print(" ");
  display.print("GY:"); display.print(gy_dps, 0); display.print(" ");
  display.print("GZ:"); display.print(gz_dps, 0);

  display.display();
}

void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);

  // Start OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("SSD1306 allocation failed");
  } else {
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println("Booting...");
    display.display();
  }

  // Sensors
  dht.begin();
  if (!bmp.begin()) {
    Serial.println("BMP180 not found (check wiring).");
  }

  mpu.initialize();
  if (!mpu.testConnection()) {
    Serial.println("MPU6050 connection failed!");
  }
  // Configure ranges to match our scaling constants
  mpu.setFullScaleGyroRange(GYRO_FS);
  mpu.setFullScaleAccelRange(ACCEL_FS);

  // Wi-Fi + Blynk
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);

  // Update every 500 ms
  timer.setInterval(500L, sendSensorData);
}

void loop() {
  Blynk.run();
  timer.run();
}

