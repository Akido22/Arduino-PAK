#define BLYNK_DEBUG
#define BLYNK_PRINT Serial    // Comment this out to disable prints and save space
#include <ESP8266WiFi.h>
#include <BlynkSimpleEsp8266.h>
#include <SimpleTimer.h>

#include <Wire.h>
#include <Adafruit_BMP085.h>

#include <BH1750.h>  // https://github.com/claws/BH1750
BH1750 lightMeter;

char auth[] = "75a3e91cbdf98b35eea63e9c1";  // Put your Auth Token here. (see Step 3 above)

SimpleTimer timer;

#define I2C_SCL 12      // Barometric Pressure Sensor (BMP085)
#define I2C_SDA 13
Adafruit_BMP085 bmp;

float dst, bt, bp, ba;
char dstmp[20], btmp[20], bprs[20], balt[20];
bool bmp085_present = true;

void setup()
{
  Serial.begin(9600); // See the connection status in Serial Monitor
  Blynk.begin(auth, "ASUS", "Jersin72"); //insert here your SSID and password
 Wire.begin(I2C_SDA, I2C_SCL);
  delay(10);
if (!bmp.begin()) {
  Serial.println("Could not find a valid BMP085 sensor, check wiring!");
  //while (1) {}
  }
  
  timer.setInterval(20000L, sendUptime);
  lightMeter.begin();
}

void sendUptime()
{
  uint16_t lux = lightMeter.readLightLevel();
  Blynk.virtualWrite(4, lux);

  float bp =  bmp.readPressure()/1;
  Blynk.virtualWrite(9, bp); // virtual pin

  float ba =  bmp.readAltitude();
  Blynk.virtualWrite(7, ba); // virtual pin

  float bt =  bmp.readTemperature();
  Blynk.virtualWrite(12, bt); // virtual pin

  float dst =  bmp.readSealevelPressure(520)/100;
  Blynk.virtualWrite(13, dst); // virtual pin

}

void loop()
{
  Blynk.run();
  timer.run();
}
