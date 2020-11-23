// Версия от 13/11/2020 до AMS v.1.2.02(f) for Tester version
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <TroykaMQ.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11  
#define SKETCH_NAME "Automatic Meteo Station"
#define SKETCH_VERSION "v.1.2.02(f) for Tester version"
#define RO_CLEAN_AIR_FACTOR (9.83)
#define PIN_MQ2         A2
#define PIN_MQ2_HEATER  13


int onLED = 13;                     // LED индикатор питания(маяк ON/OFF)
int bluerainLED = 6;                // LED индикатор дождя YES
int yellowLED = 7;                  // LED индикатор освещенности
int redLED = 10;                    // LED индикатор MQ2
int greenLED = 12;                  // LED индикатор MQ2
int buzzer = 8;                     // PIN динамика Вuzzer
int rainPinA0 = A0;                 // PIN датчика YL-83
int smokeA1 = A1;                   // PIN датчика MQ2
int pinPhotoA2 = A2;                // PIN датчика LM393
int thresholdValue = 500;           // Пороговое значение для датчика YL-83
int sensorThres = 100;              // Пороговое значение для датчика MQ-2
int raw = 0;
MQ2 mq2(PIN_MQ2, PIN_MQ2_HEATER);   // инициализация сенсора MQ2
DHT dht(DHTPIN, DHTTYPE);           // инициализация сенсора DHT11
Adafruit_BMP280 bme;                // инициализация сенсора BMP280
LiquidCrystal_I2C lcd(0x27,16,2);   // инициализация дисплея LCD1602
 
byte symb_grad[8] =                 // кодирование символа градуса
{
  B00111,
  B00101,
  B00111,
  B00000,
  B00000,
  B00000,
  B00000,
};
 
void setup()
{
  pinMode(onLED, OUTPUT);
  pinMode(bluerainLED, OUTPUT);
  pinMode(yellowLED, OUTPUT);
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(rainPinA0, INPUT);
  pinMode(smokeA1, INPUT);
  pinMode(pinPhotoA2, INPUT);
  mq2.heaterPwrHigh();            // запуск датчика MQ-2 нагревателя
  lcd.begin();                    // инициализация lcd
  lcd.createChar(1, symb_grad);   // регистрируем собственный символ с кодом 1
  dht.begin();                    // запуск датчика DHT11
  bme.begin(0x76);                // запуск датчика BMP280 - 0x76 адрес                   
  Serial.begin(9600);             // запуск передачи данных
}

//Включение светодиода
void leds() {
  digitalWrite(onLED, HIGH); 
  delay(800);                     
  digitalWrite(onLED, LOW);  
  delay(800);                     
} 

void sens_r(int rain){
  if(rain < thresholdValue){
    Serial.print("Pin A0: ");Serial.println(rain);
    Serial.println(" * В данный момент: Идет дождь");
    Serial.println(" * Статус индикатора: Включен");
    int RL = 1;  
    digitalWrite(bluerainLED, HIGH);
  }
  else {
    Serial.print("Pin A0: ");Serial.println(rain);
    Serial.println(" * В данный момент: Дождя нет");
    Serial.println(" * Статус индикатора: Выключен");  
    digitalWrite(bluerainLED, LOW);
    int RL = 0;
  }
  delay(100);  
}

void sens_l(){
  raw = analogRead(pinPhotoA2);
  float u = raw * 0.48 / 100;
  if( raw < 600){
    Serial.print("Pin A2: ");Serial.println(raw);
    Serial.print("Напряжение: ");Serial.println(u);
    Serial.println(" * Статус индикатора: Выключен");
    digitalWrite(yellowLED, LOW );
    int LL = 1;
  }
  else {
    Serial.print("Pin A2: ");Serial.println(raw);
    Serial.print("Напряжение: ");Serial.println(u);
    Serial.println(" * Статус индикатора: Включен");
    digitalWrite(yellowLED, HIGH );
    int LL = 0; 
  }
  delay(100);  
}

void sens_s(){
  // если прошёл интервал нагрева датчика
  // и калибровка не была совершена
  if (!mq2.isCalibrated() && mq2.heatingCompleted()) {
    // выполняем калибровку датчика на чистом воздухе
    mq2.calibrate();
    // выводим сопротивление датчика в чистом воздухе (Ro) в serial-порт
    Serial.print("Ro = ");
    Serial.println(mq2.getRo());
  }
  // если прошёл интервал нагрева датчика
  // и калибровка была совершена
  if (mq2.isCalibrated() && mq2.heatingCompleted()) {
    // выводим отношения текущего сопротивление датчика
    // к сопротивлению датчика в чистом воздухе (Rs/Ro)
    Serial.print("Ratio: ");
    Serial.print(mq2.readRatio());
    // выводим значения газов в ppm
    Serial.print("LPG: ");
    Serial.print(mq2.readLPG());
    Serial.print(" ppm ");
    Serial.print(" Methane: ");
    Serial.print(mq2.readMethane());
    Serial.print(" ppm ");
    Serial.print(" Smoke: ");
    Serial.print(mq2.readSmoke());
    Serial.print(" ppm ");
    Serial.print(" Hydrogen: ");
    Serial.print(mq2.readHydrogen());
    Serial.println(" ppm ");
    delay(100);
  }
}

void sens_m(float h1, float t1, float bp1, float amp1){
  Serial.println("\n\r AMS v.1.2.02(f) for Tester version");
  Serial.println("=========================================");
  Serial.println ("Параметры DHT11:");
  Serial.print ("Влажность : "); Serial.print (h1); Serial.println ("%\t");
  Serial.print ("Температура : ");  Serial.print (t1);  Serial.println (" *C");
  Serial.println ("Параметры BMP280:");
  Serial.print ("Давление : ");  Serial.print(bp1);  Serial.println (" мм.Р.ст.");
  Serial.print ("Высота : ");  Serial.print (amp1);  Serial.println (" м.");
  Serial.println ("Параметры YL-83:");
  sens_r(analogRead(rainPinA0));
  Serial.println ("Параметры MQ-2:");
  sens_s();
  Serial.println ("Параметры LM393:");
  sens_l();
  
}

// Выводим показания влажности и температуры, давления и высоты
 void lcdd(float h2, float t2, float bp2, float amp2){
  lcd.clear();                // очистка экрана
  lcd.setCursor(0, 0);        //  установка курсора в начало 1 строки
  lcd.print("H:    %");       
  lcd.setCursor(2, 0);        // установка курсора на 2 позицию
  lcd.print(dht.readHumidity(), 1);           
  lcd.setCursor(8, 0);
  lcd.print("P:     ");
  lcd.setCursor(10, 0);
  lcd.print(bme.readPressure()/133.3224, 1);
  lcd.setCursor(0, 1);         // установка курсора в начало 2 строки
  lcd.print("T:     C");       
  lcd.setCursor(6, 1);         // установка курсора на 6 позицию 
  lcd.print("\1");             
  lcd.setCursor(2, 1);         // установка курсора на 2 позицию
  lcd.print(dht.readTemperature(),1);             
 
} 

// Начало программы 
void loop(){
  delay(10000);
  float h = dht.readHumidity();         // считывание влажности DHT11
  float t = dht.readTemperature();      // считывание температуры DHT11
  float bp = bme.readPressure()/133.3224;        // считывание давление ВМР280
  float amp = bme.readAltitude(1013.25);
  sens_m(h,t,bp,amp);
  leds();
  //int sensorRain = analogRead(rainPinA0);
  //int sensorSmoke = analogRead(smokeA1);
  //raw = analogRead(pinPhotoA2);
  
  
  
}
