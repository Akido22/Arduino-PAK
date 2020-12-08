// Версия от 21/11/2020 до AMS v.1.3.0(b1) for Tester version
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <TroykaMQ.h>   // библиотека для работы с датчиками серии MQ
#include <TroykaDHT.h>  // библиотека для работы с датчиками серии DHT
#include <TroykaLight.h> // библиотека для работы с датчиками освещенности LM393
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>

#define DHTTYPE DHT11  
#define SKETCH_NAME "Automatic Meteo Station"
#define SKETCH_VERSION "v.1.3.0(b1) for Tester version"
#define RO_CLEAN_AIR_FACTOR (9.83)
#define PIN_MQ2         A2
#define PIN_MQ2_HEATER  13          // MQ2 pезерв для нагревателя, в итоге работает постоянно


int onLED = 13;                     // LED индикатор питания(маяк ON/OFF)
int bluerainLED = 6;                // LED индикатор дождя YES
int yellowLED = 7;                  // LED индикатор освещенности
int redLED = 10;                    // LED индикатор MQ-2
int greenLED = 12;                  // LED индикатор MQ-2
int buzzer = 8;                     // PIN динамика Вuzzer
int rainPinA0 = A0;                 // PIN датчика YL-83
int smokeA1 = A1;                   // PIN датчика MQ-2
int pinPhotoA2 = A2;                // PIN датчика LM393
int thresholdValue = 500;           // Пороговое значение для датчика YL-83
int sensorThres = 100;              // Пороговое значение для датчика MQ-2
int raw = 0;
TroykaLight pinPhoto(A2);
MQ2 mq2(PIN_MQ2, PIN_MQ2_HEATER);   // инициализация сенсора MQ2
DHT dht(2, DHT11);                  // инициализация сенсора DHT11
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
    Serial.println("В данный момент: Идет дождь");
    Serial.println("Статус индикатора: Включен");
    int RL = 1;  
    digitalWrite(bluerainLED, HIGH);
  }
  else {
    Serial.print("Pin A0: ");Serial.println(rain);
    Serial.println("В данный момент: Дождя нет");
    Serial.println("Статус индикатора: Выключен");  
    digitalWrite(bluerainLED, LOW);
    int RL = 0;
  }
  delay(100);  
}

void sens_l(){
  pinPhoto.read();
  Serial.print("Освещенность ");
  Serial.print(pinPhoto.getLightLux());
  Serial.println(" Lx\t");
  Serial.print(pinPhoto.getLightFootCandles());
  Serial.println(" фут-свечи");
  int raw = analogRead(pinPhotoA2);
  float u = raw * 0.48 / 100;
  if( raw < 600){
    Serial.print("Pin A2: ");Serial.println(raw);
    Serial.print("Напряжение: ");Serial.println(u);
    Serial.println("Статус индикатора: Выключен");
    digitalWrite(yellowLED, LOW );
    int LL = 1;
  }
  else {
    Serial.print("Pin A2: ");Serial.println(raw);
    Serial.print("Напряжение: ");Serial.println(u);
    Serial.println("Статус индикатора: Включен");
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
    Serial.print("Показатель: ");
    Serial.print(mq2.readRatio());
    // выводим значения газов в ppm
    Serial.print("LPG: ");
    Serial.print(mq2.readLPG());
    Serial.print(" ppm ");
    Serial.print(" Meтан: ");
    Serial.print(mq2.readMethane());
    Serial.print(" ppm ");
    Serial.print(" Задымление: ");
    Serial.print(mq2.readSmoke());
    Serial.print(" ppm ");
    Serial.print(" Водоро́д: ");
    Serial.print(mq2.readHydrogen());
    Serial.println(" ppm ");
    delay(100);
  }
}

void sens_dh(){
  dht.read();  // считывание данных с датчика
  switch(dht.getState()) { // проверяем состояние данных
    
    case DHT_OK:
      // выводим показания влажности и температуры
      Serial.print("Температура = ");
      Serial.print(dht.getTemperatureC());
      Serial.println(" C \t");
      Serial.print("Температура = ");
      Serial.print(dht.getTemperatureK());
      Serial.println(" K \t");
      Serial.print("Температура = ");
      Serial.print(dht.getTemperatureF());
      Serial.println(" F \t");
      Serial.print("Влажность = ");
      Serial.print(dht.getHumidity());
      Serial.println(" %");
      break;
    // ошибка контрольной суммы
    case DHT_ERROR_CHECKSUM:
      Serial.println("E01: Checksum error");
      break;
    // превышение времени ожидания
    case DHT_ERROR_TIMEOUT:
      Serial.println("E04: Time out error");
      break;
    // данных нет, датчик не реагирует или отсутствует
    case DHT_ERROR_NO_REPLY:
      Serial.println("E06: Sensor not connected");
      break;
  }
}
void sens_m(float bp1, float amp1){
  Serial.println("\n\r AMS v.1.3.0(b1) for Tester version");
  Serial.println("==================================");
  Serial.println ("-- Параметры DHT11 --");
  sens_dh();
  Serial.println();
  Serial.println ("-- Параметры BMP280 --");
  Serial.print ("Давление : ");  Serial.print(bp1);  Serial.println (" мм.Р.ст.");
  Serial.print ("Высота : ");  Serial.print (amp1);  Serial.println (" м.");
  Serial.println();
  Serial.println ("-- Параметры YL-83 --");
  sens_r(analogRead(rainPinA0));
  Serial.println();
  Serial.println ("-- Параметры MQ-2 --");
  sens_s();
  Serial.println();
  Serial.println ("-- Параметры LM393 --");
  sens_l();
  Serial.println();
  
}


// Начало программы 
void loop(){
  delay(30000);
  float bp = bme.readPressure()/133.3224;        // считывание давление ВМР280
  float amp = bme.readAltitude(1013.25);
  sens_m(bp,amp);
  leds();  
  
}
