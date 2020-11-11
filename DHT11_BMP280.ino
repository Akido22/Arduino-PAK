// Версия от 07/11/2020
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>
#include <Adafruit_BMP280.h>
#include <Adafruit_Sensor.h>
#include "DHT.h"
#define DHTPIN 2
#define DHTTYPE DHT11 

DHT dht(DHTPIN, DHTTYPE);               // инициализация сенсора DHT
Adafruit_BMP280 bme;                    // инициализация сенсора BMP280
LiquidCrystal_I2C lcd(0x27,16,2);       // инициализация дисплея
 
byte symb_grad[8] =                     // кодирование символа градуса
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
  pinMode(13, OUTPUT);
  lcd.begin();                          // инициализация lcd
  lcd.createChar(1, symb_grad);         // регистрируем собственный символ с кодом 1
  Serial.begin(9600);                   // запуск передачи данных
  dht.begin();                          //  запуск датчика DHT
  bme.begin(0x76);                      //  запуск датчика BMP 0x76 адрес

}
 
void loop(){
  delay(5000);
  float h = dht.readHumidity();         // считывание влажности DHT11
  float t = dht.readTemperature();      // считывание температуры DHT11
  float bp = bme.readPressure()/133.3224;        // считывание давление ВМР280
  float amp = bme.readAltitude(1013.25);
  //float bt = bme.readTemperature();      // считывание температуры ВМР280
  //on_off();
  port_view(h, t, bp, amp);
   led(h, t, bp, amp);
}
//Включение светодиода
/*void on_off() {
  digitalWrite(2, 1); 
  delay(500);                     
  digitalWrite(2, 0);  
  delay(800);                     
} */
// Выводим показания влажности и температуры, давления и высоты
 void led(float h2, float t2, float bp2, float amp2){
  lcd.clear();                         // очистка экрана
  lcd.setCursor(0, 0);                 //  установка курсора в начало 1 строки
  lcd.print("Humidity:      %");       // вывод текста
  lcd.setCursor(10, 0);                // установка курсора на 10 позицию
  lcd.print(h2, 1);                    // вывод на экран значения влажности
  lcd.setCursor(0, 1);                 // установка курсора в начало 2 строки
  lcd.print("Temperat:      C");       // вывод текста
  lcd.setCursor(14, 1);                // установка курсора на 14 позицию 
  lcd.print("\1");                     // вывод символа градуса
  lcd.setCursor(10, 1);                // установка курсора на 10 позицию
  lcd.print(t2,1);                     // вывод значения температуры
  delay(5000);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Pressure:      P");
  lcd.setCursor(10, 0); 
  lcd.print(bp2, 1); 
  lcd.setCursor(0, 1);                 // установка курсора в начало 2 строки
  lcd.print("Altitude:      m");       // вывод текста
  lcd.setCursor(10, 1);                // установка курсора на 10 позицию
  lcd.print(amp2,1);
} 

void port_view(float h1, float t1, float bp1, float amp1){
  if (isnan(h1) || isnan(t1)) {
    Serial.println("Ошибка чтения датчика");
  }
  else
  {
    Serial.print ("Влажность : ");
    Serial.print (h1);
    Serial.print ("%\t");
    Serial.print ("Температура : ");
    Serial.print (t1);
    Serial.println (" *C");
    Serial.print ("Давление : ");
    Serial.print (bp1);
    Serial.println (" мм.Р.ст.");
    Serial.print ("Высота : ");
    Serial.print (amp1);
    Serial.println (" м.");
    
 }
}
