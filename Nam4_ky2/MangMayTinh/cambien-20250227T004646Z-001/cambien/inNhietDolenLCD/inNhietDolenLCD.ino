//Thêm thư viện LiquitCrystal - nó có sẵn vì vậy bạn không cần cài thêm gì cả
#include <LiquidCrystal.h>
#include<DHT.h>
const int DHTpin = 6;
const int DHTtype = DHT11;
DHT dht(DHTpin,DHTtype); 
//Khởi tạo với các chân
LiquidCrystal lcd(12, 11, 5, 4, 3, 2);
 
void setup() {
  pinMode(9,OUTPUT);
  //Thông báo đây là LCD 1602
  lcd.begin(16, 2);
  dht.begin();
}
 
void loop() {
  // đặt con trỏ vào cột 0, dòng 1
  lcd.setCursor(0, 0);
  float temp = dht.readTemperature();
  float h = dht.readHumidity();
  lcd.print("Nhiet do:" + (String)temp + "'C");
  lcd.setCursor(0,1); 
  lcd.print("Do am:" + (String)h + "%");
}
