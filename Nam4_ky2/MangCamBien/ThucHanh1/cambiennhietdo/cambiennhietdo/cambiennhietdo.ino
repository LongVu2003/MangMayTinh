// Gọi thư viện DHT11
//#include "DHT.h"            
 
//const int DHTPIN = 2;       //Đọc dữ liệu từ DHT11 ở chân 2 trên mạch Arduino
//const int DHTTYPE = DHT11;  //Khai báo loại cảm biến, có 2 loại là DHT11 và DHT22
 
//DHT dht(DHTPIN, DHTTYPE);
 
void setup() {
  Serial.begin(9600);
  //dht.begin();         // Khởi động cảm biến
}
 
void loop() {
  int adc = analogRead(A0);
  float temp = (float)adc * 500/1023; 
  Serial.print("Nhiet do: ");
  Serial.println(temp,2);               //Xuất nhiệt độ 
  delay(1000);                     //Đợi 1 giây
}
