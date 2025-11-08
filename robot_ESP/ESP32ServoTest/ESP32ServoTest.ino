#include <ESP32Servo.h>
Servo myservo;
int servoPin = 19;

void setup() {
	myservo.setPeriodHertz(50);  
	myservo.attach(servoPin, 800, 2500);
  Serial.begin(115200);
  Serial.println();
}

void loop() {
	 myservo.write(0);  //"рука" назад 
   Serial.println(0); //выводим на Монитор порта
   delay(2000);       //задержка 2 с
   
   myservo.write(100);//"рука" вверх 
   Serial.println(100);
   delay(2000);        
   
   myservo.write(200); //"рука" вперед
   Serial.println(200);
   delay(2000);        
}
