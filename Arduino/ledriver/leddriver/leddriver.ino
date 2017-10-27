int ledPin = 9;
int pwm_value = 100;
String inputString = "";
boolean stringComplete  = false;
void setup() {
    Serial.begin(115200);    
    inputString.reserve(200);
}

void loop() {
  // put your main code here, to run repeatedly:
  analogWrite(ledPin, pwm_value);



  
    while (Serial.available()) {  
      pwm_value= (char)Serial.read();      
    }
}
