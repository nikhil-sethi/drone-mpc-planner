int ledPin = 9;
unsigned char pwm_value = 50;

void setup() {
    Serial.begin(115200);    
}

void loop() {
  if (pwm_value > 150) {
    pwm_value = 150;
  }
  // put your main code here, to run repeatedly:
  analogWrite(ledPin, pwm_value);

  static int state = 0;

  while (Serial.available()) {  
    unsigned char b = Serial.read();      

  switch(state) {
    case 0:
      if (b == 66)
        state++;
      break;
   case 1:
      if (b == 116)
        state++;
      else
        state =0;
      break;
   case 2:
      pwm_value = b;
      state = 0;
      break;
  default:
      state = 0;
    break;
  }
    
  }
  delay(20);
}
