boolean toggle2 = 0;
int sensorPin = A1;    // select the input pin for the potentiometer
int sensorValue = 0;  // variable to store the value coming from the sensor

#define PIN_B 2
#define PIN_A 3
void setup(){
    pinMode(PIN_A, OUTPUT);
    pinMode(PIN_B, OUTPUT);
    digitalWrite(PIN_A,LOW);
    digitalWrite(PIN_B,LOW);
    pinMode(sensorPin, INPUT);
}

int count = 0;
int val = 0;

#define freq_40 6e5/60000
#define freq_20 6e5/23000
#define freq_2 6e5/2300
#define freq freq_40
void loop(){
  uint32_t count = 0;
  uint8_t doBeep = true;
  uint32_t pulse_timer = 0;
  
  while (true)
  {    
    for (uint32_t i = 0; i < freq; i++)
        __asm__("nop\n\t"); 
        
    digitalWrite(PIN_B,LOW);
    
    pulse_timer++;
    count++;

    for (uint32_t i = 0; i < freq; i++)
        __asm__("nop\n\t"); //tick++;

    if (doBeep && (pulse_timer % 2000) < 400)
      digitalWrite(PIN_B,HIGH);

    if (count > 10000)
    {
      count = 0;
      sensorValue = analogRead(sensorPin);
      val = 0.1 * val + 0.9 * sensorValue;
      if (val > 10)
        doBeep = true;
      else
        doBeep = false;
  
    }    
  }
}
