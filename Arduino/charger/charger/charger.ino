float  voltages[3];
int ch=0; // pointer id to the current channel that is switch as the -
int next_ch;
const int n_ch = 3; // number of seperate channels
int led_state = 0;
#define debugln(msg, ...)  {char debug_buf[64]; sprintf(debug_buf, msg "\r\n", ##__VA_ARGS__); Serial.write(debug_buf);}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  pinMode(8, OUTPUT);
  pinMode(9, OUTPUT);
  pinMode(10, OUTPUT);
  //pinMode(PB3, OUTPUT);
  

  //while(!Serial.available());
  Serial.println("Starting");
  ch = 666;
  next_ch = 0;
}

void loop() {

  digitalWrite(LED_BUILTIN, (led_state) ? HIGH : LOW);
  led_state = !led_state;
  
  find_polarity();
  
  //Serial.println(a0);
}

void find_polarity(){
  
  if (ch!=next_ch) 
    switch_relays(); 

  delay(400); // wait for voltages to settle
  readVoltages();

  int g0 = ch;
  int g1 = (ch+1) % n_ch;
  int g2 = (ch+2) % n_ch;
  
  if (voltages[g1] > 1.f) {
    debugln("- = %d    + = %d",g0,g1);

    
  } else if (voltages[g2] > 1.f) {
    debugln("- = %d    + = %d",g0,g2);
    
  } else {
    // ch is not the -, try the next one
    next_ch = (ch+1) % n_ch;


    //debugln("ch %d: %d v, ch %d: %d v, ch %d: %d v",g0,(int)voltages[g0]*10,g1,(int)voltages[g1]*10,g2,(int)voltages[g2]*10);
    debugln("- != %d, switching...",ch);
    
  }
}

void switch_relays(){
  ch=next_ch;
  if (ch == 0){
    digitalWrite(9,  HIGH);
    digitalWrite(9,  LOW);
    digitalWrite(10,  LOW);
  } else if (ch == 1){
    digitalWrite(9,  HIGH);
    digitalWrite(10,  LOW);
    digitalWrite(8,  LOW);    
  } else {
    digitalWrite(10,  HIGH);
    digitalWrite(8,  LOW);
    digitalWrite(9,  LOW);
  } 
}

void readVoltages() {
  int tmpa;
  float a0, a1, a2;
  tmpa = analogRead(A0);
  voltages[0] = tmpa * (5.0 / 1023.0);
  tmpa = analogRead(A1);
  voltages[1] = tmpa * (5.0 / 1023.0);
  tmpa = analogRead(A2);
  voltages[2] = tmpa * (5.0 / 1023.0);
  debugln("%d, %d, %d",(int)voltages[0]*10,(int)voltages[1]*10,(int)voltages[2]*10);
}
