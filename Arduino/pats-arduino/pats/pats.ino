#include <util/atomic.h>
#include <EEPROM.h>
#include "iface_nrf24l01.h"
#include <string.h>


// ############ Wiring ################
//#define PPM_pin   2  // PPM in
//SPI Comm.pins with nRF24L01
#define MOSI_pin  2  // MOSI - D2
#define SCK_pin   4  // SCK  - D4
#define CE_pin    5  // CE   - D5
#define MISO_pin  0 // MISO - A0. Weird, this pin really is connected to A0, but the A0 define seems to be screwed up?
#define CS_pin    A1 // CS   - A1
#define LED_pin    13 // LED  - D13
#define POWERLED_pin 6

unsigned char pwm_value = 0;

// SPI outputs
#define MOSI_on PORTD |= _BV(MOSI_pin)
#define MOSI_off PORTD &= ~_BV(MOSI_pin)
#define SCK_on PORTD |= _BV(SCK_pin)
#define SCK_off PORTD &= ~_BV(SCK_pin)
#define CE_on PORTD |= _BV(CE_pin)
#define CE_off PORTD &= ~_BV(CE_pin)
#define CS_on PORTC |= _BV(1)
#define CS_off PORTC &= ~_BV(1)
// SPI input
#define  MISO_on (PINC & _BV(MISO_pin))

#define RF_POWER TX_POWER_80mW 

// PPM stream settings
#define CHANNELS 12 // number of channels in ppm stream, 12 ideally
enum chan_order{
    THROTTLE,
    AILERON,
    ELEVATOR,
    RUDDER,
    AUX1,  // (CH5)  led light, or 3 pos. rate on CX-10, H7, or inverted flight on H101
    AUX2,  // (CH6)  flip control
    AUX3,  // (CH7)  still camera (snapshot)
    AUX4,  // (CH8)  video camera
    AUX5,  // (CH9)  headless
    AUX6,  // (CH10) calibrate Y (V2x2), pitch trim (H7), RTH (Bayang, H20), 360deg flip mode (H8-3D, H22)
    AUX7,  // (CH11) calibrate X (V2x2), roll trim (H7)
    AUX8,  // (CH12) Reset / Rebind
};

#define PPM_MIN 1000
#define PPM_SAFE_THROTTLE 1050 
#define PPM_MID 1500
#define PPM_MAX 2000
#define PPM_MIN_COMMAND 1300
#define PPM_MAX_COMMAND 1700

// EEPROM locations
enum{
    ee_TXID0,
    ee_TXID1,
    ee_TXID2,
    ee_TXID3
};

uint8_t transmitterID[4];
static volatile bool ppm_ok = false;
uint8_t packet[32];
static bool reset=true;
volatile uint16_t Servo_data[12];
static uint16_t ppm[12] = {PPM_MIN,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,
                           PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,PPM_MID,};

String inputString = "";         // a string to hold incoming data
boolean stringComplete = false;  // whether the string is complete
char *p, *i;
char* c = new char[200 + 1]; // match 200 characters reserved for inputString later
char* errpt;
uint8_t ppm_cnt;

void set_txid(bool renew) {
    uint8_t i;
    for(i=0; i<4; i++)
        transmitterID[i] = EEPROM.read(ee_TXID0+i);
    if(renew || (transmitterID[0]==0xFF && transmitterID[1]==0x0FF)) {
        for(i=0; i<4; i++) {
            transmitterID[i] = random() & 0xFF;
            EEPROM.update(ee_TXID0+i, transmitterID[i]); 
        }            
    }
}

void setup() {
    randomSeed((analogRead(A4) & 0x1F) | (analogRead(A5) << 5));
    pinMode(LED_pin, OUTPUT);
    digitalWrite(LED_pin, LOW); //start LED off
    pinMode(MOSI_pin, OUTPUT);
    pinMode(SCK_pin, OUTPUT);
    pinMode(CS_pin, OUTPUT);
    pinMode(CE_pin, OUTPUT);
    pinMode(MISO_pin, INPUT);

    // PPM ISR setup
    //attachInterrupt(PPM_pin - 2, ISR_ppm, CHANGE);
    TCCR1A = 0;  //reset timer1
    TCCR1B = 0;
    TCCR1B |= (1 << CS11);  //set timer1 to increment every 1 us @ 8MHz, 0.5 us @16MHz

    set_txid(false);

    Serial.begin(115200);
    // reserve 200 bytes for the inputString:
    inputString.reserve(200);

    setPwmFrequency(9, 256); 
    pinMode(LED_BUILTIN, OUTPUT);    
}

void set_power_led() {
    if (pwm_value > 75) {
        pwm_value = 75;
      }
      // put your main code here, to run repeatedly:
      analogWrite(POWERLED_pin, pwm_value);
}

bool binding = false;
uint32_t timeout;
void send_cx10() {
  if(reset || ppm[AUX8] > PPM_MAX_COMMAND) { // rebind
        //TODO: disable power led during bind process
        reset = false;    
        binding = true;    
        ppm_ok = false; // wait for multiple complete ppm frames
        set_txid(true); // Renew Transmitter ID      
        NRF24L01_Reset();
        Serial.println("nrf24l01 reset");
        NRF24L01_Initialize();
        Serial.println("nrf24l01 init");
        CX10_init();        
        Serial.println("Waiting for binding...");
    }
    if (binding) {
      if (CX10_bind()) {
        Serial.println("cx10-initialized and bound!");
        binding = false;
      }
    } else {
      timeout = process_CX10(); // returns micros()+6000 for time to next packet. 
    }
}

void process_serial_string() {
  if (stringComplete) { //process a string received over usb
      //Serial.println(inputString);
      // process string
      
      strcpy(c, inputString.c_str());
      p = strtok_r(c,",",&i); // returns substring up to first "," delimiter
      ppm_cnt=0;
      if (!binding)
        Serial.print('#');
      while (p !=0){
        //Serial.print(p);
        int val=strtol(p, &errpt, 10);
        if (!*errpt) {
          if (!binding)
            Serial.print(val);
          if (ppm_cnt == 5)
            pwm_value = val;
          else
            ppm[ppm_cnt]=val;
        }
        else
          if (!binding)
            Serial.print("x"); // prints "x" if it could not decipher the command. Other values in string may still be assigned.
        if (!binding)
          Serial.print(";"); // a separator between ppm values
        p = strtok_r(NULL,",",&i);
        ppm_cnt+=1;
      }
      if (!binding)
        Serial.println(".");
    
      // clear the string:
      inputString = "";
      stringComplete = false;
  }

}

void receive_serial() {
  // Read the string from the serial buffer
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    // if the incoming character is a newline, set a flag
    // so the main loop can do something about it:
    if (inChar == '\n') {
      stringComplete = true;
    }
    else {      
      // add it to the inputString:
      inputString += inChar;        
    }
  }
}


void loop()
{    
  set_power_led();
  send_cx10();
  process_serial_string();  
  receive_serial();
    
  while(micros() < timeout && !binding) // timeout for CX-10 blue = 6000microseconds. 
  {
    //overrun_cnt+=1;
  }
}

/**
 * Divides a given PWM pin frequency by a divisor.
 * 
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 * 
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 * 
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 * 
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor) {
  byte mode;
  if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 64: mode = 0x03; break;
      case 256: mode = 0x04; break;
      case 1024: mode = 0x05; break;
      default: return;
    }
    if(pin == 5 || pin == 6) {
      TCCR0B = TCCR0B & 0b11111000 | mode;
    } else {
      TCCR1B = TCCR1B & 0b11111000 | mode;
    }
  } else if(pin == 3 || pin == 11) {
    switch(divisor) {
      case 1: mode = 0x01; break;
      case 8: mode = 0x02; break;
      case 32: mode = 0x03; break;
      case 64: mode = 0x04; break;
      case 128: mode = 0x05; break;
      case 256: mode = 0x06; break;
      case 1024: mode = 0x07; break;
      default: return;
    }
    TCCR2B = TCCR2B & 0b11111000 | mode;
  }
}
