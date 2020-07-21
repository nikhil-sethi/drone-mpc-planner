#ifndef __ALARM_H__
#define __ALARM_H__

#define LED_PIN 2
#define BEEPER_PIN 3

#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978

unsigned long beep_time = 0;

unsigned long get_beep_time() { 
  return millis() - beep_time;
}

void reset_beep_time() { 
  beep_time = millis();
  Serial.println("reset_beep_time");
}

void set_beep(int freq) {
  static int old_freq = 0;

  if (freq == old_freq)
    return;

  if (freq) {
    tone(BEEPER_PIN, freq);
  } else {
    noTone(BEEPER_PIN);
  }

  old_freq = freq;
}

void poly_beep(int* tones, int len, unsigned long t) {
  int total_time = 0;

  for (int i = 0; i < len / 2; i++) {
    total_time += tones[i * 2 + 1];

    if (t > total_time)
      continue;

    char buf[30];
    sprintf(buf, "tone %d %d %d", tones[i * 2], int(t), int(total_time));

    if (tones[i * 2] && total_time - t > 40) {
      set_beep(tones[i * 2]);
    } else {
      set_beep(0);
    }
    return;
  }

  if (t > total_time) {
    /* Serial.println("no_beep 2"); */
    set_beep(0);
    return;
  }
}

void set_alarm_connected() {
  int note = 500;
  int len = 12;
  int beeps[len] = {NOTE_E5, note/4,
                  NOTE_G5, note/4,
                  NOTE_E6, note/4,
                  NOTE_C6, note/4,
                  NOTE_D6, note/4,
                  NOTE_G6, note/4,
                  };
  poly_beep(beeps, len, get_beep_time());
}

void set_alarm_no_current() {
  int note = 500;
  int len = 12;
  int beeps[len] = {NOTE_C3, note/4,
                  NOTE_C4, note/4,
                  NOTE_A3, note/4,
                  NOTE_A4, note/4,
                  NOTE_AS3, note/4,
                  NOTE_AS4, note/4,
                  };
  poly_beep(beeps, len, get_beep_time() % 30000);
}

void set_alarm_no_drone() {
  int note = 1001;
  int len = 20;
  int beeps[len] = {NOTE_E5, note/8,
                  NOTE_E5, note/8,
                  0, note/8,
                  NOTE_E5, note/8,
                  0, note/8,
                  NOTE_C5, note/8,
                  NOTE_E5, note/4,
                  NOTE_G5, note/4,
                  0, note/4,
                  NOTE_G4, note/4,
                  };
  poly_beep(beeps, len, get_beep_time() % 60000);
}

void set_alarm(bool on) {
  /* set_alarm_connected(); */
  if (on) {
    digitalWrite(LED_PIN, 1);
    tone(BEEPER_PIN, 1000);
  } else {
    digitalWrite(LED_PIN, 0);
    analogWrite(BEEPER_PIN, 0);
    noTone(BEEPER_PIN);
  }
}

void set_alarm_disconnected() {
  int note = 500;
  int len = 22;
  int beeps[len] = {NOTE_B4, note/4,
                  NOTE_F5, note/4,
                  0, note/4,
                  NOTE_F5, note/4,
                  NOTE_F5, note/3,
                  NOTE_E5, note/3,
                  NOTE_D5, note/3,
                  NOTE_C5, note/2,
                  NOTE_C2, note/3,
                  NOTE_C2, note/4,
                  NOTE_C2, note/3,
                  };
  poly_beep(beeps, len, get_beep_time());
}

void set_alarm_blink_only(bool on) {
  if (on) {
    digitalWrite(LED_PIN, 1);
  } else {
    digitalWrite(LED_PIN, 0);
  }
}

void set_alarm_no_beep() {
  set_alarm(0);
}

void set_alarm_single_beep_inv() {
  if (millis() % 1500 > 100)
    set_alarm(1);
  else
    set_alarm(0);
}

void set_alarm_test_mode() {
  if (millis() % 5000 < 100)
    set_alarm(1);
  else
    set_alarm(0);
}

void set_alarm_half_on_slow() {
  if (millis() % 1500 > 750)
    set_alarm(1);
  else
    set_alarm(0);
}

void set_alarm_half_on_fast() {
  if (millis() % 300 > 150)
    set_alarm(1);
  else
    set_alarm(0);
}

void set_alarm_half_on_super_fast() {
  if (millis() % 200 > 100)
    set_alarm(1);
  else
    set_alarm(0);
}

void set_alarm_single_beep() {
  if (millis() % 3000 > 2900)
    set_alarm_blink_only(1);
  else
    set_alarm_blink_only(0);
}

void set_alarm_double_beep() {
  if (millis() % 3000 < 2600)
    set_alarm_blink_only(0);
  else if (millis() % 3000 < 2700)
    set_alarm_blink_only(1);
  else if (millis() % 3000 < 2800)
    set_alarm_blink_only(0);
  else if (millis() % 3000 < 2900)
    set_alarm_blink_only(1);
}

void set_alarm_long_double_beep() {
  if (millis() % 1500 < 1100)
    set_alarm(0);
  else if (millis() % 1500 < 1150)
    set_alarm(1);
  else if (millis() % 1500 < 1300)
    set_alarm(0);
  else if (millis() % 1500 < 1350)
    set_alarm(1);
}

void set_alarm_tripple_beep() {
  int d = 2;
  if (millis() % (3000 / d) < 2400 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2500 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 2600 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2700 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 2800 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2900 / d)
    set_alarm(1);
}

void set_alarm_long_tripple_beep() {
  int d = 2;
  if (millis() % (3000 / d) < 1300 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 1800 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 1900 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2400 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 2500 / d)
    set_alarm(0);
  else
    set_alarm(1);
}

void set_alarm_quadruple_beep() {
  int d = 2;
  if (millis() % (3000 / d) < 2200 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2300 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 2400 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2500 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 2600 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2700 / d)
    set_alarm(1);
  else if (millis() % (3000 / d) < 2800 / d)
    set_alarm(0);
  else if (millis() % (3000 / d) < 2900 / d)
    set_alarm(1);
}

#endif  //__ALARM_H__
