#ifndef __PATS_TUNES_H__
#define __PATS_TUNES_H__

#define BEEPER_PIN 5

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

unsigned long beep_timestamp = 0;

struct Tone {short freq; int time;} ;

int note = 500;

Tone charging_beeps[] = {
    {NOTE_C1, note/8},
    {0, note/8},
    {NOTE_C1, note/8},
    {0, note/8},
    {NOTE_C1, note/8},
    {0, note/8},
    {0, note/2},
};

Tone connected_beeps[] = {
    {NOTE_E5, note/8},
    {NOTE_G5, note/8},
    {NOTE_E6, note/8},
    {NOTE_C6, note/8},
    {NOTE_D6, note/8},
    {NOTE_G6, note/8}
};

Tone no_current_beeps[] = {
    {NOTE_C3, note/4},
    {NOTE_C4, note/4},
    {NOTE_A3, note/4},
    {NOTE_A4, note/4},
    {NOTE_AS3, note/4},
    {NOTE_AS4, note/4},
};

Tone calibrating_beeps[] = {{
        NOTE_C1, note/8,
    }
};

Tone no_drone_beeps[] = {
    {NOTE_E5, note/4},
    {NOTE_E5, note/4},
    {0, note/4},
    {NOTE_E5, note/4},
    {0, note/4},
    {NOTE_C5, note/4},
    {NOTE_E5, note/2},
    {NOTE_G5, note/2},
    {0, note/2},
    {NOTE_G4, note/2},
};

Tone disconnected_beeps[] = {
    {NOTE_B4, note/4},
    {NOTE_F5, note/4},
    {0, note/4},
    {NOTE_F5, note/4},
    {NOTE_F5, note/3},
    {NOTE_E5, note/3},
    {NOTE_D5, note/3},
    {NOTE_C5, note/2},
    {NOTE_C2, note/3},
    {NOTE_C2, note/4},
    {NOTE_C2, note/3},
};

unsigned long beep_time() {
    return millis() - beep_timestamp;
}

void reset_beep_time() {
    beep_timestamp = millis();
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

bool poly_beep(Tone* tones, unsigned long t, bool repeat = false) {
    int total_time = 0;
    int len = sizeof(tones)/sizeof(*tones);
    for (int i = 0; i < len; i++) {
        total_time += tones[i].time;

        if (t > total_time)
            continue;

        int silence_between_beeps_ms = 40;
        if (tones[i].time && total_time - t > silence_between_beeps_ms) {
            set_beep(tones[i].freq);
        } else {
            set_beep(0);
        }
        return false;
    }

    if (t > total_time) {
        set_beep(0);
        if (repeat)
            reset_beep_time();
        return true;
    }
}

bool play_tune_connected() {
    return poly_beep(connected_beeps, beep_time());
}

bool play_tune_charging() {
    return poly_beep(charging_beeps, beep_time(), true);
}

bool play_tune_calibrating() {
    return poly_beep(calibrating_beeps, beep_time() % 5000);
}

bool play_tune_no_current() {
    return poly_beep(no_current_beeps, beep_time() % 30000);
}

bool play_tune_no_drone() {
    return poly_beep(no_drone_beeps, beep_time() % 30000);
}

int get_version_tone(int value, int position)
{
    int zero = NOTE_C1;
    int one = NOTE_C5;
    int five = NOTE_G5;

    switch (value)
    {
    case 0: {
        if (position == 0 )
            return zero;
        break;
    }
    case 1: {
        if (position == 0 )
            return one;
        break;
    }
    case 2: {
        if (position == 0 )
            return one;
        if (position == 1 )
            return one;
        break;
    }
    case 3: {
        if (position == 0 )
            return one;
        if (position == 1 )
            return one;
        if (position == 2 )
            return one;
        break;
    }
    case 4: {
        if (position == 0 )
            return one;
        if (position == 1 )
            return five;
        break;
    }
    case 5: {
        if (position == 0 )
            return five;
        break;
    }
    case 6: {
        if (position == 0 )
            return five;
        if (position == 1 )
            return one;
        break;
    }
    case 7: {
        if (position == 0 )
            return five;
        if (position == 1 )
            return one;
        if (position == 2 )
            return one;
        break;
    }
    case 8: {
        if (position == 0 )
            return five;
        if (position == 1 )
            return one;
        if (position == 2 )
            return one;
        if (position == 3 )
            return one;
        break;
    }
    case 9: {
        if (position == 0 )
            return five;
        if (position == 1 )
            return one;
        if (position == 2 )
            return five;
        break;
    }
    }
    return 0;
}

bool play_tune_version(int version) {
    int note = 1000;
    int len = 30;
    Tone beeps[len] = {
        {get_version_tone(version/100,0), note/4},
        {get_version_tone(version/100,1), note/4},
        {get_version_tone(version/100,2), note/4},
        {get_version_tone(version/100,3), note/4},
        {0, note/8},
        {get_version_tone(version%100/10,0), note/4},
        {get_version_tone(version%100/10,1), note/4},
        {get_version_tone(version%100/10,2), note/4},
        {get_version_tone(version%100/10,3), note/4},
        {0, note/8},
        {get_version_tone(version%10,0), note/4},
        {get_version_tone(version%10,1), note/4},
        {get_version_tone(version%10,2), note/4},
        {get_version_tone(version%10,3), note/4},
        {0, note/8},
    };
    return poly_beep(beeps, len, beep_time());
}

bool play_tune_disconnected() {
    return poly_beep(disconnected_beeps, beep_time());
}
#endif  //__PATS_TUNES_H__
