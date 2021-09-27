#include "pats_tunes.h"
#include "Arduino.h"

unsigned long beep_timestamp = 0;

Tone charging_beeps[] = {
    {NOTE_C1, note/4},
    {0, note/4},
    {NOTE_C1, note/4},
    {0, note/4},
    {NOTE_C1, note/4},
    {0, note/4},
    {0, note/1},
};

Tone drone_connected_beeps[] = {
    {NOTE_E5, note/4},
    {NOTE_G5, note/4},
    {NOTE_E6, note/4},
    {NOTE_C6, note/4},
    {NOTE_D6, note/4},
    {NOTE_G6, note/4}
};

Tone drone_charging_turbo_beeps[] = {
    {NOTE_C2, note/8},
    {0, note/8},
    {NOTE_C2, note/8},
    {0, note/8},
    {NOTE_C2, note/8},
    {0, note/8},
    {0, note/2},
};

Tone drone_dected_turbo_beeps[] = {
    {NOTE_E6, note/8},
    {NOTE_G6, note/8},
    {NOTE_E7, note/8},
    {NOTE_C7, note/8},
    {NOTE_D7, note/8},
    {NOTE_G7, note/8},
    {NOTE_E6, note/8},
    {NOTE_G6, note/8},
    {NOTE_E7, note/8},
    {NOTE_C7, note/8},
    {NOTE_D7, note/8},
    {NOTE_G7, note/8}
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

Tone voltage_too_low_beeps[] = {
    {NOTE_C5, note/2},
    {0, note/4},
    {NOTE_G4, note/2},
    {0, note/4},
    {NOTE_E4, note/2},
    {0, note/4},
    {NOTE_A4, note/2},
    {NOTE_B4, note/2},
    {NOTE_AS4, note/4},
    {NOTE_A4, note/2},

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

Tone disdrone_connected_beeps[] = {
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

bool poly_beep(Tone* tones, int n_tones, unsigned long t, bool repeat, int repeat_after_ms) {
    int total_time = 0;

    for (Tone * tone = tones; tone < &tones[n_tones]; tone++) {
        total_time += tone->time;

        if (t > total_time)
            continue;

        int silence_between_beeps_ms = 40;
        if (tone->time && total_time - t > silence_between_beeps_ms) {
            set_beep(tone->freq);
        } else {
            set_beep(0);
        }
        return false;
    }

    if (t > total_time) {
        set_beep(0);
        if (repeat && t > total_time + repeat_after_ms) {
            reset_beep_time();
        }
        return true;
    }
}

bool play_tune_connected() {
    return poly_beep(drone_connected_beeps, sizeof(drone_connected_beeps)/sizeof(Tone),beep_time());
}

bool play_tune_connected_turbo() {
    return poly_beep(drone_dected_turbo_beeps, sizeof(drone_dected_turbo_beeps)/sizeof(Tone), beep_time());
}

bool play_tune_charging() {
    return poly_beep(charging_beeps, sizeof(charging_beeps)/sizeof(Tone), beep_time(), true);
}

bool play_tune_charging_turbo() {
    return poly_beep(drone_charging_turbo_beeps, sizeof(drone_charging_turbo_beeps)/sizeof(Tone), beep_time(), true);
}

bool play_tune_calibrating() {
    return poly_beep(calibrating_beeps, sizeof(calibrating_beeps)/sizeof(Tone), beep_time(), true,  5000);
}

bool play_tune_no_current() {
    return poly_beep(no_current_beeps, sizeof(no_current_beeps)/sizeof(Tone), beep_time(), true, 3000);
}

bool play_tune_no_drone() {
    return poly_beep(no_drone_beeps, sizeof(no_drone_beeps)/sizeof(Tone), beep_time(), true, 30000);
}

bool play_tune_disconnected() {
    return poly_beep(disdrone_connected_beeps, sizeof(disdrone_connected_beeps)/sizeof(Tone), beep_time());
}

bool play_tune_voltage_too_low() {
    return poly_beep(voltage_too_low_beeps, sizeof(voltage_too_low_beeps)/sizeof(Tone), beep_time(), true, 6000);
}

bool play_tune_silent() {
    set_beep(0);
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
    int len = 15;
    Tone beeps[len] = {
        {get_version_tone(version/100,0), note/2},
        {get_version_tone(version/100,1), note/2},
        {get_version_tone(version/100,2), note/2},
        {get_version_tone(version/100,3), note/2},
        {0, note/4},
        {get_version_tone(version%100/10,0), note/2},
        {get_version_tone(version%100/10,1), note/2},
        {get_version_tone(version%100/10,2), note/2},
        {get_version_tone(version%100/10,3), note/2},
        {0, note/4},
        {get_version_tone(version%10,0), note/2},
        {get_version_tone(version%10,1), note/2},
        {get_version_tone(version%10,2), note/2},
        {get_version_tone(version%10,3), note/2},
        {0, note/4},
    };
    return poly_beep(beeps, len, beep_time());
}
