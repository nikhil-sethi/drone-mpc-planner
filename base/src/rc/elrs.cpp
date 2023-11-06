#include "elrs.h"
#include "versions.h"

#define CRSF_CHANNELS 16
int rcChannels[CRSF_CHANNELS];

int32_t correction = 0;

// crc code from https://github.com/badjeff/simpleTx_esp32
static uint8_t crc8tab[256] = {
    0x00, 0xD5, 0x7F, 0xAA, 0xFE, 0x2B, 0x81, 0x54, 0x29, 0xFC, 0x56, 0x83, 0xD7, 0x02, 0xA8, 0x7D,
    0x52, 0x87, 0x2D, 0xF8, 0xAC, 0x79, 0xD3, 0x06, 0x7B, 0xAE, 0x04, 0xD1, 0x85, 0x50, 0xFA, 0x2F,
    0xA4, 0x71, 0xDB, 0x0E, 0x5A, 0x8F, 0x25, 0xF0, 0x8D, 0x58, 0xF2, 0x27, 0x73, 0xA6, 0x0C, 0xD9,
    0xF6, 0x23, 0x89, 0x5C, 0x08, 0xDD, 0x77, 0xA2, 0xDF, 0x0A, 0xA0, 0x75, 0x21, 0xF4, 0x5E, 0x8B,
    0x9D, 0x48, 0xE2, 0x37, 0x63, 0xB6, 0x1C, 0xC9, 0xB4, 0x61, 0xCB, 0x1E, 0x4A, 0x9F, 0x35, 0xE0,
    0xCF, 0x1A, 0xB0, 0x65, 0x31, 0xE4, 0x4E, 0x9B, 0xE6, 0x33, 0x99, 0x4C, 0x18, 0xCD, 0x67, 0xB2,
    0x39, 0xEC, 0x46, 0x93, 0xC7, 0x12, 0xB8, 0x6D, 0x10, 0xC5, 0x6F, 0xBA, 0xEE, 0x3B, 0x91, 0x44,
    0x6B, 0xBE, 0x14, 0xC1, 0x95, 0x40, 0xEA, 0x3F, 0x42, 0x97, 0x3D, 0xE8, 0xBC, 0x69, 0xC3, 0x16,
    0xEF, 0x3A, 0x90, 0x45, 0x11, 0xC4, 0x6E, 0xBB, 0xC6, 0x13, 0xB9, 0x6C, 0x38, 0xED, 0x47, 0x92,
    0xBD, 0x68, 0xC2, 0x17, 0x43, 0x96, 0x3C, 0xE9, 0x94, 0x41, 0xEB, 0x3E, 0x6A, 0xBF, 0x15, 0xC0,
    0x4B, 0x9E, 0x34, 0xE1, 0xB5, 0x60, 0xCA, 0x1F, 0x62, 0xB7, 0x1D, 0xC8, 0x9C, 0x49, 0xE3, 0x36,
    0x19, 0xCC, 0x66, 0xB3, 0xE7, 0x32, 0x98, 0x4D, 0x30, 0xE5, 0x4F, 0x9A, 0xCE, 0x1B, 0xB1, 0x64,
    0x72, 0xA7, 0x0D, 0xD8, 0x8C, 0x59, 0xF3, 0x26, 0x5B, 0x8E, 0x24, 0xF1, 0xA5, 0x70, 0xDA, 0x0F,
    0x20, 0xF5, 0x5F, 0x8A, 0xDE, 0x0B, 0xA1, 0x74, 0x09, 0xDC, 0x76, 0xA3, 0xF7, 0x22, 0x88, 0x5D,
    0xD6, 0x03, 0xA9, 0x7C, 0x28, 0xFD, 0x57, 0x82, 0xFF, 0x2A, 0x80, 0x55, 0x01, 0xD4, 0x7E, 0xAB,
    0x84, 0x51, 0xFB, 0x2E, 0x7A, 0xAF, 0x05, 0xD0, 0xAD, 0x78, 0xD2, 0x07, 0x53, 0x86, 0x2C, 0xF9
};

// CRC8 implementation with polynom = 0xBA
static const uint8_t crc8tab_BA[256] = {
    0x00, 0xBA, 0xCE, 0x74, 0x26, 0x9C, 0xE8, 0x52, 0x4C, 0xF6, 0x82, 0x38, 0x6A, 0xD0, 0xA4, 0x1E,
    0x98, 0x22, 0x56, 0xEC, 0xBE, 0x04, 0x70, 0xCA, 0xD4, 0x6E, 0x1A, 0xA0, 0xF2, 0x48, 0x3C, 0x86,
    0x8A, 0x30, 0x44, 0xFE, 0xAC, 0x16, 0x62, 0xD8, 0xC6, 0x7C, 0x08, 0xB2, 0xE0, 0x5A, 0x2E, 0x94,
    0x12, 0xA8, 0xDC, 0x66, 0x34, 0x8E, 0xFA, 0x40, 0x5E, 0xE4, 0x90, 0x2A, 0x78, 0xC2, 0xB6, 0x0C,
    0xAE, 0x14, 0x60, 0xDA, 0x88, 0x32, 0x46, 0xFC, 0xE2, 0x58, 0x2C, 0x96, 0xC4, 0x7E, 0x0A, 0xB0,
    0x36, 0x8C, 0xF8, 0x42, 0x10, 0xAA, 0xDE, 0x64, 0x7A, 0xC0, 0xB4, 0x0E, 0x5C, 0xE6, 0x92, 0x28,
    0x24, 0x9E, 0xEA, 0x50, 0x02, 0xB8, 0xCC, 0x76, 0x68, 0xD2, 0xA6, 0x1C, 0x4E, 0xF4, 0x80, 0x3A,
    0xBC, 0x06, 0x72, 0xC8, 0x9A, 0x20, 0x54, 0xEE, 0xF0, 0x4A, 0x3E, 0x84, 0xD6, 0x6C, 0x18, 0xA2,
    0xE6, 0x5C, 0x28, 0x92, 0xC0, 0x7A, 0x0E, 0xB4, 0xAA, 0x10, 0x64, 0xDE, 0x8C, 0x36, 0x42, 0xF8,
    0x7E, 0xC4, 0xB0, 0x0A, 0x58, 0xE2, 0x96, 0x2C, 0x32, 0x88, 0xFC, 0x46, 0x14, 0xAE, 0xDA, 0x60,
    0x6C, 0xD6, 0xA2, 0x18, 0x4A, 0xF0, 0x84, 0x3E, 0x20, 0x9A, 0xEE, 0x54, 0x06, 0xBC, 0xC8, 0x72,
    0xF4, 0x4E, 0x3A, 0x80, 0xD2, 0x68, 0x1C, 0xA6, 0xB8, 0x02, 0x76, 0xCC, 0x9E, 0x24, 0x50, 0xEA,
    0x48, 0xF2, 0x86, 0x3C, 0x6E, 0xD4, 0xA0, 0x1A, 0x04, 0xBE, 0xCA, 0x70, 0x22, 0x98, 0xEC, 0x56,
    0xD0, 0x6A, 0x1E, 0xA4, 0xF6, 0x4C, 0x38, 0x82, 0x9C, 0x26, 0x52, 0xE8, 0xBA, 0x00, 0x74, 0xCE,
    0xC2, 0x78, 0x0C, 0xB6, 0xE4, 0x5E, 0x2A, 0x90, 0x8E, 0x34, 0x40, 0xFA, 0xA8, 0x12, 0x66, 0xDC,
    0x5A, 0xE0, 0x94, 0x2E, 0x7C, 0xC6, 0xB2, 0x08, 0x16, 0xAC, 0xD8, 0x62, 0x30, 0x8A, 0xFE, 0x44
};

static uint8_t crsf_crc(const uint8_t crctab[], const uint8_t *ptr, uint8_t len)
{
    uint8_t crc = 0;
    for (uint8_t i = 0; i < len; i++)
    {
        crc = crctab[crc ^ *ptr++];
    }
    return crc;
}
uint8_t crsf_crc8(const uint8_t *ptr, uint8_t len)
{
    return crsf_crc(crc8tab, ptr, len);
}
uint8_t crsf_crc8_BA(const uint8_t *ptr, uint8_t len)
{
    return crsf_crc(crc8tab_BA, ptr, len);
}

typedef enum
{
    receiveStart,
    receiveSize,
    receivePayload,
    receiveCRC,
} CRSFReceiveState;
static CRSFReceiveState crsfReceiveState = receiveStart;

bool ELRS::connect() {
    std::cout << "Connecting ELRS TX" << std::endl;
    if (port_unopened) {
        zerothrottle();

        port_unopened = RS232_OpenComport(SERIAL_BAUDRATE, "/dev/elrs");

        if (port_unopened)
            std::cout << "Could not open ELRS TX port."  << std::endl;
        else
            std::cout << "Opened ELRS TX port."  << std::endl;
    }
    return !port_unopened;
}

int read_config_field_index = 0;
int read_chunk_index = 0;

void ELRS::ping_thread(void) {
    usleep(100);
    ping();
    _current_config_index = 0;
    while (find_modelid_parameter && _current_config_index <= no_cfg_params) {
        read_parameter(_current_config_index, 0);
        _current_config_index++;
    }
    if (_model_match_index) {
        write_parameter(_model_match_index, 20);
    }
    std::cout << "\n\nPinged\n\n" << std::endl;
    set_model_id(20);
    receive_thread_tx = std::thread(&ELRS::receive_thread, this);
    send_thread_tx = std::thread(&ELRS::send_thread, this);
    return;
}

void ELRS::send_thread(void) {
    usleep(100);
    std::cout << "ELRS send thread ready!" << std::endl;
    g_sendData.lock();
    g_sendData.lock(); // wait for first frame to arrive to prevent triggering a wdt in the MM
    std::cout << "ELRS send thread started!" << std::endl;
    while (!exitSendThread) {
        send_rc_data();
    }
}

void ELRS::receive_thread(void) {
    std::cout << "ELRS receive thread started!" << std::endl;
    while (!exitReceiveThread) {
        receive_data(CRSF_FRAMETYPE_RC_CHANNELS_PACKED);
        usleep(10);
    }
}

void ELRS::flush_buffer() {
    uint8_t buffer[1];
    int n = 1;
    while (n) {
        n = RS232_PollComport(buffer, 1);
    }
    if (n == 0) {
        // std::cout << "Flushed bytes from ELRS buffer." << std::endl;
    }
}

bool ELRS::exchange(uint8_t *packet, uint8_t size, crsf_frame_type_e sent_type) {
    if (!exitSendThread) {
        int _iteration = 0;
        watchdog_tx_connect();
        flush_buffer();
        RS232_SendBuf(packet, size);
        usleep(300);
        _exchanged = false;
        while (!_exchanged && (_iteration < 100)) { // timeout after 10*100 microseconds
            receive_data(sent_type);
            _iteration++;
            usleep(10);
        }
        if (_exchanged)
            return 0;
        else
            return 1;
    }
    return 1;
}

void ELRS::watchdog_tx_connect() {
    if (init_package_nOK_cnt) {
        init_package_nOK_cnt++;
        if (init_package_nOK_cnt > 10 * pparams.fps) {
            _init_package_failure = true;
            std::cout << "ELRS wouldn't respond to ping package within 10 seconds." << std::endl;
            exitSendThread = true;
            return;
        }
    }
}

void ELRS::init(int drone_id) {
    // todo; implement receiver number / model match
    // todo; implement resetting the module with RTS pin
    // todo; implement drone name to switch to ELRS from MM

    _drone_id_rxnum = drone_id;

    if (!port_unopened) {
        init_thread_tx = std::thread(&ELRS::ping_thread, this);
    }
    for (uint8_t i = 0; i < CRSF_CHANNELS; i++)
    {
        rcChannels[i] = CRSF_CHANNEL_VALUE_MIN;
    }
}

void ELRS::init_logger() {
    std::string logger_fn = data_output_dir  + "log_telemetry.txt";
    telem_logger.open(logger_fn, std::ofstream::out);
    logger_initialized = true;
}

void ELRS::get_elrs_info() {
    uint8_t packet[8];
    packet[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[1] = 6;
    packet[2] = CRSF_FRAMETYPE_PARAMETER_WRITE;
    packet[3] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    packet[5] = 0;
    packet[6] = 0;
    packet[7] = crsf_crc8(&packet[2], packet[1] - 1);
    usleep(100);
    RS232_SendBuf(static_cast<unsigned char *>(packet), 8);
    usleep(100);
}

void ELRS::ping() {
    uint8_t packet[6];
    packet[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[1] = 4;
    packet[2] = CRSF_FRAMETYPE_DEVICE_PING;
    packet[3] = CRSF_ADDRESS_BROADCAST;
    packet[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    packet[5] = crsf_crc8(&packet[2], packet[1] - 1);
    while (exchange(packet, 6, CRSF_FRAMETYPE_DEVICE_INFO))
        usleep(500);
    std::cout << "ELRS: Ping succesful" << std::endl;
}

void ELRS::read_parameter(uint8_t config_field_index, uint8_t chunk_index) {
    uint8_t packet[8];
    packet[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[1] = 6;
    packet[2] = CRSF_FRAMETYPE_PARAMETER_READ;
    packet[3] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[4] = CRSF_ADDRESS_ELRS_LUA;
    packet[5] = config_field_index;
    packet[6] = chunk_index;
    packet[7] = crsf_crc8(&packet[2], packet[1] - 1);
    while (exchange(packet, 8, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY))
        usleep(1000);
}

void ELRS::write_parameter(uint8_t config_field_index, uint8_t value) {
    uint8_t packet[8];
    packet[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[1] = 6;
    packet[2] = CRSF_FRAMETYPE_PARAMETER_WRITE;
    packet[3] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[4] = CRSF_ADDRESS_ELRS_LUA;
    packet[5] = config_field_index;
    packet[6] = value;
    packet[7] = crsf_crc8(&packet[2], packet[1] - 1);
    std::cout << "start writing model id" << std::endl;
    int _cnt = 0;
    while (exchange(packet, 8, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY)) {
        _cnt++;
        usleep(1000);
    }
}

void ELRS::set_model_id(uint8_t model_id) {
    uint8_t packet[9];
    packet[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
    packet[1] = 8;
    packet[2] = CRSF_FRAMETYPE_COMMAND;
    packet[3] = CRSF_ADDRESS_CRSF_TRANSMITTER;
    packet[4] = CRSF_ADDRESS_RADIO_TRANSMITTER;
    packet[5] = SUBCOMMAND_CRSF;
    packet[6] = COMMAND_MODEL_SELECT_ID;
    packet[7] = model_id;
    packet[8] = crsf_crc8(&packet[2], packet[1] - 1);
    std::cout << "start sending model id" << std::endl;
    int _cnt = 0;
    while (exchange(packet, 9, CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY) && _cnt < 10) {
        _cnt++;
        usleep(1000);
    }
}

void ELRS::zerothrottle() {
    if (dparams.mode3d)
        throttle = BF_CHN_MID;
    else
        throttle = BF_CHN_MIN;
}

void ELRS::convert_channels(uint16_t *channels, unsigned char *packet) {
    uint32_t bits = 0;
    uint8_t bitsavailable = 0;
    uint16_t ii = 0;
    // byte 4-25, channels 0..2047
    for (uint16_t i = 0; i < MULTI_CHANS; i++) {
        uint16_t value = channels[i];
        bits |= value << bitsavailable;
        bitsavailable += MULTI_CHAN_BITS;
        while (bitsavailable >= 8) {
            packet[ii++] = bits & 0xff;
            bits >>= 8;
            bitsavailable -= 8;
        }
    }
}

int ELRS::map(int inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange) {
    float x = (inValue - minInRange) / (maxInRange - minInRange);
    return minOutRange + (maxOutRange - minOutRange) * x;
}

void ELRS::send_rc_data(void) {
    uint32_t update = updateInterval + correction;
    if (update < CRSF_FRAME_PERIOD_MIN)
        update = CRSF_FRAME_PERIOD_MIN;
    else if (update > CRSF_FRAME_PERIOD_MAX)
        update = CRSF_FRAME_PERIOD_MAX;
    correction -= update - updateInterval;
    auto now = std::chrono::steady_clock::now();
    g_sendData.try_lock_until(now + std::chrono::milliseconds(30));
    g_lockData.lock();
    uint8_t packet[CRSF_PAYLOAD_SIZE_MAX] = {0};

    extern int rcChannels[CRSF_CHANNELS];

    for (uint16_t i = 0; i < 16; i++) {
        rcChannels[i] = 0;
    }

    if (calibrate_acc_cnt) {
        calibrate_acc_cnt--;
        arm_switch = BF_CHN_MIN;
        roll = BF_CHN_MID;
        pitch = BF_CHN_MIN;
        yaw = BF_CHN_MIN;
        throttle = BF_CHN_MAX;
    }
    rcChannels[0] = map(roll,  1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
    rcChannels[1] = map(pitch, 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
    rcChannels[2] = map(throttle, 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
    rcChannels[3] = map(yaw, 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);
    if (_beep || calibrate_acc_cnt) {
        rcChannels[4] = map(bf_disarmed,     1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);   // AUX 1
        rcChannels[5] = map(BF_CHN_MAX,     1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);    // AUX 2
    }
    else {
        rcChannels[4] = map(arm_switch,     1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);    // AUX 1
        rcChannels[5] = map(mode,     1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);          // AUX 2
    }
    rcChannels[6] = map(turtle_mode, 1000, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000);           // AUX 3
    if (calibrate_acc_cnt)
        rcChannels[7] = map(CRSF_CHANNEL_VALUE_1000 + CRSF_CHANNEL_VALUE_2000 / 100 * 100,     0, 2000, CRSF_CHANNEL_VALUE_1000, CRSF_CHANNEL_VALUE_2000); // AUX 4
    else
        rcChannels[7] = CRSF_CHANNEL_VALUE_1000 + CRSF_CHANNEL_VALUE_RANGE / 100 * led;               // AUX 4

    packet[0] = CRSF_ADDRESS_CRSF_TRANSMITTER;  // Header
    packet[1] = 24;                             // length of type (24) + payload + crc
    packet[2] = CRSF_FRAMETYPE_RC_CHANNELS_PACKED;
    packet[3] = static_cast<uint8_t>(rcChannels[0] & 0x07FF);
    packet[4] = static_cast<uint8_t>((rcChannels[0] & 0x07FF) >> 8 | (rcChannels[1] & 0x07FF) << 3);
    packet[5] = static_cast<uint8_t>((rcChannels[1] & 0x07FF) >> 5 | (rcChannels[2] & 0x07FF) << 6);
    packet[6] = static_cast<uint8_t>((rcChannels[2] & 0x07FF) >> 2);
    packet[7] = static_cast<uint8_t>((rcChannels[2] & 0x07FF) >> 10 | (rcChannels[3] & 0x07FF) << 1);
    packet[8] = static_cast<uint8_t>((rcChannels[3] & 0x07FF) >> 7 | (rcChannels[4] & 0x07FF) << 4);
    packet[9] = static_cast<uint8_t>((rcChannels[4] & 0x07FF) >> 4 | (rcChannels[5] & 0x07FF) << 7);
    packet[10] = static_cast<uint8_t>((rcChannels[5] & 0x07FF) >> 1);
    packet[11] = static_cast<uint8_t>((rcChannels[5] & 0x07FF) >> 9 | (rcChannels[6] & 0x07FF) << 2);
    packet[12] = static_cast<uint8_t>((rcChannels[6] & 0x07FF) >> 6 | (rcChannels[7] & 0x07FF) << 5);
    packet[13] = static_cast<uint8_t>((rcChannels[7] & 0x07FF) >> 3);
    packet[14] = static_cast<uint8_t>((rcChannels[8] & 0x07FF));
    packet[15] = static_cast<uint8_t>((rcChannels[8] & 0x07FF) >> 8 | (rcChannels[9] & 0x07FF) << 3);
    packet[16] = static_cast<uint8_t>((rcChannels[9] & 0x07FF) >> 5 | (rcChannels[10] & 0x07FF) << 6);
    packet[17] = static_cast<uint8_t>((rcChannels[10] & 0x07FF) >> 2);
    packet[18] = static_cast<uint8_t>((rcChannels[10] & 0x07FF) >> 10 | (rcChannels[11] & 0x07FF) << 1);
    packet[19] = static_cast<uint8_t>((rcChannels[11] & 0x07FF) >> 7 | (rcChannels[12] & 0x07FF) << 4);
    packet[20] = static_cast<uint8_t>((rcChannels[12] & 0x07FF) >> 4 | (rcChannels[13] & 0x07FF) << 7);
    packet[21] = static_cast<uint8_t>((rcChannels[13] & 0x07FF) >> 1);
    packet[22] = static_cast<uint8_t>((rcChannels[13] & 0x07FF) >> 9 | (rcChannels[14] & 0x07FF) << 2);
    packet[23] = static_cast<uint8_t>((rcChannels[14] & 0x07FF) >> 6 | (rcChannels[15] & 0x07FF) << 5);
    packet[24] = static_cast<uint8_t>((rcChannels[15] & 0x07FF) >> 3);

    packet[25] = crsf_crc8(&packet[2], 26 - 3); // CRC

    if (!port_unopened) {
        RS232_SendBuf(static_cast<unsigned char *>(packet), 26);
    }
    g_lockData.unlock();
    // }
}

void ELRS::parse_packet(unsigned char *buffer, crsf_frame_type_e expected_frame_type) {
    // parse type
    crsf_frame_type_e crsfFrameType = static_cast<crsf_frame_type_e>(buffer[0]);
    if (crsfFrameType != expected_frame_type) {
        if (crsfFrameType == CRSF_FRAMETYPE_RADIO_ID) {
            parseRadioID(buffer);
        } else if (crsfFrameType == CRSF_FRAMETYPE_LINK_STATISTICS) {
            parseLinkStatistics(buffer);
        } else if (crsfFrameType == CRSF_FRAMETYPE_ATTITUDE) {
            parseAttitude(buffer);
        } else if (crsfFrameType == CRSF_FRAMETYPE_BATTERY_SENSOR) {
            parseBattery(buffer);
        } else if (crsfFrameType == CRSF_FRAMETYPE_FLIGHT_MODE) {
            parseFlightMode(buffer);
        } else {
            std::cout << "ELRS: Received unexpected frame type: " << +crsfFrameType << std::endl;
        }
    } else if (crsfFrameType == CRSF_FRAMETYPE_DEVICE_INFO) {
        std::cout << "ELRS: Parsing device info" << std::endl;
        parseDeviceInfo(buffer);
    } else if (crsfFrameType == CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY) {
        // std::cout << "ELRS: Parsing parameter settings entry" << std::endl;
        parseParameterEntry(buffer);
    } else {
        std::cout << "ELRS: Received unhandled frame type: " << +crsfFrameType << std::endl;
    }
}

uint8_t getCrossfireTelemetryValue(uint8_t index, int32_t *value, uint8_t len, unsigned char *buffer)
{
    uint8_t result = 0;
    uint8_t *byte = &buffer[index];
    *value = (*byte & 0x80) ? -1 : 0;
    for (int i = 0; i < len; i++)
    {
        *value <<= 8;
        if (*byte != 0xff)
            result = 1;
        *value += *byte++;
    }
    return result;
}

void ELRS::parseRadioID(unsigned char *buffer) {
    if (buffer[1] == CRSF_ADDRESS_RADIO_TRANSMITTER && buffer[2] == CRSF_ADDRESS_CRSF_TRANSMITTER && buffer[3] == CRSF_FRAMETYPE_OPENTX_SYNC) {
        if (getCrossfireTelemetryValue(4, reinterpret_cast<int32_t *>(&updateInterval), 4, buffer) &&
                getCrossfireTelemetryValue(8, &correction, 4, buffer))
        {
            // values are in 10th of micro-seconds
            updateInterval /= 10;
            correction /= 10;
            if (correction >= 0)
                correction %= updateInterval;
            else
                correction = -((-correction) % updateInterval);
            // std::cout << "Update interval: " << updateInterval << " correction: " << correction << std::endl;
        }
    }
}

void ELRS::parseDeviceInfo(unsigned char *buffer) {
    if (buffer[1] == CRSF_ADDRESS_RADIO_TRANSMITTER && buffer[2] == CRSF_ADDRESS_CRSF_TRANSMITTER) {
        CRSF_FRAME_DEVICE_INFO deviceInfoFrame;
        // ping response
        int index = 3;

        // display name
        while (buffer[index] != 0) {
            deviceInfoFrame.display_name += char(buffer[index]);
            index++;
        }
        index += 1; // skip null string terminator

        // elrs version
        for (int i = 0; i < 4; i++) {
            deviceInfoFrame.elrs_version += char(buffer[index]);
            index++;
        }
        index += 5; // skip hardware version + leading zero version byte
        for (int i = 0; i < 3; i++) {
            deviceInfoFrame.elrs_version += std::to_string(buffer[index]);
            index++;
        }

        // number of config parameters
        deviceInfoFrame.number_of_config_parameters = buffer[index];
        index++;

        // parameter protocol version
        deviceInfoFrame.parameter_protocol_version = buffer[index];

        std::cout << deviceInfoFrame.display_name << std::endl;
        std::cout << deviceInfoFrame.elrs_version << std::endl;
        std::cout << +deviceInfoFrame.number_of_config_parameters << std::endl;
        std::cout << +deviceInfoFrame.parameter_protocol_version << std::endl;

        if (deviceInfoFrame.elrs_version == "ELRS330") {
            init_package_nOK_cnt = 0;
            _exchanged = true;
            no_cfg_params = +deviceInfoFrame.number_of_config_parameters;
        } else {
            std::cout << "ELRS: Detected wrong ELRS firmware version! Detected: " << deviceInfoFrame.elrs_version << ". Required: ELRS330." << std::endl;
        }
    } else {
        std::cout << "ELRS: Received ping response from unknown device" << std::endl;
    }
}

void ELRS::parseParameterEntry(unsigned char *buffer) {
    if (buffer[1] == CRSF_ADDRESS_RADIO_TRANSMITTER && buffer[2] == CRSF_ADDRESS_CRSF_TRANSMITTER) {
        uint8_t _config_field_index = buffer[3];
        uint8_t _field_type = buffer[6];
        (void)_field_type; // silence compiler, maybe used in the future
        int index = 7;
        string _field_label = "";
        while (!(index > 14)) {
            _field_label += char(buffer[index]);
            index ++;
        }
        if (buffer[index] == 0)
            index++;
        string _options_label = "";
        while (buffer[index] != 0) {
            _options_label += char(buffer[index]);
            index ++;
        }
        if (_current_config_index == _config_field_index) {
            if (_field_label == "Model Ma") {
                std::cout << "ELRS: model match at config field index " << +_config_field_index << std::endl;
                _model_match_index = +_config_field_index;
            }
            _exchanged = true;
        }
    } else {
        std::cout << "ELRS: Received parameter response from unknown device" << std::endl;
    }
}

void ELRS::parseLinkStatistics(unsigned char *buffer) {
    CRSF_FRAME_LINK_STATS linkStatsFrame;
    last_telemetry_time = _time;
    linkStatsFrame.uplink_RSSI_1 = buffer[1];
    linkStatsFrame.uplink_RSSI_2 = buffer[2];
    linkStatsFrame.uplink_Link_quality = buffer[3];
    linkStatsFrame.uplink_SNR = buffer[4];
    linkStatsFrame.active_antenna = buffer[5];
    linkStatsFrame.rf_Mode = buffer[6];
    linkStatsFrame.uplink_TX_Power = buffer[7];
    linkStatsFrame.downlink_RSSI = buffer[8];
    linkStatsFrame.downlink_Link_quality = buffer[9];
    linkStatsFrame.downlink_SNR = buffer[10];

    telemetry.rssi = linkStatsFrame.uplink_RSSI_1;
    telemetry.rssi_last_received = _time;
    telemetry.uplink_quality = linkStatsFrame.uplink_Link_quality;
    if (logger_initialized)
        telem_logger << _time << ";FSSP_DATAID_RSSI;" << static_cast<int>(telemetry.rssi) << "\n";
}

void ELRS::parseAttitude(unsigned char *buffer) {
    telemetry.roll_pitch_package_id++;
    telemetry.pitch = static_cast<int16_t>(((buffer)[1] << 8) | (buffer)[2]) / pow(2, 16) * 360.;
    telemetry.roll = static_cast<int16_t>(((buffer)[3] << 8) | (buffer)[4]) / pow(2, 16) * 360.;
    if (logger_initialized) {
        telem_logger << _time << ";FSSP_DATAID_ROLL;" << telemetry.roll << "\n";
        telem_logger << _time << ";FSSP_DATAID_PITCH;" << telemetry.pitch << "\n";
    }
}

void ELRS::parseBattery(unsigned char *buffer) {
    telemetry.batt_cell_v = static_cast<uint16_t>(((buffer)[1] << 8) | (buffer)[2]) / 10.;
    telemetry.batt_cell_v_package_id++;
    if (logger_initialized)
        telem_logger << _time << ";FSSP_DATAID_A4;" << telemetry.batt_cell_v << "\n";
}

void ELRS::parseFlightMode(unsigned char *buffer) {
    int index = 1;
    string flightMode = "";
    while (buffer[index] != 0) {
        flightMode += char(buffer[index]);
        index++;
    }
    // fm is !ERR* before arming
}

unsigned char crsfPacketSize = 0;
int crsfPacketIndex = 0;
unsigned char rxBuffer[CRSF_MAX_PACKET_LEN];
void ELRS::receive_data_packet(unsigned char byte, crsf_frame_type_e expected_frame_type) {
    // std::cout << +byte << std::endl;
    switch (crsfReceiveState) {
        case receiveStart:
            crsfPacketSize = 0;
            crsfReceiveState = (byte == CRSF_ADDRESS_RADIO_TRANSMITTER) ? receiveSize : receiveStart;
            break;
        case receiveSize:
            // std::cout << "ELRS: Received start byte" << std::endl;
            crsfPacketIndex = 0;
            if (byte <= CRSF_MAX_PACKET_LEN) {
                crsfPacketSize = byte;
                // std::cout << "ELRS: Packet will be of size: " << +crsfPacketSize << std::endl;
                crsfReceiveState = receivePayload;
            } else {
                crsfReceiveState = receiveStart;
            }
            break;
        case receivePayload:
            rxBuffer[crsfPacketIndex] = byte;
            if (crsfPacketIndex >= crsfPacketSize - 2) {
                crsfReceiveState = receiveCRC;
            }
            crsfPacketIndex++;
            break;
        case receiveCRC:
            // last byte is crc
            if (crsf_crc8(rxBuffer, crsfPacketSize - 1) == byte) {
                // std::cout << "ELRS: Received packet. Parsing." << std::endl;
                parse_packet(rxBuffer, expected_frame_type);

            } else {
                std::cout << "ELRS: Received packet with bad crc" << std::endl;
            }
            crsfReceiveState = receiveStart;
            break;
    }
}

void ELRS::receive_data(crsf_frame_type_e frame_type) {
    if (!port_unopened) {
        unsigned char inbuf[1];
        int n = 1;
        while (n)    {
            n = RS232_PollComport(inbuf, 1);
            if (n == 1) {
                // start parsing state machine
                receive_data_packet(inbuf[0], frame_type);
            } else if (n < 0) {
                std::cout << "ELRS read error: " << n << std::endl;
            }
        }
    }
}

void ELRS::close() {
    if (initialized) {
        std::cout << "Closing ELRS module" << std::endl;
        beep(false);
        exitSendThread = true;
        g_sendData.unlock();
        g_lockData.unlock();
        receive_thread_tx.join();
        usleep(1e5);
        exitReceiveThread = true;
        receive_thread_tx.join();

        // kill throttle when closing the module
        g_lockData.lock();
        mode = BF_CHN_MIN;
        arm_switch = BF_CHN_MIN;
        throttle = BF_CHN_MIN;
        roll = BF_CHN_MID;
        pitch = BF_CHN_MID;
        yaw = BF_CHN_MID;
        _LED_drone = 0;

        g_sendData.unlock();
        g_lockData.unlock();
        send_rc_data();
        port_unopened = 1;
        RS232_CloseComport();
        if (logger_initialized) {
            telem_logger.flush();
            telem_logger.close();
        }
    }
    initialized = false;
}
