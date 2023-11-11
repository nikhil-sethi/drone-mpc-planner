#pragma once
#include <iostream>
#include <fstream>
#include <vector>
#include <math.h>
#include <iomanip>
#include <unistd.h>
#include "common.h"

#include "rs232.h"
#include <thread>
#include <mutex>
#include "stopwatch.h"
#include "rc.h"

#define SERIAL_BAUDRATE 115200

// following defines are from ExpressLRS repository
//
// --->
//

#define CRSF_CRC_POLY 0xd5

#define CRSF_CHANNEL_VALUE_MIN  0 // 987us - actual CRSF min is 0 with E.Limits on, without 172
#define CRSF_CHANNEL_VALUE_1000 191
#define CRSF_CHANNEL_VALUE_MID  992
#define CRSF_CHANNEL_VALUE_2000 1792
#define CRSF_CHANNEL_VALUE_MAX  1984 // 2012us - actual CRSF max is 1984 with E.Limits on, without 1811
#define CRSF_CHANNEL_VALUE_RANGE (CRSF_CHANNEL_VALUE_MAX - CRSF_CHANNEL_VALUE_MIN)
#define CRSF_MAX_PACKET_LEN 64

#define CRSF_SYNC_BYTE 0xC8

#define RCframeLength 22             // length of the RC data packed bytes frame. 16 channels in 11 bits each.
#define LinkStatisticsFrameLength 10 //
#define OpenTXsyncFrameLength 11     //
#define BattSensorFrameLength 8      //
#define VTXcontrolFrameLength 12     //

#define CRSF_PAYLOAD_SIZE_MAX 62
#define CRSF_FRAME_NOT_COUNTED_BYTES 2
#define CRSF_FRAME_SIZE(payload_size) ((payload_size) + 2) // See crsf_header_t.frame_size
#define CRSF_EXT_FRAME_SIZE(payload_size) (CRSF_FRAME_SIZE(payload_size) + 2)
#define CRSF_FRAME_SIZE_MAX (CRSF_PAYLOAD_SIZE_MAX + CRSF_FRAME_NOT_COUNTED_BYTES)
#define CRSF_FRAME_CRC_SIZE 1
#define CRSF_FRAME_LENGTH_EXT_TYPE_CRC 4 // length of Extended Dest/Origin, TYPE and CRC fields combined

#define CRSF_TELEMETRY_LENGTH_INDEX 1
#define CRSF_TELEMETRY_TYPE_INDEX 2
#define CRSF_TELEMETRY_FIELD_ID_INDEX 5
#define CRSF_TELEMETRY_FIELD_CHUNK_INDEX 6
#define CRSF_TELEMETRY_CRC_LENGTH 1
#define CRSF_TELEMETRY_TOTAL_SIZE(x) (x + CRSF_FRAME_LENGTH_EXT_TYPE_CRC)

//////////////////////////////////////////////////////////////

#define CRSF_MSP_REQ_PAYLOAD_SIZE 8
#define CRSF_MSP_RESP_PAYLOAD_SIZE 58
#define CRSF_MSP_MAX_PAYLOAD_SIZE (CRSF_MSP_REQ_PAYLOAD_SIZE > CRSF_MSP_RESP_PAYLOAD_SIZE ? CRSF_MSP_REQ_PAYLOAD_SIZE : CRSF_MSP_RESP_PAYLOAD_SIZE)

typedef enum
{
    CRSF_FRAMETYPE_GPS = 0x02,
    CRSF_FRAMETYPE_VARIO = 0x07,
    CRSF_FRAMETYPE_BATTERY_SENSOR = 0x08,
    CRSF_FRAMETYPE_BARO_ALTITUDE = 0x09,
    CRSF_FRAMETYPE_LINK_STATISTICS = 0x14,
    CRSF_FRAMETYPE_OPENTX_SYNC = 0x10,
    CRSF_FRAMETYPE_RADIO_ID = 0x3A,
    CRSF_FRAMETYPE_RC_CHANNELS_PACKED = 0x16,
    CRSF_FRAMETYPE_ATTITUDE = 0x1E,
    CRSF_FRAMETYPE_FLIGHT_MODE = 0x21,
    // Extended Header Frames, range: 0x28 to 0x96
    CRSF_FRAMETYPE_DEVICE_PING = 0x28,
    CRSF_FRAMETYPE_DEVICE_INFO = 0x29,
    CRSF_FRAMETYPE_PARAMETER_SETTINGS_ENTRY = 0x2B,
    CRSF_FRAMETYPE_PARAMETER_READ = 0x2C,
    CRSF_FRAMETYPE_PARAMETER_WRITE = 0x2D,

    //CRSF_FRAMETYPE_ELRS_STATUS = 0x2E, ELRS good/bad packet count and status flags

    CRSF_FRAMETYPE_COMMAND = 0x32,
    // KISS frames
    CRSF_FRAMETYPE_KISS_REQ  = 0x78,
    CRSF_FRAMETYPE_KISS_RESP = 0x79,
    // MSP commands
    CRSF_FRAMETYPE_MSP_REQ = 0x7A,   // response request using msp sequence as command
    CRSF_FRAMETYPE_MSP_RESP = 0x7B,  // reply with 58 byte chunked binary
    CRSF_FRAMETYPE_MSP_WRITE = 0x7C, // write with 8 byte chunked binary (OpenTX outbound telemetry buffer limit)
    // Ardupilot frames
    CRSF_FRAMETYPE_ARDUPILOT_RESP = 0x80,
} crsf_frame_type_e;

typedef enum
{
    UINT8 = 0x00,
    INT8 = 0x01,
    UINT16 = 0x02,
    INT16 = 0x03,
    FLOAT = 0x08, // 4 byte
    SELECT = 0x09,
    STRING = 0x0A, // null-terminated
    FOLDER = 0x0B,
    INFO = 0x0C, //display string, not configurable
    COMMAND = 0x0D,
} crsf_parameter_info_type_e;

typedef enum {
    SUBCOMMAND_CRSF = 0x10
} crsf_command_e;

typedef enum {
    COMMAND_MODEL_SELECT_ID = 0x05
} crsf_subcommand_e;

enum {
    CRSF_FRAME_TX_MSP_FRAME_SIZE = 58,
    CRSF_FRAME_RX_MSP_FRAME_SIZE = 8,
    CRSF_FRAME_ORIGIN_DEST_SIZE = 2,
};

enum {
    CRSF_FRAME_GPS_PAYLOAD_SIZE = 15,
    CRSF_FRAME_VARIO_PAYLOAD_SIZE = 2,
    CRSF_FRAME_BARO_ALTITUDE_PAYLOAD_SIZE = 4, // TBS version is 2, ELRS is 4 (combines vario)
    CRSF_FRAME_BATTERY_SENSOR_PAYLOAD_SIZE = 8,
    CRSF_FRAME_ATTITUDE_PAYLOAD_SIZE = 6,
    CRSF_FRAME_DEVICE_INFO_PAYLOAD_SIZE = 48,
    CRSF_FRAME_FLIGHT_MODE_PAYLOAD_SIZE = 16,
    CRSF_FRAME_GENERAL_RESP_PAYLOAD_SIZE = CRSF_EXT_FRAME_SIZE(CRSF_FRAME_TX_MSP_FRAME_SIZE)
};

typedef enum
{
    CRSF_ADDRESS_BROADCAST = 0x00,
    CRSF_ADDRESS_USB = 0x10,
    CRSF_ADDRESS_TBS_CORE_PNP_PRO = 0x80,
    CRSF_ADDRESS_RESERVED1 = 0x8A,
    CRSF_ADDRESS_CURRENT_SENSOR = 0xC0,
    CRSF_ADDRESS_GPS = 0xC2,
    CRSF_ADDRESS_TBS_BLACKBOX = 0xC4,
    CRSF_ADDRESS_FLIGHT_CONTROLLER = 0xC8,
    CRSF_ADDRESS_RESERVED2 = 0xCA,
    CRSF_ADDRESS_RACE_TAG = 0xCC,
    CRSF_ADDRESS_RADIO_TRANSMITTER = 0xEA,
    CRSF_ADDRESS_CRSF_RECEIVER = 0xEC,
    CRSF_ADDRESS_CRSF_TRANSMITTER = 0xEE,
    CRSF_ADDRESS_ELRS_LUA = 0xEF
} crsf_addr_e;

//typedef struct crsf_addr_e asas;

typedef enum
{
    CRSF_UINT8 = 0,
    CRSF_INT8 = 1,
    CRSF_UINT16 = 2,
    CRSF_INT16 = 3,
    CRSF_UINT32 = 4,
    CRSF_INT32 = 5,
    CRSF_UINT64 = 6,
    CRSF_INT64 = 7,
    CRSF_FLOAT = 8,
    CRSF_TEXT_SELECTION = 9,
    CRSF_STRING = 10,
    CRSF_FOLDER = 11,
    CRSF_INFO = 12,
    CRSF_COMMAND = 13,
    CRSF_VTX = 15,
    CRSF_OUT_OF_RANGE = 127,
} crsf_value_type_e;

// These flags are or'ed with the field type above to hide the field from the normal LUA view
#define CRSF_FIELD_HIDDEN       0x80     // marked as hidden in all LUA responses
#define CRSF_FIELD_ELRS_HIDDEN  0x40     // marked as hidden when talking to ELRS specific LUA
#define CRSF_FIELD_TYPE_MASK    ~(CRSF_FIELD_HIDDEN|CRSF_FIELD_ELRS_HIDDEN)

//
// <---
//

#define CRSF_TIME_BETWEEN_FRAMES_US 5000   // 4 ms 250Hz
#define CRSF_FRAME_PERIOD_MIN 850   // 1000Hz 1ms, but allow shorter for offset cancellation
#define CRSF_FRAME_PERIOD_MAX 50000 // 25Hz  40ms, but allow longer for offset cancellation

class ELRS : public RC {

public:
    void init(int drone_id);
    bool connect();
    void init_logger();
    void close();
    void bind(bool b);
    int drone_id() {return _drone_id_rxnum;}

    // value_betaflight = value_here + 1000
    int bf_headless_enabled() {return 16;}
    int bf_headless_disabled() {return 47;}
    int bf_yaw_reset() {return 79;}
    int bf_PID_loop_disabled() {return 110;}
    int bf_spin_motor() {return 142;}
    int bf_spin_motor_reversed() {return 173;}
    int bf_airmode() {return 204;}
    int bf_sleep() {return  236;}

private:
    int _drone_id_rxnum = 0;
    uint init_package_nOK_cnt = 1;

    bool _exchanged = false;
    uint8_t no_cfg_params = 0;
    bool find_modelid_parameter = true;
    bool send_read_parameters = false;
    bool _bind = false;

    float batt_v_accepted_max = 10.f;
    float batt_cell_v_accepted_max = 10.f;
    float roll_accepted_max = 180.f;
    float pitch_accepted_max = 180.f;

    uint32_t updateInterval = CRSF_TIME_BETWEEN_FRAMES_US;

    void ping_thread(void);
    void send_thread(void);
    void receive_thread(void);
    void flush_buffer();
    bool exchange(uint8_t *packet, uint8_t size, crsf_frame_type_e sent_type);
    void send_rc_data(void);
    void receive_data(crsf_frame_type_e expected_frame_type);
    void receive_data_packet(unsigned char buffer, crsf_frame_type_e expected_frame_type);
    void parse_packet(unsigned char *buffer, crsf_frame_type_e expected_frame_type);
    void parseRadioID(unsigned char *buffer);
    void parseDeviceInfo(unsigned char *buffer);
    void parseParameterEntry(unsigned char *buffer);
    void parseLinkStatistics(unsigned char *buffer);
    void parseAttitude(unsigned char *buffer);
    void parseBattery(unsigned char *buffer);
    void parseFlightMode(unsigned char *buffer);
    void convert_channels(uint16_t *channels, unsigned char *packet);
    void zerothrottle();
    void get_elrs_info();
    void ping();
    void read_parameter(uint8_t config_field_index, uint8_t chunk_index);
    void write_parameter(uint8_t config_field_index, uint8_t value);
    void set_model_id(uint8_t model_id);
    bool receive_telemetry(std::string buffer);
    void watchdog_tx_connect();
    int map(int inValue, float minInRange, float maxInRange, float minOutRange, float maxOutRange);

    struct CRSF_FRAME_DEVICE_INFO {
        CRSF_FRAME_DEVICE_INFO() {
            display_name = "";
            elrs_version = "";
            number_of_config_parameters = 0;
            parameter_protocol_version = 0;
        };
        string display_name;
        string elrs_version;
        uint8_t number_of_config_parameters;
        uint8_t parameter_protocol_version;
    };

    struct CRSF_FRAME_PARAMETER_INFO {
        CRSF_FRAME_PARAMETER_INFO() {
            display_name = "";
            elrs_version = "";
            number_of_config_parameters = 0;
            parameter_protocol_version = 0;
        };
        string display_name;
        string elrs_version;
        uint8_t number_of_config_parameters;
        uint8_t parameter_protocol_version;
    };

    struct CRSF_FRAME_LINK_STATS {
        CRSF_FRAME_LINK_STATS() {
            uplink_RSSI_1 = 0;
            uplink_RSSI_2 = 0;
            uplink_Link_quality = 0;
            uplink_SNR = -1;
            active_antenna = 0;
            rf_Mode = 0;
            uplink_TX_Power = 0;
            downlink_RSSI = 0;
            downlink_Link_quality = 0;
            downlink_SNR = -1;
        };
        uint8_t uplink_RSSI_1;
        uint8_t uplink_RSSI_2;
        uint8_t uplink_Link_quality;
        int8_t uplink_SNR;
        uint8_t active_antenna;
        uint8_t rf_Mode;
        uint8_t uplink_TX_Power;
        uint8_t downlink_RSSI;
        uint8_t downlink_Link_quality;
        int8_t downlink_SNR;
    };

    int _current_config_index;
    int _model_match_index;
};
