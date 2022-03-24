from enum import Enum
import struct

BASEBOARD_FIRMWARE_VERSION = 9
BASEBOARD_PACKAGE_PRE_HEADER = '@'
EXECUTOR_PACKAGE_PRE_HEADER = '@'


charging_state_names = [
    'disabled          ',
    'init              ',
    'drone_not_on_pad  ',
    'contact_problem   ',
    'bat_dead          ',
    'does_not_charge   ',
    'revive_charging   ',
    'normal_charging   ',
    'trickle_charging  ',
    'discharge         ',
    'measure           ',
    'wait_drone_ready  ',
    'calibrating       '
]


class baseboard_package_headers(Enum):
    header_SerialBaseboard2NUCPackage = 'P',
    header_SerialNUC2BaseboardChargingPackage = 'C',
    header_SerialNUC2BaseboardLedPowerPackage = 'L',
    header_SerialNUC2BaseboardWatchdogPackage = 'W',
    header_SerialNUC2BaseboardFanPackage = 'F',
    header_SerialNUC2BaseboardNUCResetPackage = 'N',
    header_SerialNUC2BaseboardEEPROMPackage = 'E',
    header_SerialNUC2BaseboardRGBLEDPackage = 'R'
    header_SerialExecutor2BaseboardAllowChargingPackage = 'A',
    header_SocketDaemonLink2BaseboardLinkPackage = 'd'


class executor_package_headers(Enum):
    header_SocketExecutorStatePackage = 's'


class executor_states(Enum):
    es_daemon_disabled = 0,
    es_starting = 1,
    es_hardware_check = 2,
    es_init = 3,
    es_init_vision = 4,
    es_realsense_init = 5,
    es_realsense_reset = 6,
    es_realsense_not_found = 7,
    es_wait_for_darkness = 8,
    es_wait_for_angle = 9,
    es_wait_for_plukker = 10,
    es_pats_c = 11,
    es_pats_x = 12,
    es_closing = 13,
    es_periodic_restart = 14,
    es_watchdog_restart = 15,
    es_user_restart = 16,
    es_brightness_restart = 17,
    es_plukker_restart = 18,
    es_drone_version_mismatch = 19,
    es_drone_config_restart = 20,
    es_realsense_fps_problem = 21,
    es_realsense_frame_loss_problem = 22,
    es_rc_problem = 23,
    es_baseboard_problem = 24,
    es_daemon_problen = 25,
    es_realsense_error = 26,
    es_xml_config_problem = 27,
    es_runtime_error = 28,


class rgb_led_1_states(Enum):
    LED1_init = 0,
    LED1_inresponsive_NUC = 1,
    LED1_executor_problem = 2,  # (angle, realsense, executor proces not running, etc)
    LED1_realsense_reset = 3,
    LED1_executor_start = 4,  # start / restart / closing
    LED1_wait_for_plukkers = 5,
    LED1_wait_for_darkness = 6,
    LED1_c_OK = 7,
    LED1_x_OK = 8,
    LED1_unknown = 9,


class SerialBaseboard2NUCPackage:
    format = '=cHcHHHLBBBffffffffBLHc'  # https://docs.python.org/3/library/struct.html?highlight=struct#format-characters
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialBaseboard2NUCPackage
    hardware_version = 0
    baseboard_boot_count = 0
    watchdog_boot_count = 0
    up_duration = 0
    led_state = 0
    watchdog_state = 0
    charging_state = 0
    battery_volts = 0
    charging_volts = 0
    ground_volts = 0
    charging_amps = 0
    setpoint_amps = 0
    mah_charged = 0
    charge_resistance = 0
    drone_amps_burn = 0
    charging_pwm = 0
    charging_duration = 0
    measured_fan_speed = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.hardware_version,
         self.baseboard_boot_count,
         self.watchdog_boot_count,
         self.up_duration,
         self.led_state,
         self.watchdog_state,
         self.charging_state,
         self.battery_volts,
         self.charging_volts,
         self.ground_volts,
         self.charging_amps,
         self.setpoint_amps,
         self.mah_charged,
         self.charge_resistance,
         self.drone_amps_burn,
         self.charging_pwm,
         self.charging_duration,
         self.measured_fan_speed,
         self.ender,
         ) = fields


class SerialNUC2BaseboardEEPROMPackage:
    format = '=cHcBBBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardEEPROMPackage.value[0]
    clear_config_all = 0,
    clear_config_hard = 0,
    clear_config_log = 0,
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.clear_config_all,
         self.clear_config_hard,
         self.clear_config_log,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.clear_config_all,
                           self.clear_config_hard,
                           self.clear_config_log,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardChargingPackage:
    format = '=cHcBBfBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardChargingPackage.value[0]
    enable_charging = 0
    calibrate = 0
    volts = 0.0
    reset_calibration = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.enable_charging,
         self.calibrate,
         self.volts,
         self.reset_calibration,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.enable_charging,
                           self.calibrate,
                           self.volts,
                           self.reset_calibration,
                           bytes(self.ender, 'utf-8')
                           )


class SerialExecutor2BaseboardAllowChargingPackage:
    format = '=cHcBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialExecutor2BaseboardAllowChargingPackage.value[0]
    allow_charging = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.allow_charging,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.allow_charging,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardLedPowerPackage:
    format = '=cHcBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardLedPowerPackage.value[0]
    led_power = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.led_power,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.led_power,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardWatchdogPackage:
    format = '=cHcBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardWatchdogPackage.value[0]
    watchdog_enabled = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
            self.firmware_version,
            self.header,
            self.watchdog_enabled,
            self.ender
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.watchdog_enabled,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardFanPackage:
    format = '=cHcBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardFanPackage.value[0]
    fan_pwm = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.fan_pwm,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.fan_pwm,
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardNUCResetPackage:
    format = '=cHcc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardNUCResetPackage.value[0]
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           bytes(self.ender, 'utf-8')
                           )


class SerialNUC2BaseboardRGBLEDPackage:
    format = '=cHcBBBBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SerialNUC2BaseboardRGBLEDPackage.value[0]
    led1state = 0
    internet_OK = 0
    daemon_OK = 0
    post_processing = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
         self.firmware_version,
         self.header,
         self.led1state,
         self.internet_OK,
         self.daemon_OK,
         self.post_processing,
         self.ender,
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.led1state,
                           self.internet_OK,
                           self.daemon_OK,
                           self.post_processing,
                           bytes(self.ender, 'utf-8')
                           )


class SocketExecutorStatePackage:
    format = '=ccBdc'
    pre_header = EXECUTOR_PACKAGE_PRE_HEADER
    header = executor_package_headers.header_SocketExecutorStatePackage.value[0]
    executor_state = 0
    time = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
            self.header,
            self.executor_state,
            self.time,
            self.ender
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.executor_state,
                           self.time,
                           bytes(self.ender, 'utf-8')
                           )


class SocketDaemon2BaseboardLinkPackage:
    format = '=cHcBBBc'
    pre_header = BASEBOARD_PACKAGE_PRE_HEADER
    firmware_version = BASEBOARD_FIRMWARE_VERSION,
    header = baseboard_package_headers.header_SocketDaemonLink2BaseboardLinkPackage.value[0]
    internet_OK = 0
    post_processing = 0
    rendering = 0
    ender = '\n'

    def __init__(self):
        self.firmware_version = BASEBOARD_FIRMWARE_VERSION  # because python bug

    def parse(self, data_bytes):
        fields = struct.unpack(self.format, data_bytes)
        (self.pre_header,
            self.firmware_version,
            self.header,
            self.internet_OK,
            self.post_processing,
            self.rendering,
            self.ender
         ) = fields

    def pack(self):
        return struct.pack(self.format,
                           bytes(self.pre_header, 'utf-8'),
                           int(self.firmware_version),
                           bytes(self.header, 'utf-8'),
                           self.internet_OK,
                           self.post_processing,
                           self.rendering,
                           bytes(self.ender, 'utf-8')
                           )
