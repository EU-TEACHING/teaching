# DIFFERENT TYPE OF SHIMMERS
SHIMMER_IMU = "IMU"
SHIMMER_GSRplus = "GSR+"
SHIMMER_ExG_0 = "ExG 0"
SHIMMER_ExG_1 = "ExG 1"
SHIMMER_BA = "BA"

# DIFFERENT SENSORS
SENSOR_LOW_NOISE_ACCELEROMETER = {"name": "Low Noise Accelerometer", "info_mem_byte": 0, "or_mask": 0x01 << 7}
SENSOR_GYROSCOPE = {"name": "Gyroscope", "info_mem_byte": 0, "or_mask": 0x01 << 6}
SENSOR_MAGNETOMETER = {"name": "Magnetometer", "info_mem_byte": 0, "or_mask": 0x01 << 5}
SENSOR_ExG1_24BIT = {"name": "ExG 1 - 24 bit", "info_mem_byte": 0, "or_mask": 0x01 << 4}
SENSOR_ExG2_24BIT = {"name": "ExG 2 - 24 bit", "info_mem_byte": 0, "or_mask": 0x01 << 3}
SENSOR_GSR = {"name": "GSR", "info_mem_byte": 0, "or_mask": 0x01 << 2}
SENSOR_EXT_EXP_ADC_CH7 = {"name": "External Expansion ADC Channel 7", "info_mem_byte": 0, "or_mask": 0x01 << 1}
SENSOR_EXT_EXP_ADC_CH6 = {"name": "External Expansion ADC Channel 6", "info_mem_byte": 0, "or_mask": 0x01 << 0}

SENSOR_BRIDGE_AMPLIFIER = {"name": "Bridge Amplifier", "info_mem_byte": 1, "or_mask": 0x01 << 7}
SENSOR_BATTERY = {"name": "Battery", "info_mem_byte": 1, "or_mask": 0x01 << 5}
SENSOR_WIDE_RANGE_ACCELEROMETER = {"name": "Wide Range Accelerometer", "info_mem_byte": 1, "or_mask": 0x01 << 4}
SENSOR_EXT_EXP_ADC_CH15 = {"name": "External Expansion ADC Channel 15", "info_mem_byte": 1, "or_mask": 0x01 << 3}
SENSOR_INT_EXP_ADC_CH1 = {"name": "Internal Expansion ADC Channel 1", "info_mem_byte": 1, "or_mask": 0x01 << 2}
SENSOR_INT_EXP_ADC_CH12 = {"name": "Internal Expansion ADC Channel 12", "info_mem_byte": 1, "or_mask": 0x01 << 1}
SENSOR_INT_EXP_ADC_CH13 = {"name": "Internal Expansion ADC Channel 13", "info_mem_byte": 1, "or_mask": 0x01 << 0}

SENSOR_INT_EXP_ADC_CH14 = {"name": "Internal Expansion ADC Channel 14", "info_mem_byte": 2, "or_mask": 0x01 << 7}
SENSOR_MPU9X50_ACCELEROMETER = {"name": "MPU9X50 Accelerometer", "info_mem_byte": 2, "or_mask": 0x01 << 6}
SENSOR_MPU9X50_MAGNETOMETER = {"name": "MPU9X50 Magnetometer", "info_mem_byte": 2, "or_mask": 0x01 << 5}
SENSOR_ExG1_16BIT = {"name": "ExG 1 - 16 bit", "info_mem_byte": 2, "or_mask": 0x01 << 4}
SENSOR_ExG2_16BIT = {"name": "ExG 2 - 16 bit", "info_mem_byte": 2, "or_mask": 0x01 << 3}
SENSOR_BMPX80_PRESSURE = {"name": "BMPX80 Pressure", "info_mem_byte": 2, "or_mask": 0x01 << 2}
SENSOR_BMPX80_TEMPERATURE = {"name": "BPMX80 Temperature", "info_mem_byte": 2, "or_mask": 0x01 << 1}
SENSOR_MSP430_TEMPERATURE = {"name": "MSP430 Temperature", "info_mem_byte": 2, "or_mask": 0x01 << 0}

# WHAT SENSOR EACH TYPE HAVE
SENSORS_FOR_TYPE = {
    SHIMMER_IMU: [SENSOR_LOW_NOISE_ACCELEROMETER, SENSOR_WIDE_RANGE_ACCELEROMETER, SENSOR_GYROSCOPE,
                  SENSOR_MAGNETOMETER, SENSOR_BMPX80_PRESSURE, SENSOR_BMPX80_TEMPERATURE, SENSOR_BATTERY],
    SHIMMER_GSRplus: [SENSOR_LOW_NOISE_ACCELEROMETER, SENSOR_WIDE_RANGE_ACCELEROMETER, SENSOR_GYROSCOPE,
                      SENSOR_MAGNETOMETER, SENSOR_BMPX80_PRESSURE, SENSOR_BMPX80_TEMPERATURE, SENSOR_BATTERY,
                      SENSOR_GSR, SENSOR_INT_EXP_ADC_CH13],
    SHIMMER_BA: [SENSOR_LOW_NOISE_ACCELEROMETER, SENSOR_WIDE_RANGE_ACCELEROMETER, SENSOR_GYROSCOPE,
                 SENSOR_MAGNETOMETER, SENSOR_BMPX80_PRESSURE, SENSOR_BMPX80_TEMPERATURE, SENSOR_BATTERY,
                 SENSOR_BRIDGE_AMPLIFIER, SENSOR_INT_EXP_ADC_CH1],
    SHIMMER_ExG_0: [SENSOR_LOW_NOISE_ACCELEROMETER, SENSOR_WIDE_RANGE_ACCELEROMETER, SENSOR_GYROSCOPE,
                    SENSOR_MAGNETOMETER, SENSOR_BMPX80_PRESSURE, SENSOR_BMPX80_TEMPERATURE, SENSOR_BATTERY,
                    SENSOR_ExG1_16BIT, SENSOR_ExG2_16BIT, SENSOR_ExG1_24BIT, SENSOR_ExG2_24BIT],
    SHIMMER_ExG_1: [SENSOR_LOW_NOISE_ACCELEROMETER, SENSOR_WIDE_RANGE_ACCELEROMETER, SENSOR_GYROSCOPE,
                    SENSOR_MAGNETOMETER, SENSOR_BMPX80_PRESSURE, SENSOR_BMPX80_TEMPERATURE, SENSOR_BATTERY,
                    SENSOR_ExG1_16BIT, SENSOR_ExG2_16BIT, SENSOR_ExG1_24BIT, SENSOR_ExG2_24BIT],
}

# POSSIBLE STATES
IDLE = 0
BT_CONNECTED = 1
BT_STREAMING = 2
BT_STREAMING_SD_LOGGING = 3
SD_LOGGING = 4

# PACKET IDENTIFIERS

DATA_PACKET = 0x00
INQUIRY_COMMAND = 0x01
INQUIRY_RESPONSE = 0x02
GET_SAMPLING_RATE_COMMAND = 0x03
SAMPLING_RATE_RESPONSE = 0x04
SET_SAMPLING_RATE_COMMAND = 0x05
TOGGLE_LED_COMMAND = 0x06
START_STREAMING_COMMAND = 0x07  # //maintain compatibility with Shimmer2/2r BtStream
SET_SENSORS_COMMAND = 0x08
SET_LSM303DLHC_ACCEL_RANGE_COMMAND = 0x09
LSM303DLHC_ACCEL_RANGE_RESPONSE = 0x0A
GET_LSM303DLHC_ACCEL_RANGE_COMMAND = 0x0B
SET_CONFIG_SETUP_BYTES_COMMAND = 0x0E
CONFIG_SETUP_BYTES_RESPONSE = 0x0F
GET_CONFIG_SETUP_BYTES_COMMAND = 0x10
SET_A_ACCEL_CALIBRATION_COMMAND = 0x11
A_ACCEL_CALIBRATION_RESPONSE = 0x12
GET_A_ACCEL_CALIBRATION_COMMAND = 0x13
SET_MPU9150_GYRO_CALIBRATION_COMMAND = 0x14
MPU9150_GYRO_CALIBRATION_RESPONSE = 0x15
GET_MPU9150_GYRO_CALIBRATION_COMMAND = 0x16
SET_LSM303DLHC_MAG_CALIBRATION_COMMAND = 0x17
LSM303DLHC_MAG_CALIBRATION_RESPONSE = 0x18
GET_LSM303DLHC_MAG_CALIBRATION_COMMAND = 0x19
SET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND = 0x1A
LSM303DLHC_ACCEL_CALIBRATION_RESPONSE = 0x1B
GET_LSM303DLHC_ACCEL_CALIBRATION_COMMAND = 0x1C
STOP_STREAMING_COMMAND = 0x20  # //maintain compatibility with Shimmer2/2r BtStream
SET_GSR_RANGE_COMMAND = 0x21
GSR_RANGE_RESPONSE = 0x22
GET_GSR_RANGE_COMMAND = 0x23
DEPRECATED_GET_DEVICE_VERSION_COMMAND = 0x24  # //maintain compatibility with Shimmer2/2r BtSt
#                                               //deprecated because 0x24 ('$' ASCII) as a com
#                                               //is problematic if remote config is enable
#                                               //RN42 Bluetooth module. Replaced with 0x3F command
DEVICE_VERSION_RESPONSE = 0x25  # //maintain compatibility with Shimmer2/2r BtStream
GET_ALL_CALIBRATION_COMMAND = 0x2C
ALL_CALIBRATION_RESPONSE = 0x2D
GET_FW_VERSION_COMMAND = 0x2E  # //maintain compatibility with Shimmer2/2r BtStream
FW_VERSION_RESPONSE = 0x2F  # //maintain compatibility with Shimmer2/2r BtStream
SET_CHARGE_STATUS_LED_COMMAND = 0x30
CHARGE_STATUS_LED_RESPONSE = 0x31
GET_CHARGE_STATUS_LED_COMMAND = 0x32
BUFFER_SIZE_RESPONSE = 0x35
GET_BUFFER_SIZE_COMMAND = 0x36
SET_LSM303DLHC_MAG_GAIN_COMMAND = 0x37
LSM303DLHC_MAG_GAIN_RESPONSE = 0x38
GET_LSM303DLHC_MAG_GAIN_COMMAND = 0x39
SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND = 0x3A
LSM303DLHC_MAG_SAMPLING_RATE_RESPONSE = 0x3B
GET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND = 0x3C
UNIQUE_SERIAL_RESPONSE = 0x3D
GET_UNIQUE_SERIAL_COMMAND = 0x3E
GET_DEVICE_VERSION_COMMAND = 0x3F
SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND = 0x40
LSM303DLHC_ACCEL_SAMPLING_RATE_RESPONSE = 0x41
GET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND = 0x42
SET_LSM303DLHC_ACCEL_LPMODE_COMMAND = 0x43
LSM303DLHC_ACCEL_LPMODE_RESPONSE = 0x44
GET_LSM303DLHC_ACCEL_LPMODE_COMMAND = 0x45
SET_LSM303DLHC_ACCEL_HRMODE_COMMAND = 0x46
LSM303DLHC_ACCEL_HRMODE_RESPONSE = 0x47
GET_LSM303DLHC_ACCEL_HRMODE_COMMAND = 0x48
SET_MPU9150_GYRO_RANGE_COMMAND = 0x49
MPU9150_GYRO_RANGE_RESPONSE = 0x4A
GET_MPU9150_GYRO_RANGE_COMMAND = 0x4B
SET_MPU9150_SAMPLING_RATE_COMMAND = 0x4C
MPU9150_SAMPLING_RATE_RESPONSE = 0x4D
GET_MPU9150_SAMPLING_RATE_COMMAND = 0x4E
SET_MPU9150_ACCEL_RANGE_COMMAND = 0x4F
MPU9150_ACCEL_RANGE_RESPONSE = 0x50
GET_MPU9150_ACCEL_RANGE_COMMAND = 0x51
SET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND = 0x52
BMPX80_PRES_OVERSAMPLING_RATIO_RESPONSE = 0x53
GET_BMPX80_PRES_OVERSAMPLING_RATIO_COMMAND = 0x54
BMP180_CALIBRATION_COEFFICIENTS_RESPONSE = 0x58
GET_BMP180_CALIBRATION_COEFFICIENTS_COMMAND = 0x59
RESET_TO_DEFAULT_CONFIGURATION_COMMAND = 0x5A
RESET_CALIBRATION_VALUE_COMMAND = 0x5B
MPU9150_MAG_SENS_ADJ_VALS_RESPONSE = 0x5C
GET_MPU9150_MAG_SENS_ADJ_VALS_COMMAND = 0x5D
SET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x5E
INTERNAL_EXP_POWER_ENABLE_RESPONSE = 0x5F
GET_INTERNAL_EXP_POWER_ENABLE_COMMAND = 0x60
SET_EXG_REGS_COMMAND = 0x61
EXG_REGS_RESPONSE = 0x62
GET_EXG_REGS_COMMAND = 0x63
SET_DAUGHTER_CARD_ID_COMMAND = 0x64
DAUGHTER_CARD_ID_RESPONSE = 0x65
GET_DAUGHTER_CARD_ID_COMMAND = 0x66
SET_DAUGHTER_CARD_MEM_COMMAND = 0x67
DAUGHTER_CARD_MEM_RESPONSE = 0x68
GET_DAUGHTER_CARD_MEM_COMMAND = 0x69
SET_BT_COMMS_BAUD_RATE = 0x6A  # //11 allowable options: 0=115.2K(default), 1=1200, 2=2400, 3=4
#                                  //4=9600, 5=19.2K, 6=38.4K, 7=57.6K, 8=230.4K, 9=460.8K, 10=92
#                                  //Need to disconnect BT connection before change is active
BT_COMMS_BAUD_RATE_RESPONSE = 0x6B
GET_BT_COMMS_BAUD_RATE = 0x6C
SET_DERIVED_CHANNEL_BYTES = 0x6D
DERIVED_CHANNEL_BYTES_RESPONSE = 0x6E
GET_DERIVED_CHANNEL_BYTES = 0x6F
START_SDBT_COMMAND = 0x70
STATUS_RESPONSE = 0x71
GET_STATUS_COMMAND = 0x72
SET_TRIAL_CONFIG_COMMAND = 0x73
TRIAL_CONFIG_RESPONSE = 0x74
GET_TRIAL_CONFIG_COMMAND = 0x75
SET_CENTER_COMMAND = 0x76
CENTER_RESPONSE = 0x77
GET_CENTER_COMMAND = 0x78
SET_SHIMMERNAME_COMMAND = 0x79
SHIMMERNAME_RESPONSE = 0x7a
GET_SHIMMERNAME_COMMAND = 0x7b
SET_EXPID_COMMAND = 0x7c
EXPID_RESPONSE = 0x7d
GET_EXPID_COMMAND = 0x7e
SET_MYID_COMMAND = 0x7F
MYID_RESPONSE = 0x80
GET_MYID_COMMAND = 0x81
SET_NSHIMMER_COMMAND = 0x82
NSHIMMER_RESPONSE = 0x83
GET_NSHIMMER_COMMAND = 0x84
SET_CONFIGTIME_COMMAND = 0x85
CONFIGTIME_RESPONSE = 0x86
GET_CONFIGTIME_COMMAND = 0x87
DIR_RESPONSE = 0x88
GET_DIR_COMMAND = 0x89
INSTREAM_CMD_RESPONSE = 0x8A
SET_CRC_COMMAND = 0x8B
SET_INFOMEM_COMMAND = 0x8C
INFOMEM_RESPONSE = 0x8D
GET_INFOMEM_COMMAND = 0x8E
SET_RWC_COMMAND = 0x8F
RWC_RESPONSE = 0x90
GET_RWC_COMMAND = 0x91
START_LOGGING_COMMAND = 0x92
STOP_LOGGING_COMMAND = 0x93
VBATT_RESPONSE = 0x94
GET_VBATT_COMMAND = 0x95
DUMMY_COMMAND = 0x96
STOP_SDBT_COMMAND = 0x97
SET_CALIB_DUMP_COMMAND = 0x98
RSP_CALIB_DUMP_COMMAND = 0x99
GET_CALIB_DUMP_COMMAND = 0x9A
UPD_CALIB_DUMP_COMMAND = 0x9B
UPD_SDLOG_CFG_COMMAND = 0x9C
ROUTINE_COMMUNICATION = 0xE0
BMP280_CALIBRATION_COEFFICIENTS_RESPONSE = 0x9F
GET_BMP280_CALIBRATION_COEFFICIENTS_COMMAND = 0xA0
ACK_COMMAND_PROCESSED = 0xFF

# CHANNEL NAMES - so user can use it while coding, type util.CHANNEL_NAME
CHANNEL_LOW_ACC_X = "Low Noise Accelerometer X"
CHANNEL_LOW_ACC_Y = "Low Noise Accelerometer Y"
CHANNEL_LOW_ACC_Z = "Low Noise Accelerometer Z"
CHANNEL_BATTERY = "Battery"
CHANNEL_WIDE_ACC_X = "Wide Noise Accelerometer X"
CHANNEL_WIDE_ACC_Y = "Wide Noise Accelerometer Y"
CHANNEL_WIDE_ACC_Z = "Wide Noise Accelerometer Z"
CHANNEL_MAG_X = "Magnetometer X"
CHANNEL_MAG_Y = "Magnetometer Y"
CHANNEL_MAG_Z = "Magnetometer Z"
CHANNEL_GYRO_X = "Gyroscope X"
CHANNEL_GYRO_Y = "Gyroscope Y"
CHANNEL_GYRO_Z = "Gyroscope Z"
CHANNEL_EXT_ADC_7 = "External ADC 7"
CHANNEL_EXT_ADC_6 = "External ADC 6"
CHANNEL_EXT_ADC_15 = "External ADC 15"
CHANNEL_INT_ADC_1 = "Internal ADC 1"
CHANNEL_INT_ADC_12 = "Internal ADC 12"
CHANNEL_INT_ADC_13 = "Internal ADC 13"
CHANNEL_INT_ADC_14 = "Internal ADC 14"
CHANNEL_BMPX80_TEMP = "BMPX80 Temperature"
CHANNEL_BMPX80_PRESS = "BMPX80 Pressure"
CHANNEL_GSR = "GSR Raw"
CHANNEL_ExG_1_STATUS = "ExG_ADS1292R_1_STATUS"
CHANNEL_ExG_1_CH1_24BIT = "ExG_ADS1292R_1_CH1_24BIT"
CHANNEL_ExG_1_CH2_24BIT = "ExG_ADS1292R_1_CH2_24BIT"
CHANNEL_ExG_2_STATUS = "ExG_ADS1292R_2_STATUS"
CHANNEL_ExG_2_CH1_24BIT = "ExG_ADS1292R_2_CH1_24BIT"
CHANNEL_ExG_2_CH2_24BIT = "ExG_ADS1292R_2_CH2_24BIT"
CHANNEL_ExG_1_CH1_16BIT = "ExG_ADS1292R_1_CH1_16BIT"
CHANNEL_ExG_1_CH2_16BIT = "ExG_ADS1292R_1_CH2_16BIT"
CHANNEL_ExG_2_CH1_16BIT = "ExG_ADS1292R_2_CH1_16BIT"
CHANNEL_ExG_2_CH2_16BIT = "ExG_ADS1292R_2_CH2_16BIT"
CHANNEL_BA_HIGH = "Bridge Amplifier High"
CHANNEL_BA_LOW = "Bridge Amplifier Low"

# INQUIRY CHANNEL NAME - BYTE VALUE CORRESPONDS --- useful for private shimmer implementaton
# how to read:
# for example, 'Gyroscope X' is identified by the byte value A, that is 10 (converted from HEX to INT).
# so, INQUIRY_SIGNAL_NAMES[10] = 'Gyroscope X'
#
# suggested read: (pag. 15-16)
#   http://shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf

INQUIRY_CHANNELS_NAMES = ['Low Noise Accelerometer X', 'Low Noise Accelerometer Y', 'Low Noise Accelerometer Z',
                          'Battery', 'Wide Noise Accelerometer X', 'Wide Noise Accelerometer Y',
                          'Wide Noise Accelerometer Z', 'Magnetometer X', 'Magnetometer Y', 'Magnetometer Z',
                          'Gyroscope X', 'Gyroscope Y', 'Gyroscope Z', 'External ADC 7', 'External ADC 6',
                          'External ADC 15', 'Internal ADC 1', 'Internal ADC 12', 'Internal ADC 13', 'Internal ADC 14',
                          'None', 'None', 'None', 'None', 'None', 'None', 'BMPX80 Temperature', 'BMPX80 Pressure',
                          'GSR Raw', 'ExG_ADS1292R_1_STATUS', 'ExG_ADS1292R_1_CH1_24BIT', 'ExG_ADS1292R_1_CH2_24BIT',
                          'ExG_ADS1292R_2_STATUS', 'ExG_ADS1292R_2_CH1_24BIT', 'ExG_ADS1292R_2_CH2_24BIT',
                          'ExG_ADS1292R_1_CH1_16BIT', 'ExG_ADS1292R_1_CH2_16BIT', 'ExG_ADS1292R_2_CH1_16BIT',
                          'ExG_ADS1292R_2_CH2_16BIT', 'Bridge Amplifier High', 'Bridge Amplifier Low']

# CHANNEL DATA TYPE
CHANNEL_DATA_TYPE = {
    "Low Noise Accelerometer X": "u12",
    "Low Noise Accelerometer Y": "u12",
    "Low Noise Accelerometer Z": "u12",
    "Battery": "u12",
    "Wide Noise Accelerometer X": "i16",
    "Wide Noise Accelerometer Y": "i16",
    "Wide Noise Accelerometer Z": "i16",
    "Magnetometer X": "i16",
    "Magnetometer Y": "i16",
    "Magnetometer Z": "i16",
    "Gyroscope X": "i16*",  # with * means Big Endian -> it should be readed in inverse order
    "Gyroscope Y": "i16*",
    "Gyroscope Z": "i16*",
    "External ADC 7": "u12",
    "External ADC 6": "u12",
    "External ADC 15": "u12",
    "Internal ADC 1": "u12",
    "Internal ADC 12": "u12",
    "Internal ADC 13": "u12",
    "Internal ADC 14": "u12",
    "BMPX80 Temperature": "u16*",
    "BMPX80 Pressure": "u24*",
    "GSR Raw": "u16",
    "ExG_ADS1292R_1_STATUS": "u8",
    "ExG_ADS1292R_1_CH1_24BIT": "i24*",
    "ExG_ADS1292R_1_CH2_24BIT": "i24*",
    "ExG_ADS1292R_2_STATUS": "u8",
    "ExG_ADS1292R_2_CH1_24BIT": "i24*",
    "ExG_ADS1292R_2_CH2_24BIT": "i24*",
    "ExG_ADS1292R_1_CH1_16BIT": "i16*",
    "ExG_ADS1292R_1_CH2_16BIT": "i16*",
    "ExG_ADS1292R_2_CH1_16BIT": "i16*",
    "ExG_ADS1292R_2_CH2_16BIT": "i16*",
    "Bridge Amplifier High": "u12",
    "Bridge Amplifier Low": "u12"
}


def calculate_data_packet_size(channels):
    """
    Only for packet that contains streamed data.

    :param channels: list of active channels
    :return: size of the packet
    """
    total_size = 0
    total_size += 1  # packet id
    total_size += 3  # timestamp
    for channel in channels:
        data_type = CHANNEL_DATA_TYPE[channel]
        if data_type == "u12":
            total_size += 2  # 12 bit --> servono almeno 2byte --> 16bit
        elif data_type == "i16":
            total_size += 2
        elif data_type == "i16*":
            total_size += 2
        elif data_type == "u16":
            total_size += 2
        elif data_type == "u24":
            total_size += 3
        elif data_type == "u8":
            total_size += 1
        elif data_type == "i24":
            total_size += 3
        elif data_type == "u24*":
            total_size += 3
        elif data_type == "i24*":
            total_size += 3
    return total_size


def calculate_data_type_size(data_type):
    if data_type == "u12":
        return 2  # 12 bit --> servono almeno 2byte --> 16bit
    elif data_type == "i16":
        return 2
    elif data_type == "i16*":
        return 2
    elif data_type == "u16":
        return 2
    elif data_type == "u16*":
        return 2
    elif data_type == "u24":
        return 3
    elif data_type == "u8":
        return 1
    elif data_type == "i24":
        return 3
    elif data_type == "u24*":
        return 3
    elif data_type == "i24*":
        return 3
    else:
        print("calculate_data_byte_size -> ERROR: Something went wrong, not recognized -> ", data_type)
        return 0


# DIFFERENT MEASUREMENT UNITS
GSR_SKIN_RESISTANCE = "Skin Resistance"
GSR_SKIN_CONDUCTANCE = "Skin Conductance"

# DIFFERENT TYPES OF RANGES

# This dict is useful to map the readden value from the GET command to the range absolute value (in units of g)
WIDE_ACC_RANGES = {
    0: 2,
    1: 4,
    2: 8,
    3: 16,
}

# These macros are useful to the user: he should use one of these whithin SET method
WIDE_ACC_RANGE_2g = 0
WIDE_ACC_RANGE_4g = 1
WIDE_ACC_RANGE_8g = 2
WIDE_ACC_RANGE_16g = 3

# ExG stuff

ExG_ECG = "ECG"
ExG_EMG = "EMG"
ExG_RESP = "RESP"
ExG_TEST = "ExG Test"

ExG_CHIP1 = 0x00  # 0
ExG_CHIP2 = 0x01  # 1

# Every ExG has two chips (ExG1 and ExG2) and every chip has two channels (ExGX_CH1 and ExGX_CH2)
# For EMG only one chip should be active: the first chip, ExG1
ExG_DATA_RATE_125 = 0x00  # 0 0 0
ExG_DATA_RATE_250 = 0x01  # 0 0 1
ExG_DATA_RATE_500 = 0x02  # 0 1 0  (recommended for EMG)
ExG_DATA_RATE_1000 = 0x03  # 0 1 1
ExG_DATA_RATE_2000 = 0x04  # 1 0 0
ExG_DATA_RATE_4000 = 0x05  # 1 0 1
ExG_DATA_RATE_8000 = 0x06  # 1 1 0

ExG_DATA_RATES = {
    125: ExG_DATA_RATE_125,
    250: ExG_DATA_RATE_250,
    500: ExG_DATA_RATE_500,
    1000: ExG_DATA_RATE_1000,
    2000: ExG_DATA_RATE_2000,
    4000: ExG_DATA_RATE_4000,
    8000: ExG_DATA_RATE_8000
}

# Those are mask, to set a certain GAIN you have
# to put in an OR operation the gain mask and the
# corresponding byte.
# [byte3/4 -> 0 gain(3 bit) mux (4 bit)]
ExG_GAIN_6 = 0x00 << 4  # 0 0 0
ExG_GAIN_1 = 0x01 << 4  # 0 0 1
ExG_GAIN_2 = 0x02 << 4  # 0 1 0
ExG_GAIN_3 = 0x03 << 4  # 0 1 1
ExG_GAIN_4 = 0x04 << 4  # 1 0 0 (recommended for ECG)
ExG_GAIN_8 = 0x05 << 4  # 1 0 1
ExG_GAIN_12 = 0x06 << 4  # 1 1 0 (recommended for EMG)

ExG_GAINS_FROM_BYTE_TO_VALUE = {
    ExG_GAIN_6: 6,
    ExG_GAIN_1: 1,
    ExG_GAIN_2: 2,
    ExG_GAIN_3: 3,
    ExG_GAIN_4: 4,
    ExG_GAIN_8: 8,
    ExG_GAIN_12: 12
}

ExG_GAINS = {
    6: ExG_GAIN_6,
    1: ExG_GAIN_1,
    2: ExG_GAIN_2,
    3: ExG_GAIN_3,
    4: ExG_GAIN_4,
    8: ExG_GAIN_8,
    12: ExG_GAIN_12
}

ExG_TEST_DC = 2  # 1 0 <- INT_TEST TEST_FREQ
ExG_TEST_1HZ_SQUARE = 3  # 1 1 <- INT_TEST TEST_FREQ
ExG_TEST_NONE = 0  # 0 0 <- INT_TEST TEST_FREQ

# Respiration Presets:
# Respiration Detection Frequency - Gain - Respiration Detection Phase
RESP_PRESET_0 = 0  # 32 kHz - 3 - 112.5°
RESP_PRESET_1 = 1  # 32 kHz - 4 - 135°
RESP_PRESET_2 = 2  # 64 kHz - 2 - 135°
RESP_PRESET_3 = 3  # 64 kHz - 3 - 157°
