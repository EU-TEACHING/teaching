"""
    Assuming LogAndStream Firmware
    Version 3 -> Shimmer3
"""

import datetime
import struct
import time

import math

import numpy as np
import serial
from serial import SerialException, SerialTimeoutException

from . import shimmer_util as util


class Shimmer3:
    """
    Shimmer3 is a module crafted to handle communication with a Shimmer3 device programmed with LogAndStreamFirmware.

    This class provide an essential set of methods/properties that can be used to interact with a Shimmer3 device.

    Nothing from ShimmerSensing is requested other than sensors (e.g. the Board is not necessary).

    The whole communication is based on Bluetooth technology (using COM ports).
    """

    def __init__(self, shimmer_type=None, debug=True):
        """ Constructor method: create a Shimmer3 device abstractation.


        :param shimmer_type: it indicates what type of Shimmer this object will represent (e.g. IMU, GSR+, ExG...)
                             Check the back of your device to discover what is its type. There are some constants
                             that defines all different types of devices in the "shimmer.util.py" module.
                             e.g. SHIMMER_GSR+, SHIMMER_IMU...
        :param debug: if True, there will be some prints that suggests the current behavior/state of the computation.
        :type shimmer_type: string
        :type debug: bool

        :returns: Shimmer3 object
        :rtype: Object

        :Example:

        >>> from shimmer.shimmer import Shimmer3
        >>> from shimmer import util
        >>> shimmer = Shimmer3(shimmer_type=util.SHIMMER_IMU, debug=False)

        .. warning:: **this implementation assumes that you are using Shimmer3 devices with LogAndStream firmware on.**
        """

        # CONNECTION ATTRIBUTES

        self._current_state = util.IDLE
        """The state in which the device is, defined as a constant (see "shimmer.util.py" file).\n
            Available states are five: *IDLE*, *BT CONNECTED*, *BT STREAMING*, *SD LOGGING*, *BT STREAMING + SD LOGGING*
            \n
            When a device is power on or reset it enters in *IDLE* state.
            If Undock Start is enabled the device will progress directly from *IDLE* state to *SD LOGGING* state.\n
            If not, it move to different states based on the actions performed (e.g. open serial port -> *BT CONNECTED*
            \n
            
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pgs. 6-7-8
        """

        self._com_port = None
        """The serial port (COM port) on which the device is connected to the host"""

        self._serial_port = None
        """The serial port **interface** on which the communication takes place"""

        self._sr_number = None
        """The daughter card ID"""

        self._sr_rev = None
        """The revision number"""

        self._hardware_compatibility_code = None
        """The hardware compatibility code. It is useful for checking Shimmer version"""

        # SENSORS ATTRIBUTES

        self._enabled_sensors = []
        """List of enabled sensors (on the Shimmer device)"""
        self._channels = []
        """List of channels that make up a package data"""
        self._sampling_rate = None
        """Device sampling rate"""
        self._buffer_size = 1
        """Buffer size used in the device. Assuming always 1"""
        self._num_channels = -1  # Number of active channels == number of enabled sensors
        """Number of active channels, it is useful to calculate packet size"""

        self._rtc_ticks = None
        """Real time clock info contained inside the device, in form of ticks"""
        self._rtc_milliseconds = None
        """Real time clock info contained inside the device, in form of **milliseconds**"""

        # SETTINGS ATTRIBUTES - not yet used...

        self._configbytes0 = None
        """
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pgs. 35-36-37
        """
        self._configbytes1 = None
        """
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pgs. 35-36-37
        """
        self._configbytes2 = None
        """
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pgs. 35-36-37
        """
        self._configbytes3 = None
        """
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pgs. 35-36-37
        """

        # STREAM ATTRIBUTES

        self._clock_overflows = 0  # how much time a loop restart occurs in timestamp...
        """Number of time that the timestamp return to 0 within a stream (the value swings between 0 and 2^24)"""
        self._previous_timestamp = -1
        """Timestamp of the last packet received from the device"""
        self._first_local_timestamp_of_a_stream = -1
        """First timestamp of a stream: timestamp of the first packet received after **connect()** invocation.
            It is the real local timestamp of the first packet -> serve per poter per ogni pacchetto calcolare il tempo
            trascorso in secondi (reali, locali!) dal primo pacchetto.
        """
        self._first_unix_timestamp_of_a_stream = -1
        """The exact (RTC) time, in **seconds**, at which it was **received** (on **PC**) the first packet. It will
            be useful for synchronization purposes"""

        self._overflow_data = "".encode()
        """Contains the data that was too much in relation to the package to read. These data will be the starting
           point for the next read."""
        self._buffer_data = "".encode()
        """Similar to 'overflow data', but useful for the extended version of the reading method"""

        # WIDE RANGE ACCELEROMETER ATTRIBUTES
        self._wide_acc_range = None
        """This is the sampling range of the Wide Range Accelerometer sensor. 
        Mapping values are:
            - 0 -> +/- 2g
            - 1 -> +/- 4g
            - 2 -> +/- 8g
            - 3 -> +/- 16g
        However, this attribute will contains 2, 4, 8 or 16. 
        This value is such an important issue during the signals' calibration. 
        """

        self._exg_gain = None
        """Assuming that in both chip and both channels the gain is the same"""

        self._exg_purpose = None
        """It indicates what's the use of the ExG. If the device is not an ExG this property is None"""

        # CALIBRATION ATTRIBUTES

        self._active_gsr_mu = util.GSR_SKIN_CONDUCTANCE  # mu = measurement unit
        """The current measurement unit for GSR data"""

        self._low_accelerator_calibration = {
            "offset": np.array([[2082, 2081, 2087]]).T,
            "rx_inv": np.array([[-0.000599640215870, -1.000599640215871, -0.009994003597841],
                                [-0.998800719568259, 0.001199280431741, 0.019988007195683],
                                [-0.059964021587048, -0.059964021587048, -0.999400359784130]]),
            "kx_inv": np.array([[1 / 84, 0, 0], [0, 1 / 83, 0], [0, 0, 1 / 84]]),
        }
        """Calibration data for the low noise accelerator raw signals. \n
            The conversion is defined as follows:
                - c -> vector of calibrated data
                - u -> vector of UNcalibrated data  
                - b -> vector of Offset
                - rx -> alignment matrix
                - kx -> sensitivity matrix
                
                \n**c = rx_inv * kx_inv * (u - b)**
                
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/Shimmer_9DOF_Calibration_User_Manual_rev2.10a.pdf \n
            pgs. 34-35-36
        """

        self._wide_accelerator_calibration = {
            2: {
                "offset": np.array([[0, 0, 0]]).T,
                "rx_inv": np.array([[-1, 0, 0],
                                    [0, -1, 0],
                                    [0, 0, 1]]),
                "kx_inv": np.array([[1 / 1671, 0, 0], [0, 1 / 1671, 0], [0, 0, 1 / 1671]]),
            },
            4: {
                "offset": np.array([[0, 0, 0]]).T,
                "rx_inv": np.array([[-1, 0, 0],
                                    [0, -1, 0],
                                    [0, 0, 1]]),
                "kx_inv": np.array([[1 / 836, 0, 0], [0, 1 / 836, 0], [0, 0, 1 / 836]]),
            },
            8: {
                "offset": np.array([[0, 0, 0]]).T,
                "rx_inv": np.array([[-1, 0, 0],
                                    [0, -1, 0],
                                    [0, 0, 1]]),
                "kx_inv": np.array([[1 / 418, 0, 0], [0, 1 / 418, 0], [0, 0, 1 / 418]]),
            },
            16: {
                "offset": np.array([[0, 0, 0]]).T,
                "rx_inv": np.array([[-1, 0, 0],
                                    [0, -1, 0],
                                    [0, 0, 1]]),
                "kx_inv": np.array([[1 / 209, 0, 0], [0, 1 / 209, 0], [0, 0, 1 / 209]]),
            }
        }
        """Calibration data for the wide range accelerator raw signals. \n
            The conversion is defined as follows:
                - c -> vector of calibrated data
                - u -> vector of UNcalibrated data  
                - b -> vector of Offset
                - rx -> alignment matrix
                - kx -> sensitivity matrix

                \n**c = rx_inv * kx_inv * (u - b)**
            
            Remember this mapping's settings for the Range value:
                - 0 -> +/- 2g
                - 1 -> +/- 4g
                - 2 -> +/- 8g
                - 3 -> +/- 16g
            
        .. seealso:: 
            http://www.shimmersensing.com/images/uploads/docs/Shimmer_9DOF_Calibration_User_Manual_rev2.10a.pdf \n
            pgs. 34-35-36
        """

        # UTIL ATTRIBUTES

        self._shimmer_type = shimmer_type
        """Type of the shimmer *linked* to this object"""

        self.debug = debug
        """If True, there will be some useful prints during the executions of all methods"""

        # packet losses
        self._previous_calibrated_timestamp = None

    # PRIVATE methods

    def _wait_for_ack(self):
        """
        This methods wait for an ACK message from the device.
        The device sends a message of this type when it wants to confirm the receipt of a command (e.g. *SET_SENSORS*)

        :return: if the ACK has been correctly received by the host (computer)
        :rtype: bool
        """
        ack = struct.pack("B", util.ACK_COMMAND_PROCESSED)
        ddata = ""
        while ddata != ack:
            ddata = self._serial_port.read(1)
        return True

    # CONNECTION methods

    @property
    def current_state(self):
        """
        - 0 -> *IDLE*
        - 1 -> *BT_CONNECTED*
        - 2 -> *BT_STREAMING*
        - 3 -> *BT_STREAMING_SD_LOGGING*
        - 4 -> *SD_LOGGING*

        .. seealso::
            the module *shimmer.util.py*

        :return: the current state in which the device is.
        :rtype: int
        """
        return self._current_state

    def connect(self, com_port=None, write_rtc=True, update_all_properties=True, reset_sensors=True):
        """
        Start the connection between host (pc) and the Shimmer3. This means opening COM Port.

        :param com_port: serial port on which the devices id connected.
        :param update_all_properties: it indicates if after the connection all properties of the object should be
                                    readden (from the Shimmer3 device) and updated.
        :param write_rtc: it indicates if after the connection the RTC (real time clock) should be written from the
                          host into the Shimmer3 device.
        :param reset_sensors: it indicates if after the connection the enabled sensors of the Shimmer must be reset.
        :type com_port: str
        :type update_all_properties: bool
        :type write_rtc: bool
        :type reset_sensors: bool
        :return: True, if the connection was made correctly, False otherwise.
        :rtype: bool
        """
        if self._current_state == util.IDLE or self._current_state == util.SD_LOGGING:
            # If here, the Shimmer3 is not already connected.
            try:
                # Serial Port opening: here the connection happens
                self._serial_port = serial.Serial(com_port)
                # Flush serial port inputs (good standard)
                self._serial_port.reset_input_buffer()

                # Updating object properties
                self._current_state = util.BT_CONNECTED
                self._com_port = com_port

                if self.debug:
                    print("connect -> Succesfull connection within port ", self._com_port)

                result = True

                if write_rtc:
                    # Eventually, write RTC
                    result = self.write_real_time_clock()
                if result and update_all_properties:
                    # Eventually, update all object properties
                    result = self.update_all_properties(print_reads=False)

                if result and reset_sensors:
                    # Eventually, resetting the enabled sensors
                    result = self.set_enabled_sensors()
                    # Default choice...
                    self._active_gsr_mu = util.GSR_SKIN_CONDUCTANCE
                return result
            except (SerialException, SerialTimeoutException) as exception:
                if self.debug:
                    print("connect -> ERROR: a problem occurs during the connection within port: ", com_port)
                print("connect -> EXCEPTION: ", exception)
                return False
        else:
            if self.debug:
                print("connect -> ERROR: the Shimmer is already connected via BT.")

    def disconnect(self, reset_obj_to_init=True):
        """
        Stop the connection between host and Shimmer3 device.

        :return: True, if the connection was succesfully stopped, False otherwise.
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED or self._current_state == util.BT_STREAMING or \
                self._current_state == util.BT_STREAMING_SD_LOGGING:
            # The above clause check if the Shimmer3 is connected
            try:
                # Closing serial port
                self._serial_port.close()
                self._serial_port = None
                # Updating current state
                if self._current_state == util.BT_STREAMING_SD_LOGGING:
                    self._current_state = util.SD_LOGGING
                else:
                    self._current_state = util.IDLE

                self._com_port = None

                if self.debug:
                    print("disconnect -> Succesful disconnection. Current state: ", self._current_state)

                # Resetting obj properties to the init values. Those will be readden again at the next
                # connection or update.
                if reset_obj_to_init:
                    self._reset_to_init()

                return True
            except (SerialException, SerialTimeoutException) as exception:
                if self.debug:
                    print("disconnect -> ERROR: a problem occurs during port closing.")
                print(exception)
                return False
        else:
            if self.debug:
                print(
                    "disconnect -> ERROR: you have to establish a connection with the Shimmer before using this "
                    "command.")
            return False

    def start_bt_streaming(self):
        """
        Send the START_STREAMING command to the Shimmer3 device. After receiving it, the device will start to
        send data packet.

        :return: True, if the command arrives to the Shimmer3 (and it responds with ACK), False otherwise.
        :rtype: bool
        """

        if self._current_state == util.BT_CONNECTED:
            # Flush serial port inputs (good standard)
            self._serial_port.reset_input_buffer()
            # It is advisable to send RTC clock right before starting streaming
            self.write_real_time_clock()
            packet = struct.pack("B", util.START_STREAMING_COMMAND)
            try:
                self._serial_port.write(packet)
                self._wait_for_ack()
                if self.debug:
                    print("start_bt_streaming -> ACK received. Shimmer started streaming data.")
                # Updating the current state, from CONNECTED to STREAMING
                self._current_state = util.BT_STREAMING
                return True
            except (SerialException, SerialTimeoutException) as e:
                if self.debug:
                    print("start_bt_streaming-> ERROR: something went wrong within serial communication.")
                print(e)
                return False
        else:
            if self.debug:
                print("start_bt_streaming -> ERROR: the Shimmer has to be in 'BT Connected' state to start streaming.")
            return False

    def stop_bt_streaming(self):
        """
        Send the STOP_STREAMING command to the Shimmer3 device. After receiving it, the device will stop sending
        data packet.

        :return: True, if the command arrives to the Shimmer3 (and it responds with ACK), False otherwise.
        :rtype: bool
        """
        if self._current_state == util.BT_STREAMING or self._current_state == util.BT_STREAMING_SD_LOGGING:
            packet = struct.pack("B", util.STOP_STREAMING_COMMAND)
            try:
                self._serial_port.write(packet)
                if self._wait_for_ack():
                    if self.debug:
                        print("stop_bt_stremaing -> Shimmer stopped streaming session.")
                    # Updating state
                    if self._current_state == util.BT_STREAMING:
                        self._current_state = util.BT_CONNECTED
                    else:
                        self._current_state = util.SD_LOGGING

                    # A stream is over: all these parameters are relative to a single stream, and useful to manage
                    # the timestamp synchronization of it
                    self._clock_overflows = 0  # How many times the local timestamp exceed 2^24
                    self._previous_timestamp = -1  # Useful to check if it occurs a clock overflow
                    self._first_local_timestamp_of_a_stream = -1  # Useful to know elapsed time between packets
                    self._first_unix_timestamp_of_a_stream = -1  # RTC time (receipt time) of the first packet

                    self._buffer_data = "".encode()
                    self._overflow_data = "".encode()

                    return True
                else:
                    if self.debug:
                        print("stop_bt_streaming -> ERROR: something went wrong during ACK receipt")
                    return False
            except (SerialException, SerialTimeoutException) as e:
                if self.debug:
                    print("stop_bt_streaming -> ERROR: something went wrong within serial communication.")
                print(e)
                return False
        else:
            if self.debug:
                print("stop_bt_streaming -> ERROR: the Shimmer is not streaming data.")

    @property
    def com_port(self):
        """
        :return: the serial port on which the Shimmer is connected to the host.
        :rtype: str
        """
        return self._com_port

    # READING DATA

    def data_packet_header(self):
        """
        Build and return the header of the packet: it describe how it is formatted.

        For example: \n
        *[Local Timestamp, Start Stream Timestamp (RTC), Current Packet Timestamp (RTC), Channel 1, ... Channel X]*

        :return: the header that describe how the packet is formatted
        :rtype: list
        """
        # These are "constant"
        header = ["Local Timestamp", "Start Stream Timestamp (RTC)", "Current Packet Timestamp (RTC)"]
        # Appending all different channels enabled
        for channel in self._channels:
            header.append(channel)
        return header

    def read_data_packet_bt(self, calibrated=True):
        """
        Read the oldest packet in the receipt buffer of the connection.

        The packet is an array of that type:\n
        *[timestamp, timeRTCstart, timeRTCcurrent, sensor1 data, sensor2 data, sensor3 data, ...]*

        :param calibrated: if data should be calibrated or not
        :type calibrated: bool
        :return: the oldest packet in the receipt buffer
        :rtype: list
        """
        if self._current_state == util.BT_STREAMING or self._current_state == util.BT_STREAMING_SD_LOGGING:
            # Compute the size that a packet should have
            packet_size = util.calculate_data_packet_size(self._channels)
            # Recover old data
            ddata = self._overflow_data
            readden_byte = len(ddata)

            while readden_byte < packet_size:
                # Read data
                ddata += self._serial_port.read(packet_size)
                readden_byte = len(ddata)

            # Retrieve a packet
            data = ddata[0:packet_size]
            ddata = ddata[packet_size:]
            self._overflow_data = ddata

            # Now in 'data' we have the packet data
            #
            #   data[0] = ID packet data -> \x00
            #   data[1:4] = timestamp
            #   data[4:packet_size] = sensors data

            packet = []

            (timestamp0, timestamp1, timestamp2) = struct.unpack("BBB", data[1:4])
            timestamp = timestamp0 + timestamp1 * 256 + timestamp2 * 65536

            # Check for overflows of timestamps (when it goes over 2^24)
            if self._previous_timestamp != -1 and timestamp < self._previous_timestamp:
                self._clock_overflows += 1
            self._previous_timestamp = timestamp

            timestamp = timestamp + self._clock_overflows * 16777216

            # Update info of the stream
            if self._first_local_timestamp_of_a_stream == -1:
                self._first_local_timestamp_of_a_stream = timestamp
            if self._first_unix_timestamp_of_a_stream == -1:
                self._first_unix_timestamp_of_a_stream = time.time()

            packet.append(timestamp)  # timestamp locale
            packet.append(self._first_unix_timestamp_of_a_stream)
            packet.append(self._first_unix_timestamp_of_a_stream + self.calibrate_timestamp_time_elapsed(timestamp)[0])

            # Reading channels data
            start_index = 4
            # This array will be used for Accelerometer/Gyroscope/Magnetometer calibration
            tmp_array = []
            for i in range(self._num_channels):
                current_channel = self._channels[i]
                current_channel_data_type = util.CHANNEL_DATA_TYPE[current_channel]
                byte_size_data_type = util.calculate_data_type_size(current_channel_data_type)
                interested_data = data[start_index: start_index + byte_size_data_type]
                start_index = start_index + byte_size_data_type
                raw_data = -1
                # Convert interested_data into RAW data (byte to int)
                if current_channel_data_type == "u12":
                    raw_data = struct.unpack("<H", interested_data)[0]
                elif current_channel_data_type == "i16":
                    raw_data = struct.unpack("<H", interested_data)[0]
                elif current_channel_data_type == "i16*":
                    raw_data = struct.unpack(">H", interested_data)[0]
                elif current_channel_data_type == "u16":
                    raw_data = struct.unpack("H", interested_data)[0]
                elif current_channel_data_type == "u24":
                    raw_data = struct.unpack('<I', (data[5:8] + '\0'.encode()))[0] >> 8
                elif current_channel_data_type == "u8":
                    raw_data = struct.unpack("B", interested_data)[0]
                elif current_channel_data_type == "i24":
                    raw_data = struct.unpack('<i', (data[5:8] + '\0'.encode()))[0] >> 8
                elif current_channel_data_type == "u24*":
                    raw_data = struct.unpack('>I', (data[5:8] + '\0'.encode()))[0]
                elif current_channel_data_type == "i24*":
                    raw_data = struct.unpack('>i', (data[5:8] + '\0'.encode()))[0]

                # Now we want to calibrate data (if 'calibrated' is True)
                if not calibrated:
                    packet.append(raw_data)
                else:
                    if current_channel == util.CHANNEL_WIDE_ACC_X or current_channel == util.CHANNEL_LOW_ACC_X or \
                            current_channel == util.CHANNEL_GYRO_X or current_channel == util.CHANNEL_MAG_X:
                        tmp_array.append(raw_data)
                    elif current_channel == util.CHANNEL_WIDE_ACC_Y or current_channel == util.CHANNEL_LOW_ACC_Y or \
                            current_channel == util.CHANNEL_GYRO_Y or current_channel == util.CHANNEL_MAG_Y:
                        tmp_array.append(raw_data)
                    elif current_channel == util.CHANNEL_WIDE_ACC_Z or current_channel == util.CHANNEL_LOW_ACC_Z or \
                            current_channel == util.CHANNEL_GYRO_Z or current_channel == util.CHANNEL_MAG_Z:
                        tmp_array.append(raw_data)
                        # ora abbiamo x, y e z tutti raw in una lista [x, y, z]
                        # dobbiamo capire che sensore è in generale, stavolta precisamente
                        if current_channel == util.CHANNEL_LOW_ACC_Z:
                            calibrated_data = self.calibrate_low_acc_vector(tmp_array)
                            packet = packet + calibrated_data
                        else:
                            print("read_data -> not supported yet")
                            packet = packet + tmp_array

                        tmp_array = []
                    elif current_channel == util.CHANNEL_GSR:
                        calibrated_data = self.calibrate_gsr(raw_data)
                        packet.append(calibrated_data)
                    elif current_channel == util.CHANNEL_INT_ADC_13 and self._shimmer_type == util.SHIMMER_GSRplus:
                        calibrated_data = self.calibrate_ppg(raw_data)
                        packet.append(calibrated_data)
                    elif current_channel == util.CHANNEL_ExG_1_CH1_24BIT or \
                            current_channel == util.CHANNEL_ExG_1_CH2_24BIT \
                            or current_channel == util.CHANNEL_ExG_2_CH1_24BIT or \
                            current_channel == util.CHANNEL_ExG_2_CH2_24BIT:
                        calibrated_data = self.calibrate_exg_24bit(raw_data)
                        packet.append(calibrated_data)
                    elif current_channel == util.CHANNEL_ExG_1_STATUS or current_channel == util.CHANNEL_ExG_2_STATUS:
                        # Those aren't calibrated
                        pass
                    else:
                        print("read data -> not supported yet: ", current_channel)
                        packet.append(raw_data)
            return packet
        else:
            if self.debug:
                print("reading_data_packet_bt -> ERROR: the Shimmer is not streaming data.")
            return None

    def read_data_packet_extended(self, calibrated=True):
        """
        Read all packets in the receipt buffer of the connection.

        The packets are a arrays of that type:\n
        *[timestamp, timeRTCstart, timeRTCcurrent, sensor1 data, sensor2 data, sensor3 data, ...]* \n
        And packets are returned like: \n
        *[oldest_packet, packet1, packet2, ..., newest_packet]*

        :param calibrated: if data should be calibrated or not
        :type calibrated: bool
        :return: all the packets in the receipt buffer
        :rtype: list
        """
        if self._current_state == util.BT_STREAMING or self._current_state == util.BT_STREAMING_SD_LOGGING:
            # Compute the size that a packet should have
            packet_size = util.calculate_data_packet_size(self._channels)
            ddata = self._buffer_data

            # Check of much data are in the receipt buffer
            bytes_in_port = self._serial_port.inWaiting()

            if bytes_in_port > 0:
                # If there are data it will read all
                ddata += self._serial_port.read(bytes_in_port)
                # Now we have to check if we can infers packets from the data that we have
                remaining_data = len(ddata)

                packets = []
                n_of_packets = 0
                while remaining_data > packet_size:
                    # That code is equal to 'read_data_packet_bt'. Consider to read the above method
                    data = ddata[0:packet_size]
                    ddata = ddata[packet_size:]
                    remaining_data = len(ddata)

                    n_of_packets += 1

                    packet = []

                    (timestamp0, timestamp1, timestamp2) = struct.unpack("BBB", data[1:4])
                    timestamp = timestamp0 + timestamp1 * 256 + timestamp2 * 65536

                    # Check for overflows
                    if self._previous_timestamp != -1 and timestamp < self._previous_timestamp:
                        self._clock_overflows += 1
                    self._previous_timestamp = timestamp

                    timestamp = timestamp + self._clock_overflows * 16777216

                    if self._first_local_timestamp_of_a_stream == -1:
                        self._first_local_timestamp_of_a_stream = timestamp
                    if self._first_unix_timestamp_of_a_stream == -1:
                        self._first_unix_timestamp_of_a_stream = time.time()  # in seconds

                    packet.append(timestamp)
                    packet.append(self._first_unix_timestamp_of_a_stream)
                    calibrated_timestamp = self._first_unix_timestamp_of_a_stream + \
                                           self.calibrate_timestamp_time_elapsed(timestamp)[0]
                    packet.append(calibrated_timestamp)

                    if self._previous_calibrated_timestamp is not None:
                        # Check for packet losses
                        delta = calibrated_timestamp - self._previous_calibrated_timestamp
                        if delta > (1 / self._sampling_rate):
                            print("PACKET LOSS! Registered delta: ", delta)

                    self._previous_calibrated_timestamp = calibrated_timestamp

                    start_index = 4
                    tmp_array = []
                    for i in range(self._num_channels):
                        current_channel = self._channels[i]
                        current_channel_data_type = util.CHANNEL_DATA_TYPE[current_channel]
                        byte_size_data_type = util.calculate_data_type_size(current_channel_data_type)
                        interested_data = data[start_index: start_index + byte_size_data_type]
                        start_index = start_index + byte_size_data_type
                        raw_data = -1

                        if current_channel_data_type == "u12":
                            raw_data = struct.unpack("<H", interested_data)[0]
                        elif current_channel_data_type == "i16":
                            raw_data = struct.unpack("<H", interested_data)[0]
                        elif current_channel_data_type == "i16*":
                            raw_data = struct.unpack(">H", interested_data)[0]
                        elif current_channel_data_type == "u16":
                            raw_data = struct.unpack("H", interested_data)[0]
                        elif current_channel_data_type == "u24":
                            (raw_data0, raw_data1, raw_data2) = struct.unpack("BBB", interested_data)
                            raw_data = raw_data0 + raw_data1 * 256 + raw_data2 * 65536
                        elif current_channel_data_type == "u8":
                            raw_data = struct.unpack("B", interested_data)[0]
                        elif current_channel_data_type == "i24":
                            raw_data = struct.unpack('>i', (data[5:8] + '\0'.encode()))[0] >> 8
                        elif current_channel_data_type == "u24*":
                            (raw_data0, raw_data1, raw_data2) = struct.unpack("BBB", interested_data)
                            raw_data = raw_data2 + raw_data1 * 256 + raw_data0 * 65536
                        elif current_channel_data_type == "i24*":
                            raw_data = struct.unpack('>i', (data[5:8] + '\0'.encode()))[0] >> 8

                        if not calibrated:
                            packet.append(raw_data)
                        else:
                            if current_channel == util.CHANNEL_WIDE_ACC_X or current_channel == util.CHANNEL_LOW_ACC_X or \
                                    current_channel == util.CHANNEL_GYRO_X or current_channel == util.CHANNEL_MAG_X:
                                tmp_array.append(raw_data)
                            elif current_channel == util.CHANNEL_WIDE_ACC_Y or current_channel == util.CHANNEL_LOW_ACC_Y or \
                                    current_channel == util.CHANNEL_GYRO_Y or current_channel == util.CHANNEL_MAG_Y:
                                tmp_array.append(raw_data)
                            elif current_channel == util.CHANNEL_WIDE_ACC_Z or current_channel == util.CHANNEL_LOW_ACC_Z or \
                                    current_channel == util.CHANNEL_GYRO_Z or current_channel == util.CHANNEL_MAG_Z:
                                tmp_array.append(raw_data)
                                # ora abbiamo x, y e z tutti raw in una lista [x, y, z]
                                # dobbiamo capire che sensore è in generale, stavolta precisamente
                                if current_channel == util.CHANNEL_LOW_ACC_Z:
                                    calibrated_data = self.calibrate_low_acc_vector(tmp_array)
                                    packet = packet + calibrated_data
                                else:
                                    # print("read_data -> not supported yet")
                                    packet = packet + tmp_array

                                tmp_array = []
                            elif current_channel == util.CHANNEL_GSR:
                                calibrated_data = self.calibrate_gsr(raw_data)
                                packet.append(calibrated_data)
                            elif current_channel == util.CHANNEL_INT_ADC_13 and self._shimmer_type == util.SHIMMER_GSRplus:
                                calibrated_data = self.calibrate_ppg(raw_data)
                                packet.append(calibrated_data)
                            elif current_channel == util.CHANNEL_ExG_1_CH1_24BIT or \
                                    current_channel == util.CHANNEL_ExG_1_CH2_24BIT \
                                    or current_channel == util.CHANNEL_ExG_2_CH1_24BIT or \
                                    current_channel == util.CHANNEL_ExG_2_CH2_24BIT:
                                calibrated_data = self.calibrate_exg_24bit(raw_data)
                                packet.append(calibrated_data)
                            elif current_channel == util.CHANNEL_ExG_1_STATUS or current_channel == util.CHANNEL_ExG_2_STATUS:
                                pass
                            else:
                                packet.append(raw_data)
                    packets.append(packet)
                # The remaining data we should use at the next read
                self._buffer_data = ddata

                return n_of_packets, packets
            else:
                return 0, []
        else:
            if self.debug:
                print("reading_data_packet_extended -> ERROR: the Shimmer is not streaming data.")
            return None

    # INIT & UPDATE (object) properties and methods

    @property
    def sr_number(self):
        """
        :return: The ID of the Shimmer's daughter card
        :rtype: int
        """
        return self._sr_number

    @property
    def sr_rev(self):
        """
        :return: The revision number of the Shimmer's daughter card
        :rtype: int
        """
        return self._sr_rev

    def update_all_properties(self, print_reads=True):
        """
        This method update all (Python) object properties with the corrects (Shimmer) values.

        :param print_reads: it indicates whether the method should print readden properties.
        :return: True if the operation was successful, False otherwise.
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if print_reads:
                print("*********** READING (and UPDATING) OF ALL PROPERTIES ***********")

            # First of all, reading Daughter Card ID and the revision number
            self._serial_port.write(struct.pack('BBB', util.GET_DAUGHTER_CARD_ID_COMMAND, 0x02, 0x00))
            if not self._wait_for_ack():
                return False

            ddata = list(struct.unpack('BBBB', self._serial_port.read(4)))
            self._sr_number = ddata[2]
            self._sr_rev = ddata[3]

            # Hardware Compatibility Code reading - it is similar to SR but i took that from MATLAB
            # implementation (more complete)
            if not self.get_hardware_compatibility_code():
                if self.debug:
                    print(
                        "update_all_properties -> ERROR: something went wrong during HW Compatibility code reading...")
                return False

            # Inquiry Commands
            # Notice that with 'write=True' it goes to write all (Python) object properties that Inquiry reads.
            if not self.inquiry(write=True, print_reads=print_reads):
                if self.debug:
                    print("update_all_properties -> ERROR: INQUIRY commands failed.")
                return False
            # Enabled Sensors - must be done after inquiry (cause inquiry gets CHANNELS)
            if not self.get_enabled_sensors():
                if self.debug:
                    print("update_all_properties -> ERROR: GET SENSORS commands failed")
                return False

            if not self.read_real_time_clock(print_reads=print_reads):  # RealTimeClock settings
                if self.debug:
                    print("update_all_properties -> ERROR: READ RTC commands failed.")
                return False

            if print_reads:
                print("*********** *********** *********** *********** ***********")

            return True
        else:
            if self.debug:
                print("ERROR: you have to establish a connection with the Shimmer before using this command.")
            return False

    def get_hardware_compatibility_code(self):
        """
        Read the Hardware Compability code of the Shimmer device
        :return:
        """
        # We can assume that Shimmer3 have an HW Compatibility Code of 2
        if self._current_state == util.BT_CONNECTED:
            packet = struct.pack("B", util.GET_DAUGHTER_CARD_ID_COMMAND)
            if not self._serial_port.write(packet):
                if self.debug:
                    print("get_hardware_compatibility_code -> ERROR: something went wrong during writing.")
                return False
            # I want to read 3 byte
            packet = struct.pack("B", 3)
            if not self._serial_port.write(packet):
                if self.debug:
                    print("get_hardware_compatibility_code -> ERROR: something went wrong during writing.")
                return False
            # I want to read 3 byte starting from the 0 (from the beginning)
            packet = struct.pack("B", 0)
            if not self._serial_port.write(packet):
                if self.debug:
                    print("get_hardware_compatibility_code -> ERROR: something went wrong during writing.")
                return False

            if not self._wait_for_ack():
                if self.debug:
                    print("get_hardware_compatibility_code -> ERROR: something went wrong during ACK receiving.")
                return False

            data = self._serial_port.read(5)
            packet_id, n_of_bytes, ID_byte_0, ID_byte_1, ID_byte_2 = struct.unpack("BBBBB", data)

            # That piece of code is from MATLAB Instrument Driver of Shimmer
            if packet_id == util.DAUGHTER_CARD_ID_RESPONSE:
                hw_code = 1
                if ID_byte_0 == 8 or ID_byte_0 == 14 or ID_byte_0 == 37:
                    if ID_byte_2 == 171:
                        hw_code = 2
                elif ID_byte_0 == 31:
                    if ID_byte_1 >= 6:
                        hw_code = 2
                elif ID_byte_0 == 36 or ID_byte_0 == 38:
                    if ID_byte_1 < 3:
                        if ID_byte_2 == 171:
                            hw_code = 2
                    else:
                        hw_code = 2
                elif ID_byte_0 == 47:
                    if ID_byte_1 >= 3:
                        hw_code = 2
                elif ID_byte_0 == 48:
                    if ID_byte_1 >= 3:
                        hw_code = 2
                elif ID_byte_0 == 49:
                    if ID_byte_1 >= 2:
                        hw_code = 2
                elif ID_byte_0 == 59:
                    hw_code = 2

                self._hardware_compatibility_code = hw_code

                if self.debug:
                    print("get_hardware_compatibility_code -> Hardware Compatibility code readden.")

                return True
            else:
                if self.debug:
                    print("get_hardware_compatibility_code -> response ID not compatible: ", packet_id)
                return False
        else:
            if self.debug:
                print("get_hardware_compatibility_code -> ERROR: the Shimmer is not connected...")
            return False

    # INQUIRY
    def inquiry(self, write=True, print_reads=True):
        """ This method send to the Shimmer the INQUIRY command.

        After receiving, it responds with the Inquiry Packet. It contains some useful info as:
            - Sampling Rate
            - Config Bytes
            - Number of channels
            - Buffer Size
            - All channels
            \n
            \nInquiry Packet Structure:\n
            Byte    |     0      |      1-2      |     3-6     |   7     |   8        |   9   |   10  |  ...  |   x   |
            Value   | PacketType | Sampling Rate | ConfigBytes | N_Chans | BufferSize | Chan1 | Chan2 |  ...  | ChanX |

        .. seealso::
            http://shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pg. 15 - 16

        :param write: it indicates if the readden properties should be written to the (Python) object attribute
        :param print_reads: it indicates if the readden properties should be printed to the console
        :type write: bool
        :type print_reads: bool
        :return: True in case of succesfull operation, False otherwise.
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            packet = struct.pack("B", util.INQUIRY_COMMAND)
            self._serial_port.write(packet)
            self._wait_for_ack()
            # Now Shimmer should send the real response
            ddata = self._serial_port.read(9)  # First 10 bytes are other properties (read the docs of this method)
            packet_type = struct.unpack("B", ddata[0:1])[0]
            if packet_type == util.INQUIRY_RESPONSE:
                # Reading Properties
                sampling_rate = round(32768.0 / struct.unpack("H", ddata[1:3])[0], 2)  # Reading and rounding Hz
                # Reading confibytes -> they contains some useful information (not yet used)
                configbytes0 = struct.unpack("B", ddata[3:4])
                configbytes1 = struct.unpack("B", ddata[4:5])
                configbytes2 = struct.unpack("B", ddata[5:6])
                configbytes3 = struct.unpack("B", ddata[6:7])
                num_channels = struct.unpack("B", ddata[7:8])[0]
                buffer_size = struct.unpack("B", ddata[8:9])[0]
                if write:
                    # Writing the readden properties
                    self._sampling_rate = sampling_rate
                    self._configbytes0 = configbytes0
                    self._configbytes1 = configbytes1
                    self._configbytes2 = configbytes2
                    self._configbytes3 = configbytes3
                    self._num_channels = num_channels
                    self._buffer_size = buffer_size

                if print_reads:
                    print("Sampling Rate: ", sampling_rate)
                    print("Number of Channels: ", num_channels)
                    print("Buffer Size: ", buffer_size)

                # Reading Enabled Channels
                channels = []
                for i in range(num_channels):
                    ddata = self._serial_port.read(1)
                    byte_value = struct.unpack("B", ddata)[0]
                    signal_name = util.INQUIRY_CHANNELS_NAMES[byte_value]
                    channels.append(signal_name)
                if write:
                    self._channels = channels

                if print_reads:
                    print("Channels: ")
                    for channel in channels:
                        print("     ", channel)

                return True
            else:
                if self.debug:
                    print("inquiry -> ERROR: something went wrong...uncompatible packet type -> ", packet_type)
                return False
        else:
            if self.debug:
                print(
                    "inquiry -> ERROR: you have to establish a connection with the Shimmer before using this command.")
            return False

    # ENABLING/DISABLING sensors - TO DO! disable all, enable

    @property
    def channels(self):
        """
        :return: a list of the enabled channels
        :rtype: list
        """
        return self._channels

    @property
    def num_channels(self):
        """
        :return: the number of enabled channels
        :rtype: int
        """
        return self._num_channels

    @property
    def enabled_sensors(self):
        """
        :return: a list of the enabled sensors
        :rtype: list
        """
        return self._enabled_sensors

    def get_enabled_sensors(self):
        """
        This method reads the enabled sensors from the Shimmer.

        It's not really like that: the list of the enabled it's inferred from the list of the channels
        (there isn't a way to ask to the Shimmer directly for sensors)
        :return: a list of enabled sensors
        :rtype: list
        """
        if self._current_state == util.BT_CONNECTED:
            # Enabled sensors are inferred from active channels
            self._enabled_sensors = []
            # It will go to scan the list of active channels
            index = 0
            while index < len(self._channels):
                # Get the 'current' channel
                current_channel = self._channels[index]
                if current_channel == util.CHANNEL_LOW_ACC_X:
                    self._enabled_sensors.append(util.SENSOR_LOW_NOISE_ACCELEROMETER["name"])
                    index += 3
                elif current_channel == util.CHANNEL_WIDE_ACC_X:
                    self._enabled_sensors.append(util.SENSOR_WIDE_RANGE_ACCELEROMETER["name"])
                    index += 3
                elif current_channel == util.CHANNEL_GYRO_X:
                    self._enabled_sensors.append(util.SENSOR_GYROSCOPE["name"])
                    index += 3
                elif current_channel == util.CHANNEL_MAG_X:
                    self._enabled_sensors.append(util.SENSOR_MAGNETOMETER["name"])
                    index += 3
                elif current_channel == util.CHANNEL_INT_ADC_13:
                    self._enabled_sensors.append(util.SENSOR_INT_EXP_ADC_CH13["name"])
                    index += 1
                elif current_channel == util.CHANNEL_GSR:
                    self._enabled_sensors.append(util.SENSOR_GSR["name"])
                    index += 1
                elif current_channel == util.CHANNEL_BATTERY:
                    self._enabled_sensors.append(util.SENSOR_BATTERY["name"])
                    index += 1
                elif current_channel == util.CHANNEL_WIDE_ACC_X:
                    self._enabled_sensors.append(util.SENSOR_WIDE_RANGE_ACCELEROMETER["name"])
                    index += 3
                elif current_channel == util.CHANNEL_BMPX80_TEMP:
                    self._enabled_sensors.append(util.SENSOR_BMPX80_TEMPERATURE["name"])
                    index += 1
                elif current_channel == util.CHANNEL_BMPX80_PRESS:
                    self._enabled_sensors.append(util.SENSOR_BMPX80_PRESSURE["name"])
                    index += 1
                elif current_channel == util.CHANNEL_BA_LOW or current_channel == util.CHANNEL_BA_HIGH:
                    self._enabled_sensors.append(util.SENSOR_BRIDGE_AMPLIFIER["name"])
                # ExG Status
                elif current_channel == util.CHANNEL_ExG_1_STATUS:
                    pass
                    index += 1
                elif current_channel == util.CHANNEL_ExG_2_STATUS:
                    pass
                    index += 1
                # ExG 1 - 16 e 24 bit
                elif current_channel == util.CHANNEL_ExG_1_CH1_16BIT:
                    self._enabled_sensors.append(util.SENSOR_ExG1_16BIT["name"])
                    index += 2
                elif current_channel == util.CHANNEL_ExG_1_CH1_24BIT:
                    self._enabled_sensors.append(util.SENSOR_ExG1_24BIT["name"])
                    index += 2
                # ExG 2 - 16 e 24 bit
                elif current_channel == util.CHANNEL_ExG_2_CH1_16BIT:
                    self._enabled_sensors.append(util.SENSOR_ExG2_16BIT["name"])
                    index += 2
                elif current_channel == util.CHANNEL_ExG_2_CH1_24BIT:
                    self._enabled_sensors.append(util.SENSOR_ExG2_24BIT["name"])
                    index += 2
                else:
                    if self.debug:
                        print("get_enabled_sensors -> ERROR: something went wrong during channels reading -> ",
                              current_channel)
                    return False
            return True

        else:
            if self.debug:
                print("get_enabled_sensors -> ERROR: the Shimmer3 device is not connected.")
            return False

    def set_enabled_sensors(self, *args):
        """
        This method tell to the Shimmer device which sensors it should enable.

        .. seealso::
            http://shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf
            pg. 36

        :param args: a list of sensors (those that you want to be enabled). Check <shimmer.util.py> for a list of
                     available sensors (e.g. util.SENSOR_GYROSCOPE)
        :return: True in case of succesfull operation, False otherwise
        """
        if self._current_state == util.BT_CONNECTED:
            all_bytes = [0x00, 0x00, 0x00]

            # Building the three bytes with respect to the list of sensors passed as param
            for sensor in args:
                all_bytes[sensor["info_mem_byte"]] = all_bytes[sensor["info_mem_byte"]] | sensor["or_mask"]

            byte0 = all_bytes[0]
            byte1 = all_bytes[1]
            byte2 = all_bytes[2]

            packet = struct.pack("BBBB", util.SET_SENSORS_COMMAND, byte0, byte1, byte2)
            self._serial_port.write(packet)

            if not self._wait_for_ack():
                if self.debug:
                    print("set_enabled_sensors -> ERROR: something went wrong during ACK receipt")
                return False

            # After writing, there is a reading to mantain consistency between Python object and Shimmer device

            # Inquiry command to read channels
            if not self.inquiry(write=True, print_reads=False):
                if self.debug:
                    print("set_enabled_sensors -> ERROR: something went wrong during INQUIRY")
                return False

            # Inferring enabled sensors from active channels
            if not self.get_enabled_sensors():
                if self.debug:
                    print("set_enabled_sensors -> ERROR: something went wrong during GET SENSORS")
                return False

            if util.SENSOR_INT_EXP_ADC_CH13 in args or util.SENSOR_INT_EXP_ADC_CH1 in args or \
                    util.SENSOR_INT_EXP_ADC_CH12 in args or util.SENSOR_INT_EXP_ADC_CH14 in args:
                if not self.set_internal_expansion_power(1):
                    if self.debug:
                        print("set_enabled_sensors -> ERROR: something went wrong during INT EXP powering")
                    return False
            return True
        else:
            if self.debug:
                print("set_enabled_sensors -> ERROR: the Shimmer3 device is not connected.")
            return False

    def get_available_sensors(self):
        """
        :return: a list of all available sensors for this type of Shimmer (e.g IMU does not have ExG sensor)
        :rtype: list
        """
        return util.SENSORS_FOR_TYPE[self._shimmer_type]

    def set_internal_expansion_power(self, enable):
        """
        Power on or off the internal expansion board.
        :param enable: 0 or 1 for indicates whether the board should be powered on
        :type enable: int
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            self._serial_port.write(struct.pack("B", util.SET_INTERNAL_EXP_POWER_ENABLE_COMMAND))
            self._serial_port.write(struct.pack("B", enable))
            if not self._wait_for_ack():
                print("set_internal_expansion_power -> ERROR: something went wrong during ACK receipt.")
                return False

            if self.debug:
                print("set_internal_expansion_power -> internal expansion power set -> ", enable)

            return True
        else:
            if self.debug:
                print("set_internal_expansion_power -> ERROR: the Shimmer3 device is not connected.")
            return False

    # SAMPLING RATE
    @property
    def sampling_rate(self):
        """
        :return: the sampling rate of the device.
        """
        return self._sampling_rate

    def set_sampling_rate(self, new_sampling_rate):
        """
        Set the sampling rate of the device

        .. seealso::
            http://shimmersensing.com/images/uploads/docs/LogAndStream_for_Shimmer3_Firmware_User_Manual_rev0.11a.pdf\n
            pag. 24
        :param new_sampling_rate: the new desired sampling rate
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """

        if self._current_state == util.BT_CONNECTED:
            sampling_rate_to_send = round(32768.0 / new_sampling_rate)
            packet = struct.pack("B", util.SET_SAMPLING_RATE_COMMAND)
            if not self._serial_port.write(packet):
                if self.debug:
                    print("set_sampling_rate -> ERROR: something went wrong during writing")
                return False
            packet = struct.pack("<H", sampling_rate_to_send)
            self._serial_port.write(packet)
            if not self._wait_for_ack():
                if self.debug:
                    print("set_sampling_rate -> ERROR: something went wrong during ACK receiving.")
                return False

            # After writing, reading to maintain consistency
            if not self.get_sampling_rate():
                if self.debug:
                    print("set_sampling_rate -> ERROR: something went wrong during GET SAMPLING RATE")
                return False

            # Here the new sampling rate is readden (after set). Now we have to update
            # all sensor's specific rates
            # Sending Wide Range Accelerometer Data Rate as close as possible to the Sampling Rate, but NEVER LOWER
            res = True
            if self.sampling_rate <= 12.5:
                res = self.set_wide_acc_rate(1)
            elif self.sampling_rate <= 25:
                res = self.set_wide_acc_rate(2)
            elif self.sampling_rate <= 50:
                res = self.set_wide_acc_rate(3)
            elif self.sampling_rate <= 100:
                res = self.set_wide_acc_rate(4)
            elif self.sampling_rate <= 200:
                res = self.set_wide_acc_rate(5)
            elif self.sampling_rate <= 400:
                res = self.set_wide_acc_rate(6)
            elif self.sampling_rate <= 800:
                res = self.set_wide_acc_rate(7)
            elif self.sampling_rate <= 1600:
                res = self.set_wide_acc_rate(8)
            elif self.sampling_rate <= 3200:
                res = self.set_wide_acc_rate(9)
            elif self.sampling_rate <= 32768:
                res = self.set_wide_acc_rate(10)

            if not res:
                return False

            # Sending Gyro Rate as close as possible to the Sampling Rate, but NEVER LOWER
            gyro_rate = min(255, math.floor(8000 / self.sampling_rate - 1))
            if gyro_rate >= 0:
                if not self.set_gyro_rate(gyro_rate):
                    return False
            else:
                if not self.set_gyro_rate(0):
                    return False

            # Sending Mag Rate as close as possible to the Sampling Rate, but NEVER LOWER
            if self.sampling_rate <= 10:
                res = self.set_mag_rate(0)
            elif self.sampling_rate <= 20:
                res = self.set_mag_rate(1)
            elif self.sampling_rate <= 50:
                res = self.set_mag_rate(2)
            elif self.sampling_rate <= 32768:
                res = self.set_mag_rate(3)
            if not res:
                return False

            return True
        else:
            if self.debug:
                print("set_sampling_rate -> ERROR: the Shimmer3 device is not connected.")
            return False

    def get_sampling_rate(self):
        """
        Read the sampling rate of the device and store it in the 'sampling_rate' property.

        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            packet = struct.pack("B", util.GET_SAMPLING_RATE_COMMAND)
            self._serial_port.write(packet)
            if self._wait_for_ack():
                data = self._serial_port.read(1)
                packet_id = struct.unpack("B", data)[0]
                if packet_id == util.SAMPLING_RATE_RESPONSE:
                    data = self._serial_port.read(2)
                    sampling_rate_raw = struct.unpack("<H", data)[0]
                    sampling_rate = round(32768.0 / sampling_rate_raw, 2)
                    self._sampling_rate = sampling_rate
                    return True
                else:
                    if self.debug:
                        print("get_sampling_rate -> ERROR: incompatible response packet type -> ", packet_id)
                    return False
            else:
                if self.debug:
                    print("get_sampling_rate -> ERROR: something went wrong during ACK receipt.")
                return False
        else:
            if self.debug:
                print("get_sampling_rate -> ERROR: the Shimmer3 device is not connected.")

    # REAL TIME Clock
    @property
    def rtc_milliseconds(self):
        """
        :return: the Real Time Clock (in milliseconds) set in the device.
        """
        return self._rtc_milliseconds

    def read_real_time_clock(self, print_reads=False):
        """
        Read the RTC from the device and store it in 'rtc_milliseconds'.

        :param print_reads: if a debug print should be showed
        :type print_reads: bool
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            packet = struct.pack("B", util.GET_RWC_COMMAND)
            self._serial_port.write(packet)
            self._wait_for_ack()

            ddata = self._serial_port.read(9)
            packet_id = struct.unpack("B", ddata[0:1])[0]
            if packet_id == util.RWC_RESPONSE:
                rtc_ticks = struct.unpack("Q", ddata[1:9])[0]
                rtc_milliseconds = rtc_ticks / 32.768
                if print_reads:
                    print("=== RTC READS ===")
                    print("     RTC Ticks: ", rtc_ticks)
                    print("     RTC Milliseconds: ", rtc_milliseconds)
                    print("     RTC Date: ", datetime.datetime.fromtimestamp(rtc_milliseconds / 1000))
                self._rtc_ticks = rtc_ticks
                self._rtc_milliseconds = rtc_milliseconds
                return True
            else:
                if self.debug:
                    print("read_real_time_clock -> ERROR: incompatible response packet type -> ", packet_id)
                return False
        else:
            if self.debug:
                print("read_real_time_clock -> ERROR: can't read RTC settings. The Shimmer must be in the state "
                      "'BT_CONNECTED'")
            return False

    def write_real_time_clock(self):
        """
        Send and set the current RTC to the Shimmer device.
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            current_time_milliseconds = time.time() * 1000
            current_time_ticks = np.uint64(current_time_milliseconds * 32.768)
            packet = struct.pack("B", util.SET_RWC_COMMAND)
            self._serial_port.write(packet)
            packet = struct.pack("<Q", current_time_ticks)
            self._serial_port.write(packet)

            if not self._wait_for_ack():
                if self.debug:
                    print("write_real_time_clock -> ERROR: something went wrong during ACK receipt.")
                return False

            # After writing, reading to maintain consistency
            if not self.read_real_time_clock(print_reads=False):
                if self.debug:
                    print("write_real_time_clock -> ERROR: something went wrong during READ RTC")
                return False

            return True
        else:
            if self.debug:
                print("write_real_time_clock -> ERROR: can't write RTC settings. The Shimmer must be in the state "
                      "'BT_CONNECTED'")
            return False

    # WIDE ACC methods

    @property
    def wide_acc_range(self):
        """
        :return: the Wide Accelerometer's range of the Shimmer
        """
        return self._wide_acc_range

    def get_wide_acc_range(self):
        """
        Read the Wide Accelerometer's range from the Shimmer and set 'wide_acc_range' property.
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            packet = struct.pack("B", util.GET_LSM303DLHC_ACCEL_RANGE_COMMAND)
            self._serial_port.write(packet)
            if self._wait_for_ack():
                # The response is made of two bytes - 1 id - 1 range
                data = self._serial_port.read(1)
                packet_id = struct.unpack("B", data)[0]
                if packet_id == util.LSM303DLHC_ACCEL_RANGE_RESPONSE:
                    data = self._serial_port.read(1)
                    range_raw = struct.unpack("B", data)[0]
                    # Now the macro should be transformed to the real range
                    self._wide_acc_range = util.WIDE_ACC_RANGES[range_raw]
                    return True
                else:
                    if self.debug:
                        print("get_wide_acc_range -> ERROR: incompatible response packet type -> ", packet_id)
                    return False
            else:
                if self.debug:
                    print("get_sampling_rate -> ERROR: something went wrong during ACK receipt.")
                return False
        else:
            if self.debug:
                print("get_sampling_rate -> ERROR: the Shimmer3 device is not connected.")

    def set_wide_acc_range(self, wide_acc_range):
        """
        Set the Wide Accelerometer's range of the Shimmer.

        :param wide_acc_range: the new range to set
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if wide_acc_range == util.WIDE_ACC_RANGE_2g or wide_acc_range == util.WIDE_ACC_RANGE_4g or \
                    wide_acc_range == util.WIDE_ACC_RANGE_8g or wide_acc_range == util.WIDE_ACC_RANGE_16g:
                packet = struct.pack("B", util.SET_LSM303DLHC_ACCEL_RANGE_COMMAND)
                if not self._serial_port.write(packet):
                    if self.debug:
                        print("set_wide_acc_range -> ERROR: something went wrong during ACK receiving.")
                    return False

                packet = struct.pack("B", wide_acc_range)
                self._serial_port.write(packet)
                if not self._wait_for_ack():
                    if self.debug:
                        print("set_wide_acc_range -> ERROR: something went wrong during ACK receiving.")
                    return False

                # After writing, reading to maintain consistency
                if not self.get_wide_acc_range():
                    if self.debug:
                        print("set_wide_acc_range -> ERROR: something went wrong during GET SAMPLING RATE")
                    return False

                if self.debug:
                    print("set_wide_acc_range -> WIDE ACC range setted to: ", wide_acc_range)

                return True
            else:
                if self.debug:
                    print("set_wide_acc_range -> ERROR: the range value passed is not supported. Try 0, 1, 2 or 3")
        else:
            if self.debug:
                print("set_wide_acc_range -> ERROR: the Shimmer3 device is not connected.")
            return False

    def set_wide_acc_rate(self, wide_acc_rate):
        """
        Set the sampling rate of the Wide Range Accelerometer.

        :param wide_acc_rate: the new rate to set
        :return: True in case of succesfull operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if wide_acc_rate == 1 or wide_acc_rate == 2 or wide_acc_rate == 3 or wide_acc_rate == 4 or \
                    wide_acc_rate == 5 or wide_acc_rate == 6 or wide_acc_rate == 7 or wide_acc_rate == 8 or \
                    wide_acc_rate == 9 or wide_acc_rate == 10:
                packet = struct.pack("B", util.SET_LSM303DLHC_ACCEL_SAMPLING_RATE_COMMAND)
                if not self._serial_port.write(packet):
                    if self.debug:
                        print("set_wide_acc_rate -> ERROR: something went wrong during ACK receiving.")
                    return False

                packet = struct.pack("B", wide_acc_rate)
                self._serial_port.write(packet)
                if not self._wait_for_ack():
                    if self.debug:
                        print("set_wide_acc_rate -> ERROR: something went wrong during ACK receiving.")
                    return False

                if self.debug:
                    print("set_wide_acc_rate -> WIDE ACC rate set to: ", wide_acc_rate)
                return True
            else:
                if self.debug:
                    print("set_wide_acc_rate -> ERROR: the rate value passed is not supported. Try [0, 10]")
        else:
            if self.debug:
                print("set_wide_acc_rate -> ERROR: the Shimmer3 device is not connected.")
            return False

    # ExG methods

    @property
    def exg_gain(self):
        """
        :return: the ExG gain property
        """
        return self._exg_gain

    def set_exg_gain(self, new_gain):
        """
        Set the 'new_gain' into the 'exg_gain' property.

        :param new_gain: the new gain to set
        """
        self._exg_gain = new_gain

    @property
    def exg_purpose(self):
        """
        :return: the purpose of the ExG
        """
        return self._exg_purpose

    def exg_send_ecg_settings(self, gain):
        # BOTH CHIP SHOULD BE ENABLED AT THAT POINT
        """
        This function send to the Shimmer device the ExG register configured to handle the ECG
        signal (with recommended settings).

        Momentarily the "test" function is not supported

        Assuming:
        - The newest sampling rate is sent before that this function is called. IMPORTANT!!!
        - Previously sent "enable sensors" with ExG1 and ExG2. IMPORTANT!!!

        - Byte 0 -> 0 0 0 0 0 data_rate(3) \n
        - Byte 1 -> 1 0 1 0 X 0 INT_TEST TEST_FREQ \n
        - Byte 2 -> 0 0 0 1 0 0 0 0 \n
        - Byte 3 -> 0 Gain1(3) Mux1(4) \n
        - Byte 4 -> 0 Gain2(3) Mux2(4) \n
        - Byte 5 -> 0 0 PDB_RLD RLD_LOFF_SENS RLD_Settings(4) \n
        - Byte 6 -> 0 0 0 0 0 0 0 0 \n
        - Byte 7 -> 0 0 0 0 0 0 0 0 \n
        - Byte 8 -> 0 0 0 0 0 0 1 0 \n
        - Byte 9 -> 0 0 0 0 0 0 RLDREF_INT 1 \n

        .. seealso::
            http://shimmersensing.com/images/uploads/docs/ECG_User_Guide_Rev1.12.pdf \n
            section 3.8

        :return: True in case of successfully operation, False otherwise
        """

        # For first, check what Data Rate chips should use. We want this to be as close as possible at the
        # Shimmer sampling rate, but greater than this
        if self._sampling_rate <= 125:
            byte0 = util.ExG_DATA_RATE_125
        elif self._sampling_rate <= 250:
            byte0 = util.ExG_DATA_RATE_250
        elif self._sampling_rate <= 500:
            byte0 = util.ExG_DATA_RATE_500
        elif self._sampling_rate <= 1000:
            byte0 = util.ExG_DATA_RATE_1000
        elif self._sampling_rate <= 2000:
            byte0 = util.ExG_DATA_RATE_2000
        elif self._sampling_rate <= 4000:
            byte0 = util.ExG_DATA_RATE_4000
        elif self._sampling_rate <= 8000:
            byte0 = util.ExG_DATA_RATE_8000
        else:
            byte0 = util.ExG_DATA_RATE_500  # 500 is the suggested Data Rate for ECG

        # We have to understand if X bit should be 1 or 0: it depends on what SR_NUMBER this device has.
        # If SR_NUMBER = 47 and SR_REVISION >= 4 then X=1, otherwise X=0
        # WHEN test is FALSE
        # So, if X=1 -> 1 0 1 0 1 0 0 0 -> 0xA8
        #     if X=0 -> 1 0 1 0 0 0 0 0 -> 0xA0
        # WHEN test is TRUE
        # So, if X=1 -> 1 0 1 0 1 0 1 1 -> 0xAB
        #     if X=0 -> 1 0 1 0 0 0 1 1 -> 0xA3
        # Generally
        #   1 0 1 0 X 0 T1 T2
        #   We can start from the default 1 0 1 0 0 0 0 0 and work with OR operator

        byte1_chip1 = 0xA0
        if self._sr_number == 47 and self._sr_rev >= 4:
            byte1_chip1 = byte1_chip1 | 8  # 8 -> 0 0 0 0 1 0 0 0

        byte1_chip2 = 0xA0

        # 0x10 -> 1 0 0 0 0 -> 16
        byte2 = 0x10

        # Check doc: suggested chip1 MUX1 and MUX2 for ECG are both 0x00
        byte3_chip1 = 0x00 | gain | 0x00
        byte4_chip1 = 0x00 | gain | 0x00
        # Check doc: suggested chip2 MUX1 and MUX2 are 0 0 0 0 and 0 1 1 1 respectively
        byte3_chip2 = 0x00 | gain | 0x00
        byte4_chip2 = 0x00 | gain | 0x07

        # Check doc: suggested RLD Settings is 1 1 0 1 for chip1 and 0x00 for chip2
        #            suggested PDB_RLD is 1 for chip1 and 0 for chip2
        #            suggested RLD_LOFF_SENS is 0 for both chips
        #   chip1 -> 0 0 1 0 1 1 0 1
        #   chip2 -> 0 0 0 0 0 0 0 0
        byte5_chip1 = 0x2D
        byte5_chip2 = 0x00

        # Check doc
        byte6 = 0x00
        byte7 = 0x00
        byte8 = 0x02

        # Check doc: suggested value for RLDREF_INT is 1 for chip1 and 0 for chip2
        byte9_chip1 = 0x03  # 00000011
        byte9_chip2 = 0x01  # 00000001

        chip1 = [byte0, byte1_chip1, byte2, byte3_chip1, byte4_chip1, byte5_chip1, byte6, byte7, byte8, byte9_chip1]
        chip2 = [byte0, byte1_chip2, byte2, byte3_chip2, byte4_chip2, byte5_chip2, byte6, byte7, byte8, byte9_chip2]

        # Write config for chip1
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP1, 0x00, 0x0A] + chip1)
        self._wait_for_ack()
        # Write config for chip2
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP2, 0x00, 0x0A] + chip2)
        self._wait_for_ack()

        # Update ExG purpose as a property
        self._exg_purpose = util.ExG_ECG
        self._exg_gain = gain

        return True

    def exg_send_emg_settings(self, gain):
        # FIRST CHIP SHOULD BE ENABLED AT THAT POINT
        """
        This function send to the Shimmer device the ExG register configured to handle the EMG
        signal (with recommended settings)
        Momentarily the "test" function is not supported
        Assuming:
        - The newest sampling rate is sent before that this function is called.
        - The newest ExG Gain is sent before that this function is called.
        - Previously sent "enable sensors" with only ExG1

        - Byte 0 -> 0 0 0 0 0 data_rate(3) \n
        - Byte 1 -> 1 0 1 0 X 0 INT_TEST TEST_FREQ \n
        - Byte 2 -> 0 0 0 1 0 0 0 0 \n
        - Byte 3 -> 0 Gain1(3) Mux1(4) \n
        - Byte 4 -> 0 Gain2(3) Mux2(4) \n
        - Byte 5 -> 0 0 PDB_RLD RLD_LOFF_SENS RLD_Settings(4) \n
        - Byte 6 -> 0 0 0 0 0 0 0 0 \n
        - Byte 7 -> 0 0 0 0 0 0 0 0 \n
        - Byte 8 -> 0 0 0 0 0 0 1 0 \n
        - Byte 9 -> 0 0 0 0 0 0 RLDREF_INT 1 \
        .. seealso::
            http://shimmersensing.com/images/uploads/docs/EMG_User_Guide_Rev1.12.pdf \n
            section 3.3
        :return: True in case of successfully operation, False otherwise
        """

        # For first, check what Data Rate chips should use. We want this to be as close as possible at the
        # Shimmer sampling rate, but greater than this
        if self._sampling_rate <= 125:
            byte0 = util.ExG_DATA_RATE_125
        elif self._sampling_rate <= 250:
            byte0 = util.ExG_DATA_RATE_250
        elif self._sampling_rate <= 500:
            byte0 = util.ExG_DATA_RATE_500
        elif self._sampling_rate <= 1000:
            byte0 = util.ExG_DATA_RATE_1000
        elif self._sampling_rate <= 2000:
            byte0 = util.ExG_DATA_RATE_2000
        elif self._sampling_rate <= 4000:
            byte0 = util.ExG_DATA_RATE_4000
        elif self._sampling_rate <= 8000:
            byte0 = util.ExG_DATA_RATE_8000
        else:
            byte0 = util.ExG_DATA_RATE_500  # 500 is the suggested Data Rate for ECG

        # We have to understand if X bit should be 1 or 0: it depends on what SR_NUMBER this device has.
        # If SR_NUMBER = 47 and SR_REVISION >= 4 then X=1, otherwise X=0
        # WHEN test is FALSE
        # So, if X=1 -> 1 0 1 0 1 0 0 0 -> 0xA8
        #     if X=0 -> 1 0 1 0 0 0 0 0 -> 0xA0
        # WHEN test is TRUE
        # So, if X=1 -> 1 0 1 0 1 0 1 1 -> 0xAB
        #     if X=0 -> 1 0 1 0 0 0 1 1 -> 0xA3
        # Generally
        #   1 0 1 0 X 0 T1 T2
        #   We can start from the default 1 0 1 0 0 0 0 0 and work with OR operator
        byte1_chip1 = 0xA0
        if self._sr_number == 47 and self._sr_rev >= 4:
            byte1_chip1 = byte1_chip1 | 8  # 8 -> 0 0 0 0 1 0 0 0

        byte1_chip2 = 0xA0

        # 0x10 -> 1 0 0 0 0 -> 16
        byte2 = 0x10

        # Check doc: suggested chip1 MUX1 is 1 0 0 1
        #            suggested chip1 MUX2 is 0 0 0 0
        #            suggested chip2 MUX1 is 0 0 0 1
        #            suggested chip2 MUX2 is 0 0 0 1
        # Suppose 'gain' is one of util.ExG_GAIN_X

        byte3_chip1 = 0x00 | gain | 0x09
        byte4_chip1 = 0x00 | gain | 0x00
        # For EMG we use only the first chip. So, the second chip it is handled
        # in a slightly different way:
        # (bit7 is 1 to power down mode)
        # (gains are 0)
        byte3_chip2 = 0x80 | gain | 0x01
        byte4_chip2 = 0x80 | gain | 0x01

        # Check doc: suggested RLD Settings is 0x00 for chip1 and 0x00 for chip2
        #            suggested PDB_RLD is 1 for chip1 and 0 for chip2
        #            suggested RLD_LOFF_SENS is 0 for both chips
        #   chip1 -> 0 0 1 0 0 0 0 0
        #   chip2 -> 0 0 0 0 0 0 0 0
        byte5_chip1 = 0x20
        byte5_chip2 = 0x00
        # Check doc
        byte6 = 0x00
        byte7 = 0x00
        byte8 = 0x02
        # Check doc: suggested value for RLDREF_INT is 1 for chip1 and 0 for chip2
        byte9_chip1 = 0x03  # 00000011
        byte9_chip2 = 0x01  # 00000001

        chip1 = [byte0, byte1_chip1, byte2, byte3_chip1, byte4_chip1, byte5_chip1, byte6, byte7, byte8, byte9_chip1]
        chip2 = [byte0, byte1_chip2, byte2, byte3_chip2, byte4_chip2, byte5_chip2, byte6, byte7, byte8, byte9_chip2]

        # BRUTE FORCE

        #
        # chip1 = [byte0, 0xA8, 0x10, 0x69, 0x60, 0x20, 0x00, 0x00, 0x02, 0x03]
        # chip2 = [byte0, 0xA0, 0x10, 0xE1, 0xE1, 0x00, 0x00, 0x00, 0x02, 0x01]
        #

        # Write config for chip1
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP1, 0x00, 0x0A] + chip1)
        self._wait_for_ack()
        # Write config for chip2

        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP2, 0x00, 0x0A] + chip2)
        self._wait_for_ack()

        # Update ExG purpose as a property
        self._exg_purpose = util.ExG_EMG
        # Update ExG Gain
        self._exg_gain = gain

        return True

    def exg_send_resp_settings(self):
        """
        This function send to the Shimmer device the ExG register configured to handle the EMG
        signal (with recommended settings).

        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        # BOTH CHIP SHOULD BE ENABLED AT THAT POINT
        if self._sr_number == 47 and self._sr_rev >= 4:
            chip1 = [0x00, 0xA8, 0x10, 0x40, 0x40, 0x20, 0x00, 0x00, 0x02, 0x03]
            chip2 = [0x00, 0xA0, 0x10, 0x40, 0x40, 0x00, 0x00, 0x00, 0xEA, 0x01]
        else:
            chip1 = [0x00, 0xA0, 0x10, 0x40, 0x40, 0x20, 0x00, 0x00, 0x02, 0x03]
            chip2 = [0x00, 0xA0, 0x10, 0x40, 0x40, 0x00, 0x00, 0x00, 0xEA, 0x01]
        # Write config for chip1
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP1, 0x00, 0x0A] + chip1)
        self._wait_for_ack()
        # Write config for chip2
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP2, 0x00, 0x0A] + chip2)
        self._wait_for_ack()
        # Update ExG purpose as a property
        self._exg_purpose = util.ExG_RESP
        self._exg_gain = util.ExG_GAIN_4
        return True

    def exg_send_exg_test_settings(self):
        """
        This function send to the Shimmer device the ExG register configured to use it in test mode.

        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        # BOTH CHIP SHOULD BE ENABLED AT THAT POINT
        if self._sampling_rate <= 125:
            byte0 = util.ExG_DATA_RATE_125
        elif self._sampling_rate <= 250:
            byte0 = util.ExG_DATA_RATE_250
        elif self._sampling_rate <= 500:
            byte0 = util.ExG_DATA_RATE_500
        elif self._sampling_rate <= 1000:
            byte0 = util.ExG_DATA_RATE_1000
        elif self._sampling_rate <= 2000:
            byte0 = util.ExG_DATA_RATE_2000
        elif self._sampling_rate <= 4000:
            byte0 = util.ExG_DATA_RATE_4000
        elif self._sampling_rate <= 8000:
            byte0 = util.ExG_DATA_RATE_8000
        else:
            byte0 = util.ExG_DATA_RATE_500  # 500 is the suggested Data Rate for ECG
        # Update ExG purpose as a property
        chip1 = [byte0, 0xAB, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01]
        chip2 = [byte0, 0xA3, 0x10, 0x15, 0x15, 0x00, 0x00, 0x00, 0x02, 0x01]
        # Write config for chip1
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP1, 0x00, 0x0A] + chip1)
        self._wait_for_ack()
        # Write config for chip2
        self._serial_port.write([util.SET_EXG_REGS_COMMAND, util.ExG_CHIP2, 0x00, 0x0A] + chip2)
        self._wait_for_ack()

        self._exg_purpose = util.ExG_TEST
        self._exg_gain = util.ExG_GAIN_1
        return True

    # GSR methods

    def set_gsr_range(self, gsr_range):
        """
        Set the GSR's sampling range of the Shimmer.

        - 0 -> 10 to 56 kOhm
        - 1 -> 56 to 220 kOhm
        - 2 -> 220 to 680 kOhm
        - 3 -> 680 KOhm to 4.7 MOhm
        - 4 -> Autorange

        .. seealso::
            https://www.shimmersensing.com/images/uploads/docs/GSR_User_Guide_rev1.13.pdf
            (pg. 7)

        :param gsr_range: the new range to set
        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if gsr_range == 0 or gsr_range == 1 or gsr_range == 2 or gsr_range == 3 or gsr_range == 4:
                packet = struct.pack("B", util.SET_GSR_RANGE_COMMAND)
                if not self._serial_port.write(packet):
                    if self.debug:
                        print("send_gsr_range -> ERROR: something went wrong during ACK receiving.")
                    return False
                packet = struct.pack("B", gsr_range)
                self._serial_port.write(packet)
                if not self._wait_for_ack():
                    if self.debug:
                        print("send_gsr_range -> ERROR: something went wrong during ACK receiving.")
                    return False

                if self.debug:
                    print("send_gsr_range -> GSR range setted to: ", gsr_range)

                return True
            else:
                if self.debug:
                    print("send_gsr_range -> ERROR: invalid value for the range. Try 0, 1, 2, 3 or 4.")
                return False
        else:
            if self.debug:
                print("send_gsr_range -> ERROR: the Shimmer has to be connected")
            return False

    # GYRO methods

    def set_gyro_range(self, gyro_range):
        """
        Set the gyroscope's sampling range.

        - 0 -> +/- 250 deg/s
        - 1 -> +/- 500 deg/s
        - 2 -> +/- 1000 deg/s
        - 3 -> +/- 2000 deg/s

        .. seealso::
            https://www.shimmersensing.com/images/uploads/docs/IMU_User_Guide_rev1.4.pdf
            (pg. 21)
        :param gyro_range: the new range to set
        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if gyro_range == 0 or gyro_range == 1 or gyro_range == 2 or gyro_range == 3:
                packet = struct.pack("B", util.SET_MPU9150_GYRO_RANGE_COMMAND)
                if not self._serial_port.write(packet):
                    if self.debug:
                        print("set_gyro_range -> ERROR: something went wrong during ACK receiving.")
                    return False
                packet = struct.pack("B", gyro_range)
                self._serial_port.write(packet)
                if not self._wait_for_ack():
                    if self.debug:
                        print("set_gyro_range -> ERROR: something went wrong during ACK receiving.")
                    return False

                if self.debug:
                    print("set_gyro_range -> GYRO range set to: ", gyro_range)

                return True
            else:
                if self.debug:
                    print("set_gyro_range -> ERROR: invalid value for the range. Try 0, 1, 2 or 3.")
                return False
        else:
            if self.debug:
                print("set_gyro_range -> ERROR: the Shimmer has to be connected")
            return False

    def set_gyro_rate(self, gyro_rate):
        """
        Set the gyroscope's sampling rate.

        :param gyro_rate: the new rate to set.
        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if 0 <= gyro_rate <= 255:
                packet = struct.pack("B", util.SET_MPU9150_SAMPLING_RATE_COMMAND)
                if not self._serial_port.write(packet):
                    if self.debug:
                        print("set_gyro_rate -> ERROR: something went wrong during ACK receiving.")
                    return False

                packet = struct.pack("B", gyro_rate)
                self._serial_port.write(packet)
                if not self._wait_for_ack():
                    if self.debug:
                        print("set_gyro_rate -> ERROR: something went wrong during ACK receiving.")
                    return False

                if self.debug:
                    print("set_gyro_rate -> GYRO rate set to: ", gyro_rate, " that is, in Hz: ", 8000 / (1 + gyro_rate))
                return True
            else:
                if self.debug:
                    print("set_gyro_rate -> ERROR: the value is not supported. You have to use somewhat between 0 and "
                          "255 (included)")
        else:
            if self.debug:
                print("set_gyro_rate -> ERROR: the Shimmer has to be connected")
            return False

    # MAG methods

    def set_mag_rate(self, mag_rate):
        """
        Set the magnetometer's sampling rate.

        - 0 -> 10.0 Hz
        - 1 -> 20.0 Hz
        - 2 -> 50.0 Hz
        - 3 -> 100.0 Hz

        :param mag_rate: the new rate to set.
        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        if self._current_state == util.BT_CONNECTED:
            if mag_rate == 0 or mag_rate == 1 or mag_rate == 2 or mag_rate == 3:
                packet = struct.pack("B", util.SET_LSM303DLHC_MAG_SAMPLING_RATE_COMMAND)
                if not self._serial_port.write(packet):
                    if self.debug:
                        print("send_mag_rate -> ERROR: something went wrong during ACK receiving.")
                    return False

                packet = struct.pack("B", mag_rate)
                self._serial_port.write(packet)
                if not self._wait_for_ack():
                    if self.debug:
                        print("send_mag_rate -> ERROR: something went wrong during ACK receiving.")
                    return False

                if self.debug:
                    print("send_mag_rate -> MAG rate set to: ", mag_rate)

                return True
            else:
                if self.debug:
                    print("send_mag_rate -> ERROR: invalid value for the rate. Try 0, 1, 2 or 3.")
                return False
        else:
            if self.debug:
                print("set_mag_rate -> ERROR: the Shimmer is not connected")
            return False

    # CALIBRATE raw data

    @property
    def active_gsr_mu(self):
        """
        :return: the current measurement unit for the GSR data
        """
        return self._active_gsr_mu

    def set_active_gsr_mu(self, new_mu=None):
        """
        Set the current measurement unit for the GSR data.

        :param new_mu: the new measurement unit
        :return: True in case of successfully operation, False otherwise
        :rtype: bool
        """
        if new_mu == util.GSR_SKIN_CONDUCTANCE or new_mu == util.GSR_SKIN_RESISTANCE:
            self._active_gsr_mu = new_mu
            if self.debug:
                print("set_active_gsr_mu -> GSR Measurement unit set to: ", new_mu)
            return True
        else:
            if self.debug:
                print("set_active_gsr_mu -> ERROR: not a valid measurement unit: ", new_mu)
            return False

    def calibrate_low_acc_vector(self, raw_data):
        """
        Calibrate the data from the Low Noise Accelerometer.

        :param raw_data: [raw_acc_x, raw_acc_y, raw_acc_z]
        :return: the calibrated vector
        :rtype: list
        """

        calibrated_data = (
            self._low_accelerator_calibration['rx_inv'].dot(self._low_accelerator_calibration['kx_inv'])).dot(
            np.array([raw_data]).T - self._low_accelerator_calibration['offset'])
        return calibrated_data.T[0].tolist()

    def calibrate_wide_acc_vector(self, raw_data):
        """
        Calibrate the data from the Wide Range Accelerometer.

        :param raw_data: [raw_acc_x, raw_acc_y, raw_acc_z]
        :return: the calibrated vector
        :rtype: list
        """
        calibrated_data = (self._wide_accelerator_calibration[self._wide_acc_range]['rx_inv'].dot(
            self._wide_accelerator_calibration[self._wide_acc_range]['kx_inv'])).dot(
            np.array([raw_data]).T - self._wide_accelerator_calibration[self._wide_acc_range]['offset'])
        return calibrated_data.T[0].tolist()

    def calibrate_gsr(self, raw_data):
        """
        Calibrate the data from the GSR sensor.

        :param raw_data: GSR's raw data
        :return: the calibrated GSR data
        """
        #   How GSR Raw Data Works:
        #
        #   16 bit -> xx xx xxxx xxxx xxxx
        #             ra un ----adc-------
        #       where: ra is range
        #              un is unused
        #              adc is adc_value
        #
        range_settings = (raw_data >> 14) & 0x3  # take only two upper bits
        adc_value = raw_data & 0xfff  # take only twelve lower bits
        r_f = 3300000  # with auto_range -> range 3 -> 3300000
        if range_settings == 0:
            r_f = 40200
        elif range_settings == 1:
            r_f = 287000
        elif range_settings == 2:
            r_f = 1000000
        elif range_settings == 3:
            r_f = 3300000
        gsr_to_volts = adc_value * (3.0 / 4095.0)
        gsr_ohm = r_f / ((gsr_to_volts / 0.5) - 1.0)

        if self._active_gsr_mu == util.GSR_SKIN_CONDUCTANCE:
            skin_conductance = (1 / gsr_ohm) * 1000000  # microSiemiens - Skin Conductance
            return skin_conductance
        elif self._active_gsr_mu == util.GSR_SKIN_RESISTANCE:
            return gsr_ohm / 1000  # kOhm - Skin Resistance
        else:
            print("calibrate_gsr -> WARNING: Not supported yet...returned 'None'")
            return None

    def calibrate_exg_24bit(self, raw_data):
        """
        Calibrate the ExG 24 BIT chip's data.

        :param raw_data: raw data of the ExG 24 bit chip
        :return: calibrated data
        """
        adc_sensitivity = 2420 / (2 ** 23 - 1)
        offset = 0
        calibrated = ((raw_data - offset) * adc_sensitivity) / util.ExG_GAINS_FROM_BYTE_TO_VALUE[self._exg_gain]
        return calibrated

    @staticmethod
    def calibrate_ppg(raw_data):
        """
        Calibrate the PPG data.

        :param raw_data: the raw data of the PPG sensor
        :return: the calibrated data
        """
        return raw_data * 3000.0 / 4095.0

    def calibrate_timestamp_time_elapsed(self, raw_data):
        """
        Transform the timestamp in seconds elapsed from the first timestamp of the stream.

        :param raw_data: current timestamp
        :return: seconds elapsed from the first packet of the stream
        """
        cal_first_timestamp_of_a_stream = self._first_local_timestamp_of_a_stream / 32768
        return [raw_data / 32768 - cal_first_timestamp_of_a_stream]

    # UTIL METHODS

    @property
    def shimmer_type(self):
        """
        :return: the type of the Shimmer device
        """
        return self._shimmer_type

    def print_object_properties(self):
        """
        Pretty printing of the device's property.
        """
        print("\n**** SHIMMER3 connected on ", self._com_port, " ****\n")
        print("     Shimmer3 Type: ", self._shimmer_type)
        print("     Debug: ", self.debug)
        print("     SR Number: ", self._sr_number)
        print("     SR Rev: ", self._sr_rev)
        print("     Hardware Compatibility Code: ", self._hardware_compatibility_code)
        print("")
        print("     Current State: ", self._current_state)
        print("     COM Port: ", self._com_port)
        print("     Enabled Sensors: ", self._enabled_sensors)
        print("     Num Channels: ", self._num_channels)
        print("     Channels: ", self._channels)
        print("     Sampling Rate: ", self._sampling_rate, " Hz")
        print("")
        print("     Real Time Clock: ", self._rtc_ticks, " ticks --- ", self._rtc_milliseconds, " ms --- ",
              datetime.datetime.fromtimestamp(self._rtc_milliseconds / 1000))
        print("")
        print("     Buffer Size: ", self._buffer_size)
        print("")
        print("     Wide Accelerometer Range: ", self._wide_acc_range)
        print("     Current MU for GSR: ", self._active_gsr_mu)
        print("")
        print("     ExG Purpose: ", self._exg_purpose)
        if self._exg_gain is not None:
            to_print = util.ExG_GAINS_FROM_BYTE_TO_VALUE[self._exg_gain]
        else:
            to_print = None
        print("     ExG Gain: ", to_print)
        print("\n****************************************\n")

    # noinspection PyProtectedMember
    @staticmethod
    def encode_to_json(o):
        """
        Function that encode an object Shimmer3 'o' into JSON format.
        :param o: an object Shimmer3
        :return: the serialized object
        """
        if isinstance(o, Shimmer3):
            obj = {"__shimmer__": True, "type": o.shimmer_type, "debug": o.debug, "state": o.current_state,
                   "com_port": o.com_port, "enabled_sensors": o.enabled_sensors, "sampling_rate": o.sampling_rate,
                   "active_gsr_mu": o.active_gsr_mu, "wide_acc_range": o.wide_acc_range, "exg_gain": o.exg_gain,
                   "exg_purpose": o.exg_purpose}
            return obj
        else:
            type_name = o.__class__.__name__
            raise TypeError(f"Object of type '{type_name}' is not JSON serializable")

    @staticmethod
    def decode_from_json(dct):
        if "__shimmer__" in dct:
            shimmer = Shimmer3(shimmer_type=dct["type"], debug=dct["debug"])
            old_state = dct["state"]
            # We recover things only if the Shimmer was connected. When there is a Disconnection
            # this object will reset all properties to init value.
            if old_state == 1:  # BT Connected
                # We want to connect the device and write all properties
                shimmer.connect(com_port=dct["com_port"], write_rtc=True, update_all_properties=True)
                # We have to restore enabled sensors
                sensors_to_enable = []
                for sensor in shimmer.get_available_sensors():
                    if sensor["name"] in dct["enabled_sensors"]:
                        sensors_to_enable.append(sensor)

                shimmer.set_enabled_sensors(*sensors_to_enable)
                # Writing the old sampling rate
                shimmer.set_sampling_rate(dct["sampling_rate"])
                # Writing the old Wide Accelerometer's range
                shimmer.set_wide_acc_range(dct["wide_acc_range"])
                # Writing the old ExG purpose
                old_exg_purpose = dct["exg_purpose"]
                if old_exg_purpose is util.ExG_ECG:
                    shimmer.exg_send_ecg_settings(dct["exg_gain"])
                elif old_exg_purpose is util.ExG_EMG:
                    shimmer.exg_send_emg_settings(dct["exg_gain"])
                elif old_exg_purpose is util.ExG_RESP:
                    shimmer.exg_send_resp_settings()
                elif old_exg_purpose is util.ExG_TEST:
                    shimmer.exg_send_exg_test_settings()
                shimmer.set_active_gsr_mu(dct["active_gsr_mu"])

            return shimmer
        return dct

    def _reset_to_init(self):
        """
        Reset object's properties to constructor state.
        """
        # CONNECTION ATTRIBUTES

        self._current_state = util.IDLE
        self._com_port = None
        self._serial_port = None
        self._sr_number = None
        self._sr_rev = None

        # SENSORS ATTRIBUTES

        self._enabled_sensors = []
        self._channels = []
        self._sampling_rate = None
        self._buffer_size = 1
        self._num_channels = -1

        self._rtc_ticks = None
        self._rtc_milliseconds = None

        # SETTINGS ATTRIBUTES - not yet used...

        self._configbytes0 = None
        self._configbytes1 = None
        self._configbytes2 = None
        self._configbytes3 = None

        # STREAM ATTRIBUTES

        self._clock_overflows = 0  # how much time a loop restart occurs in timestamp...
        self._previous_timestamp = -1
        self._first_local_timestamp_of_a_stream = -1
        self._first_unix_timestamp_of_a_stream = -1

        self._overflow_data = "".encode()

        # WIDE RANGE ACCELEROMETER ATTRIBUTES
        self._wide_acc_range = None

        # ExG ATTRIBUTES

        self._exg_gain = None
        self._exg_purpose = None
        self._exg_last_resp_preset = None

        # CALIBRATION ATTRIBUTES

        self._active_gsr_mu = util.GSR_SKIN_CONDUCTANCE  # mu = measurement unit
