from datetime import datetime

from teaching.interface.communication import DataPacket
from ....node import SensorModule

from .utils.shimmer import Shimmer3
from .utils.shimmer_util import (
    SHIMMER_GSRplus,
    BT_CONNECTED,
    SENSOR_GSR,
    SENSOR_INT_EXP_ADC_CH13,
    GSR_SKIN_CONDUCTANCE,
)
from .utils.ppg_to_hr import PPGtoHR


class ShimmerGSRModule(SensorModule):
    COM_PORT = "/dev/ttyS0"

    def __init__(self, sampling_rate: int, process: bool) -> None:
        """Initializes the Shimmer GSR+ sensor device.

        Args:
            sampling_rate (int): the sampling frequency of the sensor
        """
        super().__init__()
        self.sampling_rate = sampling_rate
        self.process = process

        self._device = None
        self._pipe = None

    def run(self):
        sensing_topic = "sensor.gsr.value"
        for n, reads in self._pipe:
            if n > 0:
                timestamp = reads.pop("timestamp")
                self.send(
                    DataPacket(
                        topic=sensing_topic,
                        timestamp=[datetime.fromtimestamp(t) for t in timestamp],
                        body=[dict(zip(reads, t)) for t in zip(*reads.values())],
                    )
                )

    def build(self):
        print("Building the Shimmer GSR+ service...")
        self._device = Shimmer3(shimmer_type=SHIMMER_GSRplus, debug=True)
        self._connect()

        self._pipe = self._stream()
        if self.process:
            self._pipe = PPGtoHR(self.sampling_rate)(self._pipe)
        print("Done!")

    def _stream(self):
        """This generator handles the stream of PPG, EDA and timestamp data from the Shimmer sensor.

        Raises:
            RuntimeError: raised when bluetooth is not connected

        Yields:
            Iterator[Tuple[int, Dict]]: a dictionary with timestamp, PPG and EDA data.
        """
        if self._device.current_state == BT_CONNECTED:
            self._device.start_bt_streaming()
            print("Data streaming started.")
        else:
            raise RuntimeError("Bluetooth is not connected.")

        while True:
            n, packets = self._device.read_data_packet_extended(calibrated=True)
            reads = {"timestamp": [], "ppg": [], "eda": []}
            for pkt in packets:
                timestamp, ppg, eda = pkt[2], pkt[3], pkt[4]

                reads["timestamp"].append(timestamp)
                reads["ppg"].append(ppg)
                reads["eda"].append(eda)

            yield n, reads

    def _connect(self) -> bool:
        """This function handles the connection via bluetooth with the Shimmer sensor.

        Args:
            port (str, optional): a string with the device path (e.g., '/dev/ttyUSB0'),
                otherwise an input from the user is requested. Defaults to None.

        Returns:
            bool: True if  connected successfully.
        """

        # Starting connection with the port /dev/ttyS0
        if self._device.connect(com_port=ShimmerGSRModule.COM_PORT):
            if not self._device.set_sampling_rate(self._sampling_rate):
                return False
            # After the connection we want to enable GSR and PPG
            if not self._device.set_enabled_sensors(
                SENSOR_GSR, SENSOR_INT_EXP_ADC_CH13
            ):
                return False
            # Set the GSR measurement unit to Skin Conductance (micro Siemens)
            if not self._device.set_active_gsr_mu(GSR_SKIN_CONDUCTANCE):
                return False

            print(f"Shimmer GSR+ connected to {ShimmerGSRModule.COM_PORT}.")
            self._device.print_object_properties()
            return True

        return False

    def _disconnect(self) -> bool:
        """Disconnects the Shimmer sensor.

        Returns:
            bool: True if disconnected successfully.
        """
        if not self._device.current_state == BT_CONNECTED:
            self._device.disconnect(reset_obj_to_init=True)
            return True
        else:
            return False
