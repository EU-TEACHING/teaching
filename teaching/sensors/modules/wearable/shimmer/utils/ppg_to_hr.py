from datetime import datetime


class PPGtoHR:
    def __init__(self, sampling_rate: int = 64, seconds_per_return: float = 1):
        """This object processes the stream data from the Shimmer GSR+, aggregating samples for a seconds_per_return
        timespan and processing PPG to HR.

        Args:
            sampling_rate: the sampling rate of the device
            seconds_per_return (float, optional): the timespan covered by each yielded sample. Defaults to 1.
        """

        self._sampling_rate = sampling_rate
        self._seconds_per_return = seconds_per_return
        self._alg = PPGtoHRAlgorithm(
            sampling_rate=sampling_rate,
            number_of_beats_to_average=1,
            use_last_estimate=1,
        ).ppg_to_hr

    def __call__(self, stream):
        """This generator processes the stream data from the Shimmer GSR+, aggregating samples for a seconds_per_return
        timespan and processing PPG to HR.

        Args:
            stream: the device stream function

        Yields:
            dict: a dictionary containing the timestamp, HR and EDA of the processed sample
        """
        span = self._sampling_rate * self._seconds_per_return
        residual = {"timestamp": [], "hr": [], "eda": []}
        n_samples = 0
        for n, reads in stream:
            if n > 0:
                n_samples += n
                reads["hr"] = [
                    self._alg(ppg, reads["timestamp"][i] * 1000)[0]
                    for i, ppg in enumerate(reads["ppg"])
                ]
                reads["timestamp"] = [
                    datetime.fromtimestamp(t).strftime("%H:%M:%S.%f")[:-3]
                    for t in reads["timestamp"]
                ]

                residual["timestamp"] += reads["timestamp"]
                residual["hr"] += reads["hr"]
                residual["eda"] += reads["eda"]
            if n_samples >= span:
                to_publish = {
                    "timestamp": residual["timestamp"][:span],
                    "hr": residual["hr"][:span],
                    "eda": residual["eda"][:span],
                }

                residual = {
                    "timestamp": residual["timestamp"][span:],
                    "hr": residual["hr"][span:],
                    "eda": residual["eda"][span:],
                }
                n_samples -= span

                yield span, to_publish


INVALID_RESULT = 60


class PPGtoHRAlgorithm:
    def __init__(self, sampling_rate, number_of_beats_to_average, use_last_estimate):
        self._peak_ppg_data = []
        self._peak_timestamps = []
        self._ppg_data = []
        self._timestamps = []

        self._value_mean = None
        self._value_peak = None

        self._slope = 0.0
        self._threshold = 0.0
        self._climbing = False

        self._last_known_hr = -60.0
        self._heart_rate = INVALID_RESULT

        self._pulse_period = 0.0
        self._first_pass = True

        self._default_buffer_size = 2
        self._IBI_test_buffer_size = 3
        self._use_last_estimate = use_last_estimate
        self._sampling_rate = sampling_rate

        self._hr_upper_limit = 215  # above 215 the HR is considered invalid
        self._hr_lower_limit = 30  # below 30 the HR is considered invalid

        self._ppg_data_buffer_size = int(self._sampling_rate * 2.0)
        # print("PPG Data Buffer: ", self._ppg_data_buffer_size)

        self._default_number_of_beats_to_average = 1

        if number_of_beats_to_average < 1:
            self._number_of_beats_to_average = 1
        else:
            self._number_of_beats_to_average = number_of_beats_to_average

        self._number_of_samples_since_peak = 0

        self._max_half_beat_interval = 300.0

        self._set_parameters(
            sampling_rate, number_of_beats_to_average, self._default_buffer_size
        )

    def ppg_to_hr(self, ppg_sample, timestamp_sample):
        self._ppg_data.append(ppg_sample)
        self._timestamps.append(timestamp_sample)
        if len(self._ppg_data) < self._ppg_data_buffer_size:
            # We don't have enough data to compute an Heart Rate
            return [INVALID_RESULT, INVALID_RESULT]

        if self._first_pass:
            self._value_mean = min(self._ppg_data)
            self._value_peak = max(self._ppg_data)
            self._slope = 0.0
            self._threshold = 0.0
            self._climbing = True
            self._calculate_pulse_period()
            self._first_pass = False

        hr = self._compute_heart_rate()
        if len(self._peak_timestamps) > 2:
            return [hr, self._peak_timestamps[-1] - self._peak_timestamps[-2]]
        else:
            return [hr, INVALID_RESULT]

    def _calculate_pulse_period(self):
        self._pulse_period = self._sampling_rate / abs(self._last_known_hr / 60.0)

    def _compute_heart_rate(self):
        if len(self._ppg_data) < self._ppg_data_buffer_size:
            return self._heart_rate

        # Take only the last 'ppg_data_buffer_size' data
        for i in range(len(self._ppg_data) - self._ppg_data_buffer_size):
            self._ppg_data.pop(0)
            self._timestamps.pop(0)

        new_peak_found = False
        # Check if the threshold should be increased and check also if set climbing to True
        if (
            self._ppg_data[self._ppg_data_buffer_size - 1]
            > self._ppg_data[self._ppg_data_buffer_size - 2]
            and self._ppg_data[self._ppg_data_buffer_size - 1] > self._threshold
        ):
            self._threshold = self._ppg_data[self._ppg_data_buffer_size - 1]
            self._number_of_samples_since_peak = 0
            self._climbing = True
        else:
            # If we were climbing but the last data is less than the penultimate so the penultimate was a peak
            if self._climbing:
                self._peak_timestamps.append(
                    self._timestamps[self._ppg_data_buffer_size - 2]
                )
                self._peak_ppg_data.append(
                    self._ppg_data[self._ppg_data_buffer_size - 2]
                )
                self._value_peak = self._ppg_data[self._ppg_data_buffer_size - 2]
                self._value_mean = min(self._ppg_data)
                self._calculate_pulse_period()
                self._calculate_slope()
                new_peak_found = True
                self._number_of_samples_since_peak = 1
            else:
                # If we were not climbing we are going down so the peak has been exceeded by 'number_of_samples_since'
                # (plus one now)
                self._number_of_samples_since_peak += 1

            self._calculate_threshold()
            self._climbing = False
        if new_peak_found:
            if self._heart_rate != INVALID_RESULT:
                self._remove_false_peaks()

            number_peaks = len(self._peak_ppg_data)
            min_samples_needed = self._number_of_beats_to_average + 2
            if self._IBI_test_buffer_size + 1 > self._number_of_beats_to_average + 2:
                min_samples_needed = self._IBI_test_buffer_size + 1

            if number_peaks < min_samples_needed:
                self._heart_rate = INVALID_RESULT
            else:
                self._calculate_heart_rate(number_peaks)
                for j in range(0, len(self._peak_ppg_data) - min_samples_needed):
                    self._peak_ppg_data.pop(0)
                    self._peak_timestamps.pop(0)
        return self._heart_rate

    def _calculate_slope(self):
        self._slope = -abs(
            2.0 * (self._value_peak - self._value_mean) / (4.5 * self._pulse_period)
        )

    def _calculate_threshold(self):
        self._threshold = (
            self._value_peak + self._number_of_samples_since_peak * self._slope
        )

    def _remove_false_peaks(self):
        half_beat_size = 1000.0 / abs(self._last_known_hr / 60.0) * 0.5
        if half_beat_size > self._max_half_beat_interval:
            half_beat_size = self._max_half_beat_interval

        index = 0
        continue_loop = True
        while continue_loop:
            if len(self._peak_ppg_data) < 2 or index >= len(self._peak_ppg_data) - 2:
                continue_loop = False
            else:
                timestamp_diff = (
                    self._peak_timestamps[index + 1] - self._peak_timestamps[index]
                )
                if timestamp_diff < half_beat_size:
                    if self._peak_ppg_data[index] > self._peak_ppg_data[index + 1]:
                        self._peak_ppg_data.pop(index + 1)
                        self._peak_timestamps.pop(index + 1)
                    else:
                        self._peak_ppg_data.pop(index)
                        self._peak_timestamps.pop(index)
                else:
                    index += 1

    def _calculate_heart_rate(self, number_peaks):
        inter_beat_intervals = self._subtract_two_lists(
            self._peak_timestamps[
                number_peaks - self._number_of_beats_to_average - 1 : number_peaks - 1
            ],
            self._peak_timestamps[
                number_peaks - self._number_of_beats_to_average - 2 : number_peaks - 2
            ],
        )

        ibi_buffer = []
        for i in range(0, self._number_of_beats_to_average):
            ibi_buffer.append(inter_beat_intervals[i])

        ibi_test_buffer = self._subtract_two_lists(
            self._peak_timestamps[
                number_peaks - self._IBI_test_buffer_size : number_peaks
            ],
            self._peak_timestamps[
                number_peaks - self._IBI_test_buffer_size - 1 : number_peaks - 1
            ],
        )

        hr_test_buffer = []
        for j in range(0, self._IBI_test_buffer_size):
            hr_test_buffer.append(1.0 / ibi_test_buffer[j])

        hr_median = self._get_median(hr_test_buffer)
        hr_range = max(hr_test_buffer) - min(hr_test_buffer)

        if hr_median * 0.3 < hr_range:
            if self._use_last_estimate:
                self._heart_rate = self._last_known_hr
            else:
                self._heart_rate = INVALID_RESULT
        else:
            avg_interval = self._calculate_mean(ibi_buffer)
            if avg_interval == 0.0:
                self._heart_rate = INVALID_RESULT
            else:
                self._heart_rate = 60000.0 / avg_interval

            if (
                self._heart_rate > self._hr_upper_limit
                or self._heart_rate < self._hr_lower_limit
            ):
                if self._use_last_estimate:
                    self._heart_rate = self._last_known_hr
                else:
                    self._heart_rate = INVALID_RESULT
            else:
                self._last_known_hr = self._heart_rate

    @staticmethod
    def _subtract_two_lists(list_a, list_b):
        result = []
        for i in range(0, len(list_a)):
            result.append(list_a[i] - list_b[i])
        return result

    @staticmethod
    def _calculate_mean(lst):
        s = 0.0
        num_elements = len(lst)
        for element in lst:
            s += element
        if num_elements > 0:
            return s / num_elements

    @staticmethod
    def _get_median(lst):
        lst.sort()
        num_elements = len(lst)
        middle = int(num_elements / 2)
        if num_elements % 2 == 0:
            median_a = lst[middle]
            median_b = lst[middle - 1]
            median = (median_a + median_b) / 2.0
        else:
            median = lst[middle + 1]

        return median

    def _reset_member_variables(self):
        self._climbing = True
        self._threshold = 0.0
        self._hr_upper_limit = 215
        self._hr_lower_limit = 30
        self._last_known_hr = -60.0
        self._heart_rate = INVALID_RESULT

        self._value_peak = 0.0
        self._value_mean = 0.0
        self._number_of_samples_since_peak = 0

        self._slope = 0.0
        self._threshold = 0.0
        self._climbing = True
        self._first_pass = True

        self._ppg_data = []
        self._timestamps = []

    # noinspection PyUnusedLocal
    def _set_parameters(self, sampling_rate, num_beats_to_average, buffer_size):
        self._reset_member_variables()
        self._sampling_rate = sampling_rate
        if buffer_size <= 0:
            buffer_size = 1
        self._ppg_data_buffer_size = int(self._sampling_rate * 2.0)
        self._number_of_beats_to_average = num_beats_to_average
        if self._number_of_beats_to_average < 1:
            self._number_of_beats_to_average = 1
