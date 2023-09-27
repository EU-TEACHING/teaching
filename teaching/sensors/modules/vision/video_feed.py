import threading
import time

from ...node import SensorModule
from teaching.interface.communication import DataPacket
from vidgear.gears import WriteGear, VideoGear, CamGear, StreamGear

output_params = {
    "-preset:v": "veryfast",
    "-tune": "zerolatency",
    "-g": 60,
    "-sc_threshold": 0,
    "-bufsize": "2500",
    "-f": "flv",
    "-input_framerate": 30,
}
stream_params = {
    "-input_framerate": 1,
    "-livestream": True,
    "-f": "flv",
    "-flvflags": "no_duration_filesize",
}


class VideoFeed(SensorModule):
    def __init__(self, rtmp_server: str, rtmp_topic: str, video_source: str):
        super().__init__()
        self.rtmp_server = rtmp_server
        self.rtmp_topic = rtmp_topic
        self.video_source = video_source
        # self._record = bool(os.environ['RECORD'])

        self._stream_in = None
        self._stream_url = None
        self._streamer = None
        self._stream_thread = None
        # self._writer = None
        # # if self._record:
        # self.writer = WriteGear(output_filename=self.stream_url,logging=True,**output_params)

    def run(self):
        addr = f"rtmp://{self.rtmp_server}/live/stream"
        while True:
            self.send(
                DataPacket(topic="sensor.camera.stream_address", body={"address": addr})
            )
            time.sleep(0.5)

    def _start_streaming(self):
        """Performs a continuous stream from the given source."""
        while True:
            frame = self._stream_in.read()
            if frame is None:
                break
            self._streamer.stream(frame)

    def stop(self):
        if self._stream_in is not None:
            self._stream_in.stop()
        if self.rtmp_server is not None:
            # self.writer.close()
            self._streamer.terminate()
        if self._stream_thread is not None:
            self._stream_thread.join()

    def build(self):
        if self.video_source is not None:
            if "camera_" in self.video_source:
                self.video_source = int(self.video_source.split("_"))
                self._stream_in = CamGear(source=self.video_source).start()
            else:
                self._stream_in = VideoGear(source=self.video_source).start()

        if self.rtmp_server is not None:
            self._stream_url = f"rtmp://{self.rtmp_server}/live/stream"
            self._streamer = StreamGear(output=self._stream_url, **stream_params)

        self._stream_thread = threading.Thread(target=self._start_streaming)
        self._stream_thread.start()
