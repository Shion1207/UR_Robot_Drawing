#!/usr/bin/env python3
"""Local Vietnamese Whisper ASR with real microphone endpointing and no cloud API."""
from collections import deque
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


class SpeechRecognition(Node):
    def __init__(self):
        super().__init__('speech_recognition_node')
        root = Path(get_package_share_directory('assign7_bringup'))
        for key, value in dict(model_path=str(root/'models/whisper-base'), audio_device='pulse',
                               audio_file='', language='vi', energy_threshold=0.012,
                               silence_seconds=0.7, max_phrase_seconds=8.0,
                               max_command_age=12.0, cpu_threads=2).items():
            self.declare_parameter(key, value)
        self.options = {p: self.get_parameter(p).value for p in (
            'model_path','audio_device','audio_file','language','energy_threshold',
            'silence_seconds','max_phrase_seconds','max_command_age','cpu_threads')}
        self.text_pub = self.create_publisher(String, '/voice/text', 10)
        self.status_pub = self.create_publisher(String, '/voice/speech_status', 10)
        self.events, self.audio = queue.Queue(), queue.Queue(maxsize=2)
        self.stop_event, self.capture = threading.Event(), None
        self.worker = threading.Thread(target=self.recognize, daemon=True)
        self.worker.start()
        self.create_timer(0.1, self.flush)

    def flush(self):
        while not self.events.empty():
            kind, value = self.events.get_nowait()
            if kind == 'text':
                self.get_logger().info('NGHE ĐƯỢC: '+value)
                self.text_pub.publish(String(data=value))
            else:
                self.get_logger().info(value)
                self.status_pub.publish(String(data=value))

    def record(self):
        try:
            self.capture = subprocess.Popen(
                ['arecord','-q','-D',str(self.options['audio_device']),'-t','raw',
                 '-f','S16_LE','-r','16000','-c','1'], stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            pre = deque(maxlen=4)
            chunks, voiced, silent = [], 0, 0
            self.events.put(('status', 'MIC ĐANG NGHE. Nói một câu, rồi im lặng khoảng 1 giây.'))
            while not self.stop_event.is_set():
                pcm = self.capture.stdout.read(3200)  # 100 ms, mono 16 kHz signed PCM.
                if len(pcm) != 3200:
                    error = self.capture.stderr.read(2000).decode(errors='replace')
                    self.events.put(('status', 'Không đọc được microphone: '+error+
                                     ' Kiểm tra arecord -L; thử audio_device:=default.'))
                    break
                block = np.frombuffer(pcm, dtype='<i2').astype(np.float32)/32768
                speech = float(np.sqrt(np.mean(block*block))) >= self.options['energy_threshold']
                if not chunks:
                    pre.append(block)
                    if not speech:
                        continue
                    chunks = list(pre)
                    voiced, silent = 1, 0
                else:
                    chunks.append(block)
                    voiced += int(speech)
                    silent = 0 if speech else silent+1
                if silent*.1 >= self.options['silence_seconds'] or len(chunks)*.1 >= self.options['max_phrase_seconds']:
                    if voiced >= 4:
                        utterance = (np.concatenate(chunks), time.monotonic())
                        if self.audio.full():
                            try:
                                self.audio.get_nowait()
                            except queue.Empty:
                                pass
                        self.audio.put_nowait(utterance)
                    chunks, voiced, silent = [], 0, 0
                    pre.clear()
        except Exception as exc:
            self.events.put(('status', f'Lỗi microphone: {exc}'))

    def recognize(self):
        try:
            from faster_whisper import WhisperModel
            model_path = Path(str(self.options['model_path'])).expanduser()
            if not (model_path/'model.bin').is_file():
                raise RuntimeError('Thiếu model Whisper. Chạy ./scripts/install_deps.bash rồi build lại.')
            self.events.put(('status', 'Đang nạp Whisper tiếng Việt trên CPU...'))
            model = WhisperModel(str(model_path), device='cpu', compute_type='int8',
                                 cpu_threads=int(self.options['cpu_threads']), local_files_only=True)
            audio_file = str(self.options['audio_file'])
            if audio_file:
                if not Path(audio_file).is_file():
                    raise ValueError(f'Không có file âm thanh: {audio_file}')
                self.audio.put((audio_file, time.monotonic()))
            else:
                threading.Thread(target=self.record, daemon=True).start()
            while not self.stop_event.is_set():
                try:
                    audio, captured_at = self.audio.get(timeout=.2)
                except queue.Empty:
                    continue
                self.events.put(('status', 'Đang nhận dạng giọng nói...'))
                segments, _ = model.transcribe(
                    audio, language=str(self.options['language']), beam_size=5,
                    vad_filter=True, condition_on_previous_text=False,
                    initial_prompt='Đi đến khu vực A. Đi từ khu vực A đến khu vực B. '
                    'Đi đến phòng học. Đi đến phòng thí nghiệm. Đi qua hành lang. Dừng lại.')
                segments = [s for s in segments if s.no_speech_prob < .6 and s.avg_logprob > -1.0]
                text = ' '.join(s.text.strip() for s in segments).strip()
                if time.monotonic()-captured_at > self.options['max_command_age'] and not audio_file:
                    self.events.put(('status', 'Bỏ lệnh đã quá cũ do nhận dạng chậm. Nói lại sau khi CPU rảnh.'))
                elif text:
                    self.events.put(('text', text))
                else:
                    self.events.put(('status', 'Chưa nghe rõ; chưa phát lệnh. Hãy nói lại.'))
                if audio_file:
                    self.events.put(('status', 'Đã xử lý xong file âm thanh.'))
                    return
                self.events.put(('status', 'Sẵn sàng nghe câu tiếp theo.'))
        except Exception as exc:
            self.events.put(('status', f'Lỗi nhận dạng: {exc}'))

    def destroy_node(self):
        self.stop_event.set()
        if self.capture and self.capture.poll() is None:
            self.capture.terminate()
        self.worker.join(timeout=1)
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = SpeechRecognition()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        if node:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
