#!/usr/bin/env python3
"""
voice_ai_node.py (HybridSTT edition)

Drop-in replacement for the Vosk-based voice_ai_node. Same public
interface: listens on the mic, publishes recognized commands as
std_msgs/String on /voice_command -- so pick_place_coordinator and
everything downstream needs zero changes.

Backed by your whisper-stt repo's HybridSTT class instead of raw
faster-whisper: Whisper handles English/Arabic, and Malayalam
auto-switches to the Wav2Vec2 IndicSTT model -- see
https://github.com/Harbinger-Bong/whisper-stt

Whisper/IndicSTT aren't streaming/grammar-constrained recognizers, so
this node does its own utterance segmentation:

  1. Continuously read mic frames (sounddevice, 16 kHz mono).
  2. Track RMS energy. When energy crosses `vad_threshold`, start
     buffering an utterance.
  3. When energy stays below threshold for `silence_ms`, consider the
     utterance finished, run it through HybridSTT, publish the result.

This is a simple energy-based VAD, not webrtcvad/silero -- swap it in
if you get too many false triggers from motor/gripper noise near the
mic.

Params (ros2 run ... --ros-args -p name:=value):
  stt_repo_path      absolute path to the whisper-stt repo checkout
                     (default: ~/whisper-stt) -- added to sys.path so
                     `from src.hybrid_stt import HybridSTT` resolves
  config_path        absolute path to config.yaml (default:
                     <stt_repo_path>/config/config.yaml)
  sample_rate        16000 (expected rate -- don't change)
  vad_threshold      RMS energy to trigger recording (default 0.02)
  silence_ms         trailing silence before cutting utterance (default 700)
  max_utterance_s    hard cap per utterance (default 8.0)

Install (in the whisper-stt repo):
  pip install -r requirements.txt --break-system-packages
  # plus, for this node:
  pip install sounddevice numpy --break-system-packages

Run:
  ros2 run kuka_ros2_demo voice_ai_node \
      --ros-args -p stt_repo_path:=/home/emil/whisper-stt
"""

import os
import sys

import numpy as np
import sounddevice as sd
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class VoiceAiNode(Node):

    def __init__(self):
        super().__init__('voice_ai_node')

        self.declare_parameter('stt_repo_path', os.path.expanduser('~/hybrid-speech-to-text'))
        self.declare_parameter('config_path', '')
        self.declare_parameter('sample_rate', 16000)
        self.declare_parameter('vad_threshold', 0.02)
        self.declare_parameter('silence_ms', 700)
        self.declare_parameter('max_utterance_s', 8.0)

        self.stt_repo_path = self.get_parameter('stt_repo_path').value
        self.sample_rate = self.get_parameter('sample_rate').value
        self.vad_threshold = self.get_parameter('vad_threshold').value
        self.silence_ms = self.get_parameter('silence_ms').value
        self.max_utterance_s = self.get_parameter('max_utterance_s').value

        config_path = self.get_parameter('config_path').value
        if not config_path:
            config_path = os.path.join(self.stt_repo_path, 'config', 'config.yaml')

        # HybridSTT lives in <stt_repo_path>/src/hybrid_stt.py -- make it importable
        if self.stt_repo_path not in sys.path:
            sys.path.insert(0, self.stt_repo_path)

        try:
            from src.hybrid_stt import HybridSTT
        except ImportError as e:
            self.get_logger().error(
                f'Could not import HybridSTT from {self.stt_repo_path}: {e}. '
                f'Set the stt_repo_path parameter to your whisper-stt checkout.')
            raise

        self.get_logger().info(
            f'Loading HybridSTT (Whisper + IndicSTT) using config {config_path} ...')
        self.stt = HybridSTT(config_path=config_path)
        self.get_logger().info('HybridSTT loaded (Whisper for en/ar, IndicSTT for ml).')

        self.pub = self.create_publisher(String, '/voice_command', 10)

        # -- VAD / buffering state --------------------------------------
        self._buffer = []
        self._recording = False
        self._silence_frames = 0
        self._block_size = 1600  # 0.1s @ 16kHz
        self._silence_blocks_needed = max(
            1, int(self.silence_ms / 1000.0 / (self._block_size / self.sample_rate)))
        self._max_blocks = int(
            self.max_utterance_s / (self._block_size / self.sample_rate))

        self.stream = sd.InputStream(
            samplerate=self.sample_rate,
            channels=1,
            dtype='float32',
            blocksize=self._block_size,
            callback=self._audio_callback,
        )
        self.stream.start()
        self.get_logger().info(
            f'Listening on mic (sample_rate={self.sample_rate}, '
            f'vad_threshold={self.vad_threshold}, silence_ms={self.silence_ms})')

    # -- Audio callback (runs in sounddevice's own thread) ------------------

    def _audio_callback(self, indata, frames, time_info, status):
        if status:
            self.get_logger().warn(f'Audio stream status: {status}')

        block = indata[:, 0].copy()
        rms = float(np.sqrt(np.mean(block ** 2)))

        if not self._recording:
            if rms >= self.vad_threshold:
                self._recording = True
                self._buffer = [block]
                self._silence_frames = 0
            return

        # Already recording
        self._buffer.append(block)

        if rms < self.vad_threshold:
            self._silence_frames += 1
        else:
            self._silence_frames = 0

        finished_on_silence = self._silence_frames >= self._silence_blocks_needed
        finished_on_maxlen = len(self._buffer) >= self._max_blocks

        if finished_on_silence or finished_on_maxlen:
            audio = np.concatenate(self._buffer)
            self._recording = False
            self._buffer = []
            self._silence_frames = 0
            # Hand off to a non-realtime callback context for transcription
            self._transcribe_and_publish(audio)

    # -- Transcription -------------------------------------------------------

    def _transcribe_and_publish(self, audio: np.ndarray):
        duration = len(audio) / self.sample_rate
        if duration < 0.3:
            return  # too short, likely a spurious trigger

        try:
            result = self.stt.transcribe(audio_array=audio, sample_rate=self.sample_rate)
        except Exception as e:
            self.get_logger().error(f'HybridSTT transcription failed: {e}')
            return

        text = (result.get('text') or '').strip()
        if not text:
            self.get_logger().debug('Transcription returned empty text, skipping.')
            return

        engine = result.get('engine', '?')
        language = result.get('language', '?')
        self.get_logger().info(
            f'Heard ({duration:.1f}s) [{engine}/{language}]: "{text}"')
        msg = String()
        msg.data = text
        self.pub.publish(msg)

    def destroy_node(self):
        try:
            self.stream.stop()
            self.stream.close()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = VoiceAiNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()