#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sounddevice as sd
from vosk import Model, KaldiRecognizer
import json
import os


class VoiceAINode(Node):
    def __init__(self):
        super().__init__('voice_ai_node')
        self.publisher_ = self.create_publisher(String, '/voice_command', 10)

        # Load Vosk model (ensure you have the 'model' directory in your path)
        # You can download 'vosk-model-small-en-us' and put it in your workspace
        model_path = os.path.join(os.path.dirname(__file__), "vosk-model-small-en-us")
        self.model = Model(model_path)
        self.rec = KaldiRecognizer(self.model, 16000)

        # ── Instrument alias table ──────────────────────────────────
        # Small Vosk models frequently mis-hear these words. Each
        # instrument maps to a set of words/phrases that should count
        # as a match. Add more aliases here as you observe mis-hears
        # in the log (see "Heard so far" / "Final phrase detected").
        self.instrument_aliases = {
            'forceps': {
                'forceps', 'for', 'four', 'fours', 'force', 'forces',
                'sense', 'foreceps', 'for steps', 'four steps',
            },
            'scalpel': {
                'scalpel', 'scalp', 'scalped', 'scalpal', 'skull pill',
                'scalple',
            },
            'retractor': {
                'retractor', 'retract', 'retractors', 'protractor',
                'reactor', 'attractor', 're tractor',
            },
        }

        self.get_logger().info('Vosk Voice AI initialized. Listening...')

        # Start audio stream
        self.stream = sd.RawInputStream(samplerate=16000, blocksize=8000,
                                         dtype='int16', channels=1, callback=self.audio_callback)
        self.stream.start()

    def audio_callback(self, indata, frames, time, status):
        # 1. Capture the audio data
        data = bytes(indata)

        # 2. Check for Final result (Speech has paused)
        if self.rec.AcceptWaveform(data):
            result = json.loads(self.rec.Result())
            text = result.get('text', '')
            if text:
                self.get_logger().info(f"Final phrase detected: '{text}'")
                self.process_text(text)
        else:
            # 3. CATCH-ALL: Print Partial result (Speech in progress)
            partial = json.loads(self.rec.PartialResult())
            partial_text = partial.get('partial', '')
            if partial_text:
                # This will stream what the AI hears in real-time
                self.get_logger().info(f"Heard so far: '{partial_text}'", throttle_duration_sec=1.0)

    def process_text(self, text):
        # Catch-all: log everything that the AI considers "Final"
        self.get_logger().info(f"Processing final text: {text}")

        text_lower = text.strip().lower()
        words = text_lower.split()
        found = False
        matched_instruments = set()

        for instrument, aliases in self.instrument_aliases.items():
            # Check single-word aliases against each word in the phrase
            # (avoids "for" inside "before" triggering a false match)
            if any(w in aliases for w in words):
                matched_instruments.add(instrument)
                continue
            # Check multi-word aliases (e.g. "for steps") against the
            # full phrase, since split() would break them apart
            if any(' ' in alias and alias in text_lower for alias in aliases):
                matched_instruments.add(instrument)

        for instrument in matched_instruments:
            self.get_logger().info(f"MATCH FOUND: {instrument}")
            msg = String()
            msg.data = instrument
            self.publisher_.publish(msg)
            found = True

        if not found:
            self.get_logger().info("No instrument detected in phrase.")


def main(args=None):
    rclpy.init(args=args)
    node = VoiceAINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
