#!/usr/bin/env python3
# encoding: utf-8
"""
assistant.py

Fully local voice assistant: wake word -> record command -> transcribe
-> query local Ollama model -> synthesize speech -> play response.
No cloud services anywhere in this pipeline.

NOT hardware-tested (built and reviewed for API correctness against
each library's documented interface, but not run against real
microphone/speaker hardware or the actual models -- expect to debug
audio device selection and model paths on your end).

Requires (see requirements.txt):
    pip3 install -r requirements.txt --break-system-packages

Also requires, downloaded separately (see README.md for links):
    - An openWakeWord model file (.onnx or .tflite) for your chosen wake word
    - A Piper voice model (.onnx + .onnx.json) for text-to-speech
    - The `piper` CLI binary on your PATH (or adjust PIPER_BIN below)
    - Ollama running with a pulled model (see local_ai/README.md)

Usage:
    python3 assistant.py --wakeword-model /path/to/wakeword.onnx \\
        --piper-model /path/to/voice.onnx \\
        --ollama-model gemma3:4b
"""
import argparse
import io
import subprocess
import tempfile
import time
import wave

import numpy as np
import requests
import sounddevice as sd

SAMPLE_RATE = 16000
FRAME_SAMPLES = 1280  # 80ms at 16kHz, openWakeWord's expected chunk size
WAKE_THRESHOLD = 0.5
SILENCE_RMS_THRESHOLD = 300     # tune against your mic's noise floor
SILENCE_DURATION_S = 1.2        # stop recording after this much continuous silence
MAX_COMMAND_DURATION_S = 12.0
OLLAMA_URL = "http://127.0.0.1:11434"
PIPER_BIN = "piper"


class VoiceAssistant:
    def __init__(self, wakeword_model_path, piper_model_path, ollama_model, wake_name=None):
        from openwakeword.model import Model as WakeModel

        self.wake_model = WakeModel(wakeword_models=[wakeword_model_path])
        # openWakeWord keys its score dict by the model filename stem unless
        # given an explicit name -- if you renamed the file, pass --wake-name
        # to match, otherwise this guesses from the path.
        self.wake_name = wake_name or wakeword_model_path.split('/')[-1].rsplit('.', 1)[0]

        self.piper_model_path = piper_model_path
        self.ollama_model = ollama_model
        self.history = []  # [{"role": "user"/"assistant", "content": "..."}]

        print(f'Loaded wake word model, listening for: {self.wake_name}')

    def run(self):
        print('Listening for wake word... (Ctrl-C to stop)')
        buffer = np.zeros((0,), dtype=np.int16)

        def audio_callback(indata, frames, time_info, status):
            nonlocal buffer
            if status:
                print(f'[audio warning] {status}')
            buffer = np.concatenate([buffer, indata[:, 0]])

        with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16',
                             blocksize=FRAME_SAMPLES, callback=audio_callback):
            while True:
                if len(buffer) >= FRAME_SAMPLES:
                    chunk = buffer[:FRAME_SAMPLES]
                    buffer = buffer[FRAME_SAMPLES:]

                    scores = self.wake_model.predict(chunk)
                    score = scores.get(self.wake_name, 0.0)

                    if score > WAKE_THRESHOLD:
                        print(f'\nWake word detected (score={score:.2f})')
                        self.wake_model.reset()
                        self._handle_command()
                        print('\nListening for wake word...')
                else:
                    time.sleep(0.01)

    def _handle_command(self):
        audio = self._record_command()
        if audio is None or len(audio) < SAMPLE_RATE * 0.3:
            print('(no speech detected, ignoring)')
            return

        text = self._transcribe(audio)
        if not text.strip():
            print('(could not transcribe anything, ignoring)')
            return
        print(f'You said: {text}')

        reply = self._query_ollama(text)
        print(f'Assistant: {reply}')

        self._speak(reply)

    def _record_command(self):
        """Records from the mic until SILENCE_DURATION_S of quiet, or MAX_COMMAND_DURATION_S elapses."""
        print('Listening for your command...')
        recorded = []
        silence_start = None
        start_time = time.time()

        with sd.InputStream(samplerate=SAMPLE_RATE, channels=1, dtype='int16',
                             blocksize=FRAME_SAMPLES) as stream:
            while True:
                chunk, _ = stream.read(FRAME_SAMPLES)
                chunk = chunk[:, 0]
                recorded.append(chunk)

                rms = float(np.sqrt(np.mean(chunk.astype(np.float64) ** 2)))
                now = time.time()

                if rms < SILENCE_RMS_THRESHOLD:
                    if silence_start is None:
                        silence_start = now
                    elif now - silence_start > SILENCE_DURATION_S and now - start_time > 0.5:
                        break
                else:
                    silence_start = None

                if now - start_time > MAX_COMMAND_DURATION_S:
                    print('(max recording time reached)')
                    break

        return np.concatenate(recorded) if recorded else None

    def _transcribe(self, audio_int16):
        from faster_whisper import WhisperModel

        if not hasattr(self, '_whisper_model'):
            # loaded lazily so wake-word-only startup is fast; GPU if available,
            # falls back to CPU automatically if CUDA isn't usable
            self._whisper_model = WhisperModel('small', device='cuda', compute_type='float16')

        audio_float = audio_int16.astype(np.float32) / 32768.0
        segments, _info = self._whisper_model.transcribe(audio_float, language='en')
        return ' '.join(seg.text for seg in segments).strip()

    def _query_ollama(self, user_text):
        self.history.append({'role': 'user', 'content': user_text})
        resp = requests.post(
            f'{OLLAMA_URL}/api/chat',
            json={'model': self.ollama_model, 'messages': self.history, 'stream': False},
            timeout=60,
        )
        resp.raise_for_status()
        reply = resp.json()['message']['content']
        self.history.append({'role': 'assistant', 'content': reply})
        # keep history bounded so context doesn't grow unbounded over a long session
        if len(self.history) > 20:
            self.history = self.history[-20:]
        return reply

    def _speak(self, text):
        with tempfile.NamedTemporaryFile(suffix='.wav') as f:
            subprocess.run(
                [PIPER_BIN, '--model', self.piper_model_path, '--output_file', f.name],
                input=text.encode('utf-8'),
                check=True,
            )
            with wave.open(f.name, 'rb') as wf:
                sr = wf.getframerate()
                data = wf.readframes(wf.getnframes())
                audio = np.frombuffer(data, dtype=np.int16)
            sd.play(audio, samplerate=sr)
            sd.wait()


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--wakeword-model', required=True, help='path to openWakeWord .onnx/.tflite model')
    parser.add_argument('--wake-name', default=None, help='override wake word key if it doesn\'t match the filename')
    parser.add_argument('--piper-model', required=True, help='path to Piper voice .onnx model')
    parser.add_argument('--ollama-model', default='gemma3:4b')
    args = parser.parse_args()

    assistant = VoiceAssistant(args.wakeword_model, args.piper_model, args.ollama_model, args.wake_name)
    try:
        assistant.run()
    except KeyboardInterrupt:
        print('\nStopped.')


if __name__ == '__main__':
    main()
