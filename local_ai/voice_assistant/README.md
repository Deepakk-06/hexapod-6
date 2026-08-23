# Voice Assistant Setup

Fully local voice pipeline: wake word -> speech-to-text -> Ollama ->
text-to-speech. Standalone Python script, not a ROS2 package (unrelated
to robot control) -- run it independently of the ROS2 workspace.

**Not hardware-tested.** Built against each library's documented API,
package names/versions verified to exist on PyPI, and the script
compiles cleanly -- but it hasn't been run against real microphone/
speaker hardware or the actual downloaded models. Expect some
back-and-forth tuning audio device selection and silence-detection
thresholds for your specific mic.

## Prerequisites

- USB microphone connected to the Jetson
- USB speaker or audio output connected to the Jetson
- Ollama already running with a model pulled (see `../README.md`)

## 1. Install Python dependencies

```bash
cd local_ai/voice_assistant
pip3 install -r requirements.txt --break-system-packages
```

## 2. Install the Piper CLI binary

```bash
sudo apt install piper  # if packaged for your distro, otherwise:
# download a release binary for aarch64 from:
# https://github.com/rhasspy/piper/releases
```
Confirm it's on your PATH:
```bash
piper --help
```

## 3. Download a wake word model

openWakeWord ships several pre-trained wake words (e.g. "hey jarvis",
"alexa") -- download one from:
```
https://github.com/dscripka/openWakeWord/releases
```
Look for a `.onnx` file for the wake word you want. Note its filename
(without extension) -- that's the default `--wake-name` the script
expects to match against, unless you pass `--wake-name` explicitly.

(You can also train a fully custom wake word with openWakeWord's
training tools if you want something other than the pre-trained
options -- more setup, see their repo docs.)

## 4. Download a Piper voice

Pick a voice from:
```
https://github.com/rhasspy/piper/blob/master/VOICES.md
```
You need both the `.onnx` model file and its matching `.onnx.json`
config file, in the same directory.

## 5. Test each piece independently before running the full loop

**Microphone**: confirm your mic is detected and recording works:
```bash
python3 -c "
import sounddevice as sd
print(sd.query_devices())
"
```
Note the device index/name for your mic -- if `sounddevice` doesn't
pick the right one automatically, you may need to set it explicitly
(see sounddevice's docs on `sd.default.device`).

**Piper**: confirm text-to-speech works standalone:
```bash
echo "hello, this is a test" | piper --model /path/to/voice.onnx --output_file /tmp/test.wav
aplay /tmp/test.wav
```

**Ollama**: confirm the API responds:
```bash
curl http://127.0.0.1:11434/api/chat -d '{
  "model": "gemma3:4b",
  "messages": [{"role": "user", "content": "hello"}],
  "stream": false
}'
```

## 6. Run the full assistant

```bash
python3 assistant.py \
  --wakeword-model /path/to/hey_jarvis.onnx \
  --piper-model /path/to/voice.onnx \
  --ollama-model gemma3:4b
```

Say the wake word, wait for "Listening for your command...", speak your
question, and it should transcribe, query the model, and speak back the
response.

## Tuning notes

- `WAKE_THRESHOLD` (default 0.5) in `assistant.py`: lower if it's not
  triggering on your wake word, raise if it's triggering on background
  noise/conversation.
- `SILENCE_RMS_THRESHOLD` (default 300): depends heavily on your mic's
  sensitivity and room noise floor -- if it cuts you off mid-sentence,
  raise this; if it never stops listening, lower it. Print the `rms`
  value in `_record_command` while testing to find a good number for
  your setup.
- `faster-whisper` loads the `small` model on first use (lazy-loaded so
  wake-word-only startup is fast) with `device='cuda'` -- if that fails
  to find a usable GPU, change to `device='cpu'` in `assistant.py`'s
  `_transcribe` method; it'll be slower but still work.
