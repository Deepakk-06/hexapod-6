# Local AI Chat Assistant (Jetson, offline)

Text chat via browser (Open WebUI) + full voice assistant pipeline
(wake word -> speech-to-text -> Ollama -> text-to-speech), all running
locally on the Jetson with no cloud dependency.

## 1. Ollama (GPU-accelerated, Jetson-specific container)

Generic `curl | sh` Ollama installs frequently fail to get GPU
acceleration working on Jetson due to CUDA/JetPack version mismatches.
Use the community-maintained container built for your exact JetPack
version instead (find your version with `cat /etc/nv_tegra_release`,
then match the tag below -- e.g. R36.4.x -> `r36.4.0`):

```bash
sudo docker run -d --runtime nvidia --network=host \
  --name ollama \
  -v ~/ollama:/ollama \
  -e OLLAMA_MODELS=/ollama \
  dustynv/ollama:r36.4.0
```

Pull a model sized for your 8GB Orin Nano -- leave headroom for ROS2 +
everything else running, so a 3-4B parameter model at 4-bit
quantization is the sweet spot (roughly 2.5-4GB):

```bash
sudo docker exec -it ollama ollama pull gemma3:4b
```
(alternatives worth trying: `qwen2.5:3b`, `llama3.2:3b` -- pull a couple
and compare, `ollama list` shows what you have)

Test it directly:
```bash
sudo docker exec -it ollama ollama run gemma3:4b
```
Type a message, confirm it responds, `Ctrl-D` to exit.

## 2. Open WebUI (browser-based chat)

```bash
sudo docker run -d --network=host \
  --name open-webui \
  -v ~/open-webui:/app/backend/data \
  -e OLLAMA_BASE_URL=http://127.0.0.1:11434 \
  ghcr.io/open-webui/open-webui:main
```

Then from your VM's browser (or your laptop directly), go to:
```
http://<jetson-ip>:8080
```
First visit asks you to create a local account (stored only on the
Jetson, not sent anywhere) -- then you get a full ChatGPT-style
interface talking to your local model.

## 3. Voice assistant (wake word + speech pipeline)

See `voice_assistant/` in this workspace -- a standalone Python script
(not a ROS2 package, since it's unrelated to robot control) implementing:

    openWakeWord (listens for wake word)
      -> records your command after wake word triggers
      -> faster-whisper (speech-to-text, local)
      -> Ollama (the model, via the same API Open WebUI uses)
      -> Piper (text-to-speech, local)
      -> plays response through your speaker

Setup instructions are in `voice_assistant/README.md`.
