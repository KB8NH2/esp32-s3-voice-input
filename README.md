# esp32-s3-voice-input

ESP32-S3 voice endpoint: capture audio, VAD/PTT controls, stream WAV to a local Whisper STT server, and forward transcriptions to a Conversation service.

Quick start

- Build and flash:

```bash
idf.py build flash monitor
```

- Use `key3` as push-to-talk (toggle) or rely on VAD to auto-send on speech end.

License

This project is released under the MIT License. See `LICENSE`.
