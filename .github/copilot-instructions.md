<!-- .github/copilot-instructions.md - project-specific guidance for AI coding agents -->

# Copilot Instructions — esp32-s3-voice-input

Purpose: give an AI coding agent the minimal, concrete context needed to make safe, correct edits in this repository.

- **Build / flash / monitor:** use the ESP-IDF workflow at the repo root: `idf.py build flash monitor` (see [README.md](../README.md)).
- **ESP-IDF environment:** project expects an ESP-IDF workspace (use `export.ps1` / `export.sh` for IDF tools in your environment).

- **Big picture:** This firmware runs on an ESP32-S3 and implements a voice endpoint: capture audio, run VAD, optionally detect wake-words (via background STT), provide push-to-talk (PTT), upload WAV to an STT service, and forward transcriptions to a Conversation server. Key logic lives in `main/main.c` and low-level hardware is in `components/*`.

- **Key files / dirs:**
  - [main/main.c](../main/main.c): PTT state machine, VAD callback (`vad_ptt_notify_cb`), wake-word list, STT upload tasks, Conversation initialization.
  - [README.md](../README.md): quickstart build command.
  - `components/`: device drivers and managed components (audio, TCA9555 expander, LED ring, STT/TTS glue).
  - `CMakeLists.txt`: standard ESP-IDF project entrypoint.

- **Concrete, editable knobs:**
  - Wake-words: edit the `wakes[]` array in `main/main.c` (search for `const char *wakes[]`).
  - Conversation server: default URL is passed to `conversation_init(...)` in `app_main()` — update to point at your backend.
  - VAD callback: `vad_set_callback(vad_ptt_notify_cb)` is registered in `app_main()`; VAD behavior and capture buffering are implemented in `components`.

- **Concurrency & memory notes (important for safe edits):**
  - FreeRTOS tasks are frequently pinned to core 1 (see `xTaskCreatePinnedToCore` calls). Be cautious when increasing stack size or local allocations; large STT tasks use big stacks (`stt_sender_task` allocation uses 16384 bytes in `main.c`).
  - Audio buffers are heap-allocated and handed between tasks (`mic_take_captured_buffer`, `stt_send_wav_multipart`) — free ownership is explicit in the code. Preserve allocation/free pairs when refactoring.

- **Common patterns & APIs used:**
  - `mic_start_capture()`, `mic_stop_capture()`, `mic_take_captured_buffer(&samples)` — audio capture lifecycle.
  - `vad_set_callback()` — register VAD notifications; VAD will call with captured PCM samples.
  - `conversation_send()` / `conversation_set_callback()` — send text to conversation service and receive replies.
  - LED ring helpers: `led_ring_set_color()` / `led_ring_clear()` used for UI feedback.

- **What to avoid / watch for:**
  - Do not introduce blocking network calls on timing-critical tasks; offload STT uploads to separate tasks (existing pattern uses `stt_sender_task`).
  - Avoid unbounded stack usage in tasks. Prefer heap buffers handed between tasks when working with large audio chunks.

- **Testing & debugging tips:**
  - Use `idf.py monitor` to view `ESP_LOG*` output; main runtime logs are already instrumented (`TAG = "main"`).
  - To test wake-word logic, edit `wakes[]` and trigger via background STT (VAD segments are sent to STT when not in PTT mode).

If anything above is unclear or you want an expanded section (e.g., mapping of component names to C files, wiring for the TCA9555 expander, or examples of safe refactors), tell me which section to expand.
