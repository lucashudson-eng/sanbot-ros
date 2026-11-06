# Sanbot Human Follow (Nano face-follow)

Replacement for the missing stock **`com.qihan.HumanFollow`** package on Sanbot Nano.

This is **not** Qihan’s original APK. It uses OpenSDK **face recognition** boxes to center on a person and keep a stand-off distance with the wheel APIs. Pair it with **`Sanbot_Ros_Bridge/`**: MQTT `/autonomy` follow launches this package.

## Version

Set in `app/build.gradle` (`versionName` / `versionCode`). Shown on the Human Follow HUD as **`v{versionName} ({versionCode})`**.

Current: **`v1.2.0 (4)`** — Optional local **Google TTS / ASR** (default on; OpenSDK cloud fallback), HUD toggles, Close / Detect Human, sticky lock / quiet detect TTS.

## Behaviour

- Package id: **`com.qihan.HumanFollow`** (matches MainService voice / launch intents).
- Side-by-side **HD camera preview** + **IR radar**. Tracks **person** (face→body box + motion blob); sticky teal **LOCK** holds through FaceDetector flicker (~5.5 s follow / ~7 s detect-test).
- Follow uses **continuous** wheel keep (`LEFT_FORWARD` / `FORWARD` / …). Stand-off uses **face height**.
- Chase is **face-only**; motion blobs are search hints (not drive targets). Detect-test may show motion for sticky UI only.
- On follow, head poses **pan center (90°)** and **tilt mid–high** and re-centers periodically.
- **Accessibility gesture:** raise an open hand above your face to **start/stop** follow.
- **Voice control** (after wake): “follow me” / “stop follow” / “speech on|off” / “status”.
- **Speech feedback** (on-screen or MQTT `speech_feedback`): status phrases at **50% speed / 50% volume**. Detect-test avoids TTS storms (no “No human” / “Detect on” chatter).
- **Google TTS / Google ASR** (HUD checkboxes, default on): bypass unreliable Qihan MainService / iFlytek cloud speak-recognize via on-device Google TTS + `SpeechRecognizer`. Uncheck to fall back to OpenSDK. Needs Google TTS / Google Search on Nano + mic permission. (Same pattern as CapBot Nano Google TTS PR.)
- Listens for stock broadcasts:
  - `com.qinhancloud.humanfollow.open` / `.close` (ObstacleManager typo)
  - `com.qihancloud.obstacle.humanfollow.open` / `.close`
- Accepts intent extra `follow` (boolean) — same key MainService uses.
- Manual **Start follow** / **Stop** on the activity UI.
- Stops on obstacle reports; searches with a slow turn when the face is lost.
- No Orbbec skeleton / `com.hfisone` required — local vision only.

## Build

Requires JDK 11+ and Android SDK (`local.properties` with `sdk.dir=…`).

```bash
cd Sanbot_HumanFollow
./gradlew :app:assembleDebug
```

APK: `app/build/outputs/apk/debug/app-debug.apk`

## Install on Nano

```bash
adb connect <robot-ip>:5555
adb install -r -d app/build/outputs/apk/debug/app-debug.apk
adb shell am start -n com.qihan.HumanFollow/.HumanFollowActivity
```

Also install / run **`Sanbot_Ros_Bridge`** connected to your MQTT broker. Then:

```json
{"mode":"follow","enabled":true}
```

on topic `/autonomy` (or `/<namespace>/autonomy`). Prefer leaving the face camera clear of the ROS bridge UI while testing.

Mute status TTS (keep follow as-is):

```json
{"mode":"follow","speech_feedback":false}
```
