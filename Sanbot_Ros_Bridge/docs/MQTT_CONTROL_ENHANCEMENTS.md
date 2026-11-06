# MQTT control enhancements (wings / emotion / autonomy)

## Topics

### Wings — `/joints`

```json
{"joint":"wing_left","action":"up","speed":50}
{"joint":"wing_right","action":"down","speed":50}
{"joint":"wing_both","action":"stop","speed":50}
```

Actions: `up` / `raise` | `down` / `drop` | `stop` | `reset` | `wave`  
Angle without action: **≥ 135 → raise**, else drop.

**Nano MainService (`1.0.0_alexa_release`):** discrete **up/down/stop** use hand Order **545** (`NoAngleHandMotion`). Explicit `angle` uses **547** (absolute). `CombinationHandMotion` / cmd **4374** is ACK’d and never reaches the MCU. Keep setting `robot_arm_action=true`.

### Head pan — `/joints`

```json
{"joint":"head_pan","angle":90,"speed":50}
```

**Nano quirk:** MainService AbsoluteAngle `action` is motor axis (**1**=vertical, **2**=horizontal). Stock dsdk `ACTION_START` in `action` made pan behave as tilt. Bridge sends `(byte) 2` for pan.

### Emotion — `/emotion`

```json
{"emotion":"smile"}
```

OpenSDK `showEmotion` → MCU expression (not `FACE_IMAGE` JPEG load). Visible on the **face screen**; Sanbot-ros UI foreground hides it.

### Autonomy — `/autonomy`

```json
{"mode":"wander","enabled":true}
{"mode":"wander","enabled":false}
{"mode":"charge","enabled":true}
{"mode":"follow","enabled":true}
```

`charge` / `auto_charge` / `dock` → `switchCharge`  
`wander` → `switchWander`  
`follow` → launches in-repo face-follow app **`Sanbot_HumanFollow/`** (package **`com.qihan.HumanFollow`**). Build/install that APK; bridge notifies it via `SET_FOLLOWING` / `SET_SPEECH_FEEDBACK` (does not call `switchFollow(true)` — that arms onboard cruise).  

Follow speech mute:

```json
{"mode":"follow","speech_feedback":false}
```

Combined:

```json
{"mode":"follow","enabled":true,"speech_feedback":true}
```

## Live test status (Sanbot Nano, Oct 2026)

| Topic / mode | Status | Notes |
|--------------|--------|--------|
| `/joints` pan + wings | **Tested** | Nano MainService quirks fixed (pan `action=2`, wings **545** up/down + **547** absolute). |
| `/emotion` | **Partially tested** | MQTT + MainService cmd **1091** OK; visible only on face screen. |
| `/autonomy` **wander** | **Tested** | Starts; stops on floor joint / obstacle (expected). |
| `/autonomy` **dock** / charge | **Not physically tested** | MQTT/cmd **785** + `changebattery` lock OK; no end-to-end run to the charging pile. |
| `/autonomy` **follow** | **Face-follow stand-in** | Stock Qihan APK not found. Install **`Sanbot_HumanFollow/`** (`com.qihan.HumanFollow`). Face-bbox track only — not Orbbec body follow. |
