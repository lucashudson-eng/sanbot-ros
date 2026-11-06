package com.qihan.HumanFollow;

import android.Manifest;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Color;
import android.os.Build;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.speech.SpeechRecognizer;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.TextView;

import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.support.v4.content.ContextCompat;

import com.qihan.HumanFollow.video.VisionMediaDecoder;
import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.beans.OperationResult;
import com.sanbot.opensdk.function.beans.FaceRecognizeBean;
import com.sanbot.opensdk.function.beans.EmotionsType;
import com.sanbot.opensdk.function.beans.SpeakOption;
import com.sanbot.opensdk.function.beans.StreamOption;
import com.sanbot.opensdk.function.beans.headmotion.AbsoluteAngleHeadMotion;
import com.sanbot.opensdk.function.beans.speech.Grammar;
import com.sanbot.opensdk.function.beans.speech.GrammarConstant;
import com.sanbot.opensdk.function.beans.speech.RecognizeTextBean;
import com.sanbot.opensdk.function.beans.wheelmotion.NoAngleWheelMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.RelativeAngleWheelMotion;
import com.sanbot.opensdk.function.unit.FaceTrackManager;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.HeadMotionManager;
import com.sanbot.opensdk.function.unit.SpeechManager;
import com.sanbot.opensdk.function.unit.SystemManager;
import com.sanbot.opensdk.function.unit.WheelMotionManager;
import com.sanbot.opensdk.function.unit.interfaces.hardware.InfrareListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.ObstacleListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.PIRListener;
import com.sanbot.opensdk.function.unit.interfaces.media.FaceRecognizeListener;
import com.sanbot.opensdk.function.unit.interfaces.media.MediaStreamListener;
import com.sanbot.opensdk.function.unit.interfaces.speech.RecognizeListener;
import com.sanbot.opensdk.function.unit.interfaces.speech.WakenListener;

import java.lang.reflect.Field;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * Human follow for Sanbot Nano: camera + IR radar, person/face track with sticky lock.
 * Nano has no stock body SDK ({@code com.hfisone} / Orbbec skeleton); we use local
 * face→body expansion plus motion-blob person detect.
 */
public class HumanFollowActivity extends TopBaseActivity implements SurfaceHolder.Callback {

    public static final String EXTRA_FOLLOW = "follow";
    public static final String EXTRA_DETECT = "detect";
    public static final String ACTION_SET_FOLLOWING = "com.qihan.HumanFollow.SET_FOLLOWING";
    public static final String ACTION_SET_DETECTING = "com.qihan.HumanFollow.SET_DETECTING";
    public static final String ACTION_SET_SPEECH_FEEDBACK = "com.qihan.HumanFollow.SET_SPEECH_FEEDBACK";
    public static final String EXTRA_ENABLED = "enabled";
    /** MQTT /autonomy follow: {"speech_feedback":false} mutes status TTS. */
    public static final String EXTRA_SPEECH_FEEDBACK = "speech_feedback";

    private static final String TAG = "HumanFollow";

    /** Soft center band — mild offset uses arc (forward+turn), not yaw-in-place. */
    private static final double CENTER_LO = 0.44;
    private static final double CENTER_HI = 0.56;
    /** Far off-center: spin in place before chasing. */
    private static final double CENTER_HARD_LO = 0.32;
    private static final double CENTER_HARD_HI = 0.68;
    /**
     * Stand-off uses {@link TrackTarget#rangeHeight} (face height for face→body boxes).
     * Typical follow face ≈ 0.08–0.22 of frame at ~1.5–2.5 m.
     */
    private static final double RANGE_TOO_CLOSE = 0.34;
    private static final double RANGE_TOO_FAR = 0.09;
    private static final long CMD_INTERVAL_MS = 220L;
    /**
     * Sticky lock through FaceDetector flicker. Android FaceDetector often drops
     * for 1–2 s even when a person is still in frame.
     */
    private static final long LOCK_HOLD_MS = 5500L;
    /** Detect-test hold — longer so UI does not flap Human detected / No human. */
    private static final long DETECT_LOCK_HOLD_MS = 7000L;
    private static final long TARGET_LOST_MS = 9000L;
    /** Hard stop (furniture / crash) — well inside normal follow stand-off. */
    private static final int IR_CRASH_CM = 12;
    /**
     * Ease-back IR only when this close. With a person lock the subject often
     * lights front IR — do not treat that as “too close” or the robot retreats forever.
     */
    private static final int IR_HOLD_CM = 16;

    /** Head pose for follow camera: pan center (0…180), tilt mid–high (0…37). */
    private static final int HEAD_PAN_CENTER = 90;
    private static final int HEAD_TILT_FOLLOW = 30;
    private static final long HEAD_REPOSE_MS = 5000L;
    private static final long HAND_GESTURE_COOLDOWN_MS = 1800L;
    /** Test/a11y speech: 50% speed + volume (SpeakOption level). */
    private static final int SPEECH_SPEED = 50;
    private static final int SPEECH_VOLUME = 50;
    private static final long SPEAK_STATUS_COOLDOWN_MS = 2500L;
    private static final String PREFS = "human_follow";
    private static final String PREF_SPEECH_FEEDBACK = "speech_feedback";
    private static final String PREF_GOOGLE_TTS = "google_tts_speaker";
    private static final String PREF_GOOGLE_ASR = "google_speech_recognizer";
    private static final int REQ_RECORD_AUDIO = 42;

    private TextView statusText;
    private TextView debugText;
    private Button btnDetect;
    private Button btnStart;
    private Button btnStop;
    private Button btnClose;
    private Button btnCloseBottom;
    private CheckBox chkSpeechFeedback;
    private CheckBox chkGoogleTts;
    private CheckBox chkGoogleAsr;
    private Button btnPinHome;
    private SurfaceView surfaceView;
    private FaceOverlayView faceOverlay;
    private RadarView radarView;
    private SharedPreferences prefs;
    private boolean speechFeedbackEnabled = true;
    private boolean googleTtsEnabled = true;
    private boolean googleAsrEnabled = true;
    private String lastSpokenPhrase = "";
    private long lastSpokenAt;

    private HDCameraManager hdCameraManager;
    private WheelMotionManager wheelMotionManager;
    private HardWareManager hardWareManager;
    private HeadMotionManager headMotionManager;
    private SpeechManager speechManager;
    private SystemManager systemManager;
    private FaceTrackManager faceTrackManager;
    private AndroidTtsSpeaker androidTts;
    private AndroidSpeechRecognizer googleSpeech;
    private boolean googleAsrLoopActive;
    private final Runnable googleAsrRestartRunnable = () -> requestMicThenStartGoogleAsr();

    private final VisionMediaDecoder mediaDecoder = new VisionMediaDecoder();
    private final List<Integer> streamHandles = new ArrayList<>();
    private int streamWidth;
    private int streamHeight;

    private final Handler mainHandler = new Handler(Looper.getMainLooper());
    private LocalPersonScanner personScanner;
    private volatile boolean following;
    /** Face/person boxes only — no wheel chase (MQTT mode=detect / Detect Human). */
    private volatile boolean detecting;
    private boolean lastDetectHadHuman;
    private long lastDetectAnnounceAt;
    /** MainService hide-obstacle flag (often true when a person is close). */
    private volatile boolean obstacleFlag;
    private volatile int nearestFrontIrCm = -1;
    private volatile boolean pirFront;
    private volatile boolean pirBack;
    private long lastCmdAt;
    private long lastTargetAt;
    private long lastHeadPoseAt;
    private long lastHandGestureAt;
    private long lastVoiceCommandAt;
    private static final long VOICE_COMMAND_COOLDOWN_MS = 1500L;
    private boolean cameraListenersReady;
    private boolean speechListenersReady;
    /** Last continuous wheel action so we can refresh without stop/start chatter. */
    private byte lastWheelAction = NoAngleWheelMotion.ACTION_STOP;

    /** Sticky follow lock (person preferred). */
    private TrackTarget lockedTarget;

    private final Runnable lostTargetWatchdog = new Runnable() {
        @Override
        public void run() {
            if (!following) {
                return;
            }
            long age = System.currentTimeMillis() - lastTargetAt;
            if (age > TARGET_LOST_MS) {
                boolean firstLost = lockedTarget != null;
                lockedTarget = null;
                // Speak Searching once when lock drops — not every watchdog tick.
                if (firstLost) {
                    setStatus(getString(R.string.status_searching),
                            getString(R.string.speak_searching));
                } else {
                    setStatus(getString(R.string.status_searching));
                }
                faceOverlay.clearFaces();
                stopWheels();
                // Search turn OK unless something is crash-close
                if (!isCrashClose() && allowCmd()) {
                    RelativeAngleWheelMotion turn = new RelativeAngleWheelMotion(
                            RelativeAngleWheelMotion.TURN_RIGHT, 3, 15);
                    wheelMotionManager.doRelativeAngleMotion(turn);
                    markCmd();
                }
            } else if (age > LOCK_HOLD_MS * 0.5 && lockedTarget != null && !isCrashClose()
                    && isFaceDerived(lockedTarget)) {
                // Coast only on a face lock — never keep driving toward a motion blob.
                onPerson(lockedTarget);
            }
            if (System.currentTimeMillis() - lastHeadPoseAt >= HEAD_REPOSE_MS) {
                poseHeadForFollowCamera();
            }
            mainHandler.postDelayed(this, 450L);
        }
    };

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        register(HumanFollowActivity.class);
        super.onCreate(savedInstanceState);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        setContentView(R.layout.activity_human_follow);

        statusText = findViewById(R.id.statusText);
        debugText = findViewById(R.id.debugText);
        TextView versionText = findViewById(R.id.versionText);
        if (versionText != null) {
            versionText.setText(String.format(Locale.US, "v%s (%d)",
                    BuildConfig.VERSION_NAME, BuildConfig.VERSION_CODE));
        }
        btnDetect = findViewById(R.id.btnDetect);
        btnStart = findViewById(R.id.btnStart);
        btnStop = findViewById(R.id.btnStop);
        btnClose = findViewById(R.id.btnClose);
        btnCloseBottom = findViewById(R.id.btnCloseBottom);
        chkSpeechFeedback = findViewById(R.id.chkSpeechFeedback);
        chkGoogleTts = findViewById(R.id.chkGoogleTts);
        chkGoogleAsr = findViewById(R.id.chkGoogleAsr);
        btnPinHome = findViewById(R.id.btnPinHome);
        surfaceView = findViewById(R.id.sv_media);
        faceOverlay = findViewById(R.id.faceOverlay);
        radarView = findViewById(R.id.radarView);

        prefs = getSharedPreferences(PREFS, MODE_PRIVATE);
        HomeScreenHelper.ensurePinnedOnce(this);
        btnPinHome.setOnClickListener(v -> {
            HomeScreenHelper.pinBoth(this);
            setStatus("Home icons requested — check Nova desktop");
        });
        speechFeedbackEnabled = prefs.getBoolean(PREF_SPEECH_FEEDBACK, true);
        googleTtsEnabled = prefs.getBoolean(PREF_GOOGLE_TTS, true);
        googleAsrEnabled = prefs.getBoolean(PREF_GOOGLE_ASR, true);
        bindSpeechFeedbackCheckbox();
        chkGoogleTts.setChecked(googleTtsEnabled);
        chkGoogleTts.setOnCheckedChangeListener((buttonView, isChecked) -> {
            googleTtsEnabled = isChecked;
            prefs.edit().putBoolean(PREF_GOOGLE_TTS, isChecked).apply();
            Log.i(TAG, "Google TTS speaker=" + isChecked);
            if (isChecked) {
                speakFeedback(getString(R.string.google_tts_on), true);
            }
        });
        chkGoogleAsr.setChecked(googleAsrEnabled);
        chkGoogleAsr.setOnCheckedChangeListener((buttonView, isChecked) -> {
            googleAsrEnabled = isChecked;
            prefs.edit().putBoolean(PREF_GOOGLE_ASR, isChecked).apply();
            Log.i(TAG, "Google ASR=" + isChecked);
            if (isChecked) {
                startGoogleAsrLoop();
            } else {
                stopGoogleAsrLoop();
                tryOpenSdkWakeUp();
            }
        });

        androidTts = new AndroidTtsSpeaker(this);
        initGoogleSpeechRecognizer();

        btnDetect.setOnClickListener(v -> setDetecting(true));
        btnStart.setOnClickListener(v -> setFollowing(true));
        btnStop.setOnClickListener(v -> {
            setFollowing(false);
            setDetecting(false, false);
        });
        View.OnClickListener closeClick = v -> exitHumanFollow();
        btnClose.setOnClickListener(closeClick);
        btnCloseBottom.setOnClickListener(closeClick);

        surfaceView.getHolder().addCallback(this);

        personScanner = new LocalPersonScanner(mainHandler, this::onTargetsDetected);

        bindManagers();
        handleFollowIntent(getIntent());
    }

    @Override
    protected void onNewIntent(Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        handleFollowIntent(intent);
    }

    @Override
    protected void onResume() {
        super.onResume();
        // Re-apply after TopBaseActivity / MainService may have rewritten the intent timing.
        handleFollowIntent(getIntent());
    }

    @Override
    protected void onMainServiceConnected() {
        bindManagers();
        initListeners();
        handleFollowIntent(getIntent());
        Log.i(TAG, "MainService connected");
    }

    @Override
    public void onBackPressed() {
        exitHumanFollow();
    }

    /** Stop chase/detect, release camera, leave the activity (Nano has no nav bar). */
    private void exitHumanFollow() {
        Log.i(TAG, "exit HumanFollow");
        try {
            setFollowing(false, false);
            setDetecting(false, false);
        } catch (Exception ignored) {
        }
        try {
            closeStreams();
        } catch (Exception ignored) {
        }
        try {
            mediaDecoder.stopDecoding();
        } catch (Exception ignored) {
        }
        finish();
    }

    @Override
    protected void onDestroy() {
        setFollowing(false);
        setDetecting(false, false);
        stopGoogleAsrLoop();
        if (googleSpeech != null) {
            googleSpeech.destroy();
            googleSpeech = null;
        }
        if (androidTts != null) {
            androidTts.shutdown();
            androidTts = null;
        }
        if (personScanner != null) {
            personScanner.destroy();
        }
        closeStreams();
        mediaDecoder.stopDecoding();
        mainHandler.removeCallbacksAndMessages(null);
        super.onDestroy();
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions,
            @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode != REQ_RECORD_AUDIO) {
            return;
        }
        if (grantResults.length > 0 && grantResults[0] == PackageManager.PERMISSION_GRANTED) {
            startGoogleAsrLoop();
        } else {
            Log.w(TAG, "RECORD_AUDIO denied — falling back to OpenSDK ASR");
            googleAsrEnabled = false;
            if (prefs != null) {
                prefs.edit().putBoolean(PREF_GOOGLE_ASR, false).apply();
            }
            if (chkGoogleAsr != null) {
                chkGoogleAsr.setOnCheckedChangeListener(null);
                chkGoogleAsr.setChecked(false);
                chkGoogleAsr.setOnCheckedChangeListener((buttonView, isChecked) -> {
                    googleAsrEnabled = isChecked;
                    prefs.edit().putBoolean(PREF_GOOGLE_ASR, isChecked).apply();
                    if (isChecked) {
                        startGoogleAsrLoop();
                    } else {
                        stopGoogleAsrLoop();
                        tryOpenSdkWakeUp();
                    }
                });
            }
            tryOpenSdkWakeUp();
        }
    }

    private void bindManagers() {
        hdCameraManager = (HDCameraManager) getUnitManager(FuncConstant.HDCAMERA_MANAGER);
        wheelMotionManager = (WheelMotionManager) getUnitManager(FuncConstant.WHEELMOTION_MANAGER);
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        headMotionManager = (HeadMotionManager) getUnitManager(FuncConstant.HEADMOTION_MANAGER);
        speechManager = (SpeechManager) getUnitManager(FuncConstant.SPEECH_MANAGER);
        systemManager = (SystemManager) getUnitManager(FuncConstant.SYSTEM_MANAGER);
        faceTrackManager = (FaceTrackManager) getUnitManager(FuncConstant.FACETRACK_MANAGER);
    }

    private void initListeners() {
        initCameraListeners();
        initSpeechListeners();
    }

    private void initCameraListeners() {
        if (cameraListenersReady || hdCameraManager == null) {
            return;
        }
        cameraListenersReady = true;

        hdCameraManager.setMediaListener(new FaceRecognizeListener() {
            @Override
            public void recognizeResult(@NonNull List<FaceRecognizeBean> list) {
                if (personScanner != null && list != null && !list.isEmpty()) {
                    personScanner.markSdkFacesActive();
                }
                onTargetsDetected(fromSdkFaces(list));
            }
        });

        hdCameraManager.setMediaListener(new MediaStreamListener() {
            @Override
            public void getVideoStream(int handle, byte[] bytes, int width, int height) {
                if (width != streamWidth || height != streamHeight) {
                    mediaDecoder.onCreateCodec(width, height);
                    streamWidth = width;
                    streamHeight = height;
                    mainHandler.post(() -> faceOverlay.setFrameSize(width, height));
                }
                mediaDecoder.drawVideoSample(ByteBuffer.wrap(bytes));
                if (personScanner != null) {
                    personScanner.onVideoFrame(bytes, width, height);
                }
            }

            @Override
            public void getAudioStream(int i, @NonNull byte[] bytes) {
                // unused
            }
        });

        if (hardWareManager != null) {
            hardWareManager.setOnHareWareListener(new ObstacleListener() {
                @Override
                public void onObstacleStatus(boolean hasObstacle) {
                    // Often true when the follow target is nearby — do not freeze follow.
                    obstacleFlag = hasObstacle;
                    runOnUiThread(() -> {
                        if (radarView != null) {
                            // Show as soft warn when we have a person lock; hard when searching
                            boolean hard = hasObstacle && lockedTarget == null;
                            radarView.setObstacleActive(hard);
                        }
                        if (hasObstacle && following && lockedTarget == null && isCrashClose()) {
                            stopWheels();
                            setStatus(getString(R.string.status_obstacle),
                                    getString(R.string.speak_obstacle));
                        }
                    });
                }
            });

            hardWareManager.setOnHareWareListener(new InfrareListener() {
                @Override
                public void infrareDistance(int part, int distance) {
                    runOnUiThread(() -> {
                        if (radarView != null) {
                            radarView.setIrDistance(part, distance);
                            nearestFrontIrCm = radarView.nearestFrontCm();
                            // Crash-close only: walls / furniture. Person at follow range is OK.
                            if (following && isCrashClose()) {
                                stopWheels();
                                setStatus(getString(R.string.status_obstacle),
                                        getString(R.string.speak_obstacle));
                            }
                        }
                    });
                }
            });

            hardWareManager.setOnHareWareListener(new PIRListener() {
                @Override
                public void onPIRCheckResult(boolean isChecked, int part) {
                    if (part == 1) {
                        pirFront = isChecked;
                    } else {
                        pirBack = isChecked;
                    }
                    runOnUiThread(() -> {
                        if (radarView != null) {
                            radarView.setPir(pirFront, pirBack);
                        }
                    });
                }
            });
        }
    }

    private void initSpeechListeners() {
        if (speechListenersReady || speechManager == null) {
            return;
        }
        speechListenersReady = true;

        speechManager.setOnSpeechListener(new WakenListener() {
            @Override
            public void onWakeUp() {
                runOnUiThread(() -> {
                    // Don't clobber an active follow status with "Listening…".
                    if (!following) {
                        setStatus(getString(R.string.status_listening));
                    }
                });
                Log.i(TAG, "voice wake");
            }

            @Override
            public void onSleep() {
                // no-op
            }

            @Override
            public void onWakeUpStatus(boolean b) {
                // no-op
            }
        });

        speechManager.setOnSpeechListener(new RecognizeListener() {
            @Override
            public boolean onRecognizeResult(@NonNull Grammar grammar) {
                if (grammar == null) {
                    return false;
                }
                String topic = grammar.getTopic();
                String action = grammar.getAction();
                String text = grammar.getText();
                Log.i(TAG, "grammar topic=" + topic + " action=" + action + " text=" + text);

                // Stock follow topic from MainService grammar
                if (topic != null && topic.toLowerCase().contains("follow")) {
                    boolean start = action == null
                            || !action.toLowerCase().contains("close")
                                    && !action.toLowerCase().contains("stop")
                                    && !action.equalsIgnoreCase("0")
                                    && !action.equalsIgnoreCase("false");
                    handleVoiceCommand(start
                            ? VoiceCommandParser.Command.START_FOLLOW
                            : VoiceCommandParser.Command.STOP_FOLLOW);
                    return true;
                }
                if (GrammarConstant.TC_FOLLOW.equalsIgnoreCase(text)
                        || "follow".equalsIgnoreCase(text)) {
                    handleVoiceCommand(VoiceCommandParser.Command.START_FOLLOW);
                    return true;
                }
                return handleVoiceUtterance(text);
            }

            @Override
            public void onRecognizeText(@NonNull RecognizeTextBean bean) {
                if (bean == null) {
                    return;
                }
                handleVoiceUtterance(bean.getText());
            }

            @Override
            public void onRecognizeVolume(int volume) {
                // unused
            }

            @Override
            public void onStartRecognize() {
                runOnUiThread(() -> setStatus(getString(R.string.status_listening)));
            }

            @Override
            public void onStopRecognize() {
                // unused
            }

            @Override
            public void onError(int i, int i1) {
                Log.w(TAG, "recognize error " + i + "," + i1);
            }
        });

        if (useGoogleSpeechRecognizer()) {
            Log.i(TAG, "voice listen via Google SpeechRecognizer");
            startGoogleAsrLoop();
        } else {
            tryOpenSdkWakeUp();
        }
    }

    private void initGoogleSpeechRecognizer() {
        googleSpeech = new AndroidSpeechRecognizer(this);
        googleSpeech.setListener(new AndroidSpeechRecognizer.Listener() {
            @Override
            public void onFinalResult(String text) {
                handleVoiceUtterance(text);
                // Wait for TTS / mic settle before next listen.
                restartGoogleListeningSoon(900L);
            }

            @Override
            public void onListeningStarted() {
                runOnUiThread(() -> {
                    if (!following) {
                        setStatus(getString(R.string.status_listening));
                    }
                });
            }

            @Override
            public void onListeningEnded() {
                // Results/errors usually follow; avoid double-restart stampede.
            }

            @Override
            public void onError(int errorCode, String message) {
                Log.w(TAG, "Google STT error " + errorCode + " " + message);
                if (!googleAsrLoopActive || !useGoogleSpeechRecognizer()) {
                    return;
                }
                if (errorCode == SpeechRecognizer.ERROR_RECOGNIZER_BUSY) {
                    restartGoogleListeningSoon(2000L);
                } else if (errorCode == SpeechRecognizer.ERROR_SPEECH_TIMEOUT
                        || errorCode == SpeechRecognizer.ERROR_NO_MATCH
                        || errorCode == SpeechRecognizer.ERROR_CLIENT) {
                    restartGoogleListeningSoon(800L);
                } else if (errorCode == SpeechRecognizer.ERROR_INSUFFICIENT_PERMISSIONS) {
                    requestMicThenStartGoogleAsr();
                }
            }
        });
    }

    private boolean useGoogleSpeechRecognizer() {
        return googleAsrEnabled
                && googleSpeech != null
                && AndroidSpeechRecognizer.isRecognitionAvailable(this);
    }

    private void startGoogleAsrLoop() {
        googleAsrLoopActive = true;
        mainHandler.removeCallbacks(googleAsrRestartRunnable);
        requestMicThenStartGoogleAsr();
    }

    private void stopGoogleAsrLoop() {
        googleAsrLoopActive = false;
        mainHandler.removeCallbacks(googleAsrRestartRunnable);
        if (googleSpeech != null) {
            googleSpeech.stopListening();
        }
    }

    private void requestMicThenStartGoogleAsr() {
        if (!googleAsrLoopActive || !useGoogleSpeechRecognizer()) {
            return;
        }
        if (googleSpeech != null && googleSpeech.isListening()) {
            return;
        }
        if (Build.VERSION.SDK_INT >= 23
                && ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO)
                != PackageManager.PERMISSION_GRANTED) {
            ActivityCompat.requestPermissions(this,
                    new String[]{Manifest.permission.RECORD_AUDIO}, REQ_RECORD_AUDIO);
            return;
        }
        if (googleSpeech != null) {
            googleSpeech.startListening();
        }
    }

    private void restartGoogleListeningSoon(long delayMs) {
        if (!googleAsrLoopActive || !useGoogleSpeechRecognizer()) {
            return;
        }
        mainHandler.removeCallbacks(googleAsrRestartRunnable);
        mainHandler.postDelayed(googleAsrRestartRunnable, delayMs);
    }

    private void tryOpenSdkWakeUp() {
        if (speechManager == null) {
            return;
        }
        try {
            Log.i(TAG, "voice listen via MainService doWakeUp");
            speechManager.doWakeUp();
        } catch (Exception e) {
            Log.w(TAG, "doWakeUp", e);
        }
    }

    private void bindSpeechFeedbackCheckbox() {
        if (chkSpeechFeedback == null) {
            return;
        }
        chkSpeechFeedback.setChecked(speechFeedbackEnabled);
        chkSpeechFeedback.setText(speechFeedbackEnabled ? R.string.speech_on : R.string.speech_off);
        chkSpeechFeedback.setOnCheckedChangeListener((buttonView, isChecked) -> {
            speechFeedbackEnabled = isChecked;
            prefs.edit().putBoolean(PREF_SPEECH_FEEDBACK, isChecked).apply();
            chkSpeechFeedback.setText(isChecked ? R.string.speech_on : R.string.speech_off);
            if (isChecked) {
                speakFeedback(getString(R.string.speech_on), true);
            } else {
                LocalSpeak.stop(androidTts, speechManager);
            }
        });
    }

    /** @return true if a follow command was handled (intercept MainService). */
    private boolean handleVoiceUtterance(String text) {
        VoiceCommandParser.Command cmd = VoiceCommandParser.parse(text);
        if (cmd == VoiceCommandParser.Command.UNKNOWN) {
            return false;
        }
        handleVoiceCommand(cmd);
        return true;
    }

    private void handleVoiceCommand(VoiceCommandParser.Command cmd) {
        long now = System.currentTimeMillis();
        if (now - lastVoiceCommandAt < VOICE_COMMAND_COOLDOWN_MS) {
            return;
        }
        lastVoiceCommandAt = now;
        Log.i(TAG, "voice command " + cmd);

        switch (cmd) {
            case START_FOLLOW:
                setFollowing(true, false);
                runOnUiThread(() -> setStatus(getString(R.string.status_hand_start),
                        getString(R.string.speak_hand_start)));
                break;
            case STOP_FOLLOW:
                setFollowing(false, false);
                runOnUiThread(() -> setStatus(getString(R.string.status_hand_stop),
                        getString(R.string.speak_hand_stop)));
                break;
            case SPEECH_ON:
                applySpeechFeedbackFromRemote(true);
                speakFeedback(getString(R.string.speech_on), true);
                runOnUiThread(() -> setStatus(getString(R.string.speech_on)));
                break;
            case SPEECH_OFF:
                // Say the phrase once, then mute.
                speakFeedback(getString(R.string.speech_off), true);
                mainHandler.postDelayed(() -> applySpeechFeedbackFromRemote(false), 900L);
                runOnUiThread(() -> setStatus(getString(R.string.speech_off)));
                break;
            case STATUS:
                speakCurrentRangeStatus();
                break;
            default:
                speakFeedback(getString(R.string.speak_voice_unknown), true);
                break;
        }
    }

    private void speakCurrentRangeStatus() {
        if (lockedTarget == null) {
            speakFeedback(following
                    ? getString(R.string.speak_searching)
                    : getString(R.string.speak_idle), true);
            return;
        }
        double range = lockedTarget.rangeHeight;
        if (isCrashClose() || (nearestFrontIrCm > 0 && nearestFrontIrCm <= IR_HOLD_CM)
                || range > RANGE_TOO_CLOSE) {
            speakFeedback(getString(R.string.speak_too_close), true);
        } else if (range < RANGE_TOO_FAR) {
            speakFeedback(getString(R.string.speak_too_far), true);
        } else {
            speakFeedback(getString(R.string.speak_good_distance), true);
        }
    }

    /** True only when IR says something is almost touching — not a standing person. */
    private boolean isCrashClose() {
        int cm = nearestFrontIrCm;
        return cm > 0 && cm <= IR_CRASH_CM;
    }

    /** Inhibit forward only (turns still allowed) while following a locked person. */
    private boolean shouldHoldForward() {
        if (isCrashClose()) {
            return true;
        }
        // With a lock the person *is* the front IR — only hold when very close.
        int cm = nearestFrontIrCm;
        if (lockedTarget != null) {
            return cm > 0 && cm <= IR_HOLD_CM;
        }
        if (cm > 0 && cm <= IR_HOLD_CM) {
            return true;
        }
        // Obstacle flag without a lock → be cautious about driving forward
        return obstacleFlag;
    }

    private void onTargetsDetected(List<TrackTarget> targets) {
        if (targets == null) {
            targets = new ArrayList<>();
        }

        // Accessibility: raised hand / wave above face → toggle follow.
        if (hasRaisedHand(targets)) {
            handleRaisedHandGesture();
        }

        // Follow drive: face lock only. Detect test: face preferred, motion OK for sticky UI.
        TrackTarget best = pickFaceLock(targets, lockedTarget);
        TrackTarget motionHint = pickMotionHint(targets);
        if (best == null && detecting && !following && motionHint != null) {
            best = motionHint;
        }
        long now = System.currentTimeMillis();
        long holdMs = detecting && !following ? DETECT_LOCK_HOLD_MS : LOCK_HOLD_MS;

        if (best != null) {
            lockedTarget = best;
            lastTargetAt = now;
        } else if (lockedTarget != null && now - lastTargetAt <= holdMs
                && (detecting || isFaceDerived(lockedTarget))) {
            best = lockedTarget;
        } else {
            lockedTarget = null;
            best = null;
        }

        final List<TrackTarget> copy = new ArrayList<>(targets);
        final TrackTarget lock = lockedTarget;
        final TrackTarget drive = best;
        final TrackTarget hint = motionHint;
        final boolean lockedIsFace = isFaceDerived(lock);
        runOnUiThread(() -> {
            faceOverlay.setTargets(copy, lock != null ? lock : hint);
            if (lock != null) {
                if (following) {
                    applyRangeStatus(lock);
                } else if (detecting) {
                    applyDetectStatus(true);
                }
                debugText.setText(String.format(Locale.US,
                        "%s lock cx=%.2f rh=%.2f dets=%d %s",
                        lockedIsFace ? "FACE" : "HUMAN",
                        lock.centerX(), lock.rangeHeight, copy.size(),
                        following ? "FOLLOW" : (detecting ? "DETECT" : "preview")));
            } else if (hint != null) {
                debugText.setText(String.format(Locale.US,
                        "motion hint cx=%.2f (need face) %s",
                        hint.centerX(),
                        following ? "SEARCH" : (detecting ? "DETECT" : "preview")));
            } else if (following) {
                debugText.setText(copy.isEmpty()
                        ? "scanning — face the camera"
                        : ("dets=" + copy.size() + " (need face)"));
            } else if (detecting) {
                applyDetectStatus(false);
                debugText.setText(copy.isEmpty()
                        ? "detect — face the camera"
                        : ("dets=" + copy.size() + " (need face)"));
            } else {
                debugText.setText(copy.isEmpty() ? "no person/face" : ("dets=" + copy.size()));
            }
        });

        if (!following) {
            return;
        }
        // Drive only on face-derived lock — never chase a motion blob.
        if (drive != null && isFaceDerived(drive)) {
            onPerson(drive);
        } else if (hint != null && !isCrashClose()) {
            // No face yet: turn toward motion only — never drive straight at furniture.
            steerSearchToward(hint.centerX());
        }
    }

    private static boolean isFaceDerived(TrackTarget t) {
        return t != null && !"MOTION".equals(t.label) && !"HAND_UP".equals(t.label);
    }

    private static boolean hasRaisedHand(List<TrackTarget> targets) {
        if (targets == null) {
            return false;
        }
        for (TrackTarget t : targets) {
            if ("HAND_UP".equals(t.label)) {
                return true;
            }
        }
        return false;
    }

    /**
     * Accessibility gesture: raise an open hand above your face (wave or hold).
     * While following → stop. While idle with a face → start follow.
     */
    private void handleRaisedHandGesture() {
        long now = System.currentTimeMillis();
        if (now - lastHandGestureAt < HAND_GESTURE_COOLDOWN_MS) {
            return;
        }
        lastHandGestureAt = now;
        if (following) {
            setFollowing(false, false);
            runOnUiThread(() -> setStatus(getString(R.string.status_hand_stop),
                    getString(R.string.speak_hand_stop)));
        } else {
            setFollowing(true, false);
            runOnUiThread(() -> setStatus(getString(R.string.status_hand_start),
                    getString(R.string.speak_hand_start)));
        }
    }

    /**
     * Spoken status for testing at ~2 m. Speed/volume 50%. Honours Speech checkbox.
     * Prefers local Google TTS when enabled; falls back to OpenSDK cloud speak.
     *
     * @param force ignore duplicate cooldown (e.g. toggle confirmation)
     */
    private void speakFeedback(String phrase, boolean force) {
        if (!speechFeedbackEnabled || phrase == null || phrase.isEmpty()) {
            return;
        }
        if (!googleTtsEnabled && speechManager == null) {
            return;
        }
        long now = System.currentTimeMillis();
        if (!force && phrase.equals(lastSpokenPhrase)
                && now - lastSpokenAt < SPEAK_STATUS_COOLDOWN_MS) {
            return;
        }
        lastSpokenPhrase = phrase;
        lastSpokenAt = now;
        try {
            SpeakOption opt = new SpeakOption();
            opt.setLanguageType(SpeakOption.LAG_ENGLISH_US);
            opt.setSpeed(SPEECH_SPEED);
            opt.setIntonation(SPEECH_SPEED);
            applySpeakVolume(opt, SPEECH_VOLUME);
            LocalSpeak.speak(googleTtsEnabled, androidTts, speechManager,
                    phrase, SPEECH_SPEED, SPEECH_VOLUME, opt);
        } catch (Exception e) {
            Log.w(TAG, "speakFeedback", e);
        }
    }

    private void speakFeedback(String phrase) {
        speakFeedback(phrase, false);
    }

    /** OpenSDK stores volume in private {@code SpeakOption.level} (same as ROS bridge). */
    private static void applySpeakVolume(SpeakOption opt, int volume0to100) {
        try {
            Field level = SpeakOption.class.getDeclaredField("level");
            level.setAccessible(true);
            level.setInt(opt, Math.max(0, Math.min(100, volume0to100)));
        } catch (Exception e) {
            Log.w(TAG, "SpeakOption.level not settable", e);
        }
    }

    /** Center pan + mid/high tilt so the camera looks at standing people. */
    private void poseHeadForFollowCamera() {
        if (headMotionManager == null) {
            return;
        }
        try {
            // Nano: action byte is motor axis (1=vertical, 2=horizontal).
            headMotionManager.doAbsoluteAngleMotion(
                    new AbsoluteAngleHeadMotion(
                            AbsoluteAngleHeadMotion.ACTION_HORIZONTAL, HEAD_PAN_CENTER));
            headMotionManager.doAbsoluteAngleMotion(
                    new AbsoluteAngleHeadMotion(
                            AbsoluteAngleHeadMotion.ACTION_VERTICAL, HEAD_TILT_FOLLOW));
            try {
                headMotionManager.dohorizontalCenterLockMotion();
            } catch (Exception ignored) {
                // Optional API — absolute pan already centers.
            }
            lastHeadPoseAt = System.currentTimeMillis();
            Log.i(TAG, "head posed pan=" + HEAD_PAN_CENTER + " tilt=" + HEAD_TILT_FOLLOW);
        } catch (Exception e) {
            Log.w(TAG, "poseHeadForFollowCamera", e);
        }
    }

    private List<TrackTarget> fromSdkFaces(List<FaceRecognizeBean> list) {
        List<TrackTarget> out = new ArrayList<>();
        if (list == null) {
            return out;
        }
        double fw = streamWidth > 0 ? streamWidth : 1280.0;
        double fh = streamHeight > 0 ? streamHeight : 720.0;
        for (FaceRecognizeBean bean : list) {
            double left = bean.getLeft();
            double right = bean.getRight();
            double top = bean.getTop();
            double bottom = bean.getBottom();
            if (bean.getW() > 0 && bean.getH() > 0 && right <= left) {
                right = left + bean.getW();
                bottom = top + bean.getH();
            }
            if (right > 2.0 || bottom > 2.0 || left > 2.0 || top > 2.0) {
                left /= fw;
                right /= fw;
                top /= fh;
                bottom /= fh;
            }
            TrackTarget face = new TrackTarget(
                    TrackTarget.Kind.FACE, left, top, right, bottom, "FACE");
            out.add(face);
            out.add(TrackTarget.bodyFromFace(face));
        }
        return out;
    }

    /**
     * Face-only lock for driving. Ignores MOTION blobs (walls/furniture under ego-motion).
     */
    private TrackTarget pickFaceLock(List<TrackTarget> targets, TrackTarget previous) {
        if (targets == null || targets.isEmpty()) {
            return null;
        }
        TrackTarget bestBody = null;
        double bestBodyScore = -1;
        TrackTarget bestFace = null;
        double bestFaceArea = -1;

        for (TrackTarget t : targets) {
            if ("MOTION".equals(t.label) || "HAND_UP".equals(t.label)) {
                continue;
            }
            if (t.kind == TrackTarget.Kind.PERSON) {
                double score = t.area();
                if (previous != null && isFaceDerived(previous)) {
                    // Light stickiness — still allow quick lateral re-aim when you move.
                    double dx = Math.abs(t.centerX() - previous.centerX());
                    score *= (1.0 / (0.35 + dx));
                }
                if (score > bestBodyScore) {
                    bestBodyScore = score;
                    bestBody = t;
                }
            } else if (t.kind == TrackTarget.Kind.FACE) {
                if (t.area() > bestFaceArea) {
                    bestFaceArea = t.area();
                    bestFace = t;
                }
            }
        }
        if (bestBody != null) {
            return bestBody;
        }
        if (bestFace != null) {
            return TrackTarget.bodyFromFace(bestFace);
        }
        return null;
    }

    /** Largest motion blob — search yaw hint only, never a drive lock. */
    private TrackTarget pickMotionHint(List<TrackTarget> targets) {
        TrackTarget best = null;
        double bestArea = -1;
        if (targets == null) {
            return null;
        }
        for (TrackTarget t : targets) {
            if ("MOTION".equals(t.label) && t.area() > bestArea) {
                bestArea = t.area();
                best = t;
            }
        }
        return best;
    }

    private void steerSearchToward(double cx) {
        if (wheelMotionManager == null || !allowCmd()) {
            return;
        }
        if (cx < 0.42) {
            driveKeep(NoAngleWheelMotion.ACTION_LEFT, 4);
        } else if (cx > 0.58) {
            driveKeep(NoAngleWheelMotion.ACTION_RIGHT, 4);
        } else {
            // Ambiguous blob in center — don't charge forward without a face.
            stopWheels();
            setStatus(getString(R.string.status_searching),
                    getString(R.string.speak_searching));
        }
    }

    private void handleFollowIntent(Intent intent) {
        if (intent == null) {
            return;
        }
        Log.i(TAG, "handleFollowIntent action=" + intent.getAction()
                + " followExtra=" + intent.hasExtra(EXTRA_FOLLOW)
                + " detectExtra=" + intent.hasExtra(EXTRA_DETECT)
                + " enabledExtra=" + intent.hasExtra(EXTRA_ENABLED)
                + " speechExtra=" + intent.hasExtra(EXTRA_SPEECH_FEEDBACK)
                + " followingNow=" + following
                + " detectingNow=" + detecting);
        if (intent.hasExtra(EXTRA_SPEECH_FEEDBACK)) {
            applySpeechFeedbackFromRemote(
                    intent.getBooleanExtra(EXTRA_SPEECH_FEEDBACK, true));
        }
        if (ACTION_SET_SPEECH_FEEDBACK.equals(intent.getAction())) {
            return;
        }
        if (ACTION_SET_DETECTING.equals(intent.getAction())
                || intent.hasExtra(EXTRA_DETECT)) {
            boolean on = intent.hasExtra(EXTRA_DETECT)
                    ? intent.getBooleanExtra(EXTRA_DETECT, true)
                    : intent.getBooleanExtra(EXTRA_ENABLED, true);
            setDetecting(on);
            return;
        }
        if (ACTION_SET_FOLLOWING.equals(intent.getAction())) {
            // Broadcast open/close from switchFollow — this is the Follow Me switch.
            setFollowing(intent.getBooleanExtra(EXTRA_ENABLED, true));
            return;
        }
        if (intent.hasExtra(EXTRA_FOLLOW)) {
            // MQTT / voice launch: Follow On → follow=true enables tracking immediately.
            setFollowing(intent.getBooleanExtra(EXTRA_FOLLOW, true));
            return;
        }
        if (intent.hasExtra(EXTRA_ENABLED)
                && (intent.getBooleanExtra(EXTRA_FOLLOW, false)
                || ACTION_SET_FOLLOWING.equals(intent.getAction()))) {
            setFollowing(intent.getBooleanExtra(EXTRA_ENABLED, true));
        }
        // Launched with no extra (manual icon): stay in preview until Start follow.
    }

    /** Apply mute from MQTT / My Sanbot without speaking confirmation. */
    void applySpeechFeedbackFromRemote(boolean enabled) {
        speechFeedbackEnabled = enabled;
        if (prefs != null) {
            prefs.edit().putBoolean(PREF_SPEECH_FEEDBACK, enabled).apply();
        }
        runOnUiThread(() -> {
            if (chkSpeechFeedback != null) {
                chkSpeechFeedback.setOnCheckedChangeListener(null);
                bindSpeechFeedbackCheckbox();
            }
        });
        if (!enabled) {
            LocalSpeak.stop(androidTts, speechManager);
        }
        Log.i(TAG, "speech_feedback remote=" + enabled);
    }

    void setFollowing(boolean enabled) {
        setFollowing(enabled, true);
    }

    void setFollowing(boolean enabled, boolean announce) {
        if (enabled) {
            // Chase and detect-test are mutually exclusive.
            detecting = false;
            lastDetectHadHuman = false;
        }
        following = enabled;
        if (!enabled) {
            lockedTarget = null;
        }
        mainHandler.removeCallbacks(lostTargetWatchdog);
        setFaceTrack(enabled || detecting);
        if (!enabled) {
            stopWheels();
            runOnUiThread(() -> {
                if (!detecting) {
                    if (announce) {
                        setStatus(getString(R.string.status_idle), getString(R.string.speak_idle));
                    } else {
                        setStatus(getString(R.string.status_idle));
                    }
                    debugText.setText("preview — Detect Human or Start follow");
                }
            });
            return;
        }
        lastTargetAt = System.currentTimeMillis();
        poseHeadForFollowCamera();
        mainHandler.post(lostTargetWatchdog);
        try {
            if (systemManager != null) {
                systemManager.showEmotion(EmotionsType.SMILE);
            }
        } catch (Exception ignored) {
        }
        runOnUiThread(() -> {
            if (announce) {
                setStatus(getString(R.string.status_following),
                        getString(R.string.speak_hand_start));
            } else {
                setStatus(getString(R.string.status_following));
            }
        });
        Log.i(TAG, "follow enabled (person+face)");
    }

    void setDetecting(boolean enabled) {
        setDetecting(enabled, true);
    }

    void setDetecting(boolean enabled, boolean announce) {
        if (enabled && following) {
            setFollowing(false, false);
        }
        detecting = enabled;
        setFaceTrack(enabled || following);
        if (!enabled) {
            lastDetectHadHuman = false;
            lockedTarget = null;
            runOnUiThread(() -> {
                if (!following) {
                    // Detect mode: UI only — no TTS (saves CPU / avoids announcement storms).
                    if (statusText != null) {
                        statusText.setText(R.string.status_idle);
                    }
                    if (debugText != null) {
                        debugText.setText("preview — Detect Human or Start follow");
                    }
                    if (faceOverlay != null) {
                        faceOverlay.clearFaces();
                    }
                }
            });
            Log.i(TAG, "detect disabled");
            return;
        }
        poseHeadForFollowCamera();
        lastDetectAnnounceAt = 0L;
        lastDetectHadHuman = false;
        runOnUiThread(() -> {
            // No "Detect on" TTS — status text is enough for the test mode.
            setStatus(getString(R.string.status_detecting));
            if (debugText != null) {
                debugText.setText("detect test — face the camera");
            }
        });
        Log.i(TAG, "detect enabled (no drive)");
    }

    /**
     * Detect-test voice: only announce a positive lock once.
     * Never TTS "No human" — FaceDetector flicker made that a speech storm.
     */
    private void applyDetectStatus(boolean humanSeen) {
        if (statusText == null) {
            return;
        }
        if (humanSeen == lastDetectHadHuman) {
            statusText.setText(humanSeen
                    ? R.string.status_human_seen
                    : R.string.status_no_human);
            return;
        }
        lastDetectHadHuman = humanSeen;
        lastDetectAnnounceAt = System.currentTimeMillis();
        if (humanSeen) {
            setStatus(getString(R.string.status_human_seen),
                    getString(R.string.speak_human_seen));
        } else {
            setStatus(getString(R.string.status_no_human));
        }
    }

    private void setFaceTrack(boolean on) {
        if (faceTrackManager == null) {
            return;
        }
        try {
            faceTrackManager.switchFaceTrack(on);
        } catch (Exception e) {
            Log.w(TAG, "switchFaceTrack", e);
        }
    }

    private void onPerson(TrackTarget target) {
        if (wheelMotionManager == null) {
            return;
        }
        // Only freeze completely when almost colliding — you ARE the front IR when following.
        if (isCrashClose()) {
            stopWheels();
            runOnUiThread(() -> setStatus(getString(R.string.status_obstacle),
                    getString(R.string.speak_obstacle)));
            return;
        }

        double cx = target.centerX();
        double range = target.rangeHeight;
        int ir = nearestFrontIrCm;
        boolean holdFwd = shouldHoldForward();

        runOnUiThread(() -> {
            applyRangeStatus(target);
            if (debugText != null) {
                debugText.setText(String.format(Locale.US,
                        "%s lock cx=%.2f rh=%.2f ir=%s holdFwd=%s",
                        target.kind, cx, range,
                        ir > 0 ? (ir + "cm") : "—",
                        holdFwd ? "yes" : "no"));
            }
        });

        if (!isFaceDerived(target)) {
            steerSearchToward(cx);
            return;
        }

        // IR ease-back only when almost touching; locked person usually lights IR earlier.
        if (ir > 0 && ir <= IR_HOLD_CM) {
            driveKeep(NoAngleWheelMotion.ACTION_BACK, 3);
            return;
        }

        boolean needApproach = range < RANGE_TOO_FAR;
        boolean needBackoff = range > RANGE_TOO_CLOSE;
        boolean farLeft = cx < CENTER_HARD_LO;
        boolean farRight = cx > CENTER_HARD_HI;
        boolean softLeft = cx < CENTER_LO;
        boolean softRight = cx > CENTER_HI;

        // Turn first when you move off-center — don't keep going straight.
        if (farLeft) {
            driveKeep(NoAngleWheelMotion.ACTION_LEFT, 5);
            return;
        }
        if (farRight) {
            driveKeep(NoAngleWheelMotion.ACTION_RIGHT, 5);
            return;
        }

        if (needBackoff) {
            if (softLeft) {
                driveKeep(NoAngleWheelMotion.ACTION_LEFT_BACK, 3);
            } else if (softRight) {
                driveKeep(NoAngleWheelMotion.ACTION_RIGHT_BACK, 3);
            } else {
                driveKeep(NoAngleWheelMotion.ACTION_BACK, 3);
            }
            return;
        }

        if (holdFwd) {
            if (softLeft) {
                driveKeep(NoAngleWheelMotion.ACTION_LEFT, 3);
            } else if (softRight) {
                driveKeep(NoAngleWheelMotion.ACTION_RIGHT, 3);
            } else {
                stopWheels();
            }
            return;
        }

        // Chase face: arc when slightly off-center so direction changes are tracked.
        if (softLeft) {
            driveKeep(NoAngleWheelMotion.ACTION_LEFT_FORWARD, needApproach ? 5 : 4);
        } else if (softRight) {
            driveKeep(NoAngleWheelMotion.ACTION_RIGHT_FORWARD, needApproach ? 5 : 4);
        } else if (needApproach || range <= RANGE_TOO_CLOSE) {
            driveKeep(NoAngleWheelMotion.ACTION_FORWARD, needApproach ? 5 : 4);
        } else {
            stopWheels();
        }
    }

    /**
     * Continuous STATUS_KEEP wheel command (same pattern as the ROS bridge hold-to-drive).
     * Timed NoAngle pulses + RelativeAngle yaw were causing short/yaw-only motion.
     */
    private void driveKeep(byte action, int speed) {
        if (wheelMotionManager == null) {
            return;
        }
        if (action == lastWheelAction && !allowCmd()) {
            return;
        }
        if (action != lastWheelAction && !allowCmd()
                && lastWheelAction != NoAngleWheelMotion.ACTION_STOP) {
            // Allow direction changes a bit sooner than refreshes, but not every frame.
            if (System.currentTimeMillis() - lastCmdAt < CMD_INTERVAL_MS / 2) {
                return;
            }
        }
        try {
            int spd = Math.max(1, Math.min(10, speed));
            NoAngleWheelMotion motion = new NoAngleWheelMotion(
                    action, spd, 100, 0, NoAngleWheelMotion.STATUS_KEEP);
            wheelMotionManager.doNoAngleMotion(motion);
            lastWheelAction = action;
            markCmd();
            if (personScanner != null) {
                // Ego-motion makes walls look like “moving people” — pause blob detect.
                boolean translating = action == NoAngleWheelMotion.ACTION_FORWARD
                        || action == NoAngleWheelMotion.ACTION_LEFT_FORWARD
                        || action == NoAngleWheelMotion.ACTION_RIGHT_FORWARD
                        || action == NoAngleWheelMotion.ACTION_BACK
                        || action == NoAngleWheelMotion.ACTION_LEFT_BACK
                        || action == NoAngleWheelMotion.ACTION_RIGHT_BACK;
                personScanner.setSuppressMotion(translating);
            }
        } catch (Exception e) {
            Log.w(TAG, "driveKeep action=" + action, e);
        }
    }

    private boolean allowCmd() {
        return System.currentTimeMillis() - lastCmdAt >= CMD_INTERVAL_MS;
    }

    private void markCmd() {
        lastCmdAt = System.currentTimeMillis();
    }

    private void stopWheels() {
        if (wheelMotionManager == null) {
            return;
        }
        try {
            wheelMotionManager.doNoAngleMotion(new NoAngleWheelMotion(
                    NoAngleWheelMotion.ACTION_STOP, 0, 0, 0, NoAngleWheelMotion.STATUS_KEEP));
            lastWheelAction = NoAngleWheelMotion.ACTION_STOP;
            markCmd();
            if (personScanner != null) {
                personScanner.setSuppressMotion(false);
            }
        } catch (Exception e) {
            Log.w(TAG, "stopWheels", e);
        }
    }

    private static final int COLOR_STATUS_OK = Color.parseColor("#5EEAD4");
    private static final int COLOR_STATUS_CLOSE = Color.parseColor("#FB923C");
    private static final int COLOR_STATUS_FAR = Color.parseColor("#38BDF8");
    private static final int COLOR_STATUS_WARN = Color.parseColor("#F87171");

    /** Banner for stand-off: Too close / Too far / Good distance (preview + follow). */
    private void applyRangeStatus(TrackTarget target) {
        if (statusText == null || target == null) {
            return;
        }
        int ir = nearestFrontIrCm;
        if (isCrashClose() || (ir > 0 && ir <= IR_HOLD_CM)) {
            setStatus(getString(R.string.status_too_close), getString(R.string.speak_too_close));
            return;
        }
        double range = target.rangeHeight;
        if (range > RANGE_TOO_CLOSE) {
            setStatus(getString(R.string.status_too_close), getString(R.string.speak_too_close));
        } else if (range < RANGE_TOO_FAR) {
            setStatus(getString(R.string.status_too_far), getString(R.string.speak_too_far));
        } else if (following) {
            setStatus(getString(R.string.status_following), getString(R.string.speak_good_distance));
        } else {
            setStatus(getString(R.string.status_range_ok), getString(R.string.speak_good_distance));
        }
    }

    private void setStatus(String s) {
        setStatus(s, null);
    }

    private void setStatus(String s, String speakPhrase) {
        if (statusText != null) {
            statusText.setText(s);
            if (getString(R.string.status_obstacle).equals(s)) {
                statusText.setTextColor(COLOR_STATUS_WARN);
            } else if (getString(R.string.status_too_close).equals(s)
                    || getString(R.string.status_hand_stop).equals(s)) {
                statusText.setTextColor(COLOR_STATUS_CLOSE);
            } else if (getString(R.string.status_too_far).equals(s)
                    || getString(R.string.status_searching).equals(s)) {
                statusText.setTextColor(COLOR_STATUS_FAR);
            } else {
                statusText.setTextColor(COLOR_STATUS_OK);
            }
        }
        if (speakPhrase != null) {
            speakFeedback(speakPhrase);
        }
    }

    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        mediaDecoder.setSurface(holder.getSurface());
        openStream();
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {
        // no-op
    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        closeStreams();
        mediaDecoder.stopDecoding();
        streamWidth = 0;
        streamHeight = 0;
    }

    private void openStream() {
        if (hdCameraManager == null) {
            bindManagers();
        }
        if (hdCameraManager == null) {
            Log.w(TAG, "openStream: HDCameraManager unavailable");
            runOnUiThread(() -> debugText.setText("waiting for MainService…"));
            mainHandler.postDelayed(this::openStream, 800L);
            return;
        }
        initListeners();
        closeStreams();
        try {
            StreamOption streamOption = new StreamOption();
            streamOption.setChannel(StreamOption.MAIN_STREAM);
            streamOption.setDecodType(StreamOption.HARDWARE_DECODE);
            streamOption.setJustIframe(false);
            OperationResult operationResult = hdCameraManager.openStream(streamOption);
            String resultStr = operationResult != null ? operationResult.getResult() : null;
            Log.i(TAG, "openStream result=" + resultStr);
            if (resultStr != null) {
                int result = Integer.parseInt(resultStr);
                if (result != -1) {
                    streamHandles.add(result);
                    runOnUiThread(() -> debugText.setText("camera + person detect ready"));
                    if (personScanner != null) {
                        personScanner.start();
                    }
                } else {
                    runOnUiThread(() -> debugText.setText("openStream failed (-1)"));
                }
            }
        } catch (Exception e) {
            Log.e(TAG, "openStream", e);
            runOnUiThread(() -> debugText.setText("openStream error: " + e.getMessage()));
        }
    }

    private void closeStreams() {
        if (personScanner != null) {
            personScanner.stop();
        }
        if (hdCameraManager == null) {
            streamHandles.clear();
            return;
        }
        for (Integer handle : streamHandles) {
            try {
                hdCameraManager.closeStream(handle);
            } catch (Exception e) {
                Log.w(TAG, "closeStream " + handle, e);
            }
        }
        streamHandles.clear();
    }
}
