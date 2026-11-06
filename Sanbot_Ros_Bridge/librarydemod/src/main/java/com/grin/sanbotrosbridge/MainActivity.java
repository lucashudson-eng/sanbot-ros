package com.grin.sanbotrosbridge;

import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;
import android.widget.*;
import org.json.JSONObject;

import java.lang.reflect.Method;
import java.net.NetworkInterface;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;

import com.sanbot.opensdk.base.BindBaseInterface;
import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.beans.Order;
import com.sanbot.opensdk.function.beans.EmotionsType;
import com.sanbot.opensdk.function.beans.SpeakOption;
import com.sanbot.opensdk.function.beans.handmotion.AbsoluteAngleHandMotion;
import com.sanbot.opensdk.function.beans.handmotion.CombinationHandMotion;
import com.sanbot.opensdk.function.beans.handmotion.NoAngleHandMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.NoAngleWheelMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.RelativeAngleWheelMotion;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.function.unit.HandMotionManager;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.ModularMotionManager;
import com.sanbot.opensdk.function.unit.SpeechManager;
import com.sanbot.opensdk.function.unit.SystemManager;
import com.sanbot.opensdk.function.unit.WheelMotionManager;
import com.sanbot.opensdk.function.beans.wheelmotion.DistanceWheelMotion;
import com.sanbot.opensdk.function.unit.HeadMotionManager;
import com.sanbot.opensdk.function.beans.headmotion.AbsoluteAngleHeadMotion;
import com.sanbot.opensdk.function.unit.interfaces.hardware.TouchSensorListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.PIRListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.InfrareListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.VoiceLocateListener;
import com.sanbot.opensdk.function.beans.LED;
import com.sanbot.opensdk.function.unit.interfaces.hardware.ObstacleListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.GyroscopeListener;

import java.net.InetAddress;
import android.speech.tts.TextToSpeech;
import java.util.Locale;
import com.sanbot.opensdk.function.beans.speech.Grammar;
import com.sanbot.opensdk.function.beans.speech.RecognizeTextBean;
import com.sanbot.opensdk.function.unit.interfaces.speech.RecognizeListener;
import androidx.annotation.NonNull;
import android.Manifest;
import android.content.pm.PackageManager;
import androidx.core.app.ActivityCompat;
import androidx.core.content.ContextCompat;

import com.pedro.common.ConnectChecker;
import com.pedro.library.generic.GenericStream;
import com.pedro.encoder.input.sources.audio.MicrophoneSource;
import com.pedro.encoder.input.sources.video.BufferVideoSource;
import android.hardware.Camera;
import android.graphics.ImageFormat;
import android.graphics.SurfaceTexture;
import android.view.TextureView;
import android.text.Editable;
import android.text.TextWatcher;

public class MainActivity extends TopBaseActivity implements MqttHandler.MqttStatusListener, TextToSpeech.OnInitListener, ConnectChecker, ControlWebSocketServer.Listener, OrbbecBodyStreamer.Listener, HdCameraForeheadStreamer.Listener, UvcUsbStreamer.Listener {

    private EditText editTextName;
    private EditText editTextIp;
    private EditText editTextPort;
    private Button buttonConnect;
    private Button buttonClear;
    private Button buttonClose;
    private Button buttonCloseBottom;
    private Button buttonGoHome;
    private Button buttonGoHomeBottom;
    private Button buttonHdSpike;
    private TextView textViewVersion;
    private Spinner spinnerTopics;
    private TextView textViewMessages;
    private ScrollView scrollView;
    private Switch switchRtmp;
    private Switch switchWsControl;
    private Switch switchAutoStartProxy;
    private Switch switchOpenProxyOnConnect;
    private Button buttonStartProxy;
    private TextureView textureViewPreview;
    private TextView textViewPreviewHint;
    private Spinner spinnerLanguage;
    private CheckBox checkBoxShowLogs;
    private CheckBox checkBoxAutoConnect;
    private CheckBox checkBoxAutoLaunch;
    private CheckBox checkBoxAutoReturnHome;
    private EditText editTextAutoReturnSeconds;
    private Switch switchTtsType;
    private ArrayAdapter<String> androidLanguageAdapter;
    private ArrayAdapter<String> sdkLanguageAdapter;

    private MqttHandler mqttHandler;
    private boolean connected = false;
    private String selectedTopic = "All";
    private String robotName = "";
    /** Preferred HLS path for My Sanbot (face|body|forehead|usb|depth). */
    private String cameraLivePath = "face";
    private ControlWebSocketServer controlWsServer;
    private OrbbecBodyStreamer orbbecBodyStreamer;
    private HdCameraForeheadStreamer hdCameraForeheadStreamer;
    private UvcUsbStreamer uvcUsbStreamer;

    private static final String PREFS_NAME = "mqtt_prefs";
    private static final String KEY_LAST_IP = "last_broker_ip";
    private static final String KEY_LAST_NAME = "last_robot_name";
    private static final String KEY_LAST_PORT = "last_broker_port";
    private static final String KEY_AUTO_CONNECT = "auto_connect";
    private static final String KEY_AUTO_LAUNCH = "auto_launch";
    private static final String KEY_WS_CONTROL = "ws_control";
    private static final String KEY_CAMERA_PATH = "camera_live_path";
    private static final String KEY_AUTO_RETURN_HOME = "auto_return_home";
    private static final String KEY_SHOW_UI_LOG = "show_ui_log";
    /** One-shot: older builds defaulted auto-return ON (breaks RTMP if Bridge leaves fg). */
    private static final String KEY_AUTO_RETURN_HOME_MIGRATED_OFF = "auto_return_home_migrated_off";
    private static final String KEY_AUTO_RETURN_SECONDS = "auto_return_seconds";
    private static final int DEFAULT_AUTO_RETURN_SECONDS = 60;
    private static final int MIN_AUTO_RETURN_SECONDS = 5;
    private static final int MAX_AUTO_RETURN_SECONDS = 3600;
    private static final int CAMERA_PERMISSION_REQUEST_CODE = 200;
    private static final int WS_CONTROL_PORT = 8080;

    /** Companion (OpenSDK Proxy) asked us to background after Return to Home. */
    public static final String EXTRA_MINIMIZE_TO_HOME = "minimize_to_home";

    private SystemManager systemManager;
    /**
     * Last operator Mute/Unmute from /system. Nano often denies writing
     * Settings.System volume_master_mute — keep /info honest. Speak never changes this.
     */
    private volatile Boolean lastAppliedMasterMute;
    private HardWareManager hardWareManager;
    private WheelMotionManager wheelMotionManager;
    private HeadMotionManager headMotionManager;
    private HandMotionManager handMotionManager;
    private ModularMotionManager modularMotionManager;
    private SpeechManager speechManager;
    private HDCameraManager hdCameraManager;


    private Handler publishHandler = new Handler();

    private final Handler motionHandler = new Handler(Looper.getMainLooper());
    /** Dedicated to RTMP publish warm-up — must not share motionHandler (drive used to clear it). */
    private final Handler rtmpHandler = new Handler(Looper.getMainLooper());
    private final Runnable autoReturnHomeRunnable = this::runAutoReturnHome;
    private Runnable motionStopRunnable;
    private int rtmpPublishAttempts = 0;

    private final Map<Integer, Long> lastIRUpdate = new HashMap<>();
    private static final long IR_UPDATE_INTERVAL_MS = 1000;  // 1 segundo por sensor
    private long lastImuPublishMs = 0;
    private static final long IMU_UPDATE_INTERVAL_MS = 200; // 5 Hz — unthrottled gyro ANRs the UI
    private static final int MAX_LOG_CHARS = 8000;

    private TextToSpeech tts;

    // === RTMP Streaming ===
    private GenericStream rtmpStream;
    private BufferVideoSource faceBufferSource;
    private Camera faceHalCamera;
    private final Object faceCameraLock = new Object();
    private static final int STREAM_WIDTH = 1280;
    private static final int STREAM_HEIGHT = 720;
    private static final int VIDEO_BITRATE = 1500 * 1000; // 1.5 Mbps
    private static final int STREAM_ROTATION = 0; // 0, 90, 180 ou 270. Outros valores causarão falha no prepareVideo
    private static final int SAMPLE_RATE = 32000;
    private static final boolean IS_STEREO = true;
    private static final int AUDIO_BITRATE = 128 * 1000;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        register(MainActivity.class);
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        super.onCreate(savedInstanceState);
        setBodyView(R.layout.activity_main);

        GradientDrawable topDrawable = new GradientDrawable(
                GradientDrawable.Orientation.LEFT_RIGHT,
                new int[]{Color.parseColor("#2B6DF8"), Color.parseColor("#00A2ED")}
        );
        setHeadBackground(topDrawable);

        editTextName = findViewById(R.id.editTextName);
        editTextIp = findViewById(R.id.editTextIp);
        editTextPort = findViewById(R.id.editTextPort);
        buttonConnect = findViewById(R.id.buttonConnect);
        buttonClear = findViewById(R.id.buttonClear);
        buttonClose = findViewById(R.id.buttonClose);
        buttonCloseBottom = findViewById(R.id.buttonCloseBottom);
        buttonGoHome = findViewById(R.id.buttonGoHome);
        buttonGoHomeBottom = findViewById(R.id.buttonGoHomeBottom);
        Button buttonHideKeypad = findViewById(R.id.buttonHideKeypad);
        Button buttonHideKeypadBottom = findViewById(R.id.buttonHideKeypadBottom);
        textViewVersion = findViewById(R.id.textViewVersion);
        if (textViewVersion != null) {
            textViewVersion.setText(String.format(Locale.US, "v%s (%d)",
                    BuildConfig.VERSION_NAME, BuildConfig.VERSION_CODE));
        }
        spinnerTopics = findViewById(R.id.spinnerTopics);
        textViewMessages = findViewById(R.id.textViewMessages);
        scrollView = findViewById(R.id.scrollView);
        textureViewPreview = findViewById(R.id.textureViewPreview);
        textViewPreviewHint = findViewById(R.id.textViewPreviewHint);
        if (textureViewPreview != null) {
            textureViewPreview.setVisibility(View.VISIBLE);
            textureViewPreview.setSurfaceTextureListener(new TextureView.SurfaceTextureListener() {
                @Override
                public void onSurfaceTextureAvailable(SurfaceTexture surface, int width, int height) {
                    if (switchRtmp != null && switchRtmp.isChecked()) {
                        textureViewPreview.post(() -> initRtmpStream());
                    }
                }

                @Override
                public void onSurfaceTextureSizeChanged(SurfaceTexture surface, int width, int height) {
                }

                @Override
                public boolean onSurfaceTextureDestroyed(SurfaceTexture surface) {
                    return true;
                }

                @Override
                public void onSurfaceTextureUpdated(SurfaceTexture surface) {
                }
            });
        }
        setPreviewHintVisible(true);
        switchRtmp = findViewById(R.id.switchRtmp);
        switchWsControl = findViewById(R.id.switchWsControl);
        switchAutoStartProxy = findViewById(R.id.switchAutoStartProxy);
        switchOpenProxyOnConnect = findViewById(R.id.switchOpenProxyOnConnect);
        buttonStartProxy = findViewById(R.id.buttonStartProxy);
        spinnerLanguage = findViewById(R.id.spinnerLanguage);
        checkBoxShowLogs = findViewById(R.id.checkBoxShowLogs);
        checkBoxAutoConnect = findViewById(R.id.checkBoxAutoConnect);
        checkBoxAutoLaunch = findViewById(R.id.checkBoxAutoLaunch);
        checkBoxAutoReturnHome = findViewById(R.id.checkBoxAutoReturnHome);
        editTextAutoReturnSeconds = findViewById(R.id.editTextAutoReturnSeconds);
        switchTtsType = findViewById(R.id.switchTtsType);

        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        String lastName = prefs.getString(KEY_LAST_NAME, "");
        if (lastName != null) {
            editTextName.setText(lastName);
        }

        String lastIp = prefs.getString(KEY_LAST_IP, null);
        if (lastIp != null) {
            editTextIp.setText(lastIp);
        }
        String lastPort = prefs.getString(KEY_LAST_PORT, "1883");
        cameraLivePath = prefs.getString(KEY_CAMERA_PATH, "face");
        orbbecBodyStreamer = new OrbbecBodyStreamer(this, this);
        orbbecBodyStreamer.register();
        hdCameraForeheadStreamer = new HdCameraForeheadStreamer(this, this);
        uvcUsbStreamer = new UvcUsbStreamer(this, this);
        String brokerForBody = editTextIp != null ? editTextIp.getText().toString().trim() : "";
        String brokerIp = brokerForBody.isEmpty() ? "192.168.1.4" : brokerForBody;
        orbbecBodyStreamer.setBrokerIp(brokerIp);
        hdCameraForeheadStreamer.setBrokerIp(brokerIp);
        uvcUsbStreamer.setBrokerIp(brokerIp);
        buttonHdSpike = findViewById(R.id.buttonHdSpike);
        if (buttonHdSpike != null) {
            buttonHdSpike.setOnClickListener(v -> {
                if (switchRtmp != null && switchRtmp.isChecked()) {
                    appendMessage("📷 HD spike: turn RTMP Off first (camera exclusive on Nano)");
                    return;
                }
                appendMessage("📷 HD spike (MAIN/SUB + HAL face)…");
                if (hdCameraForeheadStreamer != null) {
                    hdCameraForeheadStreamer.runSpikeAsync(true);
                }
            });
        }
        boolean wsControlOn = prefs.getBoolean(KEY_WS_CONTROL, true);
        if (switchWsControl != null) {
            switchWsControl.setChecked(wsControlOn);
            switchWsControl.setOnCheckedChangeListener((buttonView, isChecked) -> {
                getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                        .edit()
                        .putBoolean(KEY_WS_CONTROL, isChecked)
                        .apply();
                if (isChecked) {
                    startControlWebSocket();
                } else {
                    stopControlWebSocket();
                }
            });
            if (wsControlOn) {
                startControlWebSocket();
            }
        }
        if (switchAutoStartProxy != null) {
            switchAutoStartProxy.setChecked(
                    prefs.getBoolean(OpenSdkProxyLauncher.KEY_AUTO_START_PROXY, false));
            switchAutoStartProxy.setOnCheckedChangeListener((buttonView, isChecked) ->
                    getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit()
                            .putBoolean(OpenSdkProxyLauncher.KEY_AUTO_START_PROXY, isChecked)
                            .apply());
        }
        if (switchOpenProxyOnConnect != null) {
            switchOpenProxyOnConnect.setChecked(
                    prefs.getBoolean(OpenSdkProxyLauncher.KEY_OPEN_PROXY_ON_CONNECT, false));
            switchOpenProxyOnConnect.setOnCheckedChangeListener((buttonView, isChecked) ->
                    getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit()
                            .putBoolean(OpenSdkProxyLauncher.KEY_OPEN_PROXY_ON_CONNECT, isChecked)
                            .apply());
        }
        if (buttonStartProxy != null) {
            buttonStartProxy.setOnClickListener(v -> {
                if (!OpenSdkProxyLauncher.isInstalled(this)) {
                    appendMessage("❌ OpenSDK Proxy not installed ("
                            + OpenSdkProxyLauncher.PROXY_PACKAGE + ")");
                    return;
                }
                boolean ok = OpenSdkProxyLauncher.launch(this);
                appendMessage(ok
                        ? "🚀 OpenSDK Proxy launched (WS :9181 · HTTP :9182 default)"
                        : "❌ Failed to launch OpenSDK Proxy");
            });
        }
        if (editTextPort != null && lastPort != null) {
            editTextPort.setText(lastPort);
        }
        if (checkBoxAutoConnect != null) {
            checkBoxAutoConnect.setChecked(prefs.getBoolean(KEY_AUTO_CONNECT, true));
        }
        if (checkBoxAutoLaunch != null) {
            checkBoxAutoLaunch.setChecked(prefs.getBoolean(KEY_AUTO_LAUNCH, true));
        }
        if (checkBoxAutoReturnHome != null) {
            // Default OFF — returning Home backgrounds Bridge and drops RTMP publish.
            if (!prefs.getBoolean(KEY_AUTO_RETURN_HOME_MIGRATED_OFF, false)) {
                prefs.edit()
                        .putBoolean(KEY_AUTO_RETURN_HOME, false)
                        .putBoolean(KEY_AUTO_RETURN_HOME_MIGRATED_OFF, true)
                        .apply();
            }
            checkBoxAutoReturnHome.setChecked(prefs.getBoolean(KEY_AUTO_RETURN_HOME, false));
        }
        if (editTextAutoReturnSeconds != null) {
            int seconds = clampAutoReturnSeconds(
                    prefs.getInt(KEY_AUTO_RETURN_SECONDS, DEFAULT_AUTO_RETURN_SECONDS));
            editTextAutoReturnSeconds.setText(String.valueOf(seconds));
        }
        robotName = editTextName.getText().toString().trim();

        mqttHandler = new MqttHandler(this, this);

        mqttHandler.setBrokerIp(lastIp);
        applyBrokerPortToHandler();

        // Configuração dos adaptadores de idioma
        androidLanguageAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, 
                getResources().getStringArray(R.array.language_array));
        androidLanguageAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        
        sdkLanguageAdapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, 
                getResources().getStringArray(R.array.language_sdk_array));
        sdkLanguageAdapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        
        // Configuração inicial do spinner
        spinnerLanguage.setAdapter(androidLanguageAdapter);

//        mqttHandler.connect();
//        buttonConnect.setEnabled(false);
//        buttonConnect.setText("Conectando...");

        spinnerTopics.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                selectedTopic = parent.getItemAtPosition(position).toString();
                appendMessage("Filter updated: showing messages from \"" + selectedTopic + "\"");
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {}
        });

        buttonConnect.setOnClickListener(v -> {
            String ip = editTextIp.getText().toString().trim();
            if (ip.isEmpty()) {
                Toast.makeText(this, "Enter broker IP", Toast.LENGTH_SHORT).show();
                return;
            }

            String port = readBrokerPort();
            getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                    .edit()
                    .putString(KEY_LAST_IP, ip)
                    .putString(KEY_LAST_NAME, editTextName.getText().toString().trim())
                    .putString(KEY_LAST_PORT, port)
                    .apply();

            robotName = editTextName.getText().toString().trim();

            if (!connected) {
                mqttHandler.setBrokerIp(ip);
                applyBrokerPortToHandler();
                mqttHandler.connect();
                buttonConnect.setEnabled(false);
                buttonConnect.setText("Conectando...");
            } else {
                mqttHandler.disconnect();
                connected = false;
                buttonConnect.setText("Connect");
                appendMessage("Disconnected from broker.");
            }
        });

        if (checkBoxAutoConnect != null) {
            checkBoxAutoConnect.setOnCheckedChangeListener((buttonView, isChecked) ->
                    getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                            .edit()
                            .putBoolean(KEY_AUTO_CONNECT, isChecked)
                            .apply());
        }
        if (checkBoxAutoLaunch != null) {
            checkBoxAutoLaunch.setOnCheckedChangeListener((buttonView, isChecked) ->
                    getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                            .edit()
                            .putBoolean(KEY_AUTO_LAUNCH, isChecked)
                            .apply());
        }
        bindAutoReturnHomeListener();
        if (editTextAutoReturnSeconds != null) {
            editTextAutoReturnSeconds.addTextChangedListener(new TextWatcher() {
                @Override
                public void beforeTextChanged(CharSequence s, int start, int count, int after) {
                }

                @Override
                public void onTextChanged(CharSequence s, int start, int before, int count) {
                }

                @Override
                public void afterTextChanged(Editable s) {
                    persistAutoReturnHomePrefs();
                    scheduleAutoReturnHome();
                }
            });
        }

        buttonClear.setOnClickListener(v -> textViewMessages.setText(""));
        View.OnClickListener closeClick = v -> exitBridgeUi();
        if (buttonClose != null) {
            buttonClose.setOnClickListener(closeClick);
        }
        if (buttonCloseBottom != null) {
            buttonCloseBottom.setOnClickListener(closeClick);
        }
        View.OnClickListener homeClick = v -> {
            SoftInputHider.hideActivity(this);
            cancelAutoReturnHome();
            goHomeScreen();
        };
        if (buttonGoHome != null) {
            buttonGoHome.setOnClickListener(homeClick);
        }
        if (buttonGoHomeBottom != null) {
            buttonGoHomeBottom.setOnClickListener(homeClick);
        }
        View.OnClickListener hideKeypadClick = v -> {
            SoftInputHider.hideActivity(this);
            appendMessage("⌨️ Keypad hidden");
        };
        if (buttonHideKeypad != null) {
            buttonHideKeypad.setOnClickListener(hideKeypadClick);
        }
        if (buttonHideKeypadBottom != null) {
            buttonHideKeypadBottom.setOnClickListener(hideKeypadClick);
        }

        tts = new TextToSpeech(this, this);
        
        // Configuração do Spinner de idioma
        spinnerLanguage.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                String selectedLanguage = parent.getItemAtPosition(position).toString();
                if (switchTtsType.isChecked()) {
                    // TTS SDK
                    appendMessage("🌍 TTS SDK language changed to: " + selectedLanguage);
                } else {
                    // TTS Android
                    if (tts != null) {
                        Locale locale = getLocaleForLanguage(selectedLanguage);
                        if (locale != null) {
                            tts.setLanguage(locale);
                            appendMessage("🌍 TTS Android language changed to: " + selectedLanguage);
                        }
                    }
                }
                

            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {}
        });
        
        // Configuração do Switch para alternar entre TTS Android e SDK
        switchTtsType.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                // TTS SDK
                spinnerLanguage.setAdapter(sdkLanguageAdapter);
                switchTtsType.setText("TTS SDK (Android unchecked)");
                appendMessage("🔊 TTS changed to SDK");
            } else {
                // TTS Android
                spinnerLanguage.setAdapter(androidLanguageAdapter);
                switchTtsType.setText("TTS Android (SDK unchecked)");
                appendMessage("🔊 TTS changed to Android");
            }
            // Reset da seleção do spinner
            spinnerLanguage.setSelection(0);
        });
        
        if (checkBoxShowLogs != null) {
            checkBoxShowLogs.setChecked(prefs.getBoolean(KEY_SHOW_UI_LOG, false));
            checkBoxShowLogs.setOnCheckedChangeListener((buttonView, isChecked) -> {
                getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                        .edit()
                        .putBoolean(KEY_SHOW_UI_LOG, isChecked)
                        .apply();
                if (!isChecked) {
                    if (textViewMessages != null) {
                        textViewMessages.setText("");
                    }
                } else {
                    appendMessage("📋 UI log on");
                }
            });
        }
        
        // Verifica e solicita permissões de câmera
        checkCameraPermissions();
        


        switchRtmp.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked) {
                // Start streaming if permissions are granted, otherwise request
                if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED ||
                        ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
                    appendMessage("❌ RTMP: grant Camera + Mic, then toggle RTMP On again");
                    checkCameraPermissions();
                    buttonView.setChecked(false);
                } else {
                    applyAutoReturnHomeForRtmp(true);
                    setPreviewHintVisible(false);
                    if (rtmpStream == null || !rtmpStream.isStreaming()) {
                        if (textureViewPreview != null) {
                            textureViewPreview.setVisibility(View.VISIBLE);
                            textureViewPreview.post(this::initRtmpStream);
                        } else {
                            initRtmpStream();
                        }
                    }
                    // Orbbec body starts after head RTMP connects (see onConnectionSuccess).
                }
            } else {
                rtmpHandler.removeCallbacks(startBodyAfterHeadRunnable);
                stopRtmpStream();
                stopOrbbecBody();
                setPreviewHintVisible(true);
                applyAutoReturnHomeForRtmp(false);
            }
        });
        // Honor rule on cold start if RTMP switch was left On.
        applyAutoReturnHomeForRtmp(isRtmpEnabled());

        editTextName.addTextChangedListener(new TextWatcher() {
            @Override public void beforeTextChanged(CharSequence s, int start, int count, int after) {}
            @Override public void onTextChanged(CharSequence s, int start, int before, int count) {}
            @Override public void afterTextChanged(Editable s) {
                robotName = s.toString().trim();
                if (connected) {
                    subscribeToAllTopics();
                }
            }
        });

        maybeAutoConnect();
        // Nano soft keyboard / SwiftKey covers RTMP + Options if an EditText keeps focus.
        View root = findViewById(android.R.id.content);
        if (root != null) {
            root.post(() -> SoftInputHider.hideActivity(this));
        } else {
            SoftInputHider.hideActivity(this);
        }
    }

    private String readBrokerPort() {
        if (editTextPort == null) {
            return "1883";
        }
        String port = editTextPort.getText().toString().trim();
        return port.isEmpty() ? "1883" : port;
    }

    private void applyBrokerPortToHandler() {
        if (mqttHandler == null) {
            return;
        }
        mqttHandler.setBrokerPort(readBrokerPort());
    }

    /** Connect automatically when a saved broker IP exists and the checkbox is on. */
    private void maybeAutoConnect() {
        if (checkBoxAutoConnect == null || !checkBoxAutoConnect.isChecked()) {
            return;
        }
        String ip = editTextIp.getText().toString().trim();
        if (ip.isEmpty() || connected || mqttHandler == null) {
            return;
        }
        mqttHandler.setBrokerIp(ip);
        applyBrokerPortToHandler();
        mqttHandler.connect();
        buttonConnect.setEnabled(false);
        buttonConnect.setText("Conectando...");
        appendMessage("Auto-connecting to broker...");
    }

    @Override
    public void onInit(int status) {
        if (status == TextToSpeech.SUCCESS) {
            // Define o idioma padrão baseado na seleção atual do spinner
            try {
                String selectedLanguage = spinnerLanguage.getSelectedItem().toString();
                Locale locale = getLocaleForLanguage(selectedLanguage);
                tts.setLanguage(locale);
            } catch (Exception e) {
                // Se o spinner ainda não estiver pronto, usa inglês como padrão
                tts.setLanguage(new Locale("en", "US"));
            }
        } else {
            Log.e("TTS", "Falha ao inicializar TTS");
        }
    }

    @Override
    protected void onMainServiceConnected() {
        systemManager = (SystemManager) getUnitManager(FuncConstant.SYSTEM_MANAGER);
        wheelMotionManager = (WheelMotionManager) getUnitManager(FuncConstant.WHEELMOTION_MANAGER);
        headMotionManager = (HeadMotionManager) getUnitManager(FuncConstant.HEADMOTION_MANAGER);
        modularMotionManager = (ModularMotionManager) getUnitManager(FuncConstant.MODULARMOTION_MANAGER);
        speechManager = (SpeechManager) getUnitManager(FuncConstant.SPEECH_MANAGER);
        try {
            handMotionManager = (HandMotionManager) getUnitManager(FuncConstant.HANDMOTION_MANAGER);
            appendMessage(handMotionManager != null
                    ? "✋ HandMotionManager bound"
                    : "⚠️ HandMotionManager null");
        } catch (Exception e) {
            handMotionManager = null;
            appendMessage("⚠️ HandMotionManager: " + e.getMessage());
        }
        startBatteryPublishingLoop();
        startInfoPublishingLoop();

        // zera a cabeça na horizontal
        AbsoluteAngleHeadMotion absoluteAngleHeadMotion = new AbsoluteAngleHeadMotion(AbsoluteAngleHeadMotion.DIRECTION_HEAD_HORIZONTAL,
                AbsoluteAngleHeadMotion.ACTION_START, 100, 0);
        headMotionManager.doAbsoluteAngleMotion(absoluteAngleHeadMotion);
        
        // zera a cabeça na vertical
        absoluteAngleHeadMotion = new AbsoluteAngleHeadMotion(AbsoluteAngleHeadMotion.DIRECTION_HEAD_VERTICAL,
                AbsoluteAngleHeadMotion.ACTION_START, 100, 37);
        headMotionManager.doAbsoluteAngleMotion(absoluteAngleHeadMotion);
        
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        try {
            hdCameraManager = (HDCameraManager) getUnitManager(FuncConstant.HDCAMERA_MANAGER);
            if (hdCameraForeheadStreamer != null) {
                hdCameraForeheadStreamer.setHdCameraManager(hdCameraManager);
            }
            appendMessage(hdCameraManager != null
                    ? "📷 HDCameraManager bound"
                    : "⚠️ HDCameraManager null");
        } catch (Exception e) {
            appendMessage("⚠️ HDCameraManager: " + e.getMessage());
        }
        if (uvcUsbStreamer != null) {
            uvcUsbStreamer.refreshAvailability();
        }
        // No auto HD spike — Nano camera HAL is single-client; spike races Face RTMP.
        // Use the "HD spike stills" button with RTMP Off when needed.

        // Optional companion: OpenSDK Proxy for control while this app owns cameras/RTMP.
        if (OpenSdkProxyLauncher.isAutoStartEnabled(getSharedPreferences(PREFS_NAME, MODE_PRIVATE))) {
            runOnUiThread(() -> {
                if (OpenSdkProxyLauncher.launch(MainActivity.this)) {
                    appendMessage("🚀 Auto-started OpenSDK Proxy");
                } else if (!OpenSdkProxyLauncher.isInstalled(MainActivity.this)) {
                    appendMessage("⚠️ Auto-start Proxy: not installed");
                }
            });
        }

        hardWareManager.setOnHareWareListener(
                new TouchSensorListener() {
                    @Override
                    public void onTouch(int part) {
                        try {
                            JSONObject touchJson = new JSONObject();
                            touchJson.put("part", part);
                            touchJson.put("description", getTouchPartName(part));
                            publishTelemetry("touch", touchJson);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                }
        );

        hardWareManager.setOnHareWareListener(
                new PIRListener() {
                    @Override
                    public void onPIRCheckResult(boolean isChecked, int part) {
                        try {
                            JSONObject pirJson = new JSONObject();
                            pirJson.put("part", part == 1 ? "front" : "back");
                            pirJson.put("status", isChecked ? 1 : 0);
                            publishTelemetry("pir", pirJson);
                        } catch (Exception e) {
                            e.printStackTrace();
                        }
                    }
                }
        );

        hardWareManager.setOnHareWareListener(
                new InfrareListener() {
                    @Override
                    public void infrareDistance(int part, int distance) {
                        long now = System.currentTimeMillis();
                        Long last = lastIRUpdate.get(part); // pode ser null

                        if (last == null || now - last >= IR_UPDATE_INTERVAL_MS) {
                            lastIRUpdate.put(part, now);
                            try {
                                JSONObject irJson = new JSONObject();
                                irJson.put("sensor", part);
                                irJson.put("distance_cm", distance);
                                publishTelemetry("ir", irJson);
                            } catch (Exception e) {
                                e.printStackTrace();
                            }
                        }
                    }
                }
        );

        hardWareManager.setOnHareWareListener(new VoiceLocateListener() {
            @Override
            public void voiceLocateResult(int angle) {
                try {
                    JSONObject voiceJson = new JSONObject();
                    voiceJson.put("angle", angle);
                    publishTelemetry("voice_angle", voiceJson);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        hardWareManager.setOnHareWareListener(new ObstacleListener() {
            @Override
            public void onObstacleStatus(boolean status) {
                try{
                    JSONObject json = new JSONObject();
                    json.put("status", status ? 1 : 0);
                    publishTelemetry("obstacle", json);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        hardWareManager.setOnHareWareListener(new GyroscopeListener() {
            @Override
            public void gyroscopeData(float x, float y, float z) {
                long now = System.currentTimeMillis();
                if (now - lastImuPublishMs < IMU_UPDATE_INTERVAL_MS) {
                    return;
                }
                lastImuPublishMs = now;
                try {
                    JSONObject gyroJson = new JSONObject();
                    gyroJson.put("x", x);
                    gyroJson.put("y", y);
                    gyroJson.put("z", z);
                    publishTelemetry("imu", gyroJson);
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        speechManager.setOnSpeechListener(new RecognizeListener() {
            @Override
            public boolean onRecognizeResult(@NonNull Grammar grammar) {
                String texto = grammar.getText();
                publishSpeechToMqtt(texto);
                return false; // não intercepta o reconhecimento
            }

            @Override
            public void onRecognizeText(@NonNull RecognizeTextBean recognizeTextBean) {
                String texto = recognizeTextBean.getText();
                publishSpeechToMqtt(texto);
            }

            @Override
            public void onRecognizeVolume(int volume) {
                // opcional: mostrar o volume de entrada
            }

            @Override
            public void onStartRecognize() {
                Log.d("TTS", "Reconhecimento iniciado");
            }

            @Override
            public void onStopRecognize() {
                Log.d("TTS", "Reconhecimento parado");
            }

            @Override
            public void onError(int i, int i1) {
                Log.e("TTS", "Erro no reconhecimento: " + i + ", " + i1);
            }
        });
    }

    private String getLocalIpAddress() {
        try {
            Enumeration<NetworkInterface> interfaces = NetworkInterface.getNetworkInterfaces();
            for (NetworkInterface ni : Collections.list(interfaces)) {
                for (InetAddress address : Collections.list(ni.getInetAddresses())) {
                    if (!address.isLoopbackAddress() && address instanceof java.net.Inet4Address) {
                        return address.getHostAddress();
                    }
                }
            }
        } catch (Exception ignored) {}
        return "localhost";
    }

    private String getTouchPartName(int part) {
        switch (part) {
            case 1: return "chin_right";
            case 2: return "chin_left";
            case 3: return "chest_right";
            case 4: return "chest_left";
            case 5: return "backhead_left";
            case 6: return "backhead_right";
            case 7: return "back_left";
            case 8: return "back_right";
            case 9: return "hand_left";
            case 10: return "hand_right";
            case 11: return "head_middle";
            case 12: return "head_right_front";
            case 13: return "head_left_front";
            default: return "unknown";
        }
    }

    @Override
    public void onConnectionStatusChanged(boolean connected) {
        runOnUiThread(() -> {
            this.connected = connected;
            buttonConnect.setEnabled(true);
            buttonConnect.setText(connected ? "Disconnect" : "Connect");

            appendMessage(connected ? "Successfully connected to broker!" : "Connection failed or disconnected.");

            if (connected) subscribeToAllTopics();
        });
    }

    @Override
    public void onMessageReceived(String topic, String message) {
        final String finalTopic = topic;
        final String finalMessage = message;

        // Never mirror high-rate telemetry into the on-screen log (ANR risk).
        if (!isTelemetryTopic(finalTopic)) {
            runOnUiThread(() -> {
                if ("All".equals(selectedTopic) || selectedTopic.equals(finalTopic)) {
                    appendVerboseMessage("[" + finalTopic + "]: " + finalMessage);
                }
            });
        }

        if (getPrefixedTopic("light").equals(topic)) {
            handleLightControl(message);
        } else if (getPrefixedTopic("move").equals(topic)) {
            handleMotionControl(message);
        } else if (getPrefixedTopic("joints").equals(topic)) {
            handleJointMotion(message);
        } else if (getPrefixedTopic("led").equals(topic)) {
            handleLedControl(message);
        } else if (getPrefixedTopic("camera").equals(topic)) {
            handleCameraControl(message);
        } else if (getPrefixedTopic("emotion").equals(topic)) {
            handleEmotionControl(message);
        } else if (getPrefixedTopic("autonomy").equals(topic)) {
            handleAutonomyControl(message);
        } else if (getPrefixedTopic("speak").equals(topic)) {
            try {
                JSONObject json = new JSONObject(message);
                String texto = json.optString("msg", "");
                if (!texto.isEmpty()) {
                    int speed = json.optInt("speed", 50);
                    int volume = json.optInt("volume", 70);
                    if (switchTtsType.isChecked()) {
                        handleSpeakSDK(texto, speed, volume);
                    } else {
                        handleSpeakAndroid(texto, speed, volume);
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        } else if (getPrefixedTopic("system").equals(topic)) {
            handleSystemControl(message);
        } else if (getPrefixedTopic("proxy").equals(topic)
                || getPrefixedTopic("opensdk").equals(topic)) {
            try {
                handleProxyCompanionRequest(new JSONObject(message));
            } catch (Exception e) {
                appendMessage("proxy MQTT: " + e.getMessage());
            }
        }
    }

    private void handleCameraControl(String message) {
        try {
            JSONObject json = new JSONObject(message);
            boolean touchedSource = json.has("source");
            boolean touchedRtmp = json.has("rtmp") || json.has("enabled") || json.has("streaming");
            boolean touchedLogs = json.has("show_logs") || json.has("ui_log") || json.has("show_ui_log");
            if (json.has("action")) {
                String action = json.optString("action", "").trim().toLowerCase(Locale.US);
                if ("hd_spike".equals(action) || "spike".equals(action)) {
                    appendMessage("📷 /camera hd_spike");
                    if (switchRtmp != null && switchRtmp.isChecked()) {
                        appendMessage("📷 hd_spike blocked while RTMP On");
                        return;
                    }
                    if (hdCameraForeheadStreamer != null) {
                        hdCameraForeheadStreamer.runSpikeAsync(true);
                    }
                    return;
                }
            }
            if (touchedLogs) {
                boolean on = json.has("show_logs")
                        ? json.optBoolean("show_logs", false)
                        : json.has("ui_log")
                        ? json.optBoolean("ui_log", false)
                        : json.optBoolean("show_ui_log", false);
                setShowUiLogFromRemote(on);
            }
            if (!touchedSource && !touchedRtmp && !touchedLogs) {
                appendMessage("📷 /camera needs source, rtmp, and/or show_logs");
                return;
            }
            if (!touchedSource && !touchedRtmp) {
                return;
            }
            if (touchedSource) {
                String source = json.optString("source", cameraLivePath).trim().toLowerCase(Locale.US);
                if ("stream".equals(source)) {
                    source = "face";
                }
                if (!isKnownCameraSource(source)) {
                    appendMessage("📷 Unknown camera source: " + source);
                    return;
                }
                cameraLivePath = source;
                getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                        .edit()
                        .putString(KEY_CAMERA_PATH, cameraLivePath)
                        .apply();
                appendMessage("📷 Preferred preview → live/" + cameraLivePath
                        + " (" + cameraSourceNote(cameraLivePath) + ")");
                maybeStartAuxCameraPublisher(source);
            }
            if (touchedRtmp) {
                boolean on = json.has("rtmp")
                        ? json.optBoolean("rtmp", false)
                        : json.has("enabled")
                        ? json.optBoolean("enabled", false)
                        : json.optBoolean("streaming", false);
                setRtmpEnabledFromRemote(on);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private static boolean isKnownCameraSource(String source) {
        return "face".equals(source) || "body".equals(source) || "forehead".equals(source)
                || "usb".equals(source) || "depth".equals(source);
    }

    private String cameraSourceNote(String source) {
        switch (source) {
            case "body":
                return "Orbbec color";
            case "forehead":
                return "HDCamera / forehead";
            case "usb":
                return "external UVC";
            case "depth":
                return "Orbbec depth-gray";
            default:
                return "head HAL";
        }
    }

    /** Start forehead/USB only when selected — concurrent HW encode with Face crashes Nano. */
    private void maybeStartAuxCameraPublisher(String source) {
        if (switchRtmp == null || !switchRtmp.isChecked()) {
            return;
        }
        String ip = sanitizeBrokerIp(editTextIp != null ? editTextIp.getText().toString() : "");
        if (ip.isEmpty()) {
            return;
        }
        if ("forehead".equals(source) && hdCameraForeheadStreamer != null) {
            hdCameraForeheadStreamer.setBrokerIp(ip);
            appendMessage("📷 Starting HD forehead → live/forehead…");
            hdCameraForeheadStreamer.start();
        } else if ("usb".equals(source) && uvcUsbStreamer != null) {
            uvcUsbStreamer.setBrokerIp(ip);
            uvcUsbStreamer.refreshAvailability();
            if (uvcUsbStreamer.findSecondaryCameraId() >= 0) {
                appendMessage("📷 Starting USB UVC → live/usb…");
                uvcUsbStreamer.start();
            } else {
                appendMessage("📷 USB UVC skipped (no secondary Camera HAL id)");
            }
        }
    }

    /**
     * Prefer a single IPv4. Double-paste often becomes {@code 192.168.1.4192.168.1.4}
     * (the two addresses share the digit {@code 4}). Find the first valid 0–255 octet IPv4
     * as a prefix match (no trailing {@code \\b}, which would reject that case).
     */
    private static String sanitizeBrokerIp(String raw) {
        if (raw == null) {
            return "";
        }
        String s = raw.trim();
        if (s.isEmpty() || s.contains("x") || s.contains("X")) {
            // Placeholder like 192.168.x.x
            return "";
        }
        // (?!\\d) so "192.168.1.4192…" does not yield "192.168.1.41"; glued
        // "192.168.1.4"+"192.168.1.4" still finds a real trailing/leading address.
        java.util.regex.Matcher m = java.util.regex.Pattern.compile(
                "(?:25[0-5]|2[0-4]\\d|1\\d\\d|[1-9]?\\d)"
                        + "(?:\\.(?:25[0-5]|2[0-4]\\d|1\\d\\d|[1-9]?\\d)){3}(?!\\d)")
                .matcher(s);
        if (m.find()) {
            return m.group();
        }
        return "";
    }

    /** Toggle bridge RTMP switch from MQTT/WS (keeps UI + stream in sync). */
    private void setRtmpEnabledFromRemote(boolean on) {
        runOnUiThread(() -> {
            if (switchRtmp == null) return;
            if (switchRtmp.isChecked() == on) {
                if (on) {
                    // Proxy Play may ask for On while the switch is stuck On but publish is dead.
                    appendMessage("📡 RTMP already On — force republish");
                    try {
                        stopRtmpStream();
                    } catch (Exception ignored) {
                    }
                    setPreviewHintVisible(false);
                    if (textureViewPreview != null) {
                        textureViewPreview.setVisibility(View.VISIBLE);
                        textureViewPreview.post(this::initRtmpStream);
                    } else {
                        initRtmpStream();
                    }
                } else {
                    appendMessage("📡 RTMP already Off");
                }
                return;
            }
            appendMessage("📡 RTMP → " + (on ? "On" : "Off") + " (remote)");
            switchRtmp.setChecked(on);
        });
    }

    private void handleLightControl(String message) {
        try {
            int level = -1;
            message = message.trim();
            if (message.startsWith("{")) {
                JSONObject obj = new JSONObject(message);
                if (obj.has("white")) {
                    level = obj.getInt("white");
                }
            } else {
                level = Integer.parseInt(message);
            }

            if (level == 0) {
                hardWareManager.switchWhiteLight(false);
            } else if (level >= 1 && level <= 3) {
                hardWareManager.switchWhiteLight(true);
                hardWareManager.setWhiteLightLevel(level);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void handleMotionControl(String message) {
        try {
            JSONObject json = new JSONObject(message);
            String direction = json.optString("direction", "stop");
            int speed = json.optInt("speed", 5);
            int distance = json.optInt("distance", 0);
            int duration = json.optInt("duration", 0);

            if (motionStopRunnable != null) {
                motionHandler.removeCallbacks(motionStopRunnable);
                motionStopRunnable = null;
            }
            if (duration > 0) {
                byte action = getActionFromDirection(direction);
                NoAngleWheelMotion motion = new NoAngleWheelMotion(action, speed, 100, 0, NoAngleWheelMotion.STATUS_KEEP);
                wheelMotionManager.doNoAngleMotion(motion);
                motionStopRunnable = () -> {
                    wheelMotionManager.doNoAngleMotion(new NoAngleWheelMotion(
                            NoAngleWheelMotion.ACTION_STOP, 0, 0, 0, NoAngleWheelMotion.STATUS_KEEP));
                    motionStopRunnable = null;
                };
                motionHandler.postDelayed(motionStopRunnable, duration * 1000);

                return;
            }

            if (isLinearDirection(direction) && distance > 0) {
                byte action = getLinearAction(direction);
                DistanceWheelMotion distMotion = new DistanceWheelMotion(action, speed, distance);
                wheelMotionManager.doDistanceMotion(distMotion);
                return;
            }

            if (isRotationDirection(direction) && distance > 0) {
                byte action = getRotationAction(direction);
                RelativeAngleWheelMotion angleMotion = new RelativeAngleWheelMotion(action, speed, distance);
                wheelMotionManager.doRelativeAngleMotion(angleMotion);
                return;
            }

            if ("stop".equals(direction)) {
                wheelMotionManager.doNoAngleMotion(new NoAngleWheelMotion(
                        NoAngleWheelMotion.ACTION_STOP, 0, 0, 0, NoAngleWheelMotion.STATUS_KEEP));
                return;
            }

            // fallback: movimento contínuo
            byte action = getActionFromDirection(direction);
            NoAngleWheelMotion motion = new NoAngleWheelMotion(action, speed, 100, 0, NoAngleWheelMotion.STATUS_KEEP);
            wheelMotionManager.doNoAngleMotion(motion);

        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private byte getActionFromDirection(String direction) {
        switch (direction) {
            case "forward": return NoAngleWheelMotion.ACTION_FORWARD;
            case "backward": return NoAngleWheelMotion.ACTION_BACK;
            case "left": return NoAngleWheelMotion.ACTION_LEFT_TRANSLATION;
            case "right": return NoAngleWheelMotion.ACTION_RIGHT_TRANSLATION;
            case "left_forward": return NoAngleWheelMotion.ACTION_LEFT_FORWARD;
            case "left_back": return NoAngleWheelMotion.ACTION_LEFT_BACK;
            case "right_forward": return NoAngleWheelMotion.ACTION_RIGHT_FORWARD;
            case "right_back": return NoAngleWheelMotion.ACTION_RIGHT_BACK;
            case "turn_left": return NoAngleWheelMotion.ACTION_LEFT;
            case "turn_right": return NoAngleWheelMotion.ACTION_RIGHT;
            default: return NoAngleWheelMotion.ACTION_STOP;
        }
    }

    private byte getLinearAction(String direction) {
        switch (direction) {
            case "forward": return DistanceWheelMotion.ACTION_FORWARD;
            case "backward": return DistanceWheelMotion.ACTION_BACK;
            case "left": return DistanceWheelMotion.ACTION_LEFT_TRANSLATION;
            case "right": return DistanceWheelMotion.ACTION_RIGHT_TRANSLATION;
            case "left_forward": return DistanceWheelMotion.ACTION_LEFT_FORWARD;
            case "left_back": return DistanceWheelMotion.ACTION_LEFT_BACK;
            case "right_forward": return DistanceWheelMotion.ACTION_RIGHT_FORWARD;
            case "right_back": return DistanceWheelMotion.ACTION_RIGHT_BACK;
            default: return DistanceWheelMotion.ACTION_STOP;
        }
    }

    private byte getRotationAction(String direction) {
        switch (direction) {
            case "turn_left": return RelativeAngleWheelMotion.ACTION_TURN_LEFT;
            case "turn_right": return RelativeAngleWheelMotion.ACTION_TURN_RIGHT;
            default: return RelativeAngleWheelMotion.ACTION_TURN_STOP;
        }
    }

    private boolean isLinearDirection(String direction) {
        return direction.equals("forward") || direction.equals("backward") ||
                direction.equals("left") || direction.equals("right") ||
                direction.equals("left_forward") || direction.equals("left_back") ||
                direction.equals("right_forward") || direction.equals("right_back");
    }

    private boolean isRotationDirection(String direction) {
        return "turn_left".equals(direction) || "turn_right".equals(direction);
    }

    private void handleJointMotion(String message) {
        try {
            JSONObject json = new JSONObject(message);
            String joint = json.optString("joint"); // "head_pan", "head_tilt", "wing_left", "wing_right", "wing_both"
            int angle = json.optInt("angle");
            int speed = json.optInt("speed", 50);
            String action = json.optString("action", "").trim().toLowerCase(Locale.US);
            boolean hasAngle = json.has("angle");

            AbsoluteAngleHeadMotion absoluteAngleHeadMotion;

            switch (joint) {
                case "head_pan":
                    // Nano MainService AbsoluteAngleHeadMotion: action=1 vertical, action=2 horizontal.
                    // Stock dsdk puts axis in direction and ACTION_START in action — MainService
                    // ignores direction, so ACTION_START=1 made every pan a vertical move.
                    absoluteAngleHeadMotion = new AbsoluteAngleHeadMotion(
                            AbsoluteAngleHeadMotion.DIRECTION_HEAD_HORIZONTAL,
                            (byte) 2,
                            speed,
                            angle);
                    headMotionManager.doAbsoluteAngleMotion(absoluteAngleHeadMotion);
                    break;
                case "head_tilt":
                    absoluteAngleHeadMotion = new AbsoluteAngleHeadMotion(
                            AbsoluteAngleHeadMotion.DIRECTION_HEAD_VERTICAL,
                            AbsoluteAngleHeadMotion.ACTION_START,
                            speed,
                            angle);
                    headMotionManager.doAbsoluteAngleMotion(absoluteAngleHeadMotion);
                    break;
                case "wing_left":
                    handleWingMotion(CombinationHandMotion.PART_LEFT, angle, action, speed, hasAngle);
                    break;
                case "wing_right":
                    handleWingMotion(CombinationHandMotion.PART_RIGHT, angle, action, speed, hasAngle);
                    break;
                case "wing_both":
                    handleWingMotion(CombinationHandMotion.PART_BOTH, angle, action, speed, hasAngle);
                    break;
                default:
                    break;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Nano MainService only executes hand cmds 545–547 (Combination 4374 is a silent ACK).
     * Discrete up/down/stop use HAND_MOTION_NO_ANGLE (545). Absolute 547 for explicit angles.
     * If OpenSDK Proxy currently owns MainService, forward to local proxy HTTP
     * (default :9182, legacy :8082 fallback).
     */
    private void handleWingMotion(byte part, int angle, String action, int speed, boolean hasAngle) {
        String jointName = part == CombinationHandMotion.PART_LEFT ? "wing_left"
                : part == CombinationHandMotion.PART_RIGHT ? "wing_right" : "wing_both";
        // Companion mode: proxy often holds the OpenSDK bind — forward wings there.
        if (shouldForwardWingsToProxy()) {
            forwardWingToProxy(jointName, angle, action, speed, hasAngle);
            return;
        }
        if (handMotionManager == null) {
            if (OpenSdkProxyLauncher.isInstalled(this)) {
                forwardWingToProxy(jointName, angle, action, speed, hasAngle);
            } else {
                appendMessage("wing skipped: HandMotionManager null");
            }
            return;
        }
        writeQihanSettingInt("robot_arm_action", 1);
        int clampedSpeed = Math.max(1, Math.min(100, speed));
        String resolved = action == null ? "" : action.trim().toLowerCase(Locale.US);
        try {
            if (resolved.isEmpty() && hasAngle) {
                sendHandAbsolute(part, clampedSpeed, Math.max(0, Math.min(270, angle)));
                appendMessage("wing absolute part=" + part + " angle=" + angle);
                return;
            }
            if (resolved.isEmpty()) {
                resolved = angle >= 135 ? "up" : "down";
            }
            switch (resolved) {
                case "up":
                case "raise":
                case "wave":
                    sendHandNoAngle(part, clampedSpeed, (byte) 1 /* ACTION_UP */);
                    appendMessage("wing up part=" + part + " via 545");
                    break;
                case "down":
                case "drop":
                    sendHandNoAngle(part, clampedSpeed, (byte) 2 /* ACTION_DOWN */);
                    appendMessage("wing down part=" + part + " via 545");
                    break;
                case "reset":
                    try {
                        handMotionManager.doResetMotion();
                    } catch (Exception ignored) {
                        sendHandNoAngle(part, clampedSpeed, (byte) 2);
                        sendHandAbsolute(part, clampedSpeed, 0);
                    }
                    appendMessage("wing reset");
                    break;
                case "stop":
                    try {
                        handMotionManager.doStopMotion();
                    } catch (Exception ignored) {
                        sendHandNoAngle(part, clampedSpeed, (byte) 0 /* ACTION_STOP */);
                    }
                    appendMessage("wing stop");
                    break;
                default:
                    appendMessage("Unknown wing action: " + resolved);
                    break;
            }
        } catch (Exception e) {
            Log.e("MQTT", "wing send failed", e);
            if (OpenSdkProxyLauncher.isInstalled(this)) {
                appendMessage("wing local failed — forwarding to proxy");
                forwardWingToProxy(jointName, angle, action, speed, hasAngle);
            } else {
                appendMessage("wing failed: " + e.getMessage());
            }
        }
    }

    /** True when OpenSDK Proxy process is alive (likely holds MainService). */
    private boolean shouldForwardWingsToProxy() {
        if (!OpenSdkProxyLauncher.isInstalled(this)) {
            return false;
        }
        try {
            android.app.ActivityManager am =
                    (android.app.ActivityManager) getSystemService(ACTIVITY_SERVICE);
            if (am == null) {
                return false;
            }
            for (android.app.ActivityManager.RunningAppProcessInfo p : am.getRunningAppProcesses()) {
                if (p != null && OpenSdkProxyLauncher.PROXY_PACKAGE.equals(p.processName)) {
                    return true;
                }
            }
        } catch (Exception ignored) {
        }
        return false;
    }

    private void forwardWingToProxy(String joint, int angle, String action, int speed, boolean hasAngle) {
        new Thread(() -> {
            try {
                JSONObject body = new JSONObject();
                body.put("joint", joint);
                body.put("speed", Math.max(1, Math.min(100, speed)));
                if (action != null && !action.trim().isEmpty()) {
                    body.put("action", action.trim().toLowerCase(Locale.US));
                } else if (hasAngle) {
                    body.put("angle", angle);
                } else {
                    body.put("action", angle >= 135 ? "up" : "down");
                }
                // Ensure proxy is up so it can bind MainService.
                OpenSdkProxyLauncher.launch(MainActivity.this);
                byte[] bytes = body.toString().getBytes(java.nio.charset.StandardCharsets.UTF_8);
                int code = OpenSdkProxyLauncher.postLocalHttp("/joints", bytes);
                if (code > 0) {
                    Log.i("MQTT", "wing forwarded to proxy HTTP " + code + " " + body);
                    runOnUiThread(() -> appendMessage("wing → proxy " + joint + " HTTP " + code));
                } else {
                    throw new IllegalStateException("proxy HTTP :9182/:8082 unreachable");
                }
            } catch (Exception e) {
                Log.w("MQTT", "wing proxy forward failed: " + e.getMessage());
                runOnUiThread(() -> appendMessage("wing proxy forward failed: " + e.getMessage()));
            }
        }, "wing-proxy-fwd").start();
    }

    private void sendHandNoAngle(byte part, int speed, byte action) throws Exception {
        // Prefer SDK bean + private doNoAngleMotion (correct Gson shape for this dsdk).
        try {
            NoAngleHandMotion motion = new NoAngleHandMotion(
                    part,
                    NoAngleHandMotion.DIRECTION_SHOULDER_FRONT_BACK,
                    action,
                    speed);
            invokeHandMotion(handMotionManager, "doNoAngleMotion", NoAngleHandMotion.class, motion);
            Log.i("MQTT", "wing NoAngle via SDK part=" + part + " action=" + action);
            return;
        } catch (Exception e) {
            Log.w("MQTT", "doNoAngleMotion reflect failed, raw Order 545: " + e.getMessage());
        }
        JSONObject content = new JSONObject();
        content.put("part", part);
        content.put("direction", 1);
        content.put("speed", speed);
        content.put("action", action & 0xff);
        content.put("mode", 1);
        sendHandOrder(545, content.toString());
    }

    private void sendHandAbsolute(byte part, int speed, int degrees) throws Exception {
        int deg = Math.max(0, Math.min(270, degrees));
        try {
            AbsoluteAngleHandMotion motion = new AbsoluteAngleHandMotion(
                    part,
                    AbsoluteAngleHandMotion.DIRECTION_SHOULDER_FRONT_BACK,
                    AbsoluteAngleHandMotion.ACTION_START,
                    speed,
                    deg);
            invokeHandMotion(handMotionManager, "doAbsoluteAngleMotion", AbsoluteAngleHandMotion.class, motion);
            Log.i("MQTT", "wing Absolute via SDK part=" + part + " deg=" + deg);
            return;
        } catch (Exception e) {
            Log.w("MQTT", "doAbsoluteAngleMotion reflect failed, raw Order 547: " + e.getMessage());
        }
        JSONObject content = new JSONObject();
        content.put("part", part);
        content.put("direction", 1);
        content.put("action", 1);
        content.put("speed", speed);
        content.put("lsbAngle", deg & 0xff);
        content.put("msbAngle", (deg >> 8) & 0xff);
        content.put("mode", 3);
        sendHandOrder(547, content.toString());
    }

    private void invokeHandMotion(HandMotionManager hand, String methodName, Class<?> argType, Object arg)
            throws Exception {
        Method m = hand.getClass().getDeclaredMethod(methodName, argType);
        m.setAccessible(true);
        m.invoke(hand, arg);
    }

    private void sendHandOrder(int cmd, String content) throws Exception {
        BindBaseInterface listener = resolveHandListener(handMotionManager);
        if (listener == null) {
            throw new IllegalStateException("HandMotionManager listener unavailable");
        }
        listener.sendCommandToMainService(new Order(cmd, 0, content), null);
        Log.i("MQTT", "wing Order cmd=" + cmd + " " + content);
    }

    private BindBaseInterface resolveHandListener(HandMotionManager hand) {
        if (hand == null) {
            return null;
        }
        String[] names = {
                "getListener$dsdk_NormalRelease",
                "getListener$opensdk_NormalRelease",
                "getListener"
        };
        for (String name : names) {
            try {
                Method m = hand.getClass().getMethod(name);
                Object result = m.invoke(hand);
                if (result instanceof BindBaseInterface) {
                    return (BindBaseInterface) result;
                }
            } catch (Exception ignored) {
            }
        }
        // Some dsdk builds expose listener as a field.
        try {
            java.lang.reflect.Field f = hand.getClass().getDeclaredField("listener");
            f.setAccessible(true);
            Object result = f.get(hand);
            if (result instanceof BindBaseInterface) {
                return (BindBaseInterface) result;
            }
        } catch (Exception ignored) {
        }
        return null;
    }

    /**
     * Face emotion. MQTT /emotion: {"emotion":"smile"} (case-insensitive; aliases: laugh→LAUGHTER).
     * Visible on the robot face screen (not while this bridge UI is foreground).
     */
    private void handleEmotionControl(String message) {
        try {
            if (systemManager == null) {
                return;
            }
            JSONObject json = new JSONObject(message);
            String raw = json.optString("emotion", json.optString("name", "")).trim();
            if (raw.isEmpty()) {
                return;
            }
            EmotionsType emotion = resolveEmotion(raw);
            if (emotion != null) {
                systemManager.showEmotion(emotion);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private EmotionsType resolveEmotion(String raw) {
        String key = raw.trim().toUpperCase().replace('-', '_').replace(' ', '_');
        switch (key) {
            case "LAUGH":
            case "LAUGHING":
                key = "LAUGHTER";
                break;
            case "HAPPY":
                key = "SMILE";
                break;
            case "PRAISE":
            case "PRISE":
                key = "PRISE";
                break;
            default:
                break;
        }
        try {
            return EmotionsType.valueOf(key);
        } catch (IllegalArgumentException e) {
            Log.w("MQTT", "Unknown emotion: " + raw);
            return null;
        }
    }

    /**
     * Autonomy helpers. MQTT /autonomy:
     * {"mode":"charge|wander|follow","enabled":true}
     * Follow speech mute (HumanFollow): {"mode":"follow","speech_feedback":false}
     * Combined: {"mode":"follow","enabled":true,"speech_feedback":true}
     */
    private void handleAutonomyControl(String message) {
        try {
            JSONObject json = new JSONObject(message);
            String mode = json.optString("mode", "").trim().toLowerCase();
            boolean hasEnabled = json.has("enabled");
            boolean enabled = json.optBoolean("enabled", true);
            boolean hasSpeech = json.has("speech_feedback") || json.has("speech");
            boolean speechFeedback = json.has("speech_feedback")
                    ? json.optBoolean("speech_feedback", true)
                    : json.optBoolean("speech", true);
            switch (mode) {
                case "charge":
                case "auto_charge":
                case "dock":
                    if (hasEnabled && modularMotionManager != null) {
                        modularMotionManager.switchCharge(enabled);
                    }
                    break;
                case "wander":
                    if (hasEnabled && modularMotionManager != null) {
                        modularMotionManager.switchWander(enabled);
                    }
                    break;
                case "follow":
                    // HumanFollow owns chase. Do NOT call switchFollow(true): MainService
                    // re-launches without follow=true (preview-only) and arms onboard cruise
                    // that looks like wander. Only clear the flag on off.
                    if (hasEnabled || hasSpeech) {
                        notifyHumanFollow(
                                hasEnabled ? Boolean.valueOf(enabled) : null,
                                hasSpeech ? Boolean.valueOf(speechFeedback) : null);
                    }
                    if (hasEnabled && !enabled && modularMotionManager != null) {
                        modularMotionManager.switchFollow(false);
                    }
                    break;
                default:
                    Log.w("MQTT", "Unknown autonomy mode: " + mode);
                    break;
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    /**
     * Launch / update {@code com.qihan.HumanFollow}. Pass null to leave follow or speech unchanged.
     */
    private void notifyHumanFollow(Boolean followEnabled, Boolean speechFeedback) {
        runOnUiThread(() -> {
            try {
                Log.i("MQTT", "notify HumanFollow follow=" + followEnabled
                        + " speech=" + speechFeedback);
                if (followEnabled != null) {
                    android.content.Intent followBroadcast = new android.content.Intent(
                            "com.qihan.HumanFollow.SET_FOLLOWING");
                    followBroadcast.setPackage("com.qihan.HumanFollow");
                    followBroadcast.putExtra("enabled", followEnabled.booleanValue());
                    followBroadcast.putExtra("follow", followEnabled.booleanValue());
                    if (speechFeedback != null) {
                        followBroadcast.putExtra("speech_feedback",
                                speechFeedback.booleanValue());
                    }
                    sendBroadcast(followBroadcast);
                } else if (speechFeedback != null) {
                    android.content.Intent broadcast = new android.content.Intent(
                            "com.qihan.HumanFollow.SET_SPEECH_FEEDBACK");
                    broadcast.setPackage("com.qihan.HumanFollow");
                    broadcast.putExtra("speech_feedback", speechFeedback.booleanValue());
                    sendBroadcast(broadcast);
                }
            } catch (Exception launchEx) {
                Log.w("MQTT", "HumanFollow notify failed", launchEx);
            }
        });
    }

    private void handleLedControl(String message) {
        try {
            JSONObject obj = new JSONObject(message);
            String part = obj.optString("part", "all_head");
            String mode = obj.optString("mode", "blue");
            byte duration = (byte) obj.optInt("duration", 1);
            byte random = (byte) obj.optInt("random", 1);

            byte partByte = getLedPart(part);
            byte modeByte = getLedMode(mode);

            hardWareManager.setLED(new LED(partByte, modeByte, duration, random));
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private byte getLedPart(String part) {
        switch (part.toLowerCase()) {
            case "all_hand": return LED.PART_ALL_HAND;
            case "all_head": return LED.PART_ALL_HEAD;
            case "left_hand": return LED.PART_LEFT_HAND;
            case "right_hand": return LED.PART_RIGHT_HAND;
            case "left_head": return LED.PART_LEFT_HEAD;
            case "right_head": return LED.PART_RIGHT_HEAD;
            default: return LED.PART_ALL_HEAD;
        }
    }

    private byte getLedMode(String mode) {
        switch (mode.toLowerCase()) {
            case "white": return LED.MODE_WHITE;
            case "red": return LED.MODE_RED;
            case "green": return LED.MODE_GREEN;
            case "pink": return LED.MODE_PINK;
            case "purple": return LED.MODE_PURPLE;
            case "blue": return LED.MODE_BLUE;
            case "yellow": return LED.MODE_YELLOW;
            case "flicker_white": return LED.MODE_FLICKER_WHITE;
            case "flicker_red": return LED.MODE_FLICKER_RED;
            case "flicker_green": return LED.MODE_FLICKER_GREEN;
            case "flicker_pink": return LED.MODE_FLICKER_PINK;
            case "flicker_purple": return LED.MODE_FLICKER_PURPLE;
            case "flicker_blue": return LED.MODE_FLICKER_BLUE;
            case "flicker_yellow": return LED.MODE_FLICKER_YELLOW;
            case "flicker_random": return LED.MODE_FLICKER_RANDOM;
            default: return LED.MODE_CLOSE;
        }
    }

    private void handleSpeakAndroid(String texto, int speed, int volume) {
        if (tts != null) {
            tts.stop();
            // Android TTS rate ~0.5–2.0; map MQTT speed 0–100 around 1.0
            float rate = 0.5f + (Math.max(0, Math.min(100, speed)) / 100f);
            tts.setSpeechRate(rate);
            // STREAM_MUSIC volume 0–1
            float gain = Math.max(0, Math.min(100, volume)) / 100f;
            android.os.Bundle params = new android.os.Bundle();
            params.putFloat(TextToSpeech.Engine.KEY_PARAM_VOLUME, gain);
            tts.speak(texto, TextToSpeech.QUEUE_FLUSH, params, "mqtt_speak");
        }
    }

    private void handleSpeakSDK(String texto, int speed, int volume) {
        if (speechManager != null) {
            SpeakOption speakOption = new SpeakOption();

            // Obtém o idioma selecionado no spinner
            String selectedLanguage = spinnerLanguage.getSelectedItem().toString();
            speakOption.setLanguageType(getSdkLanguageType(selectedLanguage));

            speakOption.setSpeed(Math.max(0, Math.min(100, speed)));
            speakOption.setIntonation(50);
            // volume is applied for Android TTS; SDK SpeakOption keeps speed/intonation here
            speechManager.stopSpeak();
            speechManager.startSpeak(texto, speakOption);
        }
    }

    @Override
    public void onTopicsUpdated(java.util.List<String> topics) {}

    private void subscribeToAllTopics() {
        // Commands only — do not subscribe to outbound telemetry (ir/imu/battery/…).
        // Subscribing + Show Logs caused unbounded TextView growth and ANRs.
        String[] suffixes = new String[]{
                "light", "move", "joints", "led", "speak", "emotion", "autonomy",
                "camera", "system", "proxy", "opensdk"
        };
        java.util.List<String> spinnerItems = new java.util.ArrayList<>();
        spinnerItems.add("All");

        for (String suffix : suffixes) {
            String topic = getPrefixedTopic(suffix);
            mqttHandler.subscribeToTopic(topic);
            spinnerItems.add(topic);
        }

        ArrayAdapter<String> adapter = new ArrayAdapter<>(this, android.R.layout.simple_spinner_item, spinnerItems);
        adapter.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item);
        spinnerTopics.setAdapter(adapter);
    }

    private boolean isTelemetryTopic(String topic) {
        if (topic == null) return false;
        return topic.equals(getPrefixedTopic("ir"))
                || topic.equals(getPrefixedTopic("imu"))
                || topic.equals(getPrefixedTopic("battery"))
                || topic.equals(getPrefixedTopic("info"))
                || topic.equals(getPrefixedTopic("touch"))
                || topic.equals(getPrefixedTopic("pir"))
                || topic.equals(getPrefixedTopic("voice_angle"))
                || topic.equals(getPrefixedTopic("obstacle"))
                || topic.equals(getPrefixedTopic("speech"));
    }

    private boolean isUiLogEnabled() {
        return checkBoxShowLogs != null && checkBoxShowLogs.isChecked();
    }

    private void appendMessage(String msg) {
        if (textViewMessages == null || msg == null || !isUiLogEnabled()) {
            return;
        }
        runOnUiThread(() -> {
            if (!isUiLogEnabled() || textViewMessages == null) {
                return;
            }
            textViewMessages.append(msg + "\n");
            CharSequence text = textViewMessages.getText();
            if (text != null && text.length() > MAX_LOG_CHARS) {
                textViewMessages.setText(text.subSequence(text.length() - MAX_LOG_CHARS, text.length()));
            }
            if (scrollView != null) {
                scrollView.post(() -> scrollView.fullScroll(View.FOCUS_DOWN));
            }
        });
    }

    /** MQTT topic spam — same gate as status lines (Show UI log). */
    private void appendVerboseMessage(String msg) {
        appendMessage(msg);
    }

    private void setShowUiLogFromRemote(boolean on) {
        runOnUiThread(() -> {
            if (checkBoxShowLogs == null) {
                return;
            }
            if (checkBoxShowLogs.isChecked() == on) {
                return;
            }
            checkBoxShowLogs.setChecked(on);
        });
    }

    private void setPreviewHintVisible(boolean show) {
        if (textViewPreviewHint != null) {
            textViewPreviewHint.setVisibility(show ? View.VISIBLE : View.GONE);
        }
    }

    @Override
    protected void onNewIntent(android.content.Intent intent) {
        super.onNewIntent(intent);
        setIntent(intent);
        consumeMinimizeToHome(intent);
    }

    @Override
    protected void onResume() {
        super.onResume();
        if (consumeMinimizeToHome(getIntent())) {
            return;
        }
        SoftInputHider.hideActivity(this);
        scheduleAutoReturnHome();
    }

    @Override
    protected void onPause() {
        // Keep auto-return armed while covered — otherwise we never minimize
        // after Proxy/face takes the foreground mid-countdown.
        super.onPause();
        if (rtmpStream != null) {
            if (rtmpStream.isStreaming()) {
                rtmpStream.stopStream();
            }
            if (rtmpStream.isOnPreview()) {
                rtmpStream.stopPreview();
            }
            if (switchRtmp != null) switchRtmp.setChecked(false);
        }
        if (tts != null) {
            tts.stop();
            tts.shutdown();
        }
    }

    @Override
    public boolean dispatchTouchEvent(MotionEvent ev) {
        if (ev != null && ev.getAction() == MotionEvent.ACTION_DOWN) {
            scheduleAutoReturnHome();
        }
        return super.dispatchTouchEvent(ev);
    }

    private void persistAutoReturnHomePrefs() {
        getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                .edit()
                .putBoolean(KEY_AUTO_RETURN_HOME, isAutoReturnHomeEnabled())
                .putInt(KEY_AUTO_RETURN_SECONDS, readAutoReturnSeconds())
                .apply();
    }

    private boolean isAutoReturnHomeEnabled() {
        return checkBoxAutoReturnHome != null && checkBoxAutoReturnHome.isChecked();
    }

    private int readAutoReturnSeconds() {
        if (editTextAutoReturnSeconds == null || editTextAutoReturnSeconds.getText() == null) {
            return DEFAULT_AUTO_RETURN_SECONDS;
        }
        try {
            return clampAutoReturnSeconds(
                    Integer.parseInt(editTextAutoReturnSeconds.getText().toString().trim()));
        } catch (Exception e) {
            return DEFAULT_AUTO_RETURN_SECONDS;
        }
    }

    private static int clampAutoReturnSeconds(int seconds) {
        if (seconds < MIN_AUTO_RETURN_SECONDS) {
            return MIN_AUTO_RETURN_SECONDS;
        }
        if (seconds > MAX_AUTO_RETURN_SECONDS) {
            return MAX_AUTO_RETURN_SECONDS;
        }
        return seconds;
    }

    private boolean isRtmpEnabled() {
        return switchRtmp != null && switchRtmp.isChecked();
    }

    private void bindAutoReturnHomeListener() {
        if (checkBoxAutoReturnHome == null) {
            return;
        }
        checkBoxAutoReturnHome.setOnCheckedChangeListener((buttonView, isChecked) -> {
            if (isChecked && isRtmpEnabled()) {
                // Logical rule: RTMP On ⇒ Auto Return Home must stay Off.
                appendMessage("🏠 Auto-return blocked while RTMP On");
                buttonView.setChecked(false);
                return;
            }
            persistAutoReturnHomePrefs();
            scheduleAutoReturnHome();
        });
    }

    /**
     * RTMP On ⇒ Auto Return Home must be disabled (Bridge leaving fg kills publish).
     * Saved prefs are preserved; checkbox is forced Off + disabled while streaming.
     */
    private void applyAutoReturnHomeForRtmp(boolean rtmpOn) {
        cancelAutoReturnHome();
        if (checkBoxAutoReturnHome == null) {
            return;
        }
        if (rtmpOn) {
            checkBoxAutoReturnHome.setOnCheckedChangeListener(null);
            checkBoxAutoReturnHome.setChecked(false);
            bindAutoReturnHomeListener();
            checkBoxAutoReturnHome.setEnabled(false);
            if (editTextAutoReturnSeconds != null) {
                editTextAutoReturnSeconds.setEnabled(false);
            }
            appendMessage("🏠 Auto-return disabled while RTMP On");
        } else {
            checkBoxAutoReturnHome.setEnabled(true);
            if (editTextAutoReturnSeconds != null) {
                editTextAutoReturnSeconds.setEnabled(true);
            }
            boolean saved = getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                    .getBoolean(KEY_AUTO_RETURN_HOME, false);
            checkBoxAutoReturnHome.setOnCheckedChangeListener(null);
            checkBoxAutoReturnHome.setChecked(saved);
            bindAutoReturnHomeListener();
            scheduleAutoReturnHome();
        }
    }

    private void scheduleAutoReturnHome() {
        cancelAutoReturnHome();
        if (isRtmpEnabled()) {
            appendMessage("🏠 Auto-return disabled while RTMP On");
            return;
        }
        if (!isAutoReturnHomeEnabled()) {
            appendMessage("🏠 Auto-return off");
            return;
        }
        int seconds = readAutoReturnSeconds();
        publishHandler.postDelayed(autoReturnHomeRunnable, seconds * 1000L);
        appendMessage("🏠 Auto-return armed · " + seconds + "s");
    }

    private void cancelAutoReturnHome() {
        publishHandler.removeCallbacks(autoReturnHomeRunnable);
    }

    private void runAutoReturnHome() {
        if (isRtmpEnabled()) {
            appendMessage("🏠 Auto-return blocked — RTMP On (Bridge must stay foreground)");
            return;
        }
        if (!isAutoReturnHomeEnabled()) {
            return;
        }
        appendMessage("🏠 Auto Return to Home Screen (" + readAutoReturnSeconds() + "s)…");
        goHomeScreen();
    }

    private void checkCameraPermissions() {
        java.util.List<String> needed = new java.util.ArrayList<>();
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) != PackageManager.PERMISSION_GRANTED) {
            needed.add(Manifest.permission.CAMERA);
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) != PackageManager.PERMISSION_GRANTED) {
            needed.add(Manifest.permission.RECORD_AUDIO);
        }
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.WRITE_EXTERNAL_STORAGE) != PackageManager.PERMISSION_GRANTED) {
            needed.add(Manifest.permission.WRITE_EXTERNAL_STORAGE);
        }
        if (!needed.isEmpty()) {
            ActivityCompat.requestPermissions(this, needed.toArray(new String[0]), CAMERA_PERMISSION_REQUEST_CODE);
        } else {
            if (switchRtmp != null && switchRtmp.isChecked()) {
                initRtmpStream();
            }
        }
    }

    @Override
    public void onRequestPermissionsResult(int requestCode, @NonNull String[] permissions, @NonNull int[] grantResults) {
        super.onRequestPermissionsResult(requestCode, permissions, grantResults);
        if (requestCode == CAMERA_PERMISSION_REQUEST_CODE) {
            boolean cameraGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
            boolean audioGranted = ContextCompat.checkSelfPermission(this, Manifest.permission.RECORD_AUDIO) == PackageManager.PERMISSION_GRANTED;
            if (cameraGranted && audioGranted) {
                appendMessage("✅ Camera and audio permissions granted.");
                if (switchRtmp != null && switchRtmp.isChecked()) {
                    initRtmpStream();
                }
            } else {
                appendMessage("❌ Required permissions denied. RTMP streaming will not work.");
            }
        }
    }

    @Override
    public void onBackPressed() {
        // Nano has no nav bar; Close / Back dismisses the UI without killing MQTT.
        exitBridgeUi();
    }

    /**
     * Leave the config screen (boot / auto-open). Keeps the process (and MQTT) alive
     * via {@link #moveTaskToBack(boolean)} instead of {@link #finish()}.
     */
    private void exitBridgeUi() {
        Log.i("SanbotRosBridge", "Close — move bridge UI to background");
        moveTaskToBack(true);
    }

    /**
     * Return to the stock face / home screen (OpenSDK {@code doHomeAction} + launcher Intent).
     * Close only backgrounds this app; Home brings the face launcher forward.
     */
    private void goHomeScreen() {
        cancelAutoReturnHome();
        SoftInputHider.hideActivity(this);
        appendMessage("🏠 Return to Home Screen…");
        Log.i("SanbotRosBridge", "Home — doHomeAction + face launcher + moveTaskToBack");
        // Stop open-on-connect from yanking Proxy/ROS back over the face.
        CompanionOpenGate.quietAfterHome();
        if (systemManager != null) {
            try {
                systemManager.doHomeAction();
            } catch (Exception e) {
                Log.w("SanbotRosBridge", "doHomeAction failed", e);
                appendMessage("doHomeAction: " + e.getMessage());
            }
        }
        try {
            android.content.Intent home = new android.content.Intent(android.content.Intent.ACTION_MAIN);
            home.addCategory(android.content.Intent.CATEGORY_HOME);
            home.setClassName("com.qihancloud.launcher", "com.qihancloud.launcher.MainActivity");
            home.addFlags(android.content.Intent.FLAG_ACTIVITY_NEW_TASK
                    | android.content.Intent.FLAG_ACTIVITY_RESET_TASK_IF_NEEDED
                    | android.content.Intent.FLAG_ACTIVITY_REORDER_TO_FRONT);
            startActivity(home);
        } catch (Exception e) {
            try {
                android.content.Intent home = new android.content.Intent(android.content.Intent.ACTION_MAIN);
                home.addCategory(android.content.Intent.CATEGORY_HOME);
                home.addFlags(android.content.Intent.FLAG_ACTIVITY_NEW_TASK);
                startActivity(home);
            } catch (Exception e2) {
                Log.e("SanbotRosBridge", "Home Intent failed", e2);
                appendMessage("Home failed: " + e2.getMessage());
            }
        }
        // Critical on Nano: launcher Intent alone often leaves this UI fullscreen.
        // Same path as Close — keep MQTT/WS alive in the background.
        try {
            boolean ok = moveTaskToBack(true);
            appendMessage("🏠 moveTaskToBack=" + ok);
            Log.i("SanbotRosBridge", "Home moveTaskToBack=" + ok);
        } catch (Exception e) {
            Log.e("SanbotRosBridge", "moveTaskToBack failed", e);
            exitBridgeUi();
        }
        // Proxy may still be under/over us — ask it to background too.
        MinimizeToHomeReceiver.notifyCompanions(this);
    }

    /**
     * @return true if this launch was a companion minimize request (UI already
     * backgrounded).
     */
    private boolean consumeMinimizeToHome(android.content.Intent intent) {
        if (intent == null || !intent.getBooleanExtra(EXTRA_MINIMIZE_TO_HOME, false)) {
            return false;
        }
        intent.removeExtra(EXTRA_MINIMIZE_TO_HOME);
        cancelAutoReturnHome();
        CompanionOpenGate.quietAfterHome();
        SoftInputHider.hideActivity(this);
        appendMessage("🏠 Companion minimize → moveTaskToBack");
        try {
            boolean ok = moveTaskToBack(true);
            appendMessage("🏠 moveTaskToBack=" + ok);
            Log.i("SanbotRosBridge", "Companion minimize moveTaskToBack=" + ok);
        } catch (Exception e) {
            Log.e("SanbotRosBridge", "Companion minimize failed", e);
            exitBridgeUi();
        }
        return true;
    }

    @Override
    protected void onDestroy() {
        stopControlWebSocket();
        stopOrbbecBody();
        if (orbbecBodyStreamer != null) {
            orbbecBodyStreamer.unregister();
            orbbecBodyStreamer = null;
        }
        mqttHandler.disconnect();
        publishHandler.removeCallbacksAndMessages(null);
        if (rtmpStream != null) {
            rtmpStream.release();
        }
        if (tts != null) {
            tts.stop();
            tts.shutdown();
        }

        super.onDestroy();
    }

    /** Delay so head live/face settles before second Pedro/RTMP publisher. */
    private static final long BODY_AFTER_HEAD_MS = 2500L;
    private final Runnable startBodyAfterHeadRunnable = this::startOrbbecBodyIfNeeded;

    private void startOrbbecBodyIfNeeded() {
        if (switchRtmp == null || !switchRtmp.isChecked()) return;
        String ip = editTextIp != null ? editTextIp.getText().toString().trim() : "";
        String broker = ip.isEmpty() ? "192.168.1.4" : ip;
        if (orbbecBodyStreamer != null) {
            orbbecBodyStreamer.setBrokerIp(sanitizeBrokerIp(broker));
            if (!orbbecBodyStreamer.isStreaming()) {
                appendMessage("📷 Starting Orbbec body+depth → live/body + live/depth…");
                orbbecBodyStreamer.start();
            }
        }
        // Forehead / USB are NOT auto-started here — Nano HW encoder + single-client
        // camera stack SIGSEGVs when Face + HDCamera/UVC encode at once. Start those
        // when My Sanbot selects source forehead/usb (handleCameraControl).
    }

    private void scheduleOrbbecBodyAfterHead() {
        rtmpHandler.removeCallbacks(startBodyAfterHeadRunnable);
        rtmpHandler.postDelayed(startBodyAfterHeadRunnable, BODY_AFTER_HEAD_MS);
    }

    private void stopOrbbecBody() {
        rtmpHandler.removeCallbacks(startBodyAfterHeadRunnable);
        if (orbbecBodyStreamer != null) {
            orbbecBodyStreamer.stop();
        }
        if (hdCameraForeheadStreamer != null) {
            hdCameraForeheadStreamer.stop();
        }
        if (uvcUsbStreamer != null) {
            uvcUsbStreamer.stop();
        }
    }

    /**
     * LAN host for MQTT + MediaMTX RTMP (Server IP field / last_broker_ip).
     * Exposed on /status so OpenSDK Proxy can build WHEP/HLS URLs without guessing Nano IP.
     */
    private String resolveBrokerIp() {
        String ip = sanitizeBrokerIp(editTextIp != null ? editTextIp.getText().toString() : "");
        if (ip.isEmpty()) {
            ip = sanitizeBrokerIp(getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                    .getString(KEY_LAST_IP, ""));
        }
        return ip;
    }

    private void putCameraStatusFields(JSONObject payload) throws Exception {
        boolean bodyOk = orbbecBodyStreamer != null && orbbecBodyStreamer.isHealthy();
        boolean depthOk = orbbecBodyStreamer != null && orbbecBodyStreamer.isDepthHealthy();
        boolean foreheadOk = hdCameraForeheadStreamer != null && hdCameraForeheadStreamer.isHealthy();
        boolean usbOk = uvcUsbStreamer != null && uvcUsbStreamer.isHealthy();
        payload.put("camera_source", cameraLivePath);
        payload.put("dual_camera", bodyOk);
        payload.put("body_camera_available", bodyOk);
        payload.put("depth_camera_available", depthOk);
        payload.put("forehead_camera_available", foreheadOk);
        payload.put("usb_camera_available", usbOk);
        if (hdCameraForeheadStreamer != null) {
            payload.put("hd_spike_ok", hdCameraForeheadStreamer.isSpikeOk());
            payload.put("hd_spike_distinct", hdCameraForeheadStreamer.isLikelyDistinct());
            payload.put("hd_spike_summary", hdCameraForeheadStreamer.getSpikeSummary());
        }
        StringBuilder note = new StringBuilder("Face=head HAL");
        if (foreheadOk) note.append("; Forehead=HDCamera");
        if (bodyOk) note.append("; Body=Orbbec color");
        if (depthOk) note.append("; Depth=Orbbec gray");
        if (usbOk) note.append("; USB=UVC");
        payload.put("camera_note", note.toString());
        payload.put("rtmp_on", switchRtmp != null && switchRtmp.isChecked());
        payload.put("show_logs", isUiLogEnabled());
        String broker = resolveBrokerIp();
        if (!broker.isEmpty()) {
            // Same host RTMP publish uses — Proxy / My Sanbot MediaMTX URLs.
            payload.put("broker_ip", broker);
            payload.put("mqtt_host", broker);
            payload.put("last_broker_ip", broker);
            payload.put("server_ip", broker);
            payload.put("mediamtx_host", broker);
        }
    }

    @Override
    public void onBodyLog(String message) {
        runOnUiThread(() -> appendMessage("📷 " + message));
    }

    @Override
    public void onBodyAvailabilityChanged(boolean available) {
        runOnUiThread(() -> {
            appendMessage(available
                    ? "✅ Orbbec body camera available (live/body)"
                    : "⚠️ Orbbec body camera unavailable");
            publishInfoSnapshotSoon();
        });
    }

    @Override
    public void onDepthAvailabilityChanged(boolean available) {
        runOnUiThread(() -> {
            appendMessage(available
                    ? "✅ Orbbec depth available (live/depth)"
                    : "⚠️ Orbbec depth unavailable");
            publishInfoSnapshotSoon();
        });
    }

    @Override
    public void onHdLog(String message) {
        runOnUiThread(() -> appendMessage("📷 " + message));
    }

    @Override
    public void onForeheadAvailabilityChanged(boolean available) {
        runOnUiThread(() -> {
            appendMessage(available
                    ? "✅ Forehead/HDCamera available (live/forehead)"
                    : "⚠️ Forehead/HDCamera unavailable");
            publishInfoSnapshotSoon();
        });
    }

    @Override
    public void onUsbCamLog(String message) {
        runOnUiThread(() -> appendMessage("📷 " + message));
    }

    @Override
    public void onUsbCameraAvailabilityChanged(boolean available) {
        runOnUiThread(() -> {
            appendMessage(available
                    ? "✅ USB UVC available (live/usb)"
                    : "⚠️ USB UVC unavailable");
            publishInfoSnapshotSoon();
        });
    }

    private void startControlWebSocket() {
        stopControlWebSocket();
        try {
            controlWsServer = new ControlWebSocketServer(WS_CONTROL_PORT, this);
            controlWsServer.start();
            appendMessage("🔌 WS control starting on :" + WS_CONTROL_PORT);
        } catch (Exception e) {
            appendMessage("❌ WS control failed: " + e.getMessage());
            Log.e("SanbotRosBridge", "WS start failed", e);
        }
    }

    private void stopControlWebSocket() {
        if (controlWsServer != null) {
            try {
                controlWsServer.stop();
            } catch (Exception e) {
                Log.e("SanbotRosBridge", "WS stop failed", e);
            }
            controlWsServer = null;
            appendMessage("🛑 WS control stopped");
        }
    }

    @Override
    public void onWsLog(String message) {
        runOnUiThread(() -> appendMessage(message));
    }

    @Override
    public void onWsClientConnected() {
        // My Sanbot (or proxy HTTP forward) connected to ros → open OpenSDK Proxy if enabled.
        OpenSdkProxyLauncher.maybeOpenOnClientConnect(
                getApplicationContext(),
                getSharedPreferences(PREFS_NAME, MODE_PRIVATE));
    }

    @Override
    public String onControlMessage(String path, JSONObject body) {
        final String p = path == null ? "" : path.trim();
        final JSONObject b = body != null ? body : new JSONObject();
        runOnUiThread(() -> {
            try {
                dispatchControlPath(p, b);
            } catch (Exception e) {
                appendMessage("WS dispatch error: " + e.getMessage());
            }
        });
        // Synchronous status replies
        if ("/status".equals(p) || "status".equals(p)) {
            return buildStatusReply();
        }
        if ("/camera/mjpeg".equals(p)) {
            return buildCameraHintReply();
        }
        if ("/proxy".equals(p) || "/opensdk".equals(p)) {
            return handleProxyCompanionRequest(b);
        }
        if ("/theme".equals(p) || "theme".equals(p) || "/face_theme".equals(p)
                || "face_theme".equals(p)) {
            return handleThemeRequest(b);
        }
        try {
            JSONObject ack = new JSONObject();
            ack.put("path", p);
            ack.put("ok", true);
            return ack.toString();
        } catch (Exception e) {
            return null;
        }
    }

    private void dispatchControlPath(String path, JSONObject body) throws Exception {
        String normalized = path.startsWith("/") ? path : "/" + path;
        appendMessage("WS " + normalized + " " + body.toString());
        switch (normalized) {
            case "/move":
                handleMotionControl(body.toString());
                break;
            case "/stop":
                handleMotionControl("{\"direction\":\"stop\"}");
                break;
            case "/led/white":
            case "/light":
                handleLightControl(body.toString());
                break;
            case "/led/color":
            case "/led":
                handleLedControl(body.toString());
                break;
            case "/tts":
            case "/speak": {
                String texto = body.optString("msg", body.optString("text", ""));
                if (!texto.isEmpty()) {
                    int speed = body.optInt("speed", 50);
                    int volume = body.optInt("volume", 70);
                    if (switchTtsType != null && switchTtsType.isChecked()) {
                        handleSpeakSDK(texto, speed, volume);
                    } else {
                        handleSpeakAndroid(texto, speed, volume);
                    }
                }
                break;
            }
            case "/head/absolute":
            case "/head/move":
            case "/joints":
                if (!body.has("joint")) {
                    // Controller-style absolute head may send pan/tilt fields
                    if (body.has("pan") || body.has("yaw")) {
                        JSONObject pan = new JSONObject();
                        pan.put("joint", "head_pan");
                        pan.put("angle", body.optInt("pan", body.optInt("yaw", 90)));
                        pan.put("speed", body.optInt("speed", 50));
                        handleJointMotion(pan.toString());
                    }
                    if (body.has("tilt")) {
                        JSONObject tilt = new JSONObject();
                        tilt.put("joint", "head_tilt");
                        tilt.put("angle", body.optInt("tilt", 20));
                        tilt.put("speed", body.optInt("speed", 50));
                        handleJointMotion(tilt.toString());
                    }
                    if (!body.has("pan") && !body.has("yaw") && !body.has("tilt")) {
                        handleJointMotion(body.toString());
                    }
                } else {
                    handleJointMotion(body.toString());
                }
                break;
            case "/head/center": {
                JSONObject pan = new JSONObject();
                pan.put("joint", "head_pan");
                pan.put("angle", 90);
                pan.put("speed", 50);
                handleJointMotion(pan.toString());
                JSONObject tilt = new JSONObject();
                tilt.put("joint", "head_tilt");
                tilt.put("angle", 20);
                tilt.put("speed", 50);
                handleJointMotion(tilt.toString());
                break;
            }
            case "/head/relative":
                // Map relative as absolute best-effort when joint provided
                handleJointMotion(body.toString());
                break;
            case "/arm/move":
                if (!body.has("joint")) {
                    body.put("joint", body.optString("arm", "wing_both"));
                }
                handleJointMotion(body.toString());
                break;
            case "/camera":
                handleCameraControl(body.toString());
                break;
            case "/camera/mjpeg":
            case "/status":
            case "/proxy":
            case "/opensdk":
                // replied synchronously in onControlMessage
                break;
            case "/system":
                handleSystemControl(body.toString());
                break;
            case "/emotion":
            case "/autonomy":
                appendMessage("WS " + normalized + " noted (bridge may need patched handlers)");
                break;
            case "/theme":
            case "/face_theme":
                // Handled synchronously in onControlMessage.
                break;
            default:
                appendMessage("WS unknown path: " + normalized);
                break;
        }
    }

    private String buildStatusReply() {
        try {
            JSONObject payload = new JSONObject();
            if (systemManager != null) {
                int status = systemManager.getBatteryStatus();
                String statusStr;
                switch (status) {
                    case 1:
                        statusStr = "not_charging";
                        break;
                    case 2:
                        statusStr = "charging_by_pile";
                        break;
                    default:
                        statusStr = "charging_by_wire";
                        break;
                }
                payload.put("battery_level", systemManager.getBatteryValue());
                payload.put("battery_status", statusStr);
                payload.put("device_id", systemManager.getDeviceId());
            }
            putCameraStatusFields(payload);
            payload.put("mqtt_connected", connected);
            putVolumeTelemetry(payload);
            JSONObject wrap = new JSONObject();
            wrap.put("path", "/status");
            wrap.put("payload", payload);
            // Also expose as /battery-like for My Sanbot telemetry handler
            wrap.put("topic", "/battery");
            return wrap.toString();
        } catch (Exception e) {
            return "{\"path\":\"/status\",\"error\":\"" + e.getMessage() + "\"}";
        }
    }

    private String buildCameraHintReply() {
        try {
            String ip = editTextIp != null ? editTextIp.getText().toString().trim() : "";
            JSONObject payload = new JSONObject();
            payload.put("rtmp", "rtmp://" + ip + ":1935/live/" + cameraLivePath);
            payload.put("hls", "http://" + ip + ":8888/live/" + cameraLivePath + "/index.m3u8");
            payload.put("note", "MJPEG not streamed; use HLS/RTMP");
            JSONObject wrap = new JSONObject();
            wrap.put("path", "/camera/mjpeg");
            wrap.put("payload", payload);
            return wrap.toString();
        } catch (Exception e) {
            return null;
        }
    }

    /** Remote companion control: start / status / prefs for OpenSDK Proxy. */
    private String handleProxyCompanionRequest(JSONObject body) {
        try {
            SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
            String action = body.optString("action", body.optString("cmd", "status"))
                    .trim().toLowerCase(Locale.US);
            if ("auto".equals(action) || "set_auto".equals(action)) {
                boolean enabled = body.optBoolean("enabled", body.optBoolean("auto", false));
                prefs.edit().putBoolean(OpenSdkProxyLauncher.KEY_AUTO_START_PROXY, enabled).apply();
                runOnUiThread(() -> {
                    if (switchAutoStartProxy != null) {
                        switchAutoStartProxy.setChecked(enabled);
                    }
                });
            }
            if ("open_on_connect".equals(action) || "set_open_on_connect".equals(action)) {
                boolean enabled = body.optBoolean("enabled", true);
                prefs.edit().putBoolean(OpenSdkProxyLauncher.KEY_OPEN_PROXY_ON_CONNECT, enabled).apply();
                runOnUiThread(() -> {
                    if (switchOpenProxyOnConnect != null) {
                        switchOpenProxyOnConnect.setChecked(enabled);
                    }
                });
            }
            final boolean launched;
            if ("start".equals(action) || "launch".equals(action) || "open".equals(action)) {
                launched = OpenSdkProxyLauncher.launch(this);
                runOnUiThread(() -> appendMessage(launched
                        ? "🚀 OpenSDK Proxy launched (remote /proxy)"
                        : "❌ OpenSDK Proxy launch failed"));
            } else {
                launched = false;
            }
            JSONObject payload = OpenSdkProxyLauncher.status(this, prefs);
            payload.put("launched", launched);
            payload.put("ok", payload.optBoolean("installed", false));
            JSONObject wrap = new JSONObject();
            wrap.put("path", "/proxy");
            wrap.put("ok", true);
            wrap.put("payload", payload);
            return wrap.toString();
        } catch (Exception e) {
            return "{\"path\":\"/proxy\",\"ok\":false,\"error\":\"" + e.getMessage() + "\"}";
        }
    }

    private void startBatteryPublishingLoop() {
        publishHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (connected && mqttHandler != null && systemManager != null) {
                    try {
                        int batteryLevel = systemManager.getBatteryValue();
                        int status = systemManager.getBatteryStatus();
                        String statusStr;
                        switch (status) {
                            case 1:
                                statusStr = "not_charging";
                                break;
                            case 2:
                                statusStr = "charging_by_pile";
                                break;
                            default:
                                statusStr = "charging_by_wire";
                        }

                        JSONObject json = new JSONObject();
                        json.put("battery_level", batteryLevel);
                        json.put("battery_status", statusStr);
                        putCameraStatusFields(json);

                        publishTelemetry("battery", json);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
                publishHandler.postDelayed(this, 5000);
            }
        }, 5000);
    }

    private void startInfoPublishingLoop() {
        publishHandler.postDelayed(new Runnable() {
            @Override
            public void run() {
                if (connected && mqttHandler != null && systemManager != null) {
                    try {
                        JSONObject json = new JSONObject();
                        json.put("robot_id", systemManager.getDeviceId());
                        json.put("ip", getLocalIpAddress());
                        json.put("main_service_version", systemManager.getMainServiceVersion());
                        json.put("android_version", android.os.Build.VERSION.RELEASE);
                        json.put("device_model", android.os.Build.MODEL);
                        putVolumeTelemetry(json);
                        putCameraStatusFields(json);

                        publishTelemetry("info", json);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

                publishHandler.postDelayed(this, 5000); // repete a cada 5 segundos
            }
        }, 5000);
    }

    /**
     * MQTT/WS /system:
     * {"action":"reboot"}
     * {"action":"home"} / homescreen / face / screensaver
     * {"action":"mute"} / {"action":"unmute"}
     * {"action":"volume","music_pct":80,"system_pct":50,"speak":70,"mute":false}
     * Absolute stream steps: "music":0..max, "system":0..max (optional with pct).
     */
    private void handleSystemControl(String message) {
        try {
            JSONObject json = new JSONObject(message);
            String action = json.optString("action", json.optString("cmd", "")).trim().toLowerCase(Locale.US);
            if ("reboot".equals(action) || "restart".equals(action)) {
                appendMessage("MQTT: rebooting robot…");
                new Thread(() -> {
                    try {
                        Thread.sleep(800L);
                    } catch (InterruptedException ignored) {
                        Thread.currentThread().interrupt();
                    }
                    requestDeviceReboot();
                }, "mqtt-reboot").start();
                return;
            }
            if ("home".equals(action) || "homescreen".equals(action) || "home_screen".equals(action)
                    || "face".equals(action) || "screensaver".equals(action)
                    || "screen_saver".equals(action) || "go_home".equals(action)) {
                SoftInputHider.hideActivity(this);
                goHomeScreen();
                return;
            }
            if ("hide_keypad".equals(action) || "hide_keyboard".equals(action)
                    || "keypad".equals(action) || "keyboard".equals(action)
                    || "dismiss_keyboard".equals(action)) {
                SoftInputHider.hideActivity(this);
                appendMessage("⌨️ Keypad hidden");
                return;
            }
            if ("mute".equals(action)) {
                applyMasterMute(true);
                appendMessage("🔇 Master mute on");
                publishInfoSnapshotSoon();
                return;
            }
            if ("unmute".equals(action)) {
                applyMasterMute(false);
                appendMessage("🔊 Master mute off");
                publishInfoSnapshotSoon();
                return;
            }
            if ("start_proxy".equals(action) || "start_opensdk".equals(action)
                    || "launch_proxy".equals(action)) {
                boolean ok = OpenSdkProxyLauncher.launch(this);
                appendMessage(ok ? "🚀 OpenSDK Proxy launched" : "❌ OpenSDK Proxy launch failed");
                return;
            }
            if ("show_logs".equals(action) || "ui_log".equals(action) || "logging".equals(action)) {
                boolean on = json.optBoolean("enabled", json.optBoolean("show_logs", false));
                setShowUiLogFromRemote(on);
                return;
            }
            if (json.has("show_logs") || json.has("ui_log") || json.has("show_ui_log")) {
                boolean on = json.has("show_logs")
                        ? json.optBoolean("show_logs", false)
                        : json.has("ui_log")
                        ? json.optBoolean("ui_log", false)
                        : json.optBoolean("show_ui_log", false);
                setShowUiLogFromRemote(on);
                return;
            }
            if ("volume".equals(action) || json.has("music") || json.has("system")
                    || json.has("music_pct") || json.has("system_pct") || json.has("speak")
                    || json.has("mute")) {
                applyVolumeControl(json);
                appendMessage("🔊 Volume updated");
                publishInfoSnapshotSoon();
                return;
            }
            appendMessage("Unknown /system action: " + action);
        } catch (Exception e) {
            e.printStackTrace();
            appendMessage("/system error: " + e.getMessage());
        }
    }

    private void applyVolumeControl(JSONObject json) {
        android.media.AudioManager am =
                (android.media.AudioManager) getSystemService(AUDIO_SERVICE);
        if (am != null) {
            if (json.has("music_pct")) {
                setStreamPercent(am, android.media.AudioManager.STREAM_MUSIC, json.optInt("music_pct", -1));
            } else if (json.has("music")) {
                setStreamAbsolute(am, android.media.AudioManager.STREAM_MUSIC, json.optInt("music", -1));
            }
            if (json.has("system_pct")) {
                setStreamPercent(am, android.media.AudioManager.STREAM_SYSTEM, json.optInt("system_pct", -1));
            } else if (json.has("system")) {
                setStreamAbsolute(am, android.media.AudioManager.STREAM_SYSTEM, json.optInt("system", -1));
            }
        }
        if (json.has("speak")) {
            int speak = Math.max(0, Math.min(100, json.optInt("speak", -1)));
            if (speak >= 0) {
                writeQihanSettingInt("robot_speak_volume", speak);
            }
        }
        if (json.has("mute")) {
            applyMasterMute(json.optBoolean("mute", false));
        }
    }

    private void setStreamPercent(android.media.AudioManager am, int stream, int pct) {
        if (pct < 0) return;
        int max = Math.max(1, am.getStreamMaxVolume(stream));
        int level = (Math.max(0, Math.min(100, pct)) * max) / 100;
        am.setStreamVolume(stream, level, 0);
    }

    private void setStreamAbsolute(android.media.AudioManager am, int stream, int level) {
        if (level < 0) return;
        int max = Math.max(0, am.getStreamMaxVolume(stream));
        am.setStreamVolume(stream, Math.max(0, Math.min(max, level)), 0);
    }

    private void applyMasterMute(boolean mute) {
        android.media.AudioManager am =
                (android.media.AudioManager) getSystemService(AUDIO_SERVICE);
        if (am != null) {
            // Hidden AudioManager.setMasterMute (works on Nano when Settings write is denied).
            try {
                java.lang.reflect.Method m = am.getClass()
                        .getMethod("setMasterMute", boolean.class, int.class);
                m.invoke(am, mute, 0);
            } catch (Exception e) {
                Log.w("SanbotRosBridge", "setMasterMute reflect failed", e);
            }
            try {
                int dir = mute
                        ? android.media.AudioManager.ADJUST_MUTE
                        : android.media.AudioManager.ADJUST_UNMUTE;
                am.adjustStreamVolume(android.media.AudioManager.STREAM_MUSIC, dir, 0);
                am.adjustStreamVolume(android.media.AudioManager.STREAM_SYSTEM, dir, 0);
                if (!mute) {
                    if (am.getStreamVolume(android.media.AudioManager.STREAM_MUSIC) <= 0) {
                        int max = Math.max(1, am.getStreamMaxVolume(android.media.AudioManager.STREAM_MUSIC));
                        am.setStreamVolume(android.media.AudioManager.STREAM_MUSIC, Math.max(1, max / 2), 0);
                    }
                    if (am.getStreamVolume(android.media.AudioManager.STREAM_SYSTEM) <= 0) {
                        int max = Math.max(1, am.getStreamMaxVolume(android.media.AudioManager.STREAM_SYSTEM));
                        am.setStreamVolume(android.media.AudioManager.STREAM_SYSTEM, Math.max(1, max / 2), 0);
                    }
                }
            } catch (Exception e) {
                Log.w("SanbotRosBridge", "adjustStreamVolume mute failed", e);
            }
        }
        lastAppliedMasterMute = mute;
        try {
            if (android.provider.Settings.System.canWrite(this)) {
                android.provider.Settings.System.putInt(
                        getContentResolver(), "volume_master_mute", mute ? 1 : 0);
            } else {
                appendMessage("⚠ Enable Modify system settings for Sanbot-ros to clear MUTED flag");
                try {
                    android.content.Intent intent = new android.content.Intent(
                            android.provider.Settings.ACTION_MANAGE_WRITE_SETTINGS);
                    intent.setData(android.net.Uri.parse("package:" + getPackageName()));
                    intent.addFlags(android.content.Intent.FLAG_ACTIVITY_NEW_TASK);
                    startActivity(intent);
                } catch (Exception ignored) {
                }
            }
        } catch (Exception e) {
            Log.w("SanbotRosBridge", "volume_master_mute write failed", e);
        }
    }

    private void requestDeviceReboot() {
        try {
            android.os.PowerManager pm =
                    (android.os.PowerManager) getSystemService(POWER_SERVICE);
            if (pm != null) {
                pm.reboot(null);
                return;
            }
        } catch (Throwable t) {
            Log.w("SanbotRosBridge", "PowerManager.reboot unavailable", t);
        }
        try {
            Runtime.getRuntime().exec(new String[]{"/system/bin/reboot"});
        } catch (Exception e) {
            Log.e("SanbotRosBridge", "/system/bin/reboot failed", e);
            try {
                Runtime.getRuntime().exec(new String[]{"reboot"});
            } catch (Exception e2) {
                Log.e("SanbotRosBridge", "reboot failed", e2);
            }
        }
    }

    private void publishInfoSnapshotSoon() {
        publishHandler.post(() -> {
            if (systemManager == null) return;
            try {
                JSONObject json = new JSONObject();
                json.put("robot_id", systemManager.getDeviceId());
                json.put("ip", getLocalIpAddress());
                json.put("main_service_version", systemManager.getMainServiceVersion());
                json.put("android_version", android.os.Build.VERSION.RELEASE);
                json.put("device_model", android.os.Build.MODEL);
                putVolumeTelemetry(json);
                putCameraStatusFields(json);
                publishTelemetry("info", json);
            } catch (Exception e) {
                e.printStackTrace();
            }
        });
    }

    /** Android media/system levels + Qihan speak volume for My Sanbot Robot Info. */
    private void putVolumeTelemetry(JSONObject json) {
        try {
            android.media.AudioManager am =
                    (android.media.AudioManager) getSystemService(AUDIO_SERVICE);
            if (am != null) {
                int musicMax = Math.max(1, am.getStreamMaxVolume(android.media.AudioManager.STREAM_MUSIC));
                int music = am.getStreamVolume(android.media.AudioManager.STREAM_MUSIC);
                int systemMax = Math.max(1, am.getStreamMaxVolume(android.media.AudioManager.STREAM_SYSTEM));
                int system = am.getStreamVolume(android.media.AudioManager.STREAM_SYSTEM);
                json.put("volume_music", music);
                json.put("volume_music_max", musicMax);
                json.put("volume_music_pct", (music * 100) / musicMax);
                json.put("volume_system", system);
                json.put("volume_system_max", systemMax);
                json.put("volume_system_pct", (system * 100) / systemMax);
            }
            boolean muted;
            if (lastAppliedMasterMute != null) {
                muted = lastAppliedMasterMute;
            } else {
                Boolean reflected = null;
                if (am != null) {
                    try {
                        java.lang.reflect.Method m = am.getClass().getMethod("isMasterMute");
                        Object r = m.invoke(am);
                        if (r instanceof Boolean) {
                            reflected = (Boolean) r;
                        }
                    } catch (Exception ignored) {
                    }
                }
                if (reflected != null) {
                    muted = reflected;
                } else {
                    int mute = android.provider.Settings.System.getInt(
                            getContentResolver(), "volume_master_mute", 0);
                    muted = mute != 0;
                }
            }
            json.put("master_mute", muted);
            Integer speakVol = readQihanSettingInt("robot_speak_volume");
            if (speakVol != null) {
                json.put("robot_speak_volume", speakVol);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private Integer readQihanSettingInt(String key) {
        android.database.Cursor cursor = null;
        try {
            android.net.Uri uri = android.net.Uri.parse(
                    "content://com.qihancloud.setting.Provider/setting");
            cursor = getContentResolver().query(
                    uri, new String[]{"key", "value"}, "key=?", new String[]{key}, null);
            if (cursor != null && cursor.moveToFirst()) {
                int idx = cursor.getColumnIndex("value");
                if (idx >= 0) {
                    String raw = cursor.getString(idx);
                    if (raw != null && !raw.isEmpty()) {
                        return Integer.parseInt(raw.trim());
                    }
                }
            }
        } catch (Exception ignored) {
        } finally {
            if (cursor != null) {
                cursor.close();
            }
        }
        return null;
    }

    private String handleThemeRequest(JSONObject body) {
        final JSONObject b = body != null ? body : new JSONObject();
        try {
            String theme = firstNonEmptyTheme(
                    b.optString("theme", ""),
                    b.optString("face", ""),
                    b.optString("name", ""),
                    b.optString("id", ""),
                    b.optString("value", ""));
            String action = b.optString("action", "").trim().toLowerCase(Locale.US);
            if (!theme.isEmpty()
                    || "set".equals(action) || "apply".equals(action) || "select".equals(action)) {
                String normalized = normalizeFaceTheme(theme);
                if (normalized == null) {
                    return buildThemeReply(null, false, "Unknown theme (boy|cat|flower|girl|heart)");
                }
                boolean wrote = writeQihanSettingString("robot_face", normalized);
                appendMessage("🎭 theme → " + normalized);
                runOnUiThread(this::goHomeScreen);
                return buildThemeReply(normalized, wrote, wrote ? null : "Failed to write robot_face");
            }
            return buildThemeReply(null, true, null);
        } catch (Exception e) {
            try {
                JSONObject wrap = new JSONObject();
                wrap.put("path", "/theme");
                wrap.put("ok", false);
                wrap.put("error", e.getMessage());
                return wrap.toString();
            } catch (Exception ignored) {
                return "{\"path\":\"/theme\",\"ok\":false}";
            }
        }
    }

    private String buildThemeReply(String selected, boolean ok, String error) {
        try {
            JSONObject payload = new JSONObject();
            String current = readQihanSettingString("robot_face");
            if (selected != null && !selected.isEmpty()) {
                payload.put("theme", selected);
            } else if (current != null) {
                payload.put("theme", current);
            }
            if (current != null) {
                payload.put("current", current);
            }
            org.json.JSONArray themes = new org.json.JSONArray();
            String[] known = {"boy", "cat", "flower", "girl", "heart"};
            for (String t : known) {
                themes.put(t);
            }
            payload.put("themes", themes);
            JSONObject wrap = new JSONObject();
            wrap.put("path", "/theme");
            wrap.put("ok", ok);
            if (error != null) {
                wrap.put("error", error);
            }
            wrap.put("payload", payload);
            return wrap.toString();
        } catch (Exception e) {
            return "{\"path\":\"/theme\",\"ok\":false}";
        }
    }

    private static String normalizeFaceTheme(String raw) {
        if (raw == null) {
            return null;
        }
        String key = raw.trim().toLowerCase(Locale.US).replace('-', '_').replace(' ', '_');
        if (key.isEmpty()) {
            return null;
        }
        if ("default".equals(key) || "normal".equals(key)) {
            key = "cat";
        }
        String[] known = {"boy", "cat", "flower", "girl", "heart"};
        for (String k : known) {
            if (k.equals(key)) {
                return k;
            }
        }
        java.io.File dir = new java.io.File("/sdcard/LAUNCHER_SRC/FACE_IMAGE", key);
        return dir.isDirectory() ? key : null;
    }

    private static String firstNonEmptyTheme(String... values) {
        if (values == null) {
            return "";
        }
        for (String v : values) {
            if (v != null && !v.trim().isEmpty()) {
                return v.trim();
            }
        }
        return "";
    }

    private void writeQihanSettingInt(String key, int value) {
        writeQihanSettingString(key, String.valueOf(value));
    }

    private boolean writeQihanSettingString(String key, String value) {
        try {
            android.net.Uri uri = android.net.Uri.parse(
                    "content://com.qihancloud.setting.Provider/setting");
            android.content.ContentValues values = new android.content.ContentValues();
            values.put("key", key);
            values.put("value", value);
            int updated = getContentResolver().update(uri, values, "key=?", new String[]{key});
            if (updated <= 0) {
                return getContentResolver().insert(uri, values) != null;
            }
            return true;
        } catch (Exception e) {
            Log.w("SanbotRosBridge", "writeQihanSettingString " + key + " failed", e);
            return false;
        }
    }

    private String readQihanSettingString(String key) {
        android.database.Cursor cursor = null;
        try {
            android.net.Uri uri = android.net.Uri.parse(
                    "content://com.qihancloud.setting.Provider/setting");
            cursor = getContentResolver().query(
                    uri, new String[]{"key", "value"}, "key=?", new String[]{key}, null);
            if (cursor != null && cursor.moveToFirst()) {
                int idx = cursor.getColumnIndex("value");
                if (idx >= 0) {
                    String raw = cursor.getString(idx);
                    if (raw != null && !raw.isEmpty()) {
                        return raw.trim();
                    }
                }
            }
        } catch (Exception ignored) {
        } finally {
            if (cursor != null) {
                cursor.close();
            }
        }
        return null;
    }

    private void publishSpeechToMqtt(String texto) {
        if (texto != null && !texto.isEmpty()) {
            try {
                JSONObject json = new JSONObject();
                json.put("msg", texto);
                publishTelemetry("speech", json);
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    /**
     * Publish sensor / status telemetry to MQTT (when connected) and mirror to Direct WS clients.
     * My Sanbot Radar uses /ir; Direct WS previously only saw /status → /battery.
     */
    private void publishTelemetry(String suffix, JSONObject payload) {
        if (suffix == null || payload == null) {
            return;
        }
        String topic = getPrefixedTopic(suffix);
        String body = payload.toString();
        if (mqttHandler != null) {
            mqttHandler.publishMessage(topic, body);
        }
        ControlWebSocketServer ws = controlWsServer;
        if (ws != null) {
            try {
                JSONObject wrap = new JSONObject();
                wrap.put("path", "/" + suffix);
                wrap.put("topic", topic);
                wrap.put("payload", payload);
                ws.broadcastJson(wrap.toString());
            } catch (Exception e) {
                Log.w("SanbotRosBridge", "WS telemetry broadcast failed: " + suffix, e);
            }
        }
    }

    private void initRtmpStream() {
        if (switchRtmp == null || !switchRtmp.isChecked()) {
            return;
        }
        if (textureViewPreview != null && textureViewPreview.getVisibility() != View.VISIBLE) {
            textureViewPreview.setVisibility(View.VISIBLE);
        }
        rtmpPublishAttempts = 0;
        releaseFaceHalCamera();
        if (rtmpStream != null) {
            try {
                if (rtmpStream.isStreaming()) {
                    rtmpStream.stopStream();
                }
                if (rtmpStream.isOnPreview()) {
                    rtmpStream.stopPreview();
                }
                rtmpStream.release();
            } catch (Exception ignored) {
            }
            rtmpStream = null;
        }
        faceBufferSource = null;

        // Nano V4L2 HAL is single-client: Pedro Camera1/Camera2 constructors probe camera 1
        // and throw "Too many users" / CameraAccessException. Feed Camera 0 via BufferVideoSource.
        String ip = sanitizeBrokerIp(editTextIp != null ? editTextIp.getText().toString() : "");
        if (ip.isEmpty()) {
            // Last known good / LAN MediaMTX default — don't flip RTMP Off for a blank field.
            String saved = getSharedPreferences(PREFS_NAME, MODE_PRIVATE).getString(KEY_LAST_IP, "");
            ip = sanitizeBrokerIp(saved);
            if (ip.isEmpty()) {
                ip = "192.168.1.4";
            }
            appendMessage("📡 Server IP empty/corrupt — using " + ip);
            if (editTextIp != null) {
                editTextIp.setText(ip);
            }
            getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit().putString(KEY_LAST_IP, ip).apply();
        } else if (editTextIp != null && !ip.equals(editTextIp.getText().toString().trim())) {
            appendMessage("📡 Server IP cleaned → " + ip);
            editTextIp.setText(ip);
            getSharedPreferences(PREFS_NAME, MODE_PRIVATE).edit().putString(KEY_LAST_IP, ip).apply();
        }
        Log.i("RTMP", "init with broker IP " + ip);
        if (orbbecBodyStreamer != null) {
            orbbecBodyStreamer.setBrokerIp(ip);
        }
        if (hdCameraForeheadStreamer != null) {
            hdCameraForeheadStreamer.setBrokerIp(ip);
        }
        if (uvcUsbStreamer != null) {
            uvcUsbStreamer.setBrokerIp(ip);
        }

        try {
            int[] wh = openFaceHalCameraOnly();
            if (wh == null) {
                appendMessage("❌ RTMP: Camera.open(0) failed — turn HD spike Off / close other camera apps");
                failRtmpInit();
                return;
            }
            final int camW = wh[0];
            final int camH = wh[1];
            faceBufferSource = new BufferVideoSource(BufferVideoSource.Format.NV21, VIDEO_BITRATE);
            rtmpStream = new GenericStream(this, this, faceBufferSource, new MicrophoneSource());
            boolean prepared = rtmpStream.prepareVideo(camW, camH, VIDEO_BITRATE, 15)
                    && rtmpStream.prepareAudio(SAMPLE_RATE, IS_STEREO, AUDIO_BITRATE);
            Log.d("RTMP", "prepareVideo(NV21 " + camW + "x" + camH + ")=" + prepared);
            if (!prepared) {
                appendMessage("❌ Failed to prepare RTMP streaming.");
                failRtmpInit();
                return;
            }
            String rtmpUrl = "rtmp://" + ip + ":1935/live/face";
            rtmpStream.startStream(rtmpUrl);
            attachFaceHalPreviewCallback(camW, camH);
            appendMessage("📡 Head RTMP " + camW + "x" + camH + " → " + rtmpUrl);
            Log.d("RTMP", "startStream " + rtmpUrl);
        } catch (Exception e) {
            Log.e("RTMP", "initRtmpStream failed", e);
            appendMessage("❌ RTMP init failed: " + e.getMessage());
            failRtmpInit();
        }
    }

    private void failRtmpInit() {
        releaseFaceHalCamera();
        if (rtmpStream != null) {
            try {
                if (rtmpStream.isStreaming()) {
                    rtmpStream.stopStream();
                }
                rtmpStream.release();
            } catch (Exception ignored) {
            }
            rtmpStream = null;
        }
        faceBufferSource = null;
        if (switchRtmp != null && switchRtmp.isChecked()) {
            switchRtmp.setChecked(false);
        }
    }

    /** Open only HAL camera 0 — never probe camera 1 (Nano rejects second client). */
    private int[] openFaceHalCameraOnly() {
        synchronized (faceCameraLock) {
            releaseFaceHalCameraLocked();
            try {
                faceHalCamera = Camera.open(0);
                if (faceHalCamera == null) {
                    return null;
                }
                Camera.Parameters params = faceHalCamera.getParameters();
                Camera.Size best = null;
                for (Camera.Size s : params.getSupportedPreviewSizes()) {
                    if (s.width == STREAM_WIDTH && s.height == STREAM_HEIGHT) {
                        best = s;
                        break;
                    }
                    if (best == null
                            || Math.abs(s.width - STREAM_WIDTH) + Math.abs(s.height - STREAM_HEIGHT)
                            < Math.abs(best.width - STREAM_WIDTH) + Math.abs(best.height - STREAM_HEIGHT)) {
                        best = s;
                    }
                }
                if (best != null) {
                    params.setPreviewSize(best.width, best.height);
                    try {
                        params.setPreviewFormat(ImageFormat.NV21);
                    } catch (Exception ignored) {
                    }
                    faceHalCamera.setParameters(params);
                }
                // MUST use setPreviewTexture (not setPreviewDisplay). SurfaceView broke
                // H264 publish — MediaMTX live/face became AAC-only; My Sanbot / Console failed.
                // TextureView shares the same SurfaceTexture for on-screen preview + encode path.
                int w = best != null ? best.width : STREAM_WIDTH;
                int h = best != null ? best.height : STREAM_HEIGHT;
                SurfaceTexture st = null;
                if (textureViewPreview != null && textureViewPreview.isAvailable()) {
                    st = textureViewPreview.getSurfaceTexture();
                }
                if (st != null) {
                    st.setDefaultBufferSize(w, h);
                    faceHalCamera.setPreviewTexture(st);
                    Log.i("RTMP", "Face HAL previewTexture → TextureView " + w + "x" + h);
                } else {
                    faceHalCamera.setPreviewTexture(new SurfaceTexture(10));
                    Log.i("RTMP", "Face HAL previewTexture → SurfaceTexture(10) " + w + "x" + h);
                }
                return new int[]{w, h};
            } catch (Exception e) {
                Log.e("RTMP", "Face HAL open failed", e);
                releaseFaceHalCameraLocked();
                return null;
            }
        }
    }

    private void attachFaceHalPreviewCallback(int previewW, int previewH) {
        synchronized (faceCameraLock) {
            if (faceHalCamera == null) {
                return;
            }
            // Prefer non-buffer callback on Nano — WithBuffer SIGSEGVs in JNICameraContext::copyAndPost.
            faceHalCamera.setPreviewCallback((data, camera) -> {
                BufferVideoSource src = faceBufferSource;
                GenericStream stream = rtmpStream;
                if (data == null || src == null || stream == null || !stream.isStreaming()) {
                    return;
                }
                try {
                    // Copy — HAL may reuse the array before Pedro finishes reading.
                    src.setBuffer(java.util.Arrays.copyOf(data, data.length));
                } catch (Exception e) {
                    Log.w("RTMP", "face buffer push: " + e.getMessage());
                }
            });
            faceHalCamera.startPreview();
            Log.d("RTMP", "Face HAL preview callback " + previewW + "x" + previewH);
        }
    }

    private void releaseFaceHalCamera() {
        synchronized (faceCameraLock) {
            releaseFaceHalCameraLocked();
        }
    }

    private void releaseFaceHalCameraLocked() {
        if (faceHalCamera != null) {
            try {
                faceHalCamera.setPreviewCallback(null);
            } catch (Exception ignored) {
            }
            try {
                faceHalCamera.stopPreview();
            } catch (Exception ignored) {
            }
            try {
                faceHalCamera.release();
            } catch (Exception ignored) {
            }
            faceHalCamera = null;
        }
    }

    private final Runnable rtmpPublishRunnable = () -> {
        // Retries after connection failure: re-init whole face path.
        if (switchRtmp != null && switchRtmp.isChecked()) {
            initRtmpStream();
        }
    };

    private void scheduleRtmpPublish() {
        rtmpHandler.removeCallbacks(rtmpPublishRunnable);
        rtmpHandler.postDelayed(rtmpPublishRunnable, 1500);
    }

    @Override
    public void onConnectionStarted(String url) {
                    appendMessage("🔌 RTMP: Connection started for " + url);
    }

    @Override
    public void onConnectionSuccess() {
        appendMessage("✅ RTMP: Successfully connected (live/face).");
        if (switchRtmp != null && switchRtmp.isChecked()) {
            scheduleOrbbecBodyAfterHead();
        }
    }

    @Override
    public void onConnectionFailed(String reason) {
        appendMessage("❌ RTMP: Connection failed: " + reason);
        // Retry head a few times; do not start body until face is up.
        if (switchRtmp != null && switchRtmp.isChecked() && rtmpPublishAttempts < 5) {
            rtmpPublishAttempts++;
            rtmpHandler.postDelayed(rtmpPublishRunnable, 1500);
        }
    }

    @Override
    public void onNewBitrate(long bitrate) {
        // Opcional: pode-se exibir o bitrate se desejado
    }

    @Override
    public void onDisconnect() {
        appendMessage("🔌 RTMP: Disconnected.");
    }

    @Override
    public void onAuthError() {
        appendMessage("❌ RTMP: Authentication error.");
    }

    @Override
    public void onAuthSuccess() {
        appendMessage("✅ RTMP: Authentication successful.");
    }

    private void stopRtmpStream() {
        rtmpHandler.removeCallbacks(rtmpPublishRunnable);
        rtmpPublishAttempts = 0;
        releaseFaceHalCamera();
        if (rtmpStream != null) {
            try {
                if (rtmpStream.isStreaming()) {
                    rtmpStream.stopStream();
                }
            } catch (Exception ignored) {
            }
            try {
                if (rtmpStream.isOnPreview()) {
                    rtmpStream.stopPreview();
                }
            } catch (Exception ignored) {
            }
            try {
                rtmpStream.release();
            } catch (Exception ignored) {
            }
            rtmpStream = null;
            faceBufferSource = null;
            appendMessage("🛑 RTMP streaming stopped by user.");
        }
    }



    private Locale getLocaleForLanguage(String languageName) {
        switch (languageName) {
            case "English":
                return new Locale("en", "US");
            case "Portuguese":
                return new Locale("pt", "BR");
            case "Spanish":
                return new Locale("es", "ES");
            case "French":
                return new Locale("fr", "FR");
            case "German":
                return new Locale("de", "DE");
            case "Italian":
                return new Locale("it", "IT");
            case "Chinese":
                return new Locale("zh", "CN");
            case "Japanese":
                return new Locale("ja", "JP");
            case "Korean":
                return new Locale("ko", "KR");
            case "Russian":
                return new Locale("ru", "RU");
            default:
                return new Locale("en", "US"); // English as default
        }
    }

    private int getSdkLanguageType(String languageName) {
        switch (languageName) {
            case "English":
                return SpeakOption.LAG_ENGLISH_US;
            case "Chinese":
                return SpeakOption.LAG_CHINESE;
            default:
                return SpeakOption.LAG_ENGLISH_US; // English as default
        }
    }

    private String getPrefixedTopic(String suffix) {
        if (robotName != null && !robotName.isEmpty()) {
            return "/" + robotName + "/" + suffix;
        } else {
            return "/" + suffix;
        }
    }
}
