package com.grin.sanbotmqtt;

import android.content.Intent;
import android.content.SharedPreferences;
import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.os.Bundle;
import android.os.Handler;
import android.os.Looper;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.*;
import org.json.JSONObject;

import java.net.NetworkInterface;
import java.util.Collections;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Map;
import android.media.MediaPlayer;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.SpeakOption;
import com.sanbot.opensdk.function.beans.wheelmotion.NoAngleWheelMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.RelativeAngleWheelMotion;
import com.sanbot.opensdk.function.unit.BaseManager;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.SpeechManager;
import com.sanbot.opensdk.function.unit.SystemManager;
import com.sanbot.opensdk.function.unit.WheelMotionManager;
import com.sanbot.opensdk.function.beans.wheelmotion.DistanceWheelMotion;
import com.sanbot.opensdk.function.unit.HeadMotionManager;
import com.sanbot.opensdk.function.beans.headmotion.RelativeAngleHeadMotion;
import com.sanbot.opensdk.function.unit.interfaces.hardware.TouchSensorListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.PIRListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.InfrareListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.VoiceLocateListener;
import com.sanbot.opensdk.function.beans.LED;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.beans.OperationResult;
import com.sanbot.opensdk.function.beans.StreamOption;
import com.sanbot.opensdk.function.unit.interfaces.hardware.ObstacleListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.GyroscopeListener;

import android.graphics.Bitmap;
import android.graphics.ImageFormat;
import android.graphics.YuvImage;
import android.graphics.BitmapFactory;
import android.graphics.Rect;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.InetAddress;
import android.speech.tts.TextToSpeech;
import java.util.Locale;
import com.sanbot.opensdk.function.beans.speech.Grammar;
import com.sanbot.opensdk.function.beans.speech.RecognizeTextBean;
import com.sanbot.opensdk.function.unit.interfaces.speech.RecognizeListener;
import android.support.annotation.NonNull;


public class MainActivity extends TopBaseActivity implements MqttHandler.MqttStatusListener, TextToSpeech.OnInitListener {

    private EditText editTextIp;
    private Button buttonConnect;
    private Button buttonClear;
    private Spinner spinnerTopics;
    private TextView textViewMessages;
    private ScrollView scrollView;

    private MqttHandler mqttHandler;
    private boolean connected = false;
    private String selectedTopic = "Todos";

    private static final String PREFS_NAME = "mqtt_prefs";
    private static final String KEY_LAST_IP = "last_broker_ip";

    private SystemManager systemManager;
    private HardWareManager hardWareManager;
    private WheelMotionManager wheelMotionManager;
    private HeadMotionManager headMotionManager;
    private SpeechManager speechManager;


    private Handler publishHandler = new Handler();

    private final Handler motionHandler = new Handler(Looper.getMainLooper());

    private final Map<Integer, Long> lastIRUpdate = new HashMap<>();
    private static final long IR_UPDATE_INTERVAL_MS = 1000;  // 1 segundo por sensor

    private HDCameraManager hdCameraManager;
    private MjpegServer mjpegServer;
    private int videoHandle = -1;

    private TextToSpeech tts;

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

        editTextIp = findViewById(R.id.editTextIp);
        buttonConnect = findViewById(R.id.buttonConnect);
        buttonClear = findViewById(R.id.buttonClear);
        spinnerTopics = findViewById(R.id.spinnerTopics);
        textViewMessages = findViewById(R.id.textViewMessages);
        scrollView = findViewById(R.id.scrollView);

        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        String lastIp = prefs.getString(KEY_LAST_IP, null);
        if (lastIp != null) {
            editTextIp.setText(lastIp);
        }
        lastIp = editTextIp.getText().toString().trim();

        mqttHandler = new MqttHandler(this, this);

        mqttHandler.setBrokerIp(lastIp);

        mqttHandler.connect();
        buttonConnect.setEnabled(false);
        buttonConnect.setText("Conectando...");

        spinnerTopics.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                selectedTopic = parent.getItemAtPosition(position).toString();
                appendMessage("Filtro atualizado: mostrando mensagens de \"" + selectedTopic + "\"");
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {}
        });

        buttonConnect.setOnClickListener(v -> {
            String ip = editTextIp.getText().toString().trim();
            if (ip.isEmpty()) {
                Toast.makeText(this, "Informe o IP do broker", Toast.LENGTH_SHORT).show();
                return;
            }

            getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                    .edit()
                    .putString(KEY_LAST_IP, ip)
                    .apply();

            mqttHandler.setBrokerIp(ip);

            if (!connected) {
                mqttHandler.connect();
                buttonConnect.setEnabled(false);
                buttonConnect.setText("Conectando...");
            } else {
                mqttHandler.disconnect();
                connected = false;
                buttonConnect.setText("Conectar");
                appendMessage("Desconectado do broker.");
            }
        });

        buttonClear.setOnClickListener(v -> textViewMessages.setText(""));

        tts = new TextToSpeech(this, this);
    }

    @Override
    public void onInit(int status) {
        if (status == TextToSpeech.SUCCESS) {
            tts.setLanguage(new Locale("en", "US"));
        } else {
            Log.e("TTS", "Falha ao inicializar TTS");
        }
    }

    @Override
    protected void onMainServiceConnected() {
        systemManager = (SystemManager) getUnitManager(FuncConstant.SYSTEM_MANAGER);
        wheelMotionManager = (WheelMotionManager) getUnitManager(FuncConstant.WHEELMOTION_MANAGER);
        headMotionManager = (HeadMotionManager) getUnitManager(FuncConstant.HEADMOTION_MANAGER);
        speechManager = (SpeechManager) getUnitManager(FuncConstant.SPEECH_MANAGER);
        startBatteryPublishingLoop();
        startInfoPublishingLoop();

        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);

        hardWareManager.setOnHareWareListener(
                new TouchSensorListener() {
                    @Override
                    public void onTouch(int part) {
                        try {
                            JSONObject touchJson = new JSONObject();
                            touchJson.put("part", part);
                            touchJson.put("description", getTouchPartName(part));
                            mqttHandler.publishMessage("sanbot/touch", touchJson.toString());
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
                            mqttHandler.publishMessage("sanbot/pir", pirJson.toString());
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
                                mqttHandler.publishMessage("sanbot/ir", irJson.toString());
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
                    mqttHandler.publishMessage("sanbot/voice_angle", voiceJson.toString());
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
                    json.put("status", 1);
                    mqttHandler.publishMessage("sanbot/obstacle", json.toString());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        hardWareManager.setOnHareWareListener(new GyroscopeListener() {
            @Override
            public void gyroscopeData(float x, float y, float z) {
                try {
                    JSONObject gyroJson = new JSONObject();
                    gyroJson.put("x", x);
                    gyroJson.put("y", y);
                    gyroJson.put("z", z);
                    mqttHandler.publishMessage("sanbot/gyro", gyroJson.toString());
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }
        });

        // Inicializa o MJPEG Server
        mjpegServer = new MjpegServer(8080);
        try {
            mjpegServer.start();
            appendMessage("🌐 MJPEG disponível em: http://" + getLocalIpAddress() + ":8080");
        } catch (IOException e) {
            e.printStackTrace();
            appendMessage("❌ Erro ao iniciar o MJPEG server.");
        }

        // Inicia o stream de vídeo
        hdCameraManager = (HDCameraManager) getUnitManager(FuncConstant.HDCAMERA_MANAGER);

        StreamOption streamOption = new StreamOption();
        streamOption.setChannel(StreamOption.SUB_STREAM);
        streamOption.setDecodType(StreamOption.SOFTWARE_DECODE_NV21);  // Use NV21 para gerar JPEG
        streamOption.setJustIframe(false);

        hdCameraManager.setMediaListener(new com.sanbot.opensdk.function.unit.interfaces.media.MediaStreamListener() {
            private long lastFrameTime = 0;

            @Override
            public void getVideoStream(int handle, byte[] bytes, int width, int height) {
                long now = System.currentTimeMillis();
                if (now - lastFrameTime < 100) return; // Máximo de 20 fps
                lastFrameTime = now;

                Bitmap bitmap = nv21ToBitmap(bytes, width, height);
                if (bitmap != null && mjpegServer != null) {
                    mjpegServer.updateFrame(bitmap);
                }
            }
            @Override
            public void getAudioStream(int i, byte[] bytes) {
                // Ignorado
            }
        });

        OperationResult result = hdCameraManager.openStream(streamOption);
        if (result != null) {
            String rawResult = result.getResult();
            try {
                int handle = Integer.parseInt(rawResult);
                if (handle != -1) {
                    videoHandle = handle;
                    appendMessage("🎥 Stream de vídeo iniciado.");
                } else {
                    appendMessage("❌ Falha ao iniciar stream de vídeo (handle -1).");
                }
            } catch (NumberFormatException e) {
                e.printStackTrace();
                appendMessage("❌ Falha ao iniciar stream: retorno inválido: " + rawResult);
            }
        } else {
            appendMessage("❌ Falha ao iniciar stream: OperationResult é nulo.");
        }

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

    private Bitmap nv21ToBitmap(byte[] data, int width, int height) {
        try {
            YuvImage image = new YuvImage(data, ImageFormat.NV21, width, height, null);
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            image.compressToJpeg(new Rect(0, 0, width, height), 80, baos);
            byte[] jpegData = baos.toByteArray();
            return BitmapFactory.decodeByteArray(jpegData, 0, jpegData.length);
        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
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
            buttonConnect.setText(connected ? "Desconectar" : "Conectar");

            appendMessage(connected ? "Conectado com sucesso ao broker!" : "Falha na conexão ou desconectado.");

            if (connected) subscribeToAllTopics();
        });
    }

    @Override
    public void onMessageReceived(String topic, String message) {
        final String finalTopic = topic;
        final String finalMessage = message;

        runOnUiThread(() -> {
            if ("Todos".equals(selectedTopic) || selectedTopic.equals(finalTopic)) {
                appendMessage("[" + finalTopic + "]: " + finalMessage);
            }
        });

        if ("ros/light".equals(topic)) {
            handleLightControl(message);
        } else if ("ros/move".equals(topic)) {
            handleMotionControl(message);
        } else if ("ros/head".equals(topic)) {
            handleHeadMotion(message);
        } else if ("ros/led".equals(topic)) {
            handleLedControl(message);
        } else if ("ros/speak".equals(topic)) {
//            handleSpeakAndroid(message);
            handleSpeakSDK(message);
        }
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

            motionHandler.removeCallbacksAndMessages(null);
            if (duration > 0) {
                byte action = getActionFromDirection(direction);
                NoAngleWheelMotion motion = new NoAngleWheelMotion(action, speed, 100, 0, NoAngleWheelMotion.STATUS_KEEP);
                wheelMotionManager.doNoAngleMotion(motion);
                motionHandler.postDelayed(() -> {
                    wheelMotionManager.doNoAngleMotion(new NoAngleWheelMotion(
                            NoAngleWheelMotion.ACTION_STOP, 0, 0, 0, NoAngleWheelMotion.STATUS_KEEP));
                }, duration * 1000);

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

    private void handleHeadMotion(String message) {
        try {
            JSONObject json = new JSONObject(message);
            String direction = json.optString("direction", "stop"); // "up", "down", "left", "right"
            int angle = json.optInt("angle", 10); // ângulo padrão
            int motor = json.optInt("motor", 1); // 1: pescoço, 2: vertical, 3: horizontal
            int speed = json.optInt("speed", 50); // velocidade padrão

            byte action;
            switch (direction) {
                case "up":
                    action = RelativeAngleHeadMotion.ACTION_UP;
                    break;
                case "down":
                    action = RelativeAngleHeadMotion.ACTION_DOWN;
                    break;
                case "left":
                    action = RelativeAngleHeadMotion.ACTION_LEFT;
                    break;
                case "right":
                    action = RelativeAngleHeadMotion.ACTION_RIGHT;
                    break;
                default:
                    action = RelativeAngleHeadMotion.ACTION_STOP;
            }

            RelativeAngleHeadMotion motion = new RelativeAngleHeadMotion((byte) motor, action, (byte) speed, angle);
            headMotionManager.doRelativeAngleMotion(motion);
        } catch (Exception e) {
            e.printStackTrace();
        }
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

    private void handleSpeakAndroid(String message) {
        if (tts != null) {
            try {
                JSONObject json = new JSONObject(message);
                String texto = json.optString("msg", "");
                if (!texto.isEmpty()) {
                    tts.stop();
                    tts.speak(texto, TextToSpeech.QUEUE_FLUSH, null, "mqtt_speak");
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private void handleSpeakSDK(String message){
        try {
            JSONObject json = new JSONObject(message);
            String texto = json.optString("msg", "");
            if (!texto.isEmpty() && speechManager != null) {
                SpeakOption speakOption = new SpeakOption();
                speakOption.setLanguageType(SpeakOption.LAG_ENGLISH_US);
                speakOption.setSpeed(50);      // Velocidade de fala (0 a 100)
                speakOption.setIntonation(50); // Entonação (0 a 100)
                speechManager.stopSpeak();
                speechManager.startSpeak(texto, speakOption);
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void onTopicsUpdated(java.util.List<String> topics) {}

    private void subscribeToAllTopics() {
        String[] topics = getResources().getStringArray(R.array.topics_array);
        for (int i = 1; i < topics.length; i++) {
            mqttHandler.subscribeToTopic(topics[i]);
        }
    }

    private void appendMessage(String msg) {
        textViewMessages.append(msg + "\n");
        scrollView.post(() -> scrollView.fullScroll(View.FOCUS_DOWN));
    }

    @Override
    protected void onPause() {
        super.onPause();
        if (videoHandle != -1) {
            hdCameraManager.closeStream(videoHandle);
            videoHandle = -1;
        }
        if (mjpegServer != null) {
            mjpegServer.stop();
        }
    }

    @Override
    protected void onDestroy() {
        mqttHandler.disconnect();
        publishHandler.removeCallbacksAndMessages(null);
        if (videoHandle != -1) {
            hdCameraManager.closeStream(videoHandle);
        }
        if (mjpegServer != null) {
            mjpegServer.stop();
        }
        if (tts != null) {
            tts.stop();
            tts.shutdown();
        }
        super.onDestroy();
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

                        mqttHandler.publishMessage("sanbot/battery", json.toString());
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

                        mqttHandler.publishMessage("sanbot/info", json.toString());
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }

                publishHandler.postDelayed(this, 5000); // repete a cada 5 segundos
            }
        }, 5000);
    }

    private void publishSpeechToMqtt(String texto) {
        if (mqttHandler != null && connected && texto != null && !texto.isEmpty()) {
            try {
                JSONObject json = new JSONObject();
                json.put("text", texto);
                mqttHandler.publishMessage("sanbot/speech", json.toString());
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

}
