package com.grin.sanbotopensdkmqtt;

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

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.wheelmotion.NoAngleWheelMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.RelativeAngleWheelMotion;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.SystemManager;
import com.sanbot.opensdk.function.unit.WheelMotionManager;

import org.json.JSONObject;
import com.sanbot.opensdk.function.beans.wheelmotion.DistanceWheelMotion;

public class MainActivity extends TopBaseActivity implements MqttHandler.MqttStatusListener {

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
    private Handler publishHandler = new Handler();

    private final Handler motionHandler = new Handler(Looper.getMainLooper());


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

        mqttHandler = new MqttHandler(this, this);

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
    }

    @Override
    protected void onMainServiceConnected() {
        systemManager = (SystemManager) getUnitManager(FuncConstant.SYSTEM_MANAGER);
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        wheelMotionManager = (WheelMotionManager) getUnitManager(FuncConstant.WHEELMOTION_MANAGER);
        startBatteryPublishingLoop();
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
                appendMessage("Luz desligada");
            } else if (level >= 1 && level <= 3) {
                hardWareManager.switchWhiteLight(true);
                hardWareManager.setWhiteLightLevel(level);
                appendMessage("Luz ligada no nível: " + level);
            }
        } catch (Exception e) {
            appendMessage("Erro ao processar comando de luz: " + e.getMessage());
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
                motionHandler.removeCallbacksAndMessages(null);
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
    protected void onDestroy() {
        mqttHandler.disconnect();
        publishHandler.removeCallbacksAndMessages(null);
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
                        appendMessage("Publicado bateria: " + json);
                    } catch (Exception e) {
                        e.printStackTrace();
                    }
                }
                publishHandler.postDelayed(this, 5000);
            }
        }, 5000);
    }
}
