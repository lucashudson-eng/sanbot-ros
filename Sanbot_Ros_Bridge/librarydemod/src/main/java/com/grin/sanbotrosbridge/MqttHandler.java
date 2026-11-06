package com.grin.sanbotrosbridge;

import android.content.Context;
import android.os.Handler;
import android.os.HandlerThread;
import android.util.Log;

import org.eclipse.paho.client.mqttv3.IMqttActionListener;
import org.eclipse.paho.client.mqttv3.IMqttDeliveryToken;
import org.eclipse.paho.client.mqttv3.IMqttToken;
import org.eclipse.paho.client.mqttv3.MqttAsyncClient;
import org.eclipse.paho.client.mqttv3.MqttCallback;
import org.eclipse.paho.client.mqttv3.MqttConnectOptions;
import org.eclipse.paho.client.mqttv3.MqttException;
import org.eclipse.paho.client.mqttv3.MqttMessage;
import org.eclipse.paho.client.mqttv3.persist.MemoryPersistence;

import java.util.ArrayList;
import java.util.List;
import java.util.UUID;

public class MqttHandler {
    private static final String TAG = "MqttHandler";

    private MqttAsyncClient mqttClient;
    private Context context;
    private String brokerIp = "192.168.0.101";
    private String brokerPort = "1883";
    private boolean isConnected = false;
    private List<String> topicList;
    private List<String> subscribedTopics = new ArrayList<>();
    private MqttStatusListener statusListener;
    private final HandlerThread publishThread;
    private final Handler publishHandler;

    public interface MqttStatusListener {
        void onConnectionStatusChanged(boolean connected);
        void onMessageReceived(String topic, String message);
        void onTopicsUpdated(List<String> topics);
    }

    public MqttHandler(Context context, MqttStatusListener listener) {
        this.context = context;
        this.statusListener = listener;
        this.publishThread = new HandlerThread("mqtt-publish");
        this.publishThread.start();
        this.publishHandler = new Handler(publishThread.getLooper());
        setupTopics();
    }

    private void setupTopics() {
        topicList = new ArrayList<>();
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/commands");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/image");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/escolha");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/pos");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/telemetry");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/device_space");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/maps_done");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/mapping_status");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/rosbag_status");
        topicList.add("substations/SUB001/rovers/Rover-Argo-N-0/copy_data_usb");
        topicList.add("teste");
    }

    public void setBrokerIp(String ip) {
        this.brokerIp = ip;
    }

    public void setBrokerPort(String port) {
        if (port == null || port.trim().isEmpty()) {
            this.brokerPort = "1883";
        } else {
            this.brokerPort = port.trim();
        }
    }

    public boolean isConnected() {
        return isConnected;
    }

    public List<String> getSubscribedTopics() {
        return new ArrayList<>(subscribedTopics);
    }

    public void connect() {
        Log.d(TAG, "Conectando ao broker MQTT: " + brokerIp);

        // Limpar a lista de tópicos inscritos ao iniciar nova conexão
        subscribedTopics.clear();
        if (statusListener != null) {
            statusListener.onTopicsUpdated(subscribedTopics);
        }

        // Criar identificador de cliente único
        String clientId = "AndroidClient-" + UUID.randomUUID().toString();
        String port = (brokerPort == null || brokerPort.isEmpty()) ? "1883" : brokerPort;
        String serverUri = "tcp://" + brokerIp + ":" + port;

        try {
            // Se existe uma conexão anterior, desconectar
            if (mqttClient != null && mqttClient.isConnected()) {
                mqttClient.disconnect();
            }

            // Criar novo cliente MQTT assíncrono usando diretamente a biblioteca Paho
            mqttClient = new MqttAsyncClient(serverUri, clientId, new MemoryPersistence());

            // Configurar callback para eventos MQTT
            mqttClient.setCallback(new MqttCallback() {
                @Override
                public void connectionLost(Throwable cause) {
                    Log.e(TAG, "Conexão MQTT perdida: " + cause.getMessage());
                    isConnected = false;
                    subscribedTopics.clear();
                    statusListener.onConnectionStatusChanged(false);
                    statusListener.onTopicsUpdated(subscribedTopics);
                }

                @Override
                public void messageArrived(String topic, MqttMessage message) {
                    try {
                        String payload = new String(message.getPayload());
                        // No per-message Log.d — CheckJNI log spam ANRs the Nano UI thread.
                        if (statusListener != null) {
                            statusListener.onMessageReceived(topic, payload);
                        }
                    } catch (Exception e) {
                        Log.e(TAG, "Erro ao processar mensagem recebida: " + e.getMessage(), e);
                    }
                }

                @Override
                public void deliveryComplete(IMqttDeliveryToken token) {
                    // Intentionally quiet — high-rate telemetry would flood logcat.
                }
            });

            // Configurar opções de conexão
            MqttConnectOptions options = new MqttConnectOptions();
            options.setCleanSession(true);
            options.setKeepAliveInterval(60);
            options.setAutomaticReconnect(true);

            // Conectar ao broker assincronamente
            mqttClient.connect(options, null, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.d(TAG, "Conexão MQTT bem-sucedida");
                    isConnected = true;
                    statusListener.onConnectionStatusChanged(true);

                    // Assinar tópicos
                    subscribeToTopics();
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.e(TAG, "Falha na conexão MQTT: " + exception.getMessage());
                    isConnected = false;
                    statusListener.onConnectionStatusChanged(false);
                }
            });

        } catch (MqttException e) {
            Log.e(TAG, "Erro na conexão MQTT: " + e.getMessage(), e);
            isConnected = false;
            statusListener.onConnectionStatusChanged(false);
        }
    }

    private void subscribeToTopics() {
        if (mqttClient == null || !mqttClient.isConnected()) {
            Log.e(TAG, "Não é possível assinar tópicos: cliente MQTT não conectado");
            return;
        }

        // Limpar lista de tópicos inscritos antes de reinscrever
        subscribedTopics.clear();

        for (String topic : topicList) {
            try {
                Log.d(TAG, "Tentando assinar tópico: " + topic);
                mqttClient.subscribe(topic, 0, null, new IMqttActionListener() {
                    @Override
                    public void onSuccess(IMqttToken asyncActionToken) {
                        Log.d(TAG, "Assinatura bem-sucedida no tópico: " + topic);
                        if (!subscribedTopics.contains(topic)) {
                            subscribedTopics.add(topic);
                        }
                        statusListener.onTopicsUpdated(getSubscribedTopics());
                    }

                    @Override
                    public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                        Log.e(TAG, "Erro ao assinar tópico " + topic + ": " + exception.getMessage());
                    }
                });
            } catch (MqttException e) {
                Log.e(TAG, "Erro ao assinar tópico " + topic + ": " + e.getMessage(), e);
            }
        }

        // Garantir que o tópico de comandos seja assinado explicitamente
        // (redundância para maior segurança)
        try {
            String commandTopic = "substations/SUB001/rovers/Rover-Argo-N-0/commands";
            Log.d(TAG, "Assinando explicitamente o tópico de comandos: " + commandTopic);
            mqttClient.subscribe(commandTopic, 0);

            if (!subscribedTopics.contains(commandTopic)) {
                subscribedTopics.add(commandTopic);
            }
            statusListener.onTopicsUpdated(getSubscribedTopics());
        } catch (MqttException e) {
            Log.e(TAG, "Erro ao assinar tópico de comandos: " + e.getMessage(), e);
        }
    }

    public void publishMessage(String topic, String message) {
        if (topic == null || message == null) {
            return;
        }
        // Always hop off the caller (often OpenSDK main-thread sensor callbacks).
        final String publishTopic = topic;
        final String publishPayload = message;
        publishHandler.post(() -> {
            MqttAsyncClient client = mqttClient;
            if (client == null || !client.isConnected()) {
                return;
            }
            try {
                MqttMessage mqttMessage = new MqttMessage(publishPayload.getBytes());
                mqttMessage.setQos(0);
                client.publish(publishTopic, mqttMessage);
            } catch (MqttException e) {
                Log.e(TAG, "Erro ao publicar mensagem: " + e.getMessage());
            }
        });
    }

    public void disconnect() {
        publishHandler.removeCallbacksAndMessages(null);
        if (mqttClient != null && mqttClient.isConnected()) {
            try {
                mqttClient.disconnect();
                Log.d(TAG, "MQTT Desconectado");
            } catch (MqttException e) {
                Log.e(TAG, "Erro ao desconectar: " + e.getMessage(), e);
            } finally {
                isConnected = false;
                subscribedTopics.clear();
                if (statusListener != null) {
                    statusListener.onTopicsUpdated(subscribedTopics);
                }
            }
        }
        isConnected = false;
        subscribedTopics.clear();
        if (statusListener != null) {
            statusListener.onTopicsUpdated(subscribedTopics);
        }
    }

    /**
     * Assina um tópico específico
     * @param topic o tópico a ser assinado
     */
    public void subscribeToTopic(String topic) {
        if (mqttClient == null || !mqttClient.isConnected()) {
            Log.e(TAG, "Não é possível assinar tópico: cliente MQTT não conectado");
            return;
        }

        try {
            mqttClient.subscribe(topic, 0, null, new IMqttActionListener() {
                @Override
                public void onSuccess(IMqttToken asyncActionToken) {
                    Log.d(TAG, "Assinatura explícita bem-sucedida no tópico: " + topic);
                    if (!subscribedTopics.contains(topic)) {
                        subscribedTopics.add(topic);
                        statusListener.onTopicsUpdated(getSubscribedTopics());
                    }
                }

                @Override
                public void onFailure(IMqttToken asyncActionToken, Throwable exception) {
                    Log.e(TAG, "Erro ao assinar tópico " + topic + ": " + exception.getMessage());
                }
            });
        } catch (MqttException e) {
            Log.e(TAG, "Erro ao assinar tópico " + topic + ": " + e.getMessage(), e);
        }
    }
}