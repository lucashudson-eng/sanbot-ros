package com.sanbot.librarydemod;

import android.content.SharedPreferences;
import android.os.Bundle;
import android.view.View;
import android.widget.*;
import android.support.v7.app.AppCompatActivity;

public class MainActivity extends AppCompatActivity implements MqttHandler.MqttStatusListener {

    private EditText editTextIp;
    private Button buttonConnect;
    private Spinner spinnerTopics;
    private TextView textViewMessages;

    private MqttHandler mqttHandler;
    private boolean connected = false;
    private String selectedTopic = "Todos";

    private static final String PREFS_NAME = "mqtt_prefs";
    private static final String KEY_LAST_IP = "last_broker_ip";

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        editTextIp = findViewById(R.id.editTextIp);
        buttonConnect = findViewById(R.id.buttonConnect);
        spinnerTopics = findViewById(R.id.spinnerTopics);
        textViewMessages = findViewById(R.id.textViewMessages);

        SharedPreferences prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE);
        String lastIp = prefs.getString(KEY_LAST_IP, "");
        editTextIp.setText(lastIp);

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

            // Salvar o IP no SharedPreferences
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
    }

    @Override
    public void onConnectionStatusChanged(boolean connected) {
        runOnUiThread(() -> {
            this.connected = connected;
            buttonConnect.setEnabled(true);
            buttonConnect.setText(connected ? "Desconectar" : "Conectar");

            String msg;
            if (connected) {
                subscribeToAllTopics();
                msg = "Conectado com sucesso ao broker!";
            } else {
                msg = "Falha na conexão ou desconectado.";
            }
            appendMessage(msg);
        });
    }

    @Override
    public void onMessageReceived(String topic, String message) {
        runOnUiThread(() -> {
            if ("Todos".equals(selectedTopic) || selectedTopic.equals(topic)) {
                appendMessage("[" + topic + "]: " + message);
            }
        });
    }

    @Override
    public void onTopicsUpdated(java.util.List<String> topics) {
        // Futuro: usar para atualizar a UI, se necessário
    }

    private void subscribeToAllTopics() {
        String[] topics = getResources().getStringArray(R.array.topics_array);
        for (int i = 1; i < topics.length; i++) { // pular o "Todos"
            mqttHandler.subscribeToTopic(topics[i]);
        }
    }

    private void appendMessage(String msg) {
        textViewMessages.append(msg + "\n");
    }

    @Override
    protected void onDestroy() {
        mqttHandler.disconnect();
        super.onDestroy();
    }
}
