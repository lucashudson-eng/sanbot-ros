package com.lucashudson.mqttsanbotapp

import android.os.Bundle
import android.widget.*
import androidx.appcompat.app.AppCompatActivity

class MainActivity : AppCompatActivity(), MqttHandler.MqttStatusListener {

    private lateinit var editTextIp: EditText
    private lateinit var buttonConnect: Button
    private lateinit var spinnerTopics: Spinner
    private lateinit var textViewMessages: TextView

    private var mqttHandler: MqttHandler? = null
    private var connected = false
    private var selectedTopic: String = "Todos"

    private val PREFS_NAME = "mqtt_prefs"
    private val KEY_LAST_IP = "last_broker_ip"

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        editTextIp = findViewById(R.id.editTextIp)
        buttonConnect = findViewById(R.id.buttonConnect)
        spinnerTopics = findViewById(R.id.spinnerTopics)
        textViewMessages = findViewById(R.id.textViewMessages)

        val prefs = getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
        val lastIp = prefs.getString(KEY_LAST_IP, "")
        editTextIp.setText(lastIp)

        mqttHandler = MqttHandler(this, this)

        spinnerTopics.onItemSelectedListener = object : AdapterView.OnItemSelectedListener {
            override fun onItemSelected(parent: AdapterView<*>?, view: android.view.View?, position: Int, id: Long) {
                selectedTopic = parent?.getItemAtPosition(position).toString()
                appendMessage("Filtro atualizado: mostrando mensagens de \"$selectedTopic\"")
            }

            override fun onNothingSelected(parent: AdapterView<*>?) {}
        }

        buttonConnect.setOnClickListener {
            val ip = editTextIp.text.toString().trim()
            if (ip.isEmpty()) {
                Toast.makeText(this, "Informe o IP do broker", Toast.LENGTH_SHORT).show()
                return@setOnClickListener
            }

            // Salvar o IP no SharedPreferences
            getSharedPreferences(PREFS_NAME, MODE_PRIVATE)
                .edit()
                .putString(KEY_LAST_IP, ip)
                .apply()


            mqttHandler?.setBrokerIp(ip)

            if (!connected) {
                mqttHandler?.connect()
                buttonConnect.isEnabled = false
                buttonConnect.text = "Conectando..."
            } else {
                mqttHandler?.disconnect()
                connected = false
                buttonConnect.text = "Conectar"
                appendMessage("Desconectado do broker.")
            }
        }
    }

    override fun onConnectionStatusChanged(connected: Boolean) {
        runOnUiThread {
            this.connected = connected
            buttonConnect.isEnabled = true
            buttonConnect.text = if (connected) "Desconectar" else "Conectar"

            val msg = if (connected) {
                subscribeToAllTopics()
                "Conectado com sucesso ao broker!"
            } else {
                "Falha na conexão ou desconectado."
            }
            appendMessage(msg)
        }
    }

    override fun onMessageReceived(topic: String, message: String) {
        runOnUiThread {
            if (selectedTopic == "Todos" || selectedTopic == topic) {
                appendMessage("[$topic]: $message")
            }
        }
    }

    override fun onTopicsUpdated(topics: List<String>) {
        // Futuro: usar para atualizar a UI, se necessário
    }

    private fun subscribeToAllTopics() {
        val topics = resources.getStringArray(R.array.topics_array)
        // pula o primeiro item ("Todos")
        for (topic in topics.drop(1)) {
            mqttHandler?.subscribeToTopic(topic)
        }
    }

    private fun appendMessage(msg: String) {
        textViewMessages.append("$msg\n")
    }

    override fun onDestroy() {
        mqttHandler?.disconnect()
        super.onDestroy()
    }
}
