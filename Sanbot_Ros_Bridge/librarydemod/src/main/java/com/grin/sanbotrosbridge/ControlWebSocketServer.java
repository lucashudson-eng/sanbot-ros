package com.grin.sanbotrosbridge;

import android.util.Log;

import org.java_websocket.WebSocket;
import org.java_websocket.handshake.ClientHandshake;
import org.java_websocket.server.WebSocketServer;
import org.json.JSONObject;

import java.net.InetSocketAddress;
import java.nio.ByteBuffer;
import java.util.Collections;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * LAN WebSocket control server (Controller-compatible paths) for My Sanbot Direct WS.
 * Default bind: 0.0.0.0:8080
 */
public class ControlWebSocketServer extends WebSocketServer {

    public interface Listener {
        /** Handle a control frame; return optional JSON reply (may be null). */
        String onControlMessage(String path, JSONObject body);

        void onWsLog(String message);

        /** Fired when a LAN client (e.g. My Sanbot) connects — used to open companion apps. */
        void onWsClientConnected();
    }

    private static final String TAG = "ControlWS";
    private final Listener listener;
    private final Set<WebSocket> clients = Collections.newSetFromMap(new ConcurrentHashMap<>());

    public ControlWebSocketServer(int port, Listener listener) {
        super(new InetSocketAddress(port));
        this.listener = listener;
        setReuseAddr(true);
    }

    @Override
    public void onOpen(WebSocket conn, ClientHandshake handshake) {
        clients.add(conn);
        Log.i(TAG, "Client connected: " + conn.getRemoteSocketAddress());
        if (listener != null) {
            listener.onWsLog("WS client connected: " + conn.getRemoteSocketAddress());
            try {
                listener.onWsClientConnected();
            } catch (Exception e) {
                Log.w(TAG, "onWsClientConnected", e);
            }
        }
    }

    @Override
    public void onClose(WebSocket conn, int code, String reason, boolean remote) {
        clients.remove(conn);
        Log.i(TAG, "Client closed: " + reason);
        if (listener != null) {
            listener.onWsLog("WS client closed (" + code + ")");
        }
    }

    @Override
    public void onMessage(WebSocket conn, String message) {
        try {
            JSONObject obj = new JSONObject(message);
            String path = obj.optString("path", "");
            if (path.isEmpty()) {
                path = obj.optString("topic", "");
            }
            if (path.isEmpty()) {
                conn.send(errorJson("missing path"));
                return;
            }
            JSONObject body;
            if (obj.has("payload") && obj.get("payload") instanceof JSONObject) {
                body = obj.getJSONObject("payload");
            } else {
                body = new JSONObject(obj.toString());
                body.remove("path");
                body.remove("topic");
                body.remove("payload");
            }
            String reply = listener != null ? listener.onControlMessage(path, body) : null;
            if (reply != null && !reply.isEmpty()) {
                conn.send(reply);
            }
        } catch (Exception e) {
            Log.e(TAG, "onMessage error", e);
            try {
                conn.send(errorJson(e.getMessage()));
            } catch (Exception ignored) {
            }
        }
    }

    @Override
    public void onMessage(WebSocket conn, ByteBuffer message) {
        // ignore binary
    }

    @Override
    public void onError(WebSocket conn, Exception ex) {
        Log.e(TAG, "WS error", ex);
        if (listener != null) {
            listener.onWsLog("WS error: " + (ex != null ? ex.getMessage() : "unknown"));
        }
    }

    @Override
    public void onStart() {
        Log.i(TAG, "WS control listening on " + getAddress());
        if (listener != null) {
            listener.onWsLog("WS control listening on " + getAddress());
        }
    }

    public void broadcastJson(String json) {
        for (WebSocket c : clients) {
            if (c != null && c.isOpen()) {
                c.send(json);
            }
        }
    }

    private static String errorJson(String msg) {
        try {
            JSONObject o = new JSONObject();
            o.put("path", "/error");
            o.put("error", msg != null ? msg : "error");
            return o.toString();
        } catch (Exception e) {
            return "{\"path\":\"/error\",\"error\":\"error\"}";
        }
    }
}
