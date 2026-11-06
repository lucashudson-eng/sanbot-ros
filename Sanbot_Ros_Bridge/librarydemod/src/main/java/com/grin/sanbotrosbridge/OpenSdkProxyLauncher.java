package com.grin.sanbotrosbridge;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.util.Log;

import org.json.JSONObject;

/**
 * Launch OpenSDK Proxy from Sanbot-ros.
 * Proxy 1.3.74+ defaults: WS :9181 / HTTP :9182 (configurable Listen ports).
 * Legacy 8081/8082 still probed for local HTTP forwards.
 * Cameras/RTMP stay here; proxy owns remote control.
 */
public final class OpenSdkProxyLauncher {
    private static final String TAG = "OpenSdkProxyLaunch";

    public static final String PROXY_PACKAGE = "mysanbot.opensdkproxy";
    public static final String PROXY_ACTIVITY = "mysanbot.opensdkproxy.ProxyActivity";

    /** Proxy 1.3.74+ default listen ports (away from ROS :8080 / MediaMTX 888x). */
    public static final int DEFAULT_PROXY_WS_PORT = 9181;
    public static final int DEFAULT_PROXY_HTTP_PORT = 9182;
    /** Pre-1.3.74 defaults — keep as fallback for wing HTTP forward. */
    public static final int LEGACY_PROXY_HTTP_PORT = 8082;

    public static final String KEY_AUTO_START_PROXY = "auto_start_opensdk_proxy";
    public static final String KEY_OPEN_PROXY_ON_CONNECT = "open_proxy_on_ws_connect";

    private static long lastConnectLaunchMs;

    private OpenSdkProxyLauncher() {
    }

    public static boolean isInstalled(Context ctx) {
        if (ctx == null) {
            return false;
        }
        try {
            ctx.getPackageManager().getPackageInfo(PROXY_PACKAGE, 0);
            return true;
        } catch (PackageManager.NameNotFoundException e) {
            return false;
        } catch (Exception e) {
            return false;
        }
    }

    public static boolean isAutoStartEnabled(SharedPreferences prefs) {
        return prefs != null && prefs.getBoolean(KEY_AUTO_START_PROXY, false);
    }

    public static boolean isOpenOnConnectEnabled(SharedPreferences prefs) {
        // Default false: opening Proxy Activity over Bridge kills RTMP (needs MainActivity fg).
        return prefs != null && prefs.getBoolean(KEY_OPEN_PROXY_ON_CONNECT, false);
    }

    public static boolean launch(Context ctx) {
        if (ctx == null || !isInstalled(ctx)) {
            return false;
        }
        try {
            Intent launch = new Intent();
            launch.setClassName(PROXY_PACKAGE, PROXY_ACTIVITY);
            launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK
                    | Intent.FLAG_ACTIVITY_SINGLE_TOP
                    | Intent.FLAG_ACTIVITY_REORDER_TO_FRONT);
            ctx.startActivity(launch);
            Log.i(TAG, "Launched " + PROXY_PACKAGE);
            return true;
        } catch (Exception e) {
            Log.e(TAG, "launch failed", e);
            return false;
        }
    }

    /** Debounced companion open when a control client connects to ros WS. */
    public static void maybeOpenOnClientConnect(Context ctx, SharedPreferences prefs) {
        if (ctx == null || prefs == null || !isOpenOnConnectEnabled(prefs)) {
            return;
        }
        if (CompanionOpenGate.isQuiet()) {
            Log.i(TAG, "Skip companion open — quiet after Home");
            return;
        }
        // Already running in background: do not REORDER over the face launcher.
        if (CompanionOpenGate.isPackageProcessRunning(ctx, PROXY_PACKAGE)) {
            Log.i(TAG, "Skip companion open — proxy already running");
            return;
        }
        long now = System.currentTimeMillis();
        if (now - lastConnectLaunchMs < 8000L) {
            return;
        }
        lastConnectLaunchMs = now;
        if (!isInstalled(ctx)) {
            Log.w(TAG, "OpenSDK Proxy not installed — skip companion open");
            return;
        }
        launch(ctx);
    }

    public static JSONObject status(Context ctx, SharedPreferences prefs) {
        JSONObject o = new JSONObject();
        try {
            o.put("package", PROXY_PACKAGE);
            o.put("installed", isInstalled(ctx));
            o.put("auto_start_proxy", isAutoStartEnabled(prefs));
            o.put("open_on_connect", isOpenOnConnectEnabled(prefs));
            o.put("default_ws_port", DEFAULT_PROXY_WS_PORT);
            o.put("default_http_port", DEFAULT_PROXY_HTTP_PORT);
            o.put("ok", true);
        } catch (Exception ignored) {
        }
        return o;
    }

    /**
     * POST JSON to Proxy local HTTP {@code /joints} (or other path).
     * Tries default :9182 then legacy :8082.
     *
     * @return HTTP response code, or -1 if both ports fail
     */
    public static int postLocalHttp(String path, byte[] body) {
        String rel = (path == null || path.isEmpty()) ? "/" : (path.startsWith("/") ? path : "/" + path);
        int[] ports = {DEFAULT_PROXY_HTTP_PORT, LEGACY_PROXY_HTTP_PORT};
        Exception last = null;
        for (int port : ports) {
            try {
                java.net.HttpURLConnection conn = (java.net.HttpURLConnection)
                        new java.net.URL("http://127.0.0.1:" + port + rel).openConnection();
                conn.setConnectTimeout(2500);
                conn.setReadTimeout(2500);
                conn.setRequestMethod("POST");
                conn.setDoOutput(true);
                conn.setRequestProperty("Content-Type", "application/json");
                if (body != null && body.length > 0) {
                    conn.getOutputStream().write(body);
                }
                int code = conn.getResponseCode();
                conn.disconnect();
                Log.i(TAG, "POST :" + port + rel + " → HTTP " + code);
                return code;
            } catch (Exception e) {
                last = e;
                Log.w(TAG, "POST :" + port + rel + " failed: " + e.getMessage());
            }
        }
        if (last != null) {
            Log.w(TAG, "postLocalHttp exhausted ports for " + rel, last);
        }
        return -1;
    }
}
