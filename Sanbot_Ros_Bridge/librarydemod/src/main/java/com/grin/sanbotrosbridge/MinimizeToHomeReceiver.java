package com.grin.sanbotrosbridge;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

/**
 * Companion minimize: when OpenSDK Proxy returns home, bring ROS briefly and
 * {@code moveTaskToBack} so it does not stay under the face launcher.
 */
public final class MinimizeToHomeReceiver extends BroadcastReceiver {
    public static final String ACTION = "mysanbot.intent.action.MINIMIZE_TO_HOME";
    private static final String TAG = "MinimizeToHome";

    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent == null || !ACTION.equals(intent.getAction()) || context == null) {
            return;
        }
        Log.i(TAG, "MINIMIZE_TO_HOME received → ROS");
        CompanionOpenGate.quietAfterHome();
        try {
            Intent launch = new Intent(context, MainActivity.class);
            launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK
                    | Intent.FLAG_ACTIVITY_SINGLE_TOP
                    | Intent.FLAG_ACTIVITY_REORDER_TO_FRONT);
            launch.putExtra(MainActivity.EXTRA_MINIMIZE_TO_HOME, true);
            context.startActivity(launch);
        } catch (Exception e) {
            Log.w(TAG, "minimize launch: " + e.getMessage());
        }
    }

    /** Notify companion apps (OpenSDK Proxy) to background themselves. */
    public static void notifyCompanions(Context ctx) {
        if (ctx == null) {
            return;
        }
        try {
            Intent i = new Intent(ACTION);
            i.setPackage("mysanbot.opensdkproxy");
            ctx.sendBroadcast(i);
            Log.i(TAG, "notified Proxy to minimize");
        } catch (Exception e) {
            Log.w(TAG, "notify Proxy: " + e.getMessage());
        }
    }
}
