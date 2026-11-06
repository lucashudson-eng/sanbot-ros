package com.grin.sanbotrosbridge;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.os.Handler;
import android.os.Looper;

/**
 * Starts {@link MainActivity} after boot when "Auto-launch on boot" is enabled.
 * Delay gives the launcher time to settle on Sanbot Nano (Android 6).
 */
public class BootReceiver extends BroadcastReceiver {
    private static final String PREFS_NAME = "mqtt_prefs";
    private static final String KEY_AUTO_LAUNCH = "auto_launch";
    private static final long LAUNCH_DELAY_MS = 5000L;

    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent == null || intent.getAction() == null) {
            return;
        }
        String action = intent.getAction();
        if (!Intent.ACTION_BOOT_COMPLETED.equals(action)
                && !"android.intent.action.QUICKBOOT_POWERON".equals(action)) {
            return;
        }

        SharedPreferences prefs = context.getSharedPreferences(PREFS_NAME, Context.MODE_PRIVATE);
        if (!prefs.getBoolean(KEY_AUTO_LAUNCH, true)) {
            return;
        }

        Intent launch = new Intent(context, MainActivity.class);
        launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK);
        new Handler(Looper.getMainLooper()).postDelayed(
                () -> context.startActivity(launch),
                LAUNCH_DELAY_MS
        );
    }
}
