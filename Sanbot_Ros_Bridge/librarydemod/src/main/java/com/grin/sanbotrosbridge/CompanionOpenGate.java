package com.grin.sanbotrosbridge;

import android.app.ActivityManager;
import android.content.Context;
import android.util.Log;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FileReader;
import java.nio.charset.Charset;

/**
 * Shared quiet window after Return to Home so open-on-connect does not yank
 * OpenSDK Proxy / Sanbot-ros back over the face launcher.
 * Uses a world-readable timestamp file both apps honor.
 */
public final class CompanionOpenGate {
    private static final String TAG = "CompanionOpenGate";
    private static final String[] QUIET_UNTIL_FILES = {
            "/data/local/tmp/mysanbot_companion_quiet_until",
            "/sdcard/Download/mysanbot_companion_quiet_until",
            "/sdcard/mysanbot_companion_quiet_until"
    };
    private static final Charset UTF8 = Charset.forName("UTF-8");
    private static final long DEFAULT_QUIET_MS = 90_000L;

    private CompanionOpenGate() {
    }

    public static void quietAfterHome() {
        quietFor(DEFAULT_QUIET_MS);
    }

    public static void quietFor(long durationMs) {
        long until = System.currentTimeMillis() + Math.max(5_000L, durationMs);
        byte[] payload = Long.toString(until).getBytes(UTF8);
        for (String path : QUIET_UNTIL_FILES) {
            try {
                File f = new File(path);
                File parent = f.getParentFile();
                if (parent != null && !parent.exists()) {
                    //noinspection ResultOfMethodCallIgnored
                    parent.mkdirs();
                }
                FileOutputStream out = new FileOutputStream(f);
                out.write(payload);
                out.close();
                //noinspection ResultOfMethodCallIgnored
                f.setReadable(true, false);
                //noinspection ResultOfMethodCallIgnored
                f.setWritable(true, false);
                Log.i(TAG, "Companion open quiet until " + until + " via " + path);
                return;
            } catch (Exception e) {
                Log.w(TAG, "quietFor " + path + ": " + e.getMessage());
            }
        }
    }

    public static boolean isQuiet() {
        long now = System.currentTimeMillis();
        for (String path : QUIET_UNTIL_FILES) {
            BufferedReader reader = null;
            try {
                File f = new File(path);
                if (!f.isFile()) {
                    continue;
                }
                reader = new BufferedReader(new FileReader(f));
                String line = reader.readLine();
                if (line == null || line.trim().isEmpty()) {
                    continue;
                }
                long until = Long.parseLong(line.trim());
                if (now < until) {
                    return true;
                }
            } catch (Exception ignored) {
            } finally {
                if (reader != null) {
                    try {
                        reader.close();
                    } catch (Exception ignored) {
                    }
                }
            }
        }
        return false;
    }

    @SuppressWarnings("deprecation")
    public static boolean isPackageProcessRunning(Context ctx, String packageName) {
        if (ctx == null || packageName == null || packageName.isEmpty()) {
            return false;
        }
        try {
            ActivityManager am = (ActivityManager) ctx.getSystemService(Context.ACTIVITY_SERVICE);
            if (am == null) {
                return false;
            }
            for (ActivityManager.RunningAppProcessInfo proc : am.getRunningAppProcesses()) {
                if (proc == null) {
                    continue;
                }
                if (packageName.equals(proc.processName)) {
                    return true;
                }
                if (proc.pkgList != null) {
                    for (String pkg : proc.pkgList) {
                        if (packageName.equals(pkg)) {
                            return true;
                        }
                    }
                }
            }
        } catch (Exception e) {
            Log.w(TAG, "isPackageProcessRunning: " + e.getMessage());
        }
        return false;
    }
}
