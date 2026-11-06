package com.qihan.HumanFollow;

import android.content.Context;
import android.content.Intent;
import android.content.SharedPreferences;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.Canvas;
import android.graphics.drawable.BitmapDrawable;
import android.graphics.drawable.Drawable;
import android.util.Log;

/**
 * Pins launcher shortcuts via the legacy INSTALL_SHORTCUT broadcast (Nova / Launcher3 on Nano).
 */
final class HomeScreenHelper {

    private static final String TAG = "HumanFollow-home";
    private static final String PREFS = "human_follow";
    private static final String PREF_PINNED = "home_shortcuts_pinned_v1";

    private HomeScreenHelper() {
    }

    static void ensurePinnedOnce(Context context) {
        SharedPreferences prefs = context.getSharedPreferences(PREFS, Context.MODE_PRIVATE);
        if (prefs.getBoolean(PREF_PINNED, false)) {
            return;
        }
        boolean okFollow = pin(
                context,
                "com.qihan.HumanFollow",
                "com.qihan.HumanFollow.HumanFollowActivity",
                "Human Follow");
        boolean okRos = pin(
                context,
                "com.grin.sanbotrosbridge",
                "com.grin.sanbotrosbridge.MainActivity",
                "Sanbot-ros");
        if (okFollow || okRos) {
            prefs.edit().putBoolean(PREF_PINNED, true).apply();
            Log.i(TAG, "home shortcuts requested follow=" + okFollow + " ros=" + okRos);
        }
    }

    /** Force re-pin (e.g. user tapped “Add to Home”). */
    static void pinBoth(Context context) {
        pin(context,
                "com.qihan.HumanFollow",
                "com.qihan.HumanFollow.HumanFollowActivity",
                "Human Follow");
        pin(context,
                "com.grin.sanbotrosbridge",
                "com.grin.sanbotrosbridge.MainActivity",
                "Sanbot-ros");
        context.getSharedPreferences(PREFS, Context.MODE_PRIVATE)
                .edit()
                .putBoolean(PREF_PINNED, true)
                .apply();
    }

    static boolean pin(Context context, String packageName, String className, String label) {
        try {
            PackageManager pm = context.getPackageManager();
            pm.getPackageInfo(packageName, 0);

            Intent launch = new Intent(Intent.ACTION_MAIN);
            launch.setClassName(packageName, className);
            launch.addCategory(Intent.CATEGORY_LAUNCHER);
            launch.setFlags(Intent.FLAG_ACTIVITY_NEW_TASK);

            Intent add = new Intent("com.android.launcher.action.INSTALL_SHORTCUT");
            add.putExtra(Intent.EXTRA_SHORTCUT_INTENT, launch);
            add.putExtra(Intent.EXTRA_SHORTCUT_NAME, label);
            add.putExtra("duplicate", false);

            Drawable icon = pm.getApplicationIcon(packageName);
            Bitmap bitmap = drawableToBitmap(icon);
            if (bitmap != null) {
                add.putExtra(Intent.EXTRA_SHORTCUT_ICON, bitmap);
            }

            context.sendBroadcast(add);
            return true;
        } catch (PackageManager.NameNotFoundException e) {
            Log.w(TAG, "pin skip missing package " + packageName);
            return false;
        } catch (Exception e) {
            Log.w(TAG, "pin failed " + label, e);
            return false;
        }
    }

    private static Bitmap drawableToBitmap(Drawable drawable) {
        if (drawable == null) {
            return null;
        }
        if (drawable instanceof BitmapDrawable) {
            Bitmap b = ((BitmapDrawable) drawable).getBitmap();
            if (b != null) {
                return b;
            }
        }
        int w = Math.max(1, drawable.getIntrinsicWidth());
        int h = Math.max(1, drawable.getIntrinsicHeight());
        Bitmap bitmap = Bitmap.createBitmap(w, h, Bitmap.Config.ARGB_8888);
        Canvas canvas = new Canvas(bitmap);
        drawable.setBounds(0, 0, canvas.getWidth(), canvas.getHeight());
        drawable.draw(canvas);
        return bitmap;
    }
}
