package com.grin.sanbotrosbridge;

import android.app.Activity;
import android.content.Context;
import android.view.View;
import android.view.inputmethod.InputMethodManager;

/** Dismiss soft keyboard / SwiftKey on the Nano face tablet. */
public final class SoftInputHider {
    private SoftInputHider() {
    }

    public static void hide(Context context) {
        if (context instanceof Activity) {
            hideActivity((Activity) context);
            return;
        }
        if (context == null) {
            return;
        }
        try {
            InputMethodManager imm = (InputMethodManager)
                    context.getSystemService(Context.INPUT_METHOD_SERVICE);
            if (imm != null) {
                imm.toggleSoftInput(InputMethodManager.HIDE_IMPLICIT_ONLY, 0);
            }
        } catch (Exception ignored) {
        }
    }

    public static void hideActivity(Activity activity) {
        if (activity == null || activity.isFinishing()) {
            return;
        }
        try {
            View focus = activity.getCurrentFocus();
            if (focus == null) {
                focus = activity.getWindow() != null
                        ? activity.getWindow().getDecorView()
                        : null;
            }
            if (focus != null) {
                focus.clearFocus();
                InputMethodManager imm = (InputMethodManager)
                        activity.getSystemService(Context.INPUT_METHOD_SERVICE);
                if (imm != null) {
                    imm.hideSoftInputFromWindow(focus.getWindowToken(), 0);
                }
            }
        } catch (Exception ignored) {
        }
    }
}
