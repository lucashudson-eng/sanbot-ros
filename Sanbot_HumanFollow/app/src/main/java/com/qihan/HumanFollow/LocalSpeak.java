package com.qihan.HumanFollow;

import android.util.Log;

import com.sanbot.opensdk.function.beans.SpeakOption;
import com.sanbot.opensdk.function.unit.SpeechManager;

/**
 * Speak helper: prefers local Google TTS when enabled so Nano is audible when
 * Qihan cloud TTS / phantom headset fails.
 */
public final class LocalSpeak {
    private static final String TAG = "LocalSpeak";

    private LocalSpeak() {
    }

    /**
     * @return true if local Google TTS accepted the utterance
     */
    public static boolean speak(
            boolean googleTtsEnabled,
            AndroidTtsSpeaker localTts,
            SpeechManager speechManager,
            String text,
            int speed0to100,
            int volume0to100,
            SpeakOption openSdkOption) {
        if (text == null || text.trim().isEmpty()) {
            return false;
        }
        if (googleTtsEnabled && localTts != null) {
            boolean queued = localTts.speak(text, speed0to100, volume0to100, "en", null);
            if (queued) {
                Log.i(TAG, "local Google TTS: " + text);
                return true;
            }
            Log.w(TAG, "Google TTS unavailable — falling back to OpenSDK cloud TTS");
        }
        if (speechManager != null) {
            try {
                speechManager.stopSpeak();
                speechManager.startSpeak(text, openSdkOption);
            } catch (Exception e) {
                Log.w(TAG, "OpenSDK startSpeak failed", e);
            }
        }
        return false;
    }

    public static void stop(AndroidTtsSpeaker localTts, SpeechManager speechManager) {
        if (localTts != null) {
            localTts.stop();
        }
        if (speechManager != null) {
            try {
                speechManager.stopSpeak();
            } catch (Exception ignored) {
            }
        }
    }
}
