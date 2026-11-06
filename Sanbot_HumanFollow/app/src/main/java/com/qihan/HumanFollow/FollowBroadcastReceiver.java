package com.qihan.HumanFollow;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.util.Log;

/**
 * Starts / stops face-follow when MainService ObstacleManager broadcasts
 * human-follow open/close (stock {@code switchFollow} path).
 */
public class FollowBroadcastReceiver extends BroadcastReceiver {

    private static final String TAG = "HumanFollowRx";

    @Override
    public void onReceive(Context context, Intent intent) {
        if (intent == null || intent.getAction() == null) {
            return;
        }
        String action = intent.getAction();
        Log.i(TAG, "recv " + action);

        if (HumanFollowActivity.ACTION_SET_SPEECH_FEEDBACK.equals(action)) {
            Intent launch = new Intent(context, HumanFollowActivity.class);
            launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_SINGLE_TOP
                    | Intent.FLAG_ACTIVITY_CLEAR_TOP);
            launch.setAction(HumanFollowActivity.ACTION_SET_SPEECH_FEEDBACK);
            boolean speech = intent.getBooleanExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK, true);
            launch.putExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK, speech);
            context.startActivity(launch);
            return;
        }

        if (HumanFollowActivity.ACTION_SET_DETECTING.equals(action)) {
            boolean enabled = intent.getBooleanExtra(HumanFollowActivity.EXTRA_DETECT,
                    intent.getBooleanExtra(HumanFollowActivity.EXTRA_ENABLED, true));
            Intent launch = new Intent(context, HumanFollowActivity.class);
            launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_SINGLE_TOP
                    | Intent.FLAG_ACTIVITY_CLEAR_TOP);
            launch.setAction(HumanFollowActivity.ACTION_SET_DETECTING);
            launch.putExtra(HumanFollowActivity.EXTRA_DETECT, enabled);
            launch.putExtra(HumanFollowActivity.EXTRA_ENABLED, enabled);
            context.startActivity(launch);
            return;
        }

        if (HumanFollowActivity.ACTION_SET_FOLLOWING.equals(action)) {
            boolean enabled = intent.getBooleanExtra(HumanFollowActivity.EXTRA_ENABLED,
                    intent.getBooleanExtra(HumanFollowActivity.EXTRA_FOLLOW, true));
            Intent launch = new Intent(context, HumanFollowActivity.class);
            launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_SINGLE_TOP
                    | Intent.FLAG_ACTIVITY_CLEAR_TOP);
            launch.setAction(HumanFollowActivity.ACTION_SET_FOLLOWING);
            launch.putExtra(HumanFollowActivity.EXTRA_ENABLED, enabled);
            launch.putExtra(HumanFollowActivity.EXTRA_FOLLOW, enabled);
            if (intent.hasExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK)) {
                launch.putExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK,
                        intent.getBooleanExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK, true));
            }
            context.startActivity(launch);
            return;
        }

        boolean open = action.endsWith(".open") || action.contains("humanfollow.open");
        boolean close = action.endsWith(".close") || action.contains("humanfollow.close");
        if (!open && !close) {
            return;
        }
        Intent launch = new Intent(context, HumanFollowActivity.class);
        launch.addFlags(Intent.FLAG_ACTIVITY_NEW_TASK | Intent.FLAG_ACTIVITY_SINGLE_TOP
                | Intent.FLAG_ACTIVITY_CLEAR_TOP);
        launch.setAction(HumanFollowActivity.ACTION_SET_FOLLOWING);
        launch.putExtra(HumanFollowActivity.EXTRA_ENABLED, open);
        launch.putExtra(HumanFollowActivity.EXTRA_FOLLOW, open);
        if (intent.hasExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK)) {
            launch.putExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK,
                    intent.getBooleanExtra(HumanFollowActivity.EXTRA_SPEECH_FEEDBACK, true));
        }
        context.startActivity(launch);
    }
}
