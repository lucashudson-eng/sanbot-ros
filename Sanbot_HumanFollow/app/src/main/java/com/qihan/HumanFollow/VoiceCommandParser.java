package com.qihan.HumanFollow;

/**
 * Maps free-text / grammar speech into follow commands.
 */
final class VoiceCommandParser {

    enum Command {
        START_FOLLOW,
        STOP_FOLLOW,
        SPEECH_ON,
        SPEECH_OFF,
        STATUS,
        UNKNOWN
    }

    private VoiceCommandParser() {
    }

    static Command parse(String raw) {
        if (raw == null) {
            return Command.UNKNOWN;
        }
        String t = raw.trim().toLowerCase()
                .replace('-', ' ')
                .replace('_', ' ');
        while (t.contains("  ")) {
            t = t.replace("  ", " ");
        }
        if (t.isEmpty()) {
            return Command.UNKNOWN;
        }

        if (containsAny(t,
                "speech off", "mute speech", "mute", "be quiet", "quiet",
                "silence", "stop talking", "no speech")) {
            return Command.SPEECH_OFF;
        }
        if (containsAny(t,
                "speech on", "unmute", "enable speech", "talk to me", "speak on")) {
            return Command.SPEECH_ON;
        }

        if (containsAny(t,
                "stop follow", "stop following", "cancel follow", "follow off",
                "quit follow", "end follow", "don't follow", "do not follow",
                "stop tracking")) {
            return Command.STOP_FOLLOW;
        }
        if (containsAny(t,
                "start follow", "start following", "follow me", "follow on",
                "begin follow", "come with me", "track me", "start tracking")) {
            return Command.START_FOLLOW;
        }
        // Bare "follow" after wake — start
        if (t.equals("follow") || t.equals("following")) {
            return Command.START_FOLLOW;
        }

        if (containsAny(t, "status", "how far", "distance", "where am i")) {
            return Command.STATUS;
        }

        return Command.UNKNOWN;
    }

    private static boolean containsAny(String text, String... needles) {
        for (String n : needles) {
            if (text.contains(n)) {
                return true;
            }
        }
        return false;
    }
}
