package com.grin.sanbotrosbridge;

import android.content.Context;
import androidx.test.platform.app.InstrumentationRegistry; // Updated import
import androidx.test.ext.junit.runners.AndroidJUnit4;     // Updated import

import org.junit.Test;
import org.junit.runner.RunWith;

import static org.junit.Assert.*;

/**
 * Instrumented test, which will execute on an Android device.
 *
 * @see Testing documentation
 */
@RunWith(AndroidJUnit4.class)
public class ExampleInstrumentedTest {
    @Test
    public void useAppContext() {
        // Context of the app under test.
        Context appContext = InstrumentationRegistry.getInstrumentation().getTargetContext(); // Updated line
        assertEquals("com.grin.sanbotrosbridge", appContext.getPackageName());
    }
}