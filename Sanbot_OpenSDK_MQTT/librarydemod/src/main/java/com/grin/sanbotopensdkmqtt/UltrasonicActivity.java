package com.grin.sanbotopensdkmqtt;

import android.os.Bundle;
import android.os.Handler;
import android.os.Message;
import android.widget.TextView;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.interfaces.hardware.UltrasonicListener;

import java.lang.ref.WeakReference;

/**
 * Created by admin on 2017/5/9.
 */
public class UltrasonicActivity extends TopBaseActivity {

    private TextView ir1_tv;
    private TextView ir3_tv;
    private TextView ir2_tv;
    private TextView ir4_tv;
    private TextView ir5_tv;
    private TextView ir6_tv;
    private TextView ir7_tv;
    private TextView ir8_tv;
    private TextView d1_tv;
    private TextView d2_tv;
    private TextView d3_tv;
    private TextView d4_tv;
    private TextView d5_tv;
    private TextView d6_tv;
    private TextView d7_tv;
    private TextView d8_tv;
    private TextView d9_tv;
    private byte initByte = (byte) 254;
    private HardWareManager hardWareManager;


    @Override
    protected void onCreate(Bundle savedInstanceState) {
        register(UltrasonicActivity.class);
        super.onCreate(savedInstanceState);
        setBodyView(R.layout.activity_ultrasonic);

        initView();
        initData();
    }

    private MyHandler myHandler;

    private void initData() {
        myHandler = new MyHandler(this);
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        hardWareManager.setOnHareWareListener(new UltrasonicListener() {
            @Override
            public void onUltrasonicResult(int ultrasonicNum, int data) {
                switch (ultrasonicNum) {
                    case 1:
                        if (data != 65535) {
                            ir1_tv.setText("" + data);
                        } else {
                            ir1_tv.setText("data error");
                        }
                        break;
                    case 2:
                        if (data != 65535) {
                            ir2_tv.setText("" + data);
                        } else {
                            ir2_tv.setText("data error");
                        }
                        break;
                    case 3:
                        if (data != 65535) {
                            ir3_tv.setText("" + data);
                        } else {
                            ir3_tv.setText("data error");
                        }
                        break;
                    case 4:
                        if (data != 65535) {
                            ir4_tv.setText("" + data);
                        } else {
                            ir4_tv.setText("data error");
                        }
                        break;
                    case 5:
                        if (data != 65535) {
                            ir5_tv.setText("" + data);
                        } else {
                            ir5_tv.setText("data error");
                        }
                        break;
                    case 6:
                        if (data != 65535) {
                            ir6_tv.setText("" + data);
                        } else {
                            ir6_tv.setText("data error");
                        }
                        break;
                    case 7:
                        if (data != 65535) {
                            ir7_tv.setText("" + data);
                        } else {
                            ir7_tv.setText("data error");
                        }
                        break;
                    case 8:
                        if (data != 65535) {
                            ir8_tv.setText("" + data);
                        } else {
                            ir8_tv.setText("data error");
                        }
                        break;
                }
            }
        });

    }

    private void initView() {
        ir1_tv = (TextView) findViewById(R.id.ir1_tv);
        ir2_tv = (TextView) findViewById(R.id.ir2_tv);
        ir3_tv = (TextView) findViewById(R.id.ir3_tv);
        ir4_tv = (TextView) findViewById(R.id.ir4_tv);
        ir5_tv = (TextView) findViewById(R.id.ir5_tv);
        ir6_tv = (TextView) findViewById(R.id.ir6_tv);
        ir7_tv = (TextView) findViewById(R.id.ir7_tv);
        ir8_tv = (TextView) findViewById(R.id.ir8_tv);
        d1_tv = (TextView) findViewById(R.id.d1_tv);
        d2_tv = (TextView) findViewById(R.id.d2_tv);
        d3_tv = (TextView) findViewById(R.id.d3_tv);
        d4_tv = (TextView) findViewById(R.id.d4_tv);
        d5_tv = (TextView) findViewById(R.id.d5_tv);
        d6_tv = (TextView) findViewById(R.id.d6_tv);
        d7_tv = (TextView) findViewById(R.id.d7_tv);
        d8_tv = (TextView) findViewById(R.id.d8_tv);
        d9_tv = (TextView) findViewById(R.id.d9_tv);
    }

    private byte bitSet(int bit, int bitValue) {
        if (bit == 0) {
            if (bitValue == 0) {     //用与
                initByte &= 254;
            } else {        //或
                initByte |= 1;
            }
        }
        if (bit == 1) {
            if (bitValue == 0) {
                initByte &= 253;
            } else {
                initByte |= 2;
            }
        }
        if (bit == 2) {
            if (bitValue == 0) {
                initByte &= 251;
            } else {
                initByte |= 4;
            }
        }
        if (bit == 3) {
            if (bitValue == 0) {
                initByte &= 247;
            } else {
                initByte |= 8;
            }
        }
        if (bit == 4) {
            if (bitValue == 0) {
                initByte &= 239;
            } else {
                initByte |= 16;
            }
        }
        if (bit == 5) {
            if (bitValue == 0) {
                initByte &= 223;
            } else {
                initByte |= 32;
            }
        }
        if (bit == 6) {
            if (bitValue == 0) {
                initByte &= 191;
            } else {
                initByte |= 64;
            }
        }
        if (bit == 7) {
            if (bitValue == 0) {
                initByte &= 127;
            } else {
                initByte |= 128;
            }
        }
        return initByte;
    }

    @Override
    protected void onMainServiceConnected() {
        myHandler.sendEmptyMessage(1);
    }

    private static class MyHandler extends Handler {
        private WeakReference<UltrasonicActivity> weakReference;

        public MyHandler(UltrasonicActivity ultrasonicActivity) {
            weakReference = new WeakReference<>(ultrasonicActivity);
        }

        @Override
        public void handleMessage(Message msg) {
            super.handleMessage(msg);
            UltrasonicActivity ultrasonicActivity = weakReference.get();
            if (ultrasonicActivity != null) {
                switch (msg.what) {
                    case 1:
                        ultrasonicActivity.hardWareManager.queryUltronicData();
                        sendEmptyMessageDelayed(1, 200);
                        break;
                }
            }
        }
    }

    @Override
    protected void onStop() {
        super.onStop();
        myHandler.removeCallbacksAndMessages(null);
    }
}
