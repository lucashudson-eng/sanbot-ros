package com.sanbot.librarydemod;

import android.content.Intent;
import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.Button;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Spinner;
import android.widget.TextView;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.ElectronicLock;
import com.sanbot.opensdk.function.beans.LED;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.interfaces.hardware.GyroscopeListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.IWakeSignalListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.PIRListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.TouchSensorListener;
import com.sanbot.opensdk.function.unit.interfaces.hardware.VoiceLocateListener;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;

/**
 * className: HardwareControlActivity
 * function: 硬件控制
 * <p/>
 * create at 2017/5/22 16:37
 *
 * @author gangpeng
 */

public class HardwareControlActivity extends TopBaseActivity {

    private static final String TAG = "HardwareControlActivity";
    @BindView(R.id.rb_one)
    RadioButton rbOne;
    @BindView(R.id.rb_two)
    RadioButton rbTwo;
    @BindView(R.id.rb_three)
    RadioButton rbThree;
    @BindView(R.id.rg_level)
    RadioGroup rgLevel;
    @BindView(R.id.tv_open_white_light)
    TextView tvOpenWhiteLight;
    @BindView(R.id.tv_close_white_light)
    TextView tvCloseWhiteLight;
    @BindView(R.id.tv_open_led_light)
    TextView tvOpenLedLight;
    @BindView(R.id.tv_close_led_light)
    TextView tvCloseLedLight;
    @BindView(R.id.tv_open_black_filter)
    TextView tvOpenBlackFilter;
    @BindView(R.id.tv_close_black_filter)
    TextView tvCloseBlackFilter;
    @BindView(R.id.tv_touch_sensor)
    TextView tvTouchSensor;
    @BindView(R.id.tv_voice_locate_angle)
    TextView tvVoiceLocateAngle;
    @BindView(R.id.tv_pir_sensor)
    TextView tvPirSensor;
    @BindView(R.id.btn_ir_sensor)
    Button btnIrSensor;
    @BindView(R.id.tv_gyroscope_result)
    TextView tvGyroscopeResult;
    @BindView(R.id.sv_led_part)
    Spinner svLedPart;
    @BindView(R.id.sv_led_mode)
    Spinner svLedMode;
    @BindView(R.id.et_led_duration)
    EditText etLedDuration;
    @BindView(R.id.et_led_random)
    EditText etLedRandom;
    @BindView(R.id.sv_electronic_locks)
    Spinner svElectronicLocks;
    @BindView(R.id.tv_wake_signal)
    TextView tvWakeSignal;

    private HardWareManager hardWareManager;

    /**
     * led灯
     */
    private byte[] ledPart = {LED.PART_ALL_HAND, LED.PART_ALL_HEAD, LED.PART_LEFT_HAND, LED.PART_RIGHT_HAND, LED.PART_LEFT_HEAD, LED.PART_RIGHT_HEAD};
    private byte curLedPart;

    /**
     * led模式
     */
    private byte[] ledMode = {LED.MODE_WHITE, LED.MODE_RED, LED.MODE_GREEN, LED.MODE_PINK, LED.MODE_PURPLE, LED.MODE_BLUE
            , LED.MODE_YELLOW, LED.MODE_FLICKER_WHITE, LED.MODE_FLICKER_RED, LED.MODE_FLICKER_GREEN, LED.MODE_FLICKER_PINK
            , LED.MODE_FLICKER_PURPLE, LED.MODE_FLICKER_BLUE, LED.MODE_FLICKER_YELLOW, LED.MODE_FLICKER_RANDOM};
    private byte curLedMode;

    /**
     * 电子锁
     */
    private byte[] electronicLocks = {ElectronicLock.PART_BACK, ElectronicLock.PART_FRONT};
    private byte curElectronicLock;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        register(HardwareControlActivity.class);
        //屏幕常亮
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        super.onCreate(savedInstanceState);
        setContentView(R.layout.actviity_hardware_control);
        ButterKnife.bind(this);
        //初始化变量
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        initListener();
    }

    /**
     * 初始化监听器
     */
    private void initListener() {
        //白光灯亮度控制，只能在亮的时候控制
        rgLevel.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup group, int checkedId) {
                if (checkedId == R.id.rb_one) {
                    hardWareManager.setWhiteLightLevel(1);
                } else if (checkedId == R.id.rb_two) {
                    hardWareManager.setWhiteLightLevel(2);
                } else if (checkedId == R.id.rb_three) {
                    hardWareManager.setWhiteLightLevel(3);
                }
            }
        });
        //触摸事件回调
        hardWareManager.setOnHareWareListener(
                new TouchSensorListener() {
                    @Override
                    public void onTouch(int part) {
                        switch (part) {
                            case 7:
                                tvTouchSensor.setText(R.string.touch_back_left);
                                break;
                            case 8:
                                tvTouchSensor.setText(R.string.touch_back_right);
                                break;
                            case 9:
                                tvTouchSensor.setText(R.string.touch_hand_left);
                                break;
                            case 10:
                                tvTouchSensor.setText(R.string.touch_hand_right);
                                break;
                            case 11:
                                tvTouchSensor.setText(R.string.touch_head_middle);
                                break;
                            case 14:
                                tvTouchSensor.setText(R.string.touch_elbow_right);
                                break;
                            case 15:
                                tvTouchSensor.setText(R.string.touch_muscle_right);
                                break;
                            case 16:
                                tvTouchSensor.setText(R.string.touch_shoulder_right);
                                break;
                            case 17:
                                tvTouchSensor.setText(R.string.touch_elbow_left);
                                break;
                            case 18:
                                tvTouchSensor.setText(R.string.touch_muscle_left);
                                break;
                            case 19:
                                tvTouchSensor.setText(R.string.touch_shoulder_left);
                                break;
                        }
                    }
                }
        );

        //声源定位回调
        hardWareManager.setOnHareWareListener(new VoiceLocateListener() {
            @Override
            public void voiceLocateResult(int angle) {
                tvVoiceLocateAngle.setText(getString(R.string.voice_locate_angle) + angle);
            }
        });

        //pir回调
        hardWareManager.setOnHareWareListener(new PIRListener() {
            @Override
            public void onPIRCheckResult(boolean isChecked, int part) {
                if (isChecked) {
                    tvPirSensor.setText(part == 1 ? R.string.trigger_front_pir : R.string.trigger_back_pir);
                } else {
                    tvPirSensor.setText(part == 1 ? getString(R.string.off_front_pir) : getString(R.string.off_back_pir));
                }
            }
        });
        //陀螺仪回调
        hardWareManager.setOnHareWareListener(new GyroscopeListener() {
            @Override
            public void gyroscopeData(float v, float v1, float v2) {
                Log.d(TAG, "first: " + v + ", second: " + v1 + ", third: " + v2);
                tvGyroscopeResult.setText(String.format(getResources().getString(R.string.gyroscope_result), v, v1, v2));
            }
        });
        //单片机唤醒信号回调
        hardWareManager.setOnHareWareListener(new IWakeSignalListener() {
            @Override
            public void onWakeSignal() {
                tvWakeSignal.setText(getString(R.string.got_wake_signal));
            }
        });

        //设置led模式
        svLedMode.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                curLedMode = ledMode[position];
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                curLedMode = ledMode[0];
            }
        });
        //设置led位置
        svLedPart.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                curLedPart = ledPart[position];
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                curLedPart = ledPart[0];
            }
        });
        //设置选择的电子锁种类
        svElectronicLocks.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                curElectronicLock = electronicLocks[position];
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {

            }
        });
    }

    @Override
    protected void onMainServiceConnected() {

    }

    @OnClick({R.id.tv_open_white_light, R.id.tv_close_white_light, R.id.tv_open_led_light, R.id.tv_close_led_light
            , R.id.tv_open_black_filter, R.id.tv_close_black_filter, R.id.btn_ir_sensor, R.id.btn_ultrasonic_sensor,
            R.id.query_front_pir, R.id.query_back_pir, R.id.tv_open_electronic_lock, R.id.tv_close_electronic_lock})
    public void onViewClicked(View view) {
        switch (view.getId()) {
            //打开白光灯
            case R.id.tv_open_white_light:
                hardWareManager.switchWhiteLight(true);
                break;
            //关闭白光灯
            case R.id.tv_close_white_light:
                hardWareManager.switchWhiteLight(false);
                break;
            //打开led灯
            case R.id.tv_open_led_light:
                byte duration;
                byte random;
                try {
                    duration = Byte.parseByte(etLedDuration.getText().toString());
                    random = Byte.parseByte(etLedRandom.getText().toString());
                } catch (Exception e) {
                    e.printStackTrace();
                    duration = 1;
                    random = 1;
                }
                hardWareManager.setLED(new LED(curLedPart, curLedMode, duration, random));
                break;
            //关闭led灯
            case R.id.tv_close_led_light:
                hardWareManager.setLED(new LED(curLedPart, LED.MODE_CLOSE, (byte) 1, (byte) 1));
                break;
            case R.id.tv_open_black_filter:
                hardWareManager.switchBlackLineFilter(true);
                break;
            case R.id.tv_close_black_filter:
                hardWareManager.switchBlackLineFilter(false);
                break;
            case R.id.btn_ir_sensor:
                Intent intent = new Intent(HardwareControlActivity.this, IRSensorActivity.class);
                startActivity(intent);
                break;
            case R.id.btn_ultrasonic_sensor:
                intent = new Intent(HardwareControlActivity.this, UltrasonicActivity.class);
                startActivity(intent);
                break;
            case R.id.query_back_pir://后pir状态查询
                hardWareManager.queryPirStatus(2);
                break;
            case R.id.query_front_pir://前pir状态查询
                hardWareManager.queryPirStatus(1);
                break;
            case R.id.tv_close_electronic_lock:
                hardWareManager.setElectronicLock(new ElectronicLock(curElectronicLock, ElectronicLock.STATUS_CLOSE));
                break;
            case R.id.tv_open_electronic_lock:
                hardWareManager.setElectronicLock(new ElectronicLock(curElectronicLock, ElectronicLock.STATUS_OPEN));
                break;
        }
    }
}
