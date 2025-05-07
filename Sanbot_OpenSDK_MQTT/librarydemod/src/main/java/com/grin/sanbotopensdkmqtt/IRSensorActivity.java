package com.grin.sanbotopensdkmqtt;

import android.graphics.Color;
import android.graphics.drawable.GradientDrawable;
import android.os.Bundle;
import android.view.WindowManager;
import android.widget.TextView;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.interfaces.hardware.InfrareListener;

import butterknife.BindView;
import butterknife.ButterKnife;

/**
 * className: IRSensorActivity
 * function: IR sensor
 * <p/>
 * create at 2017/5/23 11:16
 *
 * @author gangpeng
 */
public class IRSensorActivity extends TopBaseActivity {

    public HardWareManager hardWareManager;

    @BindView(R.id.ir1_tv)
    public TextView ir1_tv;
    @BindView(R.id.ir3_tv)
    public TextView ir3_tv;
    @BindView(R.id.ir2_tv)
    public TextView ir2_tv;
    @BindView(R.id.ir4_tv)
    public TextView ir4_tv;
    @BindView(R.id.ir5_tv)
    public TextView ir5_tv;
    @BindView(R.id.ir6_tv)
    public TextView ir6_tv;
    @BindView(R.id.ir7_tv)
    public TextView ir7_tv;
    @BindView(R.id.ir8_tv)
    public TextView ir8_tv;
    @BindView(R.id.ir9_tv)
    public TextView ir9_tv;
    @BindView(R.id.ir10_tv)
    public TextView ir10_tv;
    @BindView(R.id.ir11_tv)
    public TextView ir11_tv;
    @BindView(R.id.ir12_tv)
    public TextView ir12_tv;
    @BindView(R.id.ir13_tv)
    public TextView ir13_tv;
    @BindView(R.id.ir14_tv)
    public TextView ir14_tv;
    @BindView(R.id.ir16_tv)
    public TextView ir16_tv;
    @BindView(R.id.ir17_tv)
    public TextView ir17_tv;
    @BindView(R.id.ir15_tv)
    public TextView ir15_tv;
    @BindView(R.id.ir18_tv)
    public TextView ir18_tv;
    @BindView(R.id.ir19_tv)
    public TextView ir19_tv;
    @BindView(R.id.ir20_tv)
    public TextView ir20_tv;
    @BindView(R.id.ir21_tv)
    public TextView ir21_tv;
    @BindView(R.id.ir22_tv)
    public TextView ir22_tv;
    @BindView(R.id.ir23_tv)
    public TextView ir23_tv;
    @BindView(R.id.ir24_tv)
    public TextView ir24_tv;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        register(IRSensorActivity.class);
        //屏幕常亮
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        super.onCreate(savedInstanceState);
        setBodyView(R.layout.activity_ir);
        ButterKnife.bind(this);
        //设置顶部状态栏颜色
        GradientDrawable topDrawable = new GradientDrawable(GradientDrawable.Orientation.LEFT_RIGHT
                , new int[]{Color.parseColor("#2B6DF8"), Color.parseColor("#00A2ED")});
        setHeadBackground(topDrawable);
        //初始化变量
        hardWareManager = (HardWareManager) this.getUnitManager(FuncConstant.HARDWARE_MANAGER);
        //初始化数据
        initData();
    }

    /**
     * 初始化数据
     */
    public void initData() {
        hardWareManager.setOnHareWareListener(new InfrareListener() {
            @Override
            public void infrareDistance(int part, int distance) {
                switch (part) {
                    case 1:
                        ir1_tv.setText(String.valueOf(distance));
                        break;
                    case 2:
                        ir2_tv.setText(String.valueOf(distance));
                        break;
                    case 3:
                        ir3_tv.setText(String.valueOf(distance));
                        break;
                    case 4:
                        ir4_tv.setText(String.valueOf(distance));
                        break;
                    case 5:
                        ir5_tv.setText(String.valueOf(distance));
                        break;
                    case 6:
                        ir6_tv.setText(String.valueOf(distance));
                        break;
                    case 7:
                        ir7_tv.setText(String.valueOf(distance));
                        break;
                    case 8:
                        ir8_tv.setText(String.valueOf(distance));
                        break;
                    case 9:
                        ir9_tv.setText(String.valueOf(distance));
                        break;
                    case 10:
                        ir10_tv.setText(String.valueOf(distance));
                        break;
                    case 11:
                        ir11_tv.setText(String.valueOf(distance));
                        break;
                    case 12:
                        ir12_tv.setText(String.valueOf(distance));
                        break;
                    case 13:
                        ir13_tv.setText(String.valueOf(distance));
                        break;
                    case 14:
                        ir14_tv.setText(String.valueOf(distance));
                        break;
                    case 15:
                        ir15_tv.setText(String.valueOf(distance));
                        break;
                    case 16:
                        ir16_tv.setText(String.valueOf(distance));
                        break;
                    case 17:
                        ir17_tv.setText(String.valueOf(distance));
                        break;
                    case 18:
                        ir18_tv.setText(String.valueOf(distance));
                        break;
                    case 19:
                        ir19_tv.setText(String.valueOf(distance));
                        break;
                    case 20:
                        ir20_tv.setText(String.valueOf(distance));
                        break;
                    case 21:
                        ir21_tv.setText(String.valueOf(distance));
                        break;
                    case 22:
                        ir22_tv.setText(String.valueOf(distance));
                        break;
                    case 23:
                        ir23_tv.setText(String.valueOf(distance));
                        break;
                    case 24:
                        ir24_tv.setText(String.valueOf(distance));
                        break;
                }
            }
        });
    }

    @Override
    protected void onMainServiceConnected() {

    }
}
