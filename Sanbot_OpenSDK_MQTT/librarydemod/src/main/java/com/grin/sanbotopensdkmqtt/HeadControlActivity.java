package com.grin.sanbotopensdkmqtt;

import android.os.Bundle;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Spinner;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.headmotion.AbsoluteAngleHeadMotion;
import com.sanbot.opensdk.function.beans.headmotion.RelativeAngleHeadMotion;
import com.sanbot.opensdk.function.unit.HeadMotionManager;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;

/**
 * className: HeadControlActivity
 * function: 头部控制
 * <p/>
 * create at 2017/5/23 17:37
 *
 * @author gangpeng
 */

public class HeadControlActivity extends TopBaseActivity {

    private static final String TAG = "HeadControlActivity";
    @BindView(R.id.sv_head_relative_action)
    Spinner svHeadRelativeAction;
    @BindView(R.id.et_head_relative_angle)
    EditText etHeadRelativeAngle;
    @BindView(R.id.et_head_absolute_angle)
    EditText etHeadAbsoluteAngle;

    @BindView(R.id.head_rg)
    RadioGroup head_rg;
    @BindView(R.id.neck_rb)
    RadioButton neck_rb;

    private HeadMotionManager headMotionManager;
    /**
     * 当前相对运动方向
     */
    private byte curRelativeSpeed = 50;
    /**
     * 当前绝对运动方向
     */
    private byte curAbsoluteSpeed = 50;

    private byte[] releateMode = new byte[]{RelativeAngleHeadMotion.ACTION_UP, RelativeAngleHeadMotion.ACTION_DOWN, RelativeAngleHeadMotion.ACTION_LEFT, RelativeAngleHeadMotion.ACTION_RIGHT};

    @Override
    public void onCreate(Bundle savedInstanceState) {
        register(HeadControlActivity.class);
        //屏幕常亮
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_head_control);
        ButterKnife.bind(this);
        //初始化变量
        headMotionManager = (HeadMotionManager) getUnitManager(FuncConstant.HEADMOTION_MANAGER);
        initListener();
    }

    private int currentMotor = 1;
    private byte currentRelativeMode = releateMode[0];

    /**
     * 初始化监听器
     */
    private void initListener() {
        head_rg.setOnCheckedChangeListener(new RadioGroup.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(RadioGroup radioGroup, int i) {
                switch (i) {
                    case R.id.horizontal_motor:
                        currentMotor = 3;
                        break;
                    case R.id.vertical_motor:
                        currentMotor = 2;
                        break;
                    case R.id.neck_rb:
                        currentMotor = 1;
                        break;
                }
            }
        });
        neck_rb.setChecked(true);
        //相对运动方向下拉框
        svHeadRelativeAction.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                currentRelativeMode = releateMode[position];
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                currentRelativeMode = releateMode[0];
            }
        });
    }

    @Override
    protected void onMainServiceConnected() {

    }

    @OnClick({R.id.tv_head_relative_start, R.id.tv_head_relative_end, R.id.tv_head_absolute_start})
    public void onViewClicked(View view) {
        switch (view.getId()) {
            //开始相对运动
            case R.id.tv_head_relative_start:
                int angle;
                try {
                    angle = Integer.parseInt(etHeadRelativeAngle.getText().toString());
                } catch (Exception e) {
                    e.printStackTrace();
                    Log.e("HeadControlActivity:", "please input number");
                    angle = 0;
                }
                RelativeAngleHeadMotion relativeAngleHeadMotion = new RelativeAngleHeadMotion((byte) currentMotor, currentRelativeMode, curRelativeSpeed, angle);
                headMotionManager.doRelativeAngleMotion(relativeAngleHeadMotion);
                break;
            //结束相对运动
            case R.id.tv_head_relative_end:
                headMotionManager.doStopMotion();
                break;
            //开始绝对运动
            case R.id.tv_head_absolute_start:
                try {
                    angle = Integer.parseInt(etHeadAbsoluteAngle.getText().toString());
                } catch (Exception e) {
                    e.printStackTrace();
                    Log.e("HeadControlActivity:", "please input number");
                    angle = 90;
                }
                AbsoluteAngleHeadMotion absoluteAngleHeadMotion = new AbsoluteAngleHeadMotion((byte) currentMotor, AbsoluteAngleHeadMotion.ACTION_START, curAbsoluteSpeed, angle);
                headMotionManager.doAbsoluteAngleMotion(absoluteAngleHeadMotion);
                break;
        }
    }
}
