package com.sanbot.librarydemod;

import android.os.Bundle;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.EditText;
import android.widget.Spinner;
import android.widget.TextView;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.wheelmotion.DistanceWheelMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.NoAngleWheelMotion;
import com.sanbot.opensdk.function.beans.wheelmotion.RelativeAngleWheelMotion;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.WheelMotionManager;
import com.sanbot.opensdk.function.unit.interfaces.hardware.WheelStatusListener;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;

/**
 * className: WheelControlActivity
 * function: 轮子控制
 * <p/>
 * create at 2017/5/24 15:37
 *
 * @author gangpeng
 */

public class WheelControlActivity extends TopBaseActivity {


    @BindView(R.id.sv_wheel_no_angle_action)
    Spinner svWheelNoAngleAction;
    @BindView(R.id.et_wheel_no_angle_speed)
    EditText etWheelNoAngleSpeed;
    @BindView(R.id.et_wheel_no_angle_duration)
    EditText etWheelNoAngleDuration;
    @BindView(R.id.et_wheel_no_angle_angle)
    EditText etWheelNoAngleAngle;
    @BindView(R.id.tv_wheel_no_angle_start)
    TextView tvWheelNoAngleStart;
    @BindView(R.id.tv_wheel_no_angle_end_turn)
    TextView tvWheelNoAngleEndTurn;
    @BindView(R.id.tv_wheel_no_angle_end_run)
    TextView tvWheelNoAngleEndRun;
    @BindView(R.id.sv_wheel_relative_action)
    Spinner svWheelRelativeAction;
    @BindView(R.id.et_wheel_relative_speed)
    EditText etWheelRelativeSpeed;
    @BindView(R.id.et_wheel_relative_angle)
    EditText etWheelRelativeAngle;
    @BindView(R.id.tv_wheel_relative_start)
    TextView tvWheelRelativeStart;
    @BindView(R.id.tv_wheel_relative_end)
    TextView tvWheelRelativeEnd;
    @BindView(R.id.et_wheel_distance_speed)
    EditText etWheelDistanceSpeed;
    @BindView(R.id.et_wheel_distance)
    EditText etWheelDistance;
    @BindView(R.id.tv_wheel_distance_start)
    TextView tvWheelDistanceStart;
    @BindView(R.id.tv_wheel_distance_end)
    TextView tvWheelDistanceEnd;
    @BindView(R.id.show_status_tv)
    TextView tvWheelStatus;

    private WheelMotionManager wheelMotionManager;
    private HardWareManager hardWareManager;

    /**
     * 无角度运动action
     */
    private byte[] noAngleAction = {NoAngleWheelMotion.ACTION_FORWARD,//    前进
            NoAngleWheelMotion.ACTION_BACK,//  后退
            NoAngleWheelMotion.ACTION_LEFT,//   左走
            NoAngleWheelMotion.ACTION_RIGHT,//   右走
            NoAngleWheelMotion.ACTION_LEFT_FORWARD,//   左前
            NoAngleWheelMotion.ACTION_RIGHT_FORWARD,//   右前
            NoAngleWheelMotion.ACTION_LEFT_BACK,//  左后
            NoAngleWheelMotion.ACTION_RIGHT_BACK,//    右后
            NoAngleWheelMotion.ACTION_RESET,//    动作归位
            NoAngleWheelMotion.ACTION_LEFT_TRANSLATION,//    左平移
            NoAngleWheelMotion.ACTION_RIGHT_TRANSLATION,//    右平移
            NoAngleWheelMotion.ACTION_TURN_LEFT,//    左转弯
            NoAngleWheelMotion.ACTION_TURN_RIGHT,//   右转弯
            NoAngleWheelMotion.ACTION_STOP_TURN,//   停止转弯
            NoAngleWheelMotion.ACTION_TURN_LEFT_BY_ANGLE,//   左转
            NoAngleWheelMotion.ACTION_TURN_RIGHT_BY_ANGEL,//    右转
            NoAngleWheelMotion.ACTION_STOP,//   停止
    };

    private byte curNoAngleAction;

    /**
     * 相对运动action
     */
    private byte[] relativeAction = {RelativeAngleWheelMotion.ACTION_TURN_LEFT, RelativeAngleWheelMotion.ACTION_TURN_RIGHT};
    private byte curRelativeAction;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        register(WheelControlActivity.class);
        //屏幕常亮
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_wheel_control);
        ButterKnife.bind(this);
        wheelMotionManager = (WheelMotionManager) getUnitManager(FuncConstant.WHEELMOTION_MANAGER);
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        initListener();
    }

    /**
     * 初始化监听器
     */
    private void initListener() {
        //无角度运动action
        svWheelNoAngleAction.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                curNoAngleAction = noAngleAction[position];
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                curNoAngleAction = noAngleAction[0];
            }
        });
        //相对运动action
        svWheelRelativeAction.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> parent, View view, int position, long id) {
                curRelativeAction = relativeAction[position];
            }

            @Override
            public void onNothingSelected(AdapterView<?> parent) {
                curRelativeAction = relativeAction[0];
            }
        });
        hardWareManager.setOnHareWareListener(new WheelStatusListener() {
            @Override
            public void onWheelStatus(int i) {
                if (i == 0) {
                    tvWheelStatus.setText(getString(R.string.wheel_state_stop));
                } else {
                    tvWheelStatus.setText(getString(R.string.wheel_state_running));
                }
            }
        });
    }

    @Override
    protected void onMainServiceConnected() {

    }

    @OnClick({R.id.tv_wheel_no_angle_start, R.id.tv_wheel_no_angle_end_turn, R.id.tv_wheel_no_angle_end_run
            , R.id.tv_wheel_relative_start, R.id.tv_wheel_relative_end, R.id.tv_wheel_distance_start, R.id.tv_wheel_distance_end})
    public void onViewClicked(View view) {
        switch (view.getId()) {
            //无角度运动
            case R.id.tv_wheel_no_angle_start:
                int speed;
                int duration;
                int angle1;
                byte state;
                try {
                    speed = Integer.parseInt(etWheelNoAngleSpeed.getText().toString());
                    duration = Integer.parseInt(etWheelNoAngleDuration.getText().toString());
                    angle1 = Integer.parseInt(etWheelNoAngleAngle.getText().toString());
                } catch (Exception e) {
                    e.printStackTrace();
                    speed = 5;
                    angle1 = 100;
                    duration = 0;
                }

                if (duration == 0) {
                    state = NoAngleWheelMotion.STATUS_KEEP;
                } else {
                    state = NoAngleWheelMotion.STATUS_TIME;
                }
                NoAngleWheelMotion noAngleWheelMotion = new NoAngleWheelMotion(curNoAngleAction, speed, angle1, duration, state);
                wheelMotionManager.doNoAngleMotion(noAngleWheelMotion);
                break;
            //无角度停止转圈
            case R.id.tv_wheel_no_angle_end_turn:
                noAngleWheelMotion = new NoAngleWheelMotion(NoAngleWheelMotion.ACTION_STOP_TURN, 1, 0, 1, NoAngleWheelMotion.STATUS_TIME);
                wheelMotionManager.doNoAngleMotion(noAngleWheelMotion);
                break;
            //无角度停止行走
            case R.id.tv_wheel_no_angle_end_run:
                noAngleWheelMotion = new NoAngleWheelMotion(NoAngleWheelMotion.ACTION_STOP, 1, 0, 1, NoAngleWheelMotion.STATUS_TIME);
                wheelMotionManager.doNoAngleMotion(noAngleWheelMotion);
                break;
            //相对角度运动
            case R.id.tv_wheel_relative_start:
                int angle;
                try {
                    speed = Integer.parseInt(etWheelRelativeSpeed.getText().toString());
                    angle = Integer.parseInt(etWheelRelativeAngle.getText().toString());
                } catch (Exception e) {
                    e.printStackTrace();
                    speed = 5;
                    angle = 90;
                }
                RelativeAngleWheelMotion relativeAngleWheelMotion = new RelativeAngleWheelMotion(curRelativeAction, speed, angle);
                wheelMotionManager.doRelativeAngleMotion(relativeAngleWheelMotion);
                break;
            //停止相对角度运动
            case R.id.tv_wheel_relative_end:
                relativeAngleWheelMotion = new RelativeAngleWheelMotion(RelativeAngleWheelMotion.ACTION_TURN_STOP, 1, 1);
                wheelMotionManager.doRelativeAngleMotion(relativeAngleWheelMotion);
                break;
            //距离运动
            case R.id.tv_wheel_distance_start:
                int distance;
                try {
                    speed = Integer.parseInt(etWheelDistanceSpeed.getText().toString());
                    distance = Integer.parseInt(etWheelDistance.getText().toString());
                } catch (Exception e) {
                    e.printStackTrace();
                    speed = 5;
                    distance = 10;
                }
                DistanceWheelMotion distanceWheelMotion = new DistanceWheelMotion(DistanceWheelMotion.ACTION_FORWARD, speed, distance);
                wheelMotionManager.doDistanceMotion(distanceWheelMotion);
                break;
            //结束距离运动
            case R.id.tv_wheel_distance_end:
                distanceWheelMotion = new DistanceWheelMotion(DistanceWheelMotion.ACTION_STOP, 1, 1);
                wheelMotionManager.doDistanceMotion(distanceWheelMotion);
                break;
            default:
                break;
        }
    }
}
