package com.sanbot.librarydemod;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentManager;
import android.support.v4.app.FragmentTransaction;
import android.util.Log;
import android.view.View;
import android.widget.Button;
import android.widget.TextView;

import com.sanbot.librarydemod.control.BothHandFragment;
import com.sanbot.librarydemod.control.LeftHandFragment;
import com.sanbot.librarydemod.control.RightHangFragment;
import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.finger.CombinationFingerMotion;
import com.sanbot.opensdk.function.beans.handmotion.CombinationHandMotion;
import com.sanbot.opensdk.function.unit.FingerMotionManager;
import com.sanbot.opensdk.function.unit.HandMotionManager;
import com.sanbot.opensdk.function.unit.HardWareManager;
import com.sanbot.opensdk.function.unit.interfaces.hardware.HandStatusListener;
import com.sanbot.opensdk.setting.Setting;

/**
 * xxxx.java
 * "Functional Description"
 * <p>
 * Created by Cris_Peng on 2019/5/27
 * Copyright (c) 2016 QihanCloud, Inc. All Rights Reserved.
 */
public class HandControlActivity extends TopBaseActivity implements View.OnClickListener {
    private static final String TAG = "HandControlActivity";

    private Button left_hand;
    private Button right_hand;
    private Button two_hand;
    private TextView tvHandStatus;
    private HandMotionManager handMotionManager;
    private FingerMotionManager fingerMotionManager;
    private HardWareManager hardWareManager;

    private FragmentManager manager;
    private Fragment leftHandFragment;
    private Fragment rightHandFragment;
    private Fragment bothHandFragment;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        getWindow().setFlags(128, 128);
        super.onCreate(savedInstanceState);
        register(MainActivity.class);
        setBodyView2(R.layout.activity_controlhand);
        initView();
        initData();
    }


    private void initView() {
        left_hand = findViewById(R.id.left_hand);
        left_hand.setOnClickListener(this);
        right_hand = findViewById(R.id.right_hand);
        right_hand.setOnClickListener(this);
        two_hand = findViewById(R.id.two_hand);
        two_hand.setOnClickListener(this);
        tvHandStatus = findViewById(R.id.hand_status);
        findViewById(R.id.finger_1).setOnClickListener(this);
        findViewById(R.id.finger_2).setOnClickListener(this);
        findViewById(R.id.finger_3).setOnClickListener(this);
        findViewById(R.id.finger_4).setOnClickListener(this);
        findViewById(R.id.finger_5).setOnClickListener(this);
        findViewById(R.id.finger_7).setOnClickListener(this);
        findViewById(R.id.finger_8).setOnClickListener(this);
        findViewById(R.id.finger_9).setOnClickListener(this);
        findViewById(R.id.finger_10).setOnClickListener(this);
        findViewById(R.id.finger_11).setOnClickListener(this);
    }

    private void initData() {
        handMotionManager = (HandMotionManager) getUnitManager(FuncConstant.HANDMOTION_MANAGER);
        fingerMotionManager = (FingerMotionManager) getUnitManager(FuncConstant.FINGER_MANAGER);
        hardWareManager = (HardWareManager) getUnitManager(FuncConstant.HARDWARE_MANAGER);
        hardWareManager.setOnHareWareListener(new HandStatusListener() {
            @Override
            public void onHandStatus(byte part, byte status) {
                String handPart = getResources().getStringArray(R.array.hand_parts)[part];
                String handState = getResources().getStringArray(R.array.hand_status)[part];
                tvHandStatus.setText(handPart + ":" + handState);
            }
        });

        manager = getSupportFragmentManager();
        leftHandFragment = new LeftHandFragment();
        rightHandFragment = new RightHangFragment();
        bothHandFragment = new BothHandFragment();
        showFragment(leftHandFragment);
    }

    private int whichHand = 1;

    private Fragment currentFragment;

    /**
     * 展示Fragment
     */
    private void showFragment(Fragment fragment) {
        if (currentFragment != fragment) {
            FragmentTransaction transaction = manager.beginTransaction();
            if (null != currentFragment)
                transaction.hide(currentFragment);
            currentFragment = fragment;
            if (!fragment.isAdded()) {
                transaction.add(R.id.contain_fl, fragment).show(fragment).commit();
            } else {
                transaction.show(fragment).commit();
            }
        }
    }

    @Override
    public void onClick(View v) {
        switch (v.getId()) {
            case R.id.left_hand:
                whichHand = 1;
                left_hand.setBackgroundResource(R.drawable.add_new_bg);
                right_hand.setBackgroundResource(R.drawable.stop_bg);
                two_hand.setBackgroundResource(R.drawable.stop_bg);
                showFragment(leftHandFragment);
                break;
            case R.id.right_hand:
                whichHand = 2;
                right_hand.setBackgroundResource(R.drawable.add_new_bg);
                left_hand.setBackgroundResource(R.drawable.stop_bg);
                two_hand.setBackgroundResource(R.drawable.stop_bg);
                showFragment(rightHandFragment);
                break;
            case R.id.two_hand:
                whichHand = 3;
                two_hand.setBackgroundResource(R.drawable.add_new_bg);
                right_hand.setBackgroundResource(R.drawable.stop_bg);
                left_hand.setBackgroundResource(R.drawable.stop_bg);
                showFragment(bothHandFragment);
                break;
            case R.id.finger_1:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_STRETCH_FINGER,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_2:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_OK,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_3:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_FIST,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_4:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_V,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_5:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_THUMB,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_7:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_LITTLE_FINGER,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_8:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_LOVE,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_9:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_NUM1,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_10:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_NUM3,
                        CombinationFingerMotion.ACTION_START));
                break;
            case R.id.finger_11:
                fingerMotionManager.doCombinationMotion(new CombinationFingerMotion((byte) whichHand, CombinationFingerMotion.MOTION_NUM4,
                        CombinationFingerMotion.ACTION_START));
                break;
        }
    }

    public void sendHandCommand(int whichHand, int handAction) {
        if (isAllowArmMove()) {
            handMotionManager.doCombinationMotionReset(new CombinationHandMotion((byte) whichHand, (byte) handAction, CombinationHandMotion.ACTION_START), true);
        } else {
            Log.i("", "sendHandCommand: ");
        }
    }

    private boolean isAllowArmMove() {
        String value = Setting.getString(getContentResolver(), ROBOT_ARM_ACTION_B);
        return value.equals("true");
    }

    public static final String ROBOT_ARM_ACTION_B = "robot_arm_action";

    @Override
    protected void onMainServiceConnected() {

    }


}
