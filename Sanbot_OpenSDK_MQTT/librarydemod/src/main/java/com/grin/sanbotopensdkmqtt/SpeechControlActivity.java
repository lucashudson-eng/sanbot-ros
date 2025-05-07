package com.grin.sanbotopensdkmqtt;

import android.os.Bundle;
import android.support.annotation.NonNull;
import android.text.TextUtils;
import android.util.Log;
import android.view.View;
import android.view.WindowManager;
import android.widget.AdapterView;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Spinner;
import android.widget.TextView;

import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.function.beans.SpeakOption;
import com.sanbot.opensdk.function.beans.WakeUpOption;
import com.sanbot.opensdk.function.beans.speech.Grammar;
import com.sanbot.opensdk.function.beans.speech.RecognizeTextBean;
import com.sanbot.opensdk.function.beans.speech.SpeakStatus;
import com.sanbot.opensdk.function.unit.SpeechManager;
import com.sanbot.opensdk.function.unit.interfaces.speech.RecognizeListener;
import com.sanbot.opensdk.function.unit.interfaces.speech.SpeakListener;
import com.sanbot.opensdk.function.unit.interfaces.speech.WakenListener;

import butterknife.BindView;
import butterknife.ButterKnife;
import butterknife.OnClick;

/**
 * className: SpeechControlActivity
 * function: 语音管理
 * 1. 语音识别拦截，需要在AndroidMainfest中配置RECOGNIZE_MODE为1，且回调方法返回true
 * 2. 语音合成时，英文语言无法识别出中文
 * <p/>
 * create at 2017/5/22 11:37
 *
 * @author gangpeng
 */

public class SpeechControlActivity extends TopBaseActivity {

    private static final String TAG = "SpeechControlActivity";

    @BindView(R.id.et_text)
    EditText etText;
    @BindView(R.id.rb_english)
    RadioButton rbEnglish;
    @BindView(R.id.rb_chinese)
    RadioButton rbChinese;
    @BindView(R.id.rg_lang)
    RadioGroup rgLang;
    @BindView(R.id.et_speed)
    EditText etSpeed;
    @BindView(R.id.et_tone)
    EditText etTone;
    @BindView(R.id.tv_speech_synthesize_start)
    TextView tvSpeechSynthesizeStart;
    @BindView(R.id.tv_speech_synthesize_pause)
    TextView tvSpeechSynthesizePause;
    @BindView(R.id.tv_speech_synthesize_continue)
    TextView tvSpeechSynthesizeContinue;
    @BindView(R.id.tv_speech_synthesize_stop)
    TextView tvSpeechSynthesizeStop;
    @BindView(R.id.tv_speech_synthesize_finish)
    TextView tvSpeechSynthesizeFinish;
    @BindView(R.id.tv_speech_synthesize_progress)
    TextView tvSpeechSynthesizeProgress;
    @BindView(R.id.tv_speech_sleep)
    TextView tvSpeechSleep;
    @BindView(R.id.tv_speech_wakeup)
    TextView tvSpeechWakeup;
    @BindView(R.id.tv_speech_status)
    TextView tvSpeechStatus;
    @BindView(R.id.tv_speech_recognize_volume)
    TextView tvSpeechRecognizeVolume;
    @BindView(R.id.tv_speech_recognize_result)
    TextView tvSpeechRecognizeResult;
    @BindView(R.id.cb_intercept_message)
    CheckBox cbInterceptMessage;
    @BindView(R.id.tv_speech_speaking)
    TextView tvSpeaking;
    @BindView(R.id.tv_speech_speaking_result)
    TextView tvSpeakingResult;
    private SpeechManager speechManager;

    @BindView(R.id.wake_option_sp)
    Spinner wake_option_sp;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        register(SpeechControlActivity.class);
        //屏幕常亮
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_speech_control);
        ButterKnife.bind(this);
        //初始化变量
        speechManager = (SpeechManager) getUnitManager(FuncConstant.SPEECH_MANAGER);

        initListener();
    }

    private String wakeupOption = "default";

    /**
     * 初始化监听器
     */
    private void initListener() {
        final String[] stringArray = getResources().getStringArray(R.array.wakeup_option);
        wake_option_sp.setOnItemSelectedListener(new AdapterView.OnItemSelectedListener() {
            @Override
            public void onItemSelected(AdapterView<?> adapterView, View view, int i, long l) {
                wakeupOption = stringArray[i];
            }

            @Override
            public void onNothingSelected(AdapterView<?> adapterView) {

            }
        });

        //设置唤醒，休眠回调
        speechManager.setOnSpeechListener(new WakenListener() {
            @Override
            public void onWakeUp() {
                tvSpeechStatus.setText(R.string.speech_wakeup_status);
            }

            @Override
            public void onSleep() {
                tvSpeechStatus.setText(R.string.speech_sleep_status);
            }

            @Override
            public void onWakeUpStatus(boolean b) {

            }
        });

        speechManager.setOnSpeechListener(new RecognizeListener() {
            @Override
            public boolean onRecognizeResult(@NonNull Grammar grammar) {
                //只有在配置了RECOGNIZE_MODE为1，且返回为true的情况下，才会拦截
                tvSpeechRecognizeResult.setText(grammar.getText());
                if (grammar.getTopic().equals("test_topic")) {
                    speechManager.startSpeak("接收到了自定义语义");
                    return true;
                }
                return cbInterceptMessage.isChecked();
            }

            @Override
            public void onRecognizeText(@NonNull RecognizeTextBean recognizeTextBean) {
                tvSpeechRecognizeResult.setText(recognizeTextBean.getText());
            }

            @Override
            public void onRecognizeVolume(int i) {
                tvSpeechRecognizeVolume.setText(String.valueOf(i));
            }

            @Override
            public void onStartRecognize() {
                Log.i(TAG, "onStartRecognize: ");
            }

            @Override
            public void onStopRecognize() {
                Log.i(TAG, "onStopRecognize: ");
            }

            @Override
            public void onError(int i, int i1) {
                Log.i(TAG, "onError: i=" + i + " i1=" + i1);
            }
        });

        //语音合成状态回调
        speechManager.setOnSpeechListener(new SpeakListener() {
            @Override
            public void onSpeakStatus(@NonNull SpeakStatus speakStatus) {
                Log.e(TAG, "" + speakStatus.getProgress());
                tvSpeechSynthesizeProgress.setText("" + speakStatus.getProgress());
            }
        });
    }

    @Override
    protected void onMainServiceConnected() {

    }

    /**
     * 处理所有的点击事件
     *
     * @param view
     */
    @OnClick({R.id.tv_speech_synthesize_start, R.id.tv_speech_synthesize_pause, R.id.tv_speech_synthesize_continue
            , R.id.tv_speech_synthesize_stop, R.id.tv_speech_sleep, R.id.tv_speech_wakeup, R.id.tv_speech_speaking})
    public void onViewClicked(View view) {
        switch (view.getId()) {
            //开始合成语音
            case R.id.tv_speech_synthesize_start:
                SpeakOption speakOption = new SpeakOption();
                //设置合成语言
                if (rgLang.getCheckedRadioButtonId() == R.id.rb_chinese) {
                    speakOption.setLanguageType(SpeakOption.LAG_CHINESE);
                } else if (rgLang.getCheckedRadioButtonId() == R.id.rb_english) {
                    speakOption.setLanguageType(SpeakOption.LAG_ENGLISH_US);
                }
                //设置合成语速
                String speed = etSpeed.getText().toString();
                if (!TextUtils.isEmpty(speed) && Integer.parseInt(speed) >= 0 && Integer.parseInt(speed) <= 100) {
                    speakOption.setSpeed(Integer.parseInt(speed));
                }
                //设置合成声调
                String tone = etTone.getText().toString();
                if (!TextUtils.isEmpty(tone) && Integer.parseInt(tone) >= 0 && Integer.parseInt(tone) <= 100) {
                    speakOption.setIntonation(Integer.parseInt(tone));
                }
                speechManager.startSpeak(etText.getText().toString(), speakOption);
                break;
            //暂停合成语音
            case R.id.tv_speech_synthesize_pause:
                speechManager.pauseSpeak();
                break;
            //继续合成语音
            case R.id.tv_speech_synthesize_continue:
                speechManager.resumeSpeak();
                break;
            //停止合成语音
            case R.id.tv_speech_synthesize_stop:
                speechManager.stopSpeak();
                break;
            //休眠
            case R.id.tv_speech_sleep:
                speechManager.doSleep();
                break;
            //唤醒
            case R.id.tv_speech_wakeup:
                if (wakeupOption.equals("default")) {
                    speechManager.doWakeUp();
                    return;
                }
                WakeUpOption wakeUpOption = new WakeUpOption();
                wakeUpOption.setLanguageType(wakeupOption);
                speechManager.doWakeUp(wakeUpOption);
                break;
            //机器人是否正在说话
            case R.id.tv_speech_speaking:
                if (speechManager.isSpeaking().getResult().equals("1")) {
                    tvSpeakingResult.setText(R.string.speaking);
                } else if (speechManager.isSpeaking().getResult().equals("0")) {
                    tvSpeakingResult.setText(R.string.not_speaking);
                }
                break;
        }
    }
}
