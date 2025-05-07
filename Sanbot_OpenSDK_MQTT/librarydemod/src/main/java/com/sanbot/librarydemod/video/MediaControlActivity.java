package com.sanbot.librarydemod.video;

import android.graphics.Bitmap;
import android.os.Bundle;
import android.os.Environment;
import android.util.Log;
import android.view.SurfaceHolder;
import android.view.SurfaceView;
import android.view.View;
import android.view.WindowManager;
import android.widget.ImageView;
import android.widget.TextView;

import com.google.gson.Gson;
import com.sanbot.librarydemod.R;
import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.beans.OperationResult;
import com.sanbot.opensdk.function.beans.FaceRecognizeBean;
import com.sanbot.opensdk.function.beans.StreamOption;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.function.unit.interfaces.media.FaceRecognizeListener;
import com.sanbot.opensdk.function.unit.interfaces.media.MediaStreamListener;

import java.io.File;
import java.io.FileOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;

/**
 * className: MediaControlActivity
 * function: 多媒体控制
 * <p/> 人脸识别和抓图功能 需要安装“家庭成员app”
 * create at 2017/5/25 9:25
 *
 * @author gangpeng
 */

public class MediaControlActivity extends TopBaseActivity implements SurfaceHolder.Callback {

    private final static String TAG = MediaControlActivity.class.getSimpleName();

    SurfaceView svMedia;
    TextView tvCapture;
    ImageView iv_capture;
    TextView tvFace;


    private HDCameraManager hdCameraManager;
    private VisionMediaDecoder mediaDecoder;
    private List<Integer> handleList = new ArrayList<>();

    private int type = 1;

    @Override
    public void onCreate(Bundle savedInstanceState) {
        register(MediaControlActivity.class);
        //屏幕常亮
        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);
        super.onCreate(savedInstanceState);
        type = getIntent().getIntExtra("type", 1);
        setContentView(R.layout.activity_media_control);
        tvFace = findViewById(R.id.tv_face);
        iv_capture = findViewById(R.id.iv_capture);
        svMedia = findViewById(R.id.sv_media);
        tvCapture = findViewById(R.id.tv_capture);
        hdCameraManager = (HDCameraManager) getUnitManager(FuncConstant.HDCAMERA_MANAGER);

        svMedia.getHolder().addCallback(this);
        tvCapture.setOnClickListener(new View.OnClickListener() {
            @Override
            public void onClick(View view) {
                //        storeImage(mediaManager.getVideoImage());
                iv_capture.setImageBitmap(hdCameraManager.getVideoImage());
            }
        });

        mediaDecoder = new VisionMediaDecoder();

        initListener();
    }

    @Override
    protected void onResume() {
        super.onResume();
    }

    @Override
    protected void onPause() {
        super.onPause();
    }

    private int mWidth, mHeight;

    /**
     * 初始化监听器
     */
    private void initListener() {
        hdCameraManager.setMediaListener(new MediaStreamListener() {
            @Override
            public void getVideoStream(int handle, byte[] bytes, int width, int height) {
                if (mediaDecoder != null) {
                    if (width != mWidth || height != mHeight) {
                        mediaDecoder.onCreateCodec(width, height);
                        mWidth = width;
                        mHeight = height;
                    }
                    mediaDecoder.drawVideoSample(ByteBuffer.wrap(bytes));
                    Log.i(TAG, "getVideoStream: 视频数据:" + bytes.length);
                }
            }

            @Override
            public void getAudioStream(int i, byte[] bytes) {

            }
        });
        hdCameraManager.setMediaListener(new FaceRecognizeListener() {
            @Override
            public void recognizeResult(List<FaceRecognizeBean> list) {
                StringBuilder sb = new StringBuilder();
                for (FaceRecognizeBean bean : list) {
                    sb.append(new Gson().toJson(bean));
                    sb.append("\n");
                }
                tvFace.setText(sb.toString());
            }
        });
    }

    @Override
    protected void onMainServiceConnected() {

    }


    @Override
    public void surfaceCreated(SurfaceHolder holder) {
        //设置参数并打开媒体流
        StreamOption streamOption = new StreamOption();
        if (type == 1) {
            streamOption.setChannel(StreamOption.MAIN_STREAM);
        } else {
            streamOption.setChannel(StreamOption.SUB_STREAM);
        }
        streamOption.setDecodType(StreamOption.HARDWARE_DECODE);
        streamOption.setJustIframe(false);
        OperationResult operationResult = hdCameraManager.openStream(streamOption);
        Log.i(TAG, "surfaceCreated: operationResult=" + operationResult.getResult());
        int result = Integer.valueOf(operationResult.getResult());
        if (result != -1) {
            handleList.add(result);
        }
        mediaDecoder.setSurface(holder.getSurface());
    }

    @Override
    public void surfaceChanged(SurfaceHolder holder, int format, int width, int height) {

    }

    @Override
    public void surfaceDestroyed(SurfaceHolder holder) {
        Log.i(TAG, "surfaceDestroyed: ");
        //关闭媒体流
        if (handleList.size() > 0) {
            for (int handle : handleList) {
                Log.i(TAG, "surfaceDestroyed: " + hdCameraManager.closeStream(handle));
            }
        }
        handleList = null;
        mediaDecoder.stopDecoding();
    }

    public void storeImage(Bitmap bitmap) {
        String dir = Environment.getExternalStorageDirectory() + "/FACE_REG/IMG/" + "DCIM/";
        final File f = new File(dir);
        if (!f.exists()) {
            f.mkdirs();
        }
        String fileName = System.currentTimeMillis() + ".jpg";
        File file = new File(f, fileName);
        try {
            FileOutputStream fos = new FileOutputStream(file);
            bitmap.compress(Bitmap.CompressFormat.JPEG, 100, fos);
            fos.flush();
            fos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        Log.i(TAG, "onDestroy: ");
    }
}
