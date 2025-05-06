package com.sanbot.librarydemod.video;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Rect;
import android.graphics.YuvImage;
import android.os.Bundle;
import android.util.Log;
import android.widget.ImageView;

import com.sanbot.librarydemod.R;
import com.sanbot.opensdk.base.TopBaseActivity;
import com.sanbot.opensdk.beans.FuncConstant;
import com.sanbot.opensdk.beans.OperationResult;
import com.sanbot.opensdk.function.beans.StreamOption;
import com.sanbot.opensdk.function.unit.HDCameraManager;
import com.sanbot.opensdk.function.unit.interfaces.media.MediaStreamListener;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;


public class SoftDecodeActivity extends TopBaseActivity {

    private HDCameraManager hdCameraManager;
    private List<Integer> handleList=new ArrayList<>();
    private ImageView soft_iv;
    private int mWidth;
    private int mHeight;
    private int type=1;

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        register(SoftDecodeActivity.class);
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_soft_decode);

        soft_iv = findViewById(R.id.soft_iv);
        type=getIntent().getIntExtra("type",1);
        hdCameraManager = (HDCameraManager) getUnitManager(FuncConstant.HDCAMERA_MANAGER);

        hdCameraManager.setMediaListener(new MediaStreamListener() {
            @Override
            public void getVideoStream(int handle, byte[] bytes, int width, int height) {
                mWidth=width;
                mHeight=height;
//                setRGB365(bytes,width,height);
                setNV21(bytes);
            }

            @Override
            public void getAudioStream(int i, byte[] bytes) {

            }
        });
        StreamOption streamOption = new StreamOption();
        if (type==1){
            streamOption.setChannel(StreamOption.MAIN_STREAM);
        }else {
            streamOption.setChannel(StreamOption.SUB_STREAM);
        }
        streamOption.setDecodType(StreamOption.SOFTWARE_DECODE_NV21);//RGB365使用setRGB365方法显示,NV21使用setNV21方式显示
        streamOption.setJustIframe(false);
        OperationResult operationResult = hdCameraManager.openStream(streamOption);
        Log.i("Cris", "surfaceCreated: operationResult="+operationResult.getResult());
        int result=Integer.valueOf(operationResult.getResult());
        if (result!=-1){
            handleList.add(result);
        }
    }

    private void setNV21(byte[] bytes) {
        try {
            YuvImage image = new YuvImage(bytes, ImageFormat.NV21, mWidth, mHeight, null);
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            image.compressToJpeg(new Rect(0, 0, mWidth, mHeight), 80, baos);
            final Bitmap bitmap = BitmapFactory.decodeByteArray(baos.toByteArray(), 0, baos.toByteArray().length);
            runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    soft_iv.setImageBitmap(bitmap);
                }
            });
            baos.flush();
            baos.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    protected void onMainServiceConnected() {

    }

    @Override
    protected void onPause() {
        super.onPause();
        if (handleList.size()>0){
            for (int handle:handleList){
                Log.i("Cris", "surfaceDestroyed: "+hdCameraManager.closeStream(handle));
            }
        }
        handleList=null;
    }

    private void setRGB365(byte[] data,int width,int height){
        Bitmap bmp = null;
        try {
            bmp = Bitmap.createBitmap(width, height, Bitmap.Config.RGB_565);
        } catch (OutOfMemoryError e) {
            if (bmp != null) {
                bmp.recycle();
                bmp = null;
            }
            System.gc();
            return;
        }
        try {
            if (bmp != null && data != null && data.length > 0) {
                ByteBuffer.wrap(data).rewind();
                bmp.copyPixelsFromBuffer(ByteBuffer.wrap(data));
                ByteBuffer.wrap(data).position(0);
                final Bitmap finalBmp = bmp;
                runOnUiThread(new Runnable() {
                    @Override
                    public void run() {
                        soft_iv.setImageBitmap(finalBmp);
                    }
                });
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
    }
}
