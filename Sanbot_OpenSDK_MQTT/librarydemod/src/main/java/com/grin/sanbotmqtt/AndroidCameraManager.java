package com.grin.sanbotmqtt;

import android.Manifest;
import android.content.Context;
import android.content.pm.PackageManager;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.ImageFormat;
import android.graphics.Matrix;
import android.graphics.SurfaceTexture;
import android.hardware.Camera;
import android.os.Handler;
import android.os.HandlerThread;
import android.support.annotation.NonNull;
import android.support.v4.app.ActivityCompat;
import android.util.Log;

import java.io.IOException;

/**
 * Gerenciador para captura de frames da câmera Android usando Camera API (compatível com API mais baixa)
 */
public class AndroidCameraManager {
    private static final String TAG = "AndroidCameraManager";
    
    private Context context;
    private Camera camera;
    private Handler backgroundHandler;
    private HandlerThread backgroundThread;
    private OnFrameAvailableListener frameListener;
    private boolean isCapturing = false;
    
    public interface OnFrameAvailableListener {
        void onFrameAvailable(Bitmap frame);
    }
    
    public AndroidCameraManager(Context context) {
        this.context = context;
    }
    
    public void setOnFrameAvailableListener(OnFrameAvailableListener listener) {
        this.frameListener = listener;
    }
    
    public boolean startCamera() {
        if (!checkCameraPermission()) {
            Log.e(TAG, "Permissão de câmera não concedida");
            return false;
        }
        
        startBackgroundThread();
        
        try {
            // Tenta abrir a câmera traseira primeiro
            int cameraId = findBackCamera();
            if (cameraId == -1) {
                // Se não encontrar câmera traseira, usa a frontal
                cameraId = Camera.CameraInfo.CAMERA_FACING_FRONT;
            }
            
            camera = Camera.open(cameraId);
            if (camera == null) {
                Log.e(TAG, "Não foi possível abrir a câmera");
                return false;
            }
            
            setupCamera();
            startPreview();
            return true;
            
        } catch (Exception e) {
            Log.e(TAG, "Erro ao acessar câmera", e);
            return false;
        }
    }
    
    private int findBackCamera() {
        int numberOfCameras = Camera.getNumberOfCameras();
        Camera.CameraInfo cameraInfo = new Camera.CameraInfo();
        
        for (int i = 0; i < numberOfCameras; i++) {
            Camera.getCameraInfo(i, cameraInfo);
            if (cameraInfo.facing == Camera.CameraInfo.CAMERA_FACING_BACK) {
                return i;
            }
        }
        return -1; // Não encontrou câmera traseira
    }
    
    private void setupCamera() {
        try {
            Camera.Parameters parameters = camera.getParameters();
            
            // Define uma resolução adequada
            Camera.Size previewSize = getBestPreviewSize(parameters);
            if (previewSize != null) {
                parameters.setPreviewSize(previewSize.width, previewSize.height);
            }
            
            // Define formato de imagem
            parameters.setPreviewFormat(ImageFormat.NV21);
            
            // Configura auto-foco se disponível
            if (parameters.getSupportedFocusModes().contains(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE)) {
                parameters.setFocusMode(Camera.Parameters.FOCUS_MODE_CONTINUOUS_PICTURE);
            }
            
            camera.setParameters(parameters);
            Log.d(TAG, "Câmera configurada com sucesso");
            
        } catch (Exception e) {
            Log.e(TAG, "Erro ao configurar câmera", e);
        }
    }
    
    private Camera.Size getBestPreviewSize(Camera.Parameters parameters) {
        Camera.Size bestSize = null;
        for (Camera.Size size : parameters.getSupportedPreviewSizes()) {
            if (size.width <= 800 && size.height <= 600) {
                if (bestSize == null || (size.width * size.height) > (bestSize.width * bestSize.height)) {
                    bestSize = size;
                }
            }
        }
        return bestSize != null ? bestSize : parameters.getSupportedPreviewSizes().get(0);
    }
    
    private void startPreview() {
        try {
            camera.setPreviewCallback(new Camera.PreviewCallback() {
                private long lastFrameTime = 0;
                
                @Override
                public void onPreviewFrame(byte[] data, Camera camera) {
                    long now = System.currentTimeMillis();
                    if (now - lastFrameTime < 100) return; // Limita a ~10 FPS
                    lastFrameTime = now;
                    
                    if (frameListener != null && isCapturing) {
                        backgroundHandler.post(new Runnable() {
                            @Override
                            public void run() {
                                processFrame(data, camera);
                            }
                        });
                    }
                }
            });
            
            camera.startPreview();
            isCapturing = true;
            Log.d(TAG, "Preview da câmera Android iniciado");
            
        } catch (Exception e) {
            Log.e(TAG, "Erro ao iniciar preview", e);
        }
    }
    
    private void processFrame(byte[] data, Camera camera) {
        try {
            Camera.Parameters parameters = camera.getParameters();
            Camera.Size previewSize = parameters.getPreviewSize();
            
            Bitmap bitmap = convertYuvToBitmap(data, previewSize.width, previewSize.height);
            if (bitmap != null && frameListener != null) {
                // Rotaciona a imagem se necessário
                bitmap = rotateBitmap(bitmap);
                frameListener.onFrameAvailable(bitmap);
            }
        } catch (Exception e) {
            Log.e(TAG, "Erro ao processar frame", e);
        }
    }
    
    private Bitmap convertYuvToBitmap(byte[] data, int width, int height) {
        try {
            // Converte YUV para RGB
            int[] rgb = new int[width * height];
            decodeYUV420SP(rgb, data, width, height);
            
            // Cria bitmap a partir dos dados RGB
            Bitmap bitmap = Bitmap.createBitmap(rgb, width, height, Bitmap.Config.ARGB_8888);
            return bitmap;
        } catch (Exception e) {
            Log.e(TAG, "Erro ao converter YUV para bitmap", e);
            return null;
        }
    }
    
    private void decodeYUV420SP(int[] rgb, byte[] yuv420sp, int width, int height) {
        final int frameSize = width * height;
        
        for (int j = 0, yp = 0; j < height; j++) {
            int uvp = frameSize + (j >> 1) * width, u = 0, v = 0;
            for (int i = 0; i < width; i++, yp++) {
                int y = (0xff & ((int) yuv420sp[yp])) - 16;
                if (y < 0) y = 0;
                if ((i & 1) == 0) {
                    v = (0xff & yuv420sp[uvp++]) - 128;
                    u = (0xff & yuv420sp[uvp++]) - 128;
                }
                
                int y1192 = 1192 * y;
                int r = (y1192 + 1634 * v);
                int g = (y1192 - 833 * v - 400 * u);
                int b = (y1192 + 2066 * u);
                
                if (r < 0) r = 0; else if (r > 262143) r = 262143;
                if (g < 0) g = 0; else if (g > 262143) g = 262143;
                if (b < 0) b = 0; else if (b > 262143) b = 262143;
                
                rgb[yp] = 0xff000000 | ((r << 6) & 0xff0000) | ((g >> 2) & 0xff00) | ((b >> 10) & 0xff);
            }
        }
    }
    
    private Bitmap rotateBitmap(Bitmap bitmap) {
        try {
            // Rotaciona a imagem 90 graus para corrigir orientação (se necessário)
            Matrix matrix = new Matrix();
            matrix.postRotate(90);
            return Bitmap.createBitmap(bitmap, 0, 0, bitmap.getWidth(), bitmap.getHeight(), matrix, true);
        } catch (Exception e) {
            Log.e(TAG, "Erro ao rotacionar bitmap", e);
            return bitmap;
        }
    }
    
    public void stopCamera() {
        isCapturing = false;
        
        if (camera != null) {
            try {
                camera.stopPreview();
                camera.setPreviewCallback(null);
                camera.release();
                camera = null;
            } catch (Exception e) {
                Log.e(TAG, "Erro ao parar câmera", e);
            }
        }
        
        stopBackgroundThread();
        Log.d(TAG, "Câmera Android parada");
    }
    
    private void startBackgroundThread() {
        backgroundThread = new HandlerThread("CameraBackground");
        backgroundThread.start();
        backgroundHandler = new Handler(backgroundThread.getLooper());
    }
    
    private void stopBackgroundThread() {
        if (backgroundThread != null) {
            backgroundThread.quitSafely();
            try {
                backgroundThread.join();
                backgroundThread = null;
                backgroundHandler = null;
            } catch (InterruptedException e) {
                Log.e(TAG, "Erro ao parar thread em background", e);
            }
        }
    }
    
    private boolean checkCameraPermission() {
        return ActivityCompat.checkSelfPermission(context, Manifest.permission.CAMERA) == PackageManager.PERMISSION_GRANTED;
    }
} 