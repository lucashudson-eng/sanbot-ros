package com.grin.sanbotmqtt;

import android.graphics.Bitmap;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.atomic.AtomicReference;

import fi.iki.elonen.NanoHTTPD;

/**
 * Servidor HTTP MJPEG que transmite frames de múltiplas câmeras em tempo real.
 * Câmera 1 (SDK): http://<ip_do_robot>:8080/camera1
 * Câmera 2 (Android): http://<ip_do_robot>:8080/camera2
 */
public class MultiCameraMjpegServer extends NanoHTTPD {

    private final ConcurrentHashMap<String, AtomicReference<Bitmap>> cameraFrames = new ConcurrentHashMap<>();

    public MultiCameraMjpegServer(int port) {
        super(port);
        // Inicializa as câmeras
        cameraFrames.put("camera1", new AtomicReference<>());
        cameraFrames.put("camera2", new AtomicReference<>());
    }

    public void updateFrame(String cameraId, Bitmap frame) {
        AtomicReference<Bitmap> frameRef = cameraFrames.get(cameraId);
        if (frameRef != null) {
            frameRef.set(frame);
        }
    }

    @Override
    public Response serve(IHTTPSession session) {
        String uri = session.getUri();
        
        // Página principal com links para as câmeras
        if ("/".equals(uri)) {
            String html = "<html><head><title>Multi-Camera MJPEG Server</title></head>" +
                    "<body style='font-family: Arial; padding: 20px;'>" +
                    "<h1>Multi-Camera MJPEG Server</h1>" +
                    "<h2>Câmeras Disponíveis:</h2>" +
                    "<p><a href='/camera1' target='_blank'>Câmera 1 (SDK)</a></p>" +
                    "<p><a href='/camera2' target='_blank'>Câmera 2 (Android)</a></p>" +
                    "<hr>" +
                    "<h3>Visualização em Tempo Real:</h3>" +
                    "<div style='display: flex; gap: 20px;'>" +
                    "<div><h4>Câmera 1 (SDK)</h4><img src='/camera1' style='max-width: 400px; border: 1px solid #ccc;'></div>" +
                    "<div><h4>Câmera 2 (Android)</h4><img src='/camera2' style='max-width: 400px; border: 1px solid #ccc;'></div>" +
                    "</div></body></html>";
            return newFixedLengthResponse(Response.Status.OK, MIME_HTML, html);
        }
        
        // Verifica se é um endpoint de câmera válido
        String cameraId = null;
        if ("/camera1".equals(uri)) {
            cameraId = "camera1";
        } else if ("/camera2".equals(uri)) {
            cameraId = "camera2";
        }
        
        if (cameraId == null) {
            return newFixedLengthResponse(Response.Status.NOT_FOUND, MIME_PLAINTEXT, 
                "Endpoint não encontrado. Use /camera1 ou /camera2");
        }

        return streamCamera(cameraId);
    }

    private Response streamCamera(String cameraId) {
        try {
            final AtomicReference<Bitmap> frameRef = cameraFrames.get(cameraId);
            final PipedOutputStream out = new PipedOutputStream();
            PipedInputStream in = new PipedInputStream(out);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        while (true) {
                            Bitmap frame = frameRef.get();
                            if (frame != null) {
                                ByteArrayOutputStream baos = new ByteArrayOutputStream();
                                frame.compress(Bitmap.CompressFormat.JPEG, 70, baos);
                                byte[] imageBytes = baos.toByteArray();

                                String header = "\r\n--BoundaryString\r\n" +
                                        "Content-type: image/jpeg\r\n" +
                                        "Content-Length: " + imageBytes.length + "\r\n\r\n";

                                out.write(header.getBytes());
                                out.write(imageBytes);
                                out.flush();
                                Thread.sleep(100); // ~10fps
                            } else {
                                Thread.sleep(50);
                            }
                        }
                    } catch (Exception e) {
                        e.printStackTrace();
                    } finally {
                        try {
                            out.close();
                        } catch (IOException ignored) {}
                    }
                }
            }).start();

            return newChunkedResponse(Response.Status.OK,
                    "multipart/x-mixed-replace; boundary=--BoundaryString",
                    in);
        } catch (IOException e) {
            e.printStackTrace();
            return newFixedLengthResponse(Response.Status.INTERNAL_ERROR,
                    MIME_PLAINTEXT, "Erro ao iniciar stream da " + cameraId);
        }
    }
} 