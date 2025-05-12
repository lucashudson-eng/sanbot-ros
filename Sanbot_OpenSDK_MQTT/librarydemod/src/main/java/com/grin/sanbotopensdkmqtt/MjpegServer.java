package com.grin.sanbotopensdkmqtt;

import android.graphics.Bitmap;

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.PipedInputStream;
import java.io.PipedOutputStream;
import java.util.concurrent.atomic.AtomicReference;

import fi.iki.elonen.NanoHTTPD;

/**
 * Servidor HTTP MJPEG que transmite frames da câmera do Sanbot em tempo real.
 * Acesse pelo navegador: http://<ip_do_robot>:8080
 */
public class MjpegServer extends NanoHTTPD {

    private final AtomicReference<Bitmap> latestFrame = new AtomicReference<>();

    public MjpegServer(int port) {
        super(port);
    }

    public void updateFrame(Bitmap frame) {
        latestFrame.set(frame);
    }

    @Override
    public Response serve(IHTTPSession session) {
        try {
            final PipedOutputStream out = new PipedOutputStream();
            PipedInputStream in = new PipedInputStream(out);

            new Thread(new Runnable() {
                @Override
                public void run() {
                    try {
                        while (true) {
                            Bitmap frame = latestFrame.get();
                            if (frame != null) {
                                ByteArrayOutputStream baos = new ByteArrayOutputStream();
                                frame.compress(Bitmap.CompressFormat.JPEG, 30, baos); // menor qualidade = menos dados
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
                    MIME_PLAINTEXT, "Erro ao iniciar servidor MJPEG.");
        }
    }
}
