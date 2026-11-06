#include <jni.h>
#include <android/log.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <mutex>
#include <cstring>
#include <string>
#include <vector>
#include <cstdint>
#include <cstdio>
#include <algorithm>

#define LOG_TAG "SanbotOrbbec"
#define LOGI(...) __android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#define LOGE(...) __android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)

extern "C" {
typedef uint32_t XnUInt32;
typedef uint16_t XnUInt16;
typedef char XnChar;
typedef int32_t XnStatus;
typedef void *XnContext;
typedef void *XnNodeHandle;
typedef void *XnEnumerationErrors;
typedef XnUInt16 XnDepthPixel;
typedef uint8_t XnUInt8;
typedef struct {
    XnUInt8 nRed;
    XnUInt8 nGreen;
    XnUInt8 nBlue;
} XnRGB24Pixel;

#define XN_STATUS_OK 0
#define XN_NODE_TYPE_DEPTH 2
#define XN_NODE_TYPE_IMAGE 3

XnStatus xnInitFromXmlFile(const XnChar *strFileName, XnContext **ppContext,
                           XnEnumerationErrors **ppErrors);
XnStatus xnContextRelease(XnContext *pContext);
XnStatus xnFindExistingNodeByType(XnContext *pContext, XnUInt32 type,
                                  XnNodeHandle *phNode);
XnStatus xnCreateDepthGenerator(XnContext *pContext, XnNodeHandle *phDepthGenerator,
                                void *pQuery, void *pErrors);
XnStatus xnCreateImageGenerator(XnContext *pContext, XnNodeHandle *phImageGenerator,
                                void *pQuery, void *pErrors);
XnStatus xnStartGeneratingAll(XnContext *pContext);
XnStatus xnWaitAndUpdateAll(XnContext *pContext);
const XnDepthPixel *xnGetDepthMap(XnNodeHandle hInstance);
const XnRGB24Pixel *xnGetRGB24ImageMap(XnNodeHandle hInstance);
typedef struct {
    XnUInt32 nXRes;
    XnUInt32 nYRes;
    XnUInt32 nFPS;
} XnMapOutputMode;
XnStatus xnGetMapOutputMode(XnNodeHandle hInstance, XnMapOutputMode *pOutputMode);
const XnChar *xnGetStatusString(XnStatus status);
XnStatus xnOSGetApplicationFilesDir(XnChar *cpDest, XnUInt32 nDestLength);
XnStatus xnOSGetApplicationLibDir(XnChar *cpDest, XnUInt32 nDestLength);
XnStatus xnOSSetCurrentDir(const XnChar *cpDir);
}

namespace {
std::mutex gMu;
XnContext *gCtx = nullptr;
XnNodeHandle gDepth = nullptr;
XnNodeHandle gImage = nullptr;
bool gRunning = false;
bool gUseColor = false;
XnUInt32 gW = 0;
XnUInt32 gH = 0;

std::string statusMsg(XnStatus st) {
    const char *s = xnGetStatusString(st);
    char buf[160];
    snprintf(buf, sizeof(buf), "status=%d (%s)", (int) st, s ? s : "?");
    return std::string(buf);
}
} // namespace

extern "C" JNIEXPORT jstring JNICALL
Java_com_grin_sanbotrosbridge_OrbbecNative_nativeOpen(
        JNIEnv *env, jclass /*clazz*/, jstring jXmlPath) {
    std::lock_guard<std::mutex> lock(gMu);
    if (gRunning) {
        return env->NewStringUTF("OK already open");
    }
    const char *xmlPath = env->GetStringUTFChars(jXmlPath, nullptr);
    XnChar filesDir[512] = {0};
    XnChar libDir[512] = {0};
    xnOSGetApplicationFilesDir(filesDir, sizeof(filesDir));
    xnOSGetApplicationLibDir(libDir, sizeof(libDir));
    if (filesDir[0]) xnOSSetCurrentDir(filesDir);
    LOGI("nativeOpen xml=%s files=%s", xmlPath, filesDir);

    XnContext *ctx = nullptr;
    XnEnumerationErrors *errors = nullptr;
    XnStatus st = xnInitFromXmlFile(xmlPath, &ctx, &errors);
    if (st != XN_STATUS_OK || ctx == nullptr) {
        std::string err = "xnInitFromXmlFile failed " + statusMsg(st);
        LOGE("%s", err.c_str());
        env->ReleaseStringUTFChars(jXmlPath, xmlPath);
        return env->NewStringUTF(err.c_str());
    }

    XnNodeHandle depth = nullptr;
    st = xnFindExistingNodeByType(ctx, XN_NODE_TYPE_DEPTH, &depth);
    if (st != XN_STATUS_OK || depth == nullptr) {
        st = xnCreateDepthGenerator(ctx, &depth, nullptr, nullptr);
    }
    if (st != XN_STATUS_OK || depth == nullptr) {
        std::string err = "No depth node " + statusMsg(st);
        xnContextRelease(ctx);
        env->ReleaseStringUTFChars(jXmlPath, xmlPath);
        return env->NewStringUTF(err.c_str());
    }

    XnNodeHandle image = nullptr;
    bool useColor = false;
    XnStatus ist = xnFindExistingNodeByType(ctx, XN_NODE_TYPE_IMAGE, &image);
    if (ist != XN_STATUS_OK || image == nullptr) {
        ist = xnCreateImageGenerator(ctx, &image, nullptr, nullptr);
    }
    if (ist == XN_STATUS_OK && image != nullptr) {
        useColor = true;
        LOGI("Image/color node available");
    } else {
        LOGI("No Image node (%s) — depth gray fallback", statusMsg(ist).c_str());
        image = nullptr;
    }

    st = xnStartGeneratingAll(ctx);
    if (st != XN_STATUS_OK) {
        std::string err = "xnStartGeneratingAll failed " + statusMsg(st);
        xnContextRelease(ctx);
        env->ReleaseStringUTFChars(jXmlPath, xmlPath);
        return env->NewStringUTF(err.c_str());
    }

    // Warm-up
    for (int i = 0; i < 5; i++) xnWaitAndUpdateAll(ctx);

    XnMapOutputMode mode{};
    XnNodeHandle modeNode = useColor ? image : depth;
    st = xnGetMapOutputMode(modeNode, &mode);
    XnUInt32 w = (st == XN_STATUS_OK && mode.nXRes > 0) ? mode.nXRes : 160;
    XnUInt32 h = (st == XN_STATUS_OK && mode.nYRes > 0) ? mode.nYRes : 120;

    gCtx = ctx;
    gDepth = depth;
    gImage = image;
    gUseColor = useColor;
    gW = w;
    gH = h;
    gRunning = true;
    env->ReleaseStringUTFChars(jXmlPath, xmlPath);

    char msg[160];
    snprintf(msg, sizeof(msg), "OK open %ux%u mode=%s", w, h, useColor ? "color" : "depth");
    LOGI("%s", msg);
    return env->NewStringUTF(msg);
}

extern "C" JNIEXPORT void JNICALL
Java_com_grin_sanbotrosbridge_OrbbecNative_nativeClose(JNIEnv *, jclass) {
    std::lock_guard<std::mutex> lock(gMu);
    if (gCtx) {
        xnContextRelease(gCtx);
    }
    gCtx = nullptr;
    gDepth = nullptr;
    gImage = nullptr;
    gRunning = false;
    gUseColor = false;
    gW = gH = 0;
    LOGI("nativeClose");
}

extern "C" JNIEXPORT jintArray JNICALL
Java_com_grin_sanbotrosbridge_OrbbecNative_nativeFrameSize(JNIEnv *env, jclass) {
    std::lock_guard<std::mutex> lock(gMu);
    jintArray arr = env->NewIntArray(3);
    jint v[3] = {(jint) gW, (jint) gH, gUseColor ? 1 : 0};
    env->SetIntArrayRegion(arr, 0, 3, v);
    return arr;
}

/** Fill ARGB_8888 int[] of length width*height (native frame size). Returns fps-ish ok=1. */
extern "C" JNIEXPORT jint JNICALL
Java_com_grin_sanbotrosbridge_OrbbecNative_nativeGrabArgb(
        JNIEnv *env, jclass /*clazz*/, jintArray jArgb) {
    std::lock_guard<std::mutex> lock(gMu);
    if (!gRunning || !gCtx || !gDepth || jArgb == nullptr) return -1;

    XnStatus st = xnWaitAndUpdateAll(gCtx);
    if (st != XN_STATUS_OK) {
        LOGE("wait %s", statusMsg(st).c_str());
        return -2;
    }

    jsize len = env->GetArrayLength(jArgb);
    if (len < (jsize) (gW * gH)) return -3;
    jint *out = env->GetIntArrayElements(jArgb, nullptr);
    if (!out) return -4;

    if (gUseColor && gImage) {
        const XnRGB24Pixel *rgb = xnGetRGB24ImageMap(gImage);
        if (!rgb) {
            env->ReleaseIntArrayElements(jArgb, out, JNI_ABORT);
            return -5;
        }
        for (XnUInt32 i = 0; i < gW * gH; i++) {
            out[i] = (jint) (0xFF000000u | (rgb[i].nRed << 16) | (rgb[i].nGreen << 8) | rgb[i].nBlue);
        }
    } else {
        const XnDepthPixel *map = xnGetDepthMap(gDepth);
        if (!map) {
            env->ReleaseIntArrayElements(jArgb, out, JNI_ABORT);
            return -5;
        }
        uint16_t maxNz = 1;
        for (XnUInt32 i = 0; i < gW * gH; i++) {
            if (map[i] > maxNz) maxNz = map[i];
        }
        for (XnUInt32 i = 0; i < gW * gH; i++) {
            uint8_t g = (uint8_t) ((map[i] * 255u) / maxNz);
            out[i] = (jint) (0xFF000000u | (g << 16) | (g << 8) | g);
        }
    }

    env->ReleaseIntArrayElements(jArgb, out, 0);
    return 1;
}

/**
 * Depth-gray ARGB from gDepth using the last WaitAndUpdate (call after nativeGrabArgb
 * in the same loop — does not wait again).
 */
extern "C" JNIEXPORT jint JNICALL
Java_com_grin_sanbotrosbridge_OrbbecNative_nativeGrabDepthArgb(
        JNIEnv *env, jclass /*clazz*/, jintArray jArgb) {
    std::lock_guard<std::mutex> lock(gMu);
    if (!gRunning || !gCtx || !gDepth || jArgb == nullptr) return -1;

    jsize len = env->GetArrayLength(jArgb);
    if (len < (jsize) (gW * gH)) return -3;
    jint *out = env->GetIntArrayElements(jArgb, nullptr);
    if (!out) return -4;

    const XnDepthPixel *map = xnGetDepthMap(gDepth);
    if (!map) {
        env->ReleaseIntArrayElements(jArgb, out, JNI_ABORT);
        return -5;
    }
    uint16_t maxNz = 1;
    for (XnUInt32 i = 0; i < gW * gH; i++) {
        if (map[i] > maxNz) maxNz = map[i];
    }
    for (XnUInt32 i = 0; i < gW * gH; i++) {
        uint8_t g = (uint8_t) ((map[i] * 255u) / maxNz);
        out[i] = (jint) (0xFF000000u | (g << 16) | (g << 8) | g);
    }

    env->ReleaseIntArrayElements(jArgb, out, 0);
    return 1;
}

/** One-shot probe still used by Probe button — writes PGM. */
extern "C" JNIEXPORT jstring JNICALL
Java_com_grin_sanbotrosbridge_OrbbecNative_nativeProbe(
        JNIEnv *env, jclass clazz, jstring jXmlPath, jstring jOutPath,
        jstring /*jUsbPath*/) {
    jstring open = Java_com_grin_sanbotrosbridge_OrbbecNative_nativeOpen(env, clazz, jXmlPath);
    const char *openStr = env->GetStringUTFChars(open, nullptr);
    bool ok = openStr && strncmp(openStr, "OK", 2) == 0;
    env->ReleaseStringUTFChars(open, openStr);
    if (!ok) return open;

    jint sizeArr[3];
    {
        std::lock_guard<std::mutex> lock(gMu);
        sizeArr[0] = (jint) gW;
        sizeArr[1] = (jint) gH;
        sizeArr[2] = gUseColor ? 1 : 0;
    }
    jintArray argb = env->NewIntArray(sizeArr[0] * sizeArr[1]);
    jint rc = Java_com_grin_sanbotrosbridge_OrbbecNative_nativeGrabArgb(env, clazz, argb);
    if (rc != 1) {
        Java_com_grin_sanbotrosbridge_OrbbecNative_nativeClose(env, clazz);
        char err[64];
        snprintf(err, sizeof(err), "grab failed rc=%d", rc);
        return env->NewStringUTF(err);
    }

    const char *outPath = env->GetStringUTFChars(jOutPath, nullptr);
    std::string outPathCopy = outPath ? outPath : "";
    jint *pixels = env->GetIntArrayElements(argb, nullptr);
    FILE *f = fopen(outPathCopy.c_str(), "wb");
    if (!f) {
        env->ReleaseIntArrayElements(argb, pixels, JNI_ABORT);
        env->ReleaseStringUTFChars(jOutPath, outPath);
        Java_com_grin_sanbotrosbridge_OrbbecNative_nativeClose(env, clazz);
        return env->NewStringUTF("fopen failed");
    }
    int w = sizeArr[0], h = sizeArr[1];
    fprintf(f, "P5\n%d %d\n255\n", w, h);
    for (int i = 0; i < w * h; i++) {
        uint8_t g = (uint8_t) (pixels[i] & 0xFF);
        fwrite(&g, 1, 1, f);
    }
    fclose(f);
    env->ReleaseIntArrayElements(argb, pixels, JNI_ABORT);
    env->ReleaseStringUTFChars(jOutPath, outPath);

    char msg[256];
    snprintf(msg, sizeof(msg), "OK depth %dx%d mode=%s wrote %s",
             w, h, sizeArr[2] ? "color→gray" : "depth", outPathCopy.c_str());
    Java_com_grin_sanbotrosbridge_OrbbecNative_nativeClose(env, clazz);
    return env->NewStringUTF(msg);
}
