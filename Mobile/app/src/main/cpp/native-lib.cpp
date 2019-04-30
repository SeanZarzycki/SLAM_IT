#include <jni.h>
#include <string>

extern "C" JNIEXPORT jstring JNICALL
Java_drexel_ece_group13_slam_1it_MainActivity_stringFromJNI(
        JNIEnv* env,
        jobject /* this */) {
    std::string hello = "Hello from C++";
    return env->NewStringUTF(hello.c_str());
}
