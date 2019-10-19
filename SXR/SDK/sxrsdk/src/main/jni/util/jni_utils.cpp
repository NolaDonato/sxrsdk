/* Copyright 2016 Samsung Electronics Co., LTD
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "jni_utils.h"
#include "util/sxr_log.h"

namespace sxr {
SmartLocalRef::~SmartLocalRef()
{
    if (mJavaObj)
    {
        JNIEnv* env = getEnv();
        env->DeleteLocalRef(mJavaObj);
        mJavaObj = nullptr;
    }
}

SmartLocalRef::SmartLocalRef(JavaVM& jvm, jobject object)
: mJVM(&jvm),
  mJavaObj(object)
{
    if (object)
    {
        JNIEnv* env = getEnv();
        mJavaObj = env->NewLocalRef(object);
    }
}

SmartLocalRef::SmartLocalRef(const SmartLocalRef& src)
:   mJVM(src.getJVM()),
    mJavaObj(nullptr)
{
    if (src.getObject())
    {
        JNIEnv* env = getEnv();
        mJavaObj = env->NewLocalRef(src.getObject());
    }
}

SmartLocalRef::SmartLocalRef(const SmartGlobalRef& src)
:   mJVM(src.getJVM()),
    mJavaObj(nullptr)
{
    if (src.getObject())
    {
        JNIEnv* env = getEnv();
        mJavaObj = env->NewLocalRef(src.getObject());
    }
}

JNIEnv* SmartLocalRef::getEnv() const
{
        JNIEnv* env = nullptr;
        env = getCurrentEnv(mJVM);
        if (env == nullptr)
        {
            mJVM->AttachCurrentThread(&env, nullptr);
            env = getCurrentEnv(mJVM);
        }
        return env;

}

SmartLocalRef& SmartLocalRef::operator=(const SmartLocalRef& src)
{
    jobject oldObj = mJavaObj;

    JNIEnv* env = getEnv();
    if (src.getObject())
    {
        mJavaObj = env->NewLocalRef(src.getObject());
    }
    else
    {
        mJavaObj = nullptr;
    }
    if (oldObj)
    {
        env->DeleteLocalRef(oldObj);
    }
    return *this;
}

SmartLocalRef& SmartLocalRef::operator=(const SmartGlobalRef& src)
{
    jobject oldObj = mJavaObj;

    JNIEnv* env = getEnv();
    if (src.getObject())
    {
        mJavaObj = env->NewLocalRef(src.getObject());
    }
    else
    {
        mJavaObj = nullptr;
    }
    if (oldObj)
    {
        env->DeleteLocalRef(oldObj);
    }
    return *this;
}

SmartLocalRef& SmartLocalRef::operator=(jobject javaObj)
{
    jobject oldObj = mJavaObj;
    JNIEnv* env = getEnv();

    if (javaObj)
    {
        mJavaObj = env->NewLocalRef(javaObj);
    }
    else
    {
        mJavaObj = nullptr;
    }
    if (oldObj)
    {
        env->DeleteLocalRef(oldObj);
    }
    return *this;
}

SmartGlobalRef::~SmartGlobalRef()
{
    if (mJavaObj)
    {
        JNIEnv* env = getEnv();
        env->DeleteGlobalRef(mJavaObj);
        mJavaObj = nullptr;
    }
}

SmartGlobalRef::SmartGlobalRef(JavaVM& vm, jobject object)
: SmartLocalRef(vm, nullptr)
{
    if (object)
    {
        JNIEnv* env = getEnv();
        mJavaObj = env->NewGlobalRef(object);
    }
}


SmartGlobalRef::SmartGlobalRef(const SmartGlobalRef& src)
:   SmartLocalRef(*src.getJVM(), nullptr)
{
    if (src.getObject())
    {
        JNIEnv* env = getEnv();
        mJavaObj = env->NewGlobalRef(src.getObject());
    }
}

SmartGlobalRef::SmartGlobalRef(const SmartLocalRef& src)
:   SmartLocalRef(*src.getJVM(), nullptr)
{
    if (src.getObject())
    {
        JNIEnv* env = getEnv();
        mJavaObj = env->NewGlobalRef(src.getObject());
    }
}

SmartGlobalRef& SmartGlobalRef::operator=(const SmartGlobalRef& src)
{
    jobject oldObj = mJavaObj;

    JNIEnv* env = src.getEnv();
    if (src.getObject())
    {
        mJavaObj = env->NewGlobalRef(src.getObject());
    }
    else
    {
        mJavaObj = nullptr;
    }
    if (oldObj != nullptr)
    {
        env->DeleteGlobalRef(oldObj);
    }
    return *this;
}

SmartGlobalRef& SmartGlobalRef::operator=(const SmartLocalRef& src)
{
    jobject oldObj = mJavaObj;
    JNIEnv* env = src.getEnv();

    if (src.getObject())
    {
        mJavaObj = env->NewGlobalRef(src.getObject());
    }
    else
    {
        mJavaObj = nullptr;
    }
    if (oldObj != nullptr)
    {
        env->DeleteGlobalRef(oldObj);
    }
    return *this;
}

SmartGlobalRef& SmartGlobalRef::operator=(jobject javaObj)
{
    jobject oldObj = mJavaObj;
    JNIEnv* env = getEnv();

    if (javaObj)
    {
        mJavaObj = env->NewGlobalRef(javaObj);
    }
    else
    {
        mJavaObj = nullptr;
    }
    if (oldObj != nullptr)
    {
        env->DeleteGlobalRef(oldObj);
    }
    return *this;
}

JNIEnv* getCurrentEnv(JavaVM *javaVm)
{
    JNIEnv* result;
    if (JNI_OK != javaVm->GetEnv(reinterpret_cast<void**>(&result), JNI_VERSION_1_6))
    {
        FAIL("GetEnv failed");
    }
    return result;
}

jclass GetGlobalClassReference(JNIEnv &env, const char *className)
{
    jclass lc = env.FindClass(className);
    if (0 == lc) {
        FAIL("unable to find class %s", className);
    }
    // Turn it into a global ref, so we can safely use it in the VR thread
    jclass gc = static_cast<jclass>(env.NewGlobalRef(lc));
    env.DeleteLocalRef(lc);

    return gc;
}

jmethodID GetMethodId(JNIEnv &env, const jclass clazz, const char *name, const char *signature)
{
    const jmethodID mid = env.GetMethodID(clazz, name, signature);
    if (nullptr == mid)
    {
        FAIL("unable to find method %s", name);
    }
    return mid;
}

jmethodID GetStaticMethodID(JNIEnv &env, jclass clazz, const char *name, const char *signature)
{
    jmethodID mid = env.GetStaticMethodID(clazz, name, signature);
    if (!mid)
    {
        FAIL("unable to find static method %s", name);
    }
    return mid;
}

jobject CreateInstance(JNIEnv& env, const char* className, const char* signature, ...)
{
    jclass clazz = env.FindClass(className);

    if (NULL == clazz)
    {
        return 0;
    }
    jmethodID ctr_id = env.GetMethodID(clazz, "<init>", signature);

    if (NULL == ctr_id)
    {
        return 0;
    }
    va_list args;
    va_start(args, signature);
    jobject javaObj = env.NewObjectV(clazz, ctr_id, args);
    va_end(args);
    return javaObj;
}

jint throwOutOfMemoryError(JNIEnv* env, const char *message)
{
    jclass exClass = env->FindClass("java/lang/OutOfMemoryError");

    if (exClass != NULL)
    {
        return env->ThrowNew(exClass, message);
    }
    return -1;
}

}