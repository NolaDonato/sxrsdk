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

#ifndef JNI_UTILS_H
#define JNI_UTILS_H

#include <jni.h>


namespace sxr {

// Automatically deletes a local ref when it goes out of scope
class SmartLocalRef
{

protected:
    JNIEnv* mJniEnv;
    jobject mJavaObj;
    JNIEnv* getEnv() const { return mJniEnv; }

public:
    SmartLocalRef(JNIEnv* env, jobject object)
    : mJniEnv(env),
      mJavaObj(object)
    {
    }

    ~SmartLocalRef()
    {
        if (mJavaObj != NULL)
        {
            mJniEnv->DeleteLocalRef(mJavaObj);
        }
    }

    SmartLocalRef()
    :    mJniEnv(nullptr),
         mJavaObj(nullptr)
    {
    }

    SmartLocalRef(const SmartLocalRef& src)
    :   mJniEnv(src.getEnv()),
        mJavaObj(nullptr)
    {
        if (src.getObject())
        {
            mJavaObj = mJniEnv->NewLocalRef(src.getObject());
        }
    }


    SmartLocalRef& operator=(const SmartLocalRef& src)
    {
        if (mJavaObj != nullptr)
        {
            mJniEnv->DeleteLocalRef(mJavaObj);
        }
        mJniEnv = src.getEnv();
        if (src.getObject())
        {
            mJavaObj = mJniEnv->NewLocalRef(src.getObject());
        }
        return *this;
    }

    jobject getObject() const { return mJavaObj; }

    bool operator==(const SmartLocalRef& r)
    {
        return r.getObject() == getObject();
    }
};

jobject   CreateInstance(JNIEnv* env, const char* className, const char* signature, ...);

jmethodID GetStaticMethodID(JNIEnv& env, jclass clazz, const char * name,
        const char * signature);

jmethodID GetMethodId(JNIEnv& env, const jclass clazz, const char* name, const char* signature);

/**
 * @return global reference; caller must delete the global reference
 */
jclass GetGlobalClassReference(JNIEnv& env, const char * className);

/**
 * Assuming this is called only by threads that are already attached to jni; it is the
 * responsibility of the caller to ensure that.
 */
JNIEnv* getCurrentEnv(JavaVM* javaVm);

jint throwOutOfMemoryError(JNIEnv* env, const char *message);

constexpr int SUPPORTED_JNI_VERSION = JNI_VERSION_1_6;
}

#endif
