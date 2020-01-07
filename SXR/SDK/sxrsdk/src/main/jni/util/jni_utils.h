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
class SmartGlobalRef;

// Automatically deletes a local ref when it goes out of scope
class SmartLocalRef
{
protected:
    JavaVM* mJVM;
    jobject mJavaObj;

public:
    SmartLocalRef(JavaVM& vm, jobject object);
    SmartLocalRef(const SmartLocalRef& src);
    SmartLocalRef(const SmartGlobalRef& src);

    SmartLocalRef()
    : mJVM(nullptr),
      mJavaObj(nullptr)
    { }


    virtual ~SmartLocalRef();
    virtual SmartLocalRef& operator=(const SmartLocalRef& src);
    virtual SmartLocalRef& operator=(const SmartGlobalRef& src);
    virtual SmartLocalRef& operator=(jobject javaObj);

    jobject getObject() const { return mJavaObj; }

    JavaVM* getJVM() const { return mJVM; }
    JNIEnv* getEnv() const;

    virtual bool operator==(const SmartLocalRef& r)
    {
        return r.getObject() == getObject();
    }
};

class SmartGlobalRef : public SmartLocalRef
{
public:
    SmartGlobalRef(JavaVM& jvm, jobject object);
    SmartGlobalRef(const SmartGlobalRef& src);
    SmartGlobalRef(const SmartLocalRef& src);
    SmartGlobalRef() { }

    jobject getObject() const { return mJavaObj; }

    virtual bool operator==(const SmartGlobalRef& r)
    {
        return r.getObject() == getObject();
    }

    virtual ~SmartGlobalRef();
    virtual SmartGlobalRef& operator=(const SmartGlobalRef& src);
    virtual SmartGlobalRef& operator=(const SmartLocalRef& src);
    virtual SmartGlobalRef& operator=(jobject javaObj);
};

jobject   CreateInstance(JNIEnv& env, const char* className, const char* signature, ...);

void CallVoidMethod(JNIEnv& env, jobject obj, const char* className, const char* methodName, const char* signature, ...);

jlong CallLongMethod(JNIEnv& env, jobject obj, const char* className, const char* methodName, const char* signature, ...);

jobject CallObjectMethod(JNIEnv& env, jobject obj, const char* className, const char* methodName, const char* signature, ...);

jmethodID GetStaticMethodID(JNIEnv& env, jclass clazz, const char * name, const char * signature);

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
