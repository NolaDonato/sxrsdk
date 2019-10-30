/* Copyright 2015 Samsung Electronics Co., LTD
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

#include <android/asset_manager_jni.h>
#include "../physics_world.h"
#include "../physics_constraint.h"
#include "bullet_fileloader.h"
#include "bullet_rigidbody.h"
#include "bullet_joint.h"
#include "util/jni_utils.h"

class AAssetManager;

static char tag[] = "PHYSICS";

namespace sxr {
extern "C" {

JNIEXPORT jlong JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_ctor(JNIEnv* env, jclass clazz, jobject context, jobject assetManager)
{
    JavaVM* jvm;
    env->GetJavaVM(&jvm);
    AAssetManager* am = AAssetManager_fromJava(env, assetManager);
    BulletFileLoader* loader = new BulletFileLoader(context, *jvm, am);
    return reinterpret_cast<jlong>(loader);
}

JNIEXPORT bool JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_parse(JNIEnv* env, jclass clazz,
                                                   jlong jloader,
                                                   jbyteArray byteArr,
                                                   jint len, jboolean ignoreUpAxis)
{
    jbyte* data = env->GetByteArrayElements(byteArr, NULL);

    if (data == NULL)
    {
        return 0;
    }
    BulletFileLoader* loader = reinterpret_cast<BulletFileLoader*>(jloader);

    bool result = loader->parse((char*) data, len, ignoreUpAxis);
    env->ReleaseByteArrayElements(byteArr, data, JNI_ABORT);
    return result;
}

JNIEXPORT bool JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_parseMB(JNIEnv* env, jclass clazz,
                                                    jlong jloader,
                                                    jlong jworld,
                                                    jbyteArray byteArr,
                                                    jint len, jboolean ignoreUpAxis)
{
    jbyte* data = env->GetByteArrayElements(byteArr, NULL);

    if (data == NULL)
    {
        return 0;
    }
    BulletFileLoader* loader = reinterpret_cast<BulletFileLoader*>(jloader);
    BulletWorld* world = reinterpret_cast<BulletWorld*>(jworld);
    bool result = loader->parse(world, (char*) data, len, ignoreUpAxis);
    env->ReleaseByteArrayElements(byteArr, data, JNI_ABORT);
    return result;
}

JNIEXPORT bool JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_parseURDF(JNIEnv* env, jclass clazz,
                                                      jlong jloader,
                                                      jlong jworld,
                                                      jstring jxmldata,
                                                      jboolean ignoreUpAxis)
{
    const char* xmldata = env->GetStringUTFChars(jxmldata, 0);
    if (xmldata == NULL)
    {
        return 0;
    }
    BulletFileLoader* loader = reinterpret_cast<BulletFileLoader*>(jloader);
    BulletWorld* world = reinterpret_cast<BulletWorld*>(jworld);
    bool result = loader->parseURDF(world, (char*) xmldata, ignoreUpAxis);
    env->ReleaseStringUTFChars(jxmldata, xmldata);
    return result;
}

JNIEXPORT void JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_clear(JNIEnv* env, jclass clazz, jlong jloader)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    loader->clear();
}

JNIEXPORT jobject JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getCollider(JNIEnv* env, jclass clazz,
                                                       jlong jloader, jstring jname)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    const char* name = env->GetStringUTFChars(jname, 0);
    jobject result = loader->getCollider(name);
    env->ReleaseStringUTFChars(jname, name);
    return env->NewLocalRef(result);
}

JNIEXPORT jobjectArray JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getConstraints(JNIEnv* env, jclass clazz,
                                                            jlong jloader)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    return loader->getConstraints();
}

JNIEXPORT jobjectArray JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getRigidBodies(JNIEnv* env, jclass clazz,
                                                             jlong jloader)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    return loader->getRigidBodies();
}

JNIEXPORT jobjectArray JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getJoints(JNIEnv* env, jclass clazz,
                                                             jlong jloader)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    return loader->getJoints();
}

JNIEXPORT jstring JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getConstraintName(JNIEnv* env, jclass clazz,
                                                           jlong jloader, jlong jconstraint)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    PhysicsConstraint* constraint = reinterpret_cast<PhysicsConstraint*>(jconstraint);
    const char* name = loader->getConstraintName(constraint);
    return env->NewStringUTF(name);
}

}

}