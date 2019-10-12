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

#include "../physics_world.h"
#include "../physics_constraint.h"
#include "bullet_fileloader.h"
#include "bullet_rigidbody.h"
#include "bullet_joint.h"
#include "util/jni_utils.h"

static char tag[] = "PHYSICS";

namespace sxr {
extern "C" {

JNIEXPORT jlong JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_ctor(JNIEnv* env, jclass clazz,
                                                   jobject context, jbyteArray byteArr,
                                                   jint len, jboolean ignoreUpAxis)
{
    jbyte* data = env->GetByteArrayElements(byteArr, NULL);

    if (data == NULL)
    {
        return 0;
    }
    char *buffer = new char[len];
    memcpy(buffer, data, len);
    env->ReleaseByteArrayElements(byteArr, data, JNI_ABORT);

    BulletFileLoader *loader = new BulletFileLoader(context, env, buffer, len, ignoreUpAxis);
    delete[] buffer;
    return reinterpret_cast<jlong>(loader);
}

JNIEXPORT void JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_delete(JNIEnv* env, jclass clazz, jlong jloader)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    delete loader;
}

JNIEXPORT jobject JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getRigidBody(JNIEnv* env, jclass clazz,
        jlong jloader, jstring jname)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    const char* name = env->GetStringUTFChars(jname, 0);
    jobject result = loader->getRigidBody(name);
    env->ReleaseStringUTFChars(jname, name);
    return result;
}

JNIEXPORT jobject JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getJoint(JNIEnv* env, jclass clazz,
                                                           jlong jloader, jstring jname)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    const char* name = env->GetStringUTFChars(jname, 0);
    jobject result = loader->getJoint(name);
    env->ReleaseStringUTFChars(jname, name);
    return result;
}


JNIEXPORT jobject JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getCollider(JNIEnv* env, jclass clazz,
                                                       jlong jloader, jstring jname)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    const char* name = env->GetStringUTFChars(jname, 0);
    jobject result = loader->getCollider(name);
    env->ReleaseStringUTFChars(jname, name);
    return result;
}

JNIEXPORT jobject JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getConstraint(JNIEnv* env, jclass clazz,
                                                          jlong jloader, jstring jname)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    const char* name = env->GetStringUTFChars(jname, 0);
    jobject result = loader->getConstraint(name);
    env->ReleaseStringUTFChars(jname, name);
    return result;
}

JNIEXPORT jobject JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getConstraintBodyA(JNIEnv* env, jclass clazz,
                                                                 jlong jloader, jlong nativeConstraint)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    PhysicsConstraint* constraint = reinterpret_cast<PhysicsConstraint*>(nativeConstraint);
    return loader->getConstraintBodyA(constraint);
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
Java_com_samsungxr_physics_NativeBulletLoader_getRigidBodyName(JNIEnv* env, jclass clazz,
                                                        jlong jloader, jlong jbody)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    BulletRigidBody* body = reinterpret_cast<BulletRigidBody*>(jbody);
    const char* name = loader->getRigidBodyName(body);
    return env->NewStringUTF(name);
}

JNIEXPORT jstring JNICALL
Java_com_samsungxr_physics_NativeBulletLoader_getJointName(JNIEnv* env, jclass clazz,
                                                           jlong jloader, jlong jjoint)
{
    BulletFileLoader *loader = reinterpret_cast<BulletFileLoader*>(jloader);
    BulletJoint* joint = reinterpret_cast<BulletJoint*>(jjoint);
    const char* name = loader->getJointName(joint);
    return env->NewStringUTF(name);
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