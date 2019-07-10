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

#include "physics_world.h"
#include "physics_joint.h"
#include "bullet/bullet_joint.h"
#include "bullet/bullet_world.h"
#include "jni_utils.h"

namespace sxr {
extern "C"
{
    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_ctorRoot(JNIEnv* env, jclass obj, jfloat mass, jint numBones);

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_ctorLink(JNIEnv* env, jclass obj, jobject jparent, jint boneID, jfloat mass);

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getComponentType(JNIEnv* env, jobject obj);

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getBoneID(JNIEnv* env, jclass obj, jlong jjoint);

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getMass(JNIEnv* env, jclass obj, jlong jjoint);

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_setMass(JNIEnv* env, jclass obj, jlong jjoint, jfloat mass);

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getFriction(JNIEnv* env, jclass obj, jlong jjoint);

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_setFriction(JNIEnv* env, jclass obj, jlong jjoint, jfloat friction);

    JNIEXPORT jobject JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getName(JNIEnv* env, jclass obj, jlong jjoint);

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_setName(JNIEnv* env, jclass obj, jlong jjoint, jobject jname);
}

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_ctorRoot(JNIEnv* env, jclass obj, jfloat mass, jint numBones)
    {
        return reinterpret_cast<jlong>(new PhysicsJoint(mass, numBones));
    }

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_ctorLink(JNIEnv* env, jclass obj, jobject jparent, jfloat mass, jint boneID)
    {
        PhysicsJoint* parent = reinterpret_cast<PhysicsJoint*>(jparent);
        return reinterpret_cast<jlong>(new PhysicsJoint(parent, boneID, mass));
    }

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getComponentType(JNIEnv * env, jobject obj)
    {
        return PhysicsJoint::getComponentType();
    }

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getMass(JNIEnv* env, jclass obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getMass();
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_setMass(JNIEnv* env, jclass obj, jlong jjoint, jfloat mass)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        mb->setMass(mass);
    }

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getFriction(JNIEnv* env, jclass obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getFriction();
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_setFriction(JNIEnv* env, jclass obj, jlong jjoint, jfloat friction)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        mb->setFriction(friction())
    }

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getBoneID(JNIEnv* env, jclass obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getBoneID();
    }

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getBoneID(JNIEnv* env, jclass obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getBoneID();
    }

    JNIEXPORT jobject JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_getName(JNIEnv* env, jclass obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        const char* name = mb->getName();
        return env->NewStringUTF(name);
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsJoint_setName(JNIEnv* env, jclass obj, jlong jjoint, jobject jname)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        const char* name = env->GetStringUTFChars(jname, 0);

        mb->setName(name);
        env->ReleaseStringUTFChars(jname, name);
    }

}