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

namespace sxr {
extern "C"
{
    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_ctorRoot(JNIEnv * env, jobject obj, jfloat mass, int numBones);

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_ctorLink(JNIEnv * env, jobject obj, jobject jparent, int boneID, jfloat mass);

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getComponentType(JNIEnv * env, jobject obj);

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getBoneID(JNIEnv * env, jobject obj, jlong jjoint);


    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getMass(JNIEnv * env, jobject obj, jlong jjoint);

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_setMass(JNIEnv * env, jobject obj, jlong jjoint, jfloat mass);

    JNIEXPORT jobject JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getName(JNIEnv * env, jobject obj, jlong jjoint);

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_setName(JNIEnv * env, jobject obj, jlong jjoint, jobject jname);
}

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_ctorRoot(JNIEnv * env, jobject obj, jfloat mass, int numBones)
    {
        return reinterpret_cast<jlong>(new PhysicsJoint(mass, numBones));
    }

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_ctorLink(JNIEnv * env, jobject obj, jobject jparent, jfloat mass, int boneID)
    {
        PhysicsJoint* parent = reinterpret_cast<PhysicsJoint*>(jparent);
        return reinterpret_cast<jlong>(new PhysicsJoint(parent, boneID, mass));
    }

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getComponentType(JNIEnv * env, jobject obj)
    {
        return PhysicsJoint::getComponentType();
    }

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getMass(JNIEnv * env, jobject obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getMass();
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_setMass(JNIEnv * env, jobject obj, jlong jjoint, jfloat mass)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        mb->setMass(mass);
    }

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getBoneID(JNIEnv * env, jobject obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getBoneID();
    }

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getBoneID(JNIEnv * env, jobject obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        return mb->getBoneID();
    }

    JNIEXPORT jobject JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getName(JNIEnv * env, jobject obj, jlong jjoint)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        const char* name = mb->getName();
        return env->NewStringUTF(name);
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_setName(JNIEnv * env, jobject obj, jlong jjoint, jobject jname)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jjoint);
        const char* name = env->GetStringUTFChars(jname, 0);

        mb->setName(name);
        env->ReleaseStringUTFChars(jname, name);
    }

}