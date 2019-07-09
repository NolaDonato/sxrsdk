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
    Java_com_samsungxr_physics_NativePhysicsLink_ctor(JNIEnv * env, jobject obj, jfloat mass);

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getComponentType(JNIEnv * env, jobject obj);

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getBoneID(JNIEnv * env, jobject obj, jlong jmultibody);


    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getMass(JNIEnv * env, jobject obj, jlong jmultibody);

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_setMass(JNIEnv * env, jobject obj, jlong jmultibody, jfloat mass);
}

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_ctor(JNIEnv * env, jobject obj, jfloat mass)
    {
        return reinterpret_cast<jlong>(new PhysicsJoint(mass));
    }

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getComponentType(JNIEnv * env, jobject obj)
    {
        return PhysicsJoint::getComponentType();
    }

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getMass(JNIEnv * env, jobject obj, jlong jmultibody)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jmultibody);
        return mb->getMass();
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getMass(JNIEnv * env, jobject obj, jlong jmultibody, jfloat mass)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jmultibody);
        mb->setMass(mass);
    }

    JNIEXPORT jint JNICALL
    Java_com_samsungxr_physics_NativePhysicsLink_getBoneID(JNIEnv * env, jobject obj, jlong jmultibody)
    {
        PhysicsJoint* mb = reinterpret_cast<PhysicsJoint*>(jmultibody);
        return mb->getBoneID();
    }

}