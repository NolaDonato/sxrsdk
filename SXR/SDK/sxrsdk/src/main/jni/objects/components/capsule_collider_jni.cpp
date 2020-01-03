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

#include "capsule_collider.h"
#include "glm/gtc/type_ptr.hpp"

namespace sxr
{
    extern "C" {

    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_ctor(JNIEnv* env, jclass obj)
    {
        return reinterpret_cast<jlong>(new CapsuleCollider());
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_setRadius(JNIEnv* env, jclass obj, jlong jcollider,
                                                       jfloat radius)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        collider->setRadius(radius);
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_setHeight(JNIEnv* env, jclass obj, jlong jcollider,
                                                       jfloat height)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        collider->setHeight(height);
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_setToXDirection(JNIEnv* env, jclass obj,
                                                             jlong jcollider)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        collider->setToXDirection();
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_setToYDirection(JNIEnv* env, jclass obj,
                                                             jlong jcollider)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        collider->setToYDirection();
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_setToZDirection(JNIEnv* env, jclass obj,
                                                             jlong jcollider)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        collider->setToZDirection();
    }

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_getHeight(JNIEnv* env, jclass obj, jlong jcollider)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        return collider->getHeight();
    }

    JNIEXPORT jfloat JNICALL
    Java_com_samsungxr_NativeCapsuleCollider_getRadius(JNIEnv* env, jclass obj, jlong jcollider)
    {
        CapsuleCollider* collider = reinterpret_cast<CapsuleCollider*>(jcollider);
        return collider->getRadius();
    }
    }
}