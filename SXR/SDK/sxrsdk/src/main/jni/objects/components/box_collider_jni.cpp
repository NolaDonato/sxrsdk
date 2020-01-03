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
#include <jni.h>
#include "box_collider.h"
#include "glm/gtc/type_ptr.hpp"

namespace sxr
{
    extern "C"
    {
    JNIEXPORT jlong JNICALL
    Java_com_samsungxr_NativeBoxCollider_ctor(JNIEnv* env, jclass obj)
    {
        return reinterpret_cast<jlong>(new BoxCollider());
    }

    JNIEXPORT void JNICALL
    Java_com_samsungxr_NativeBoxCollider_setHalfExtents(JNIEnv* env, jclass obj, jlong jcollider,
                                                        jfloat x, jfloat y, jfloat z)
    {
        BoxCollider* collider = reinterpret_cast<BoxCollider*>(jcollider);
        collider->set_half_extents(x, y, z);
    }

    JNIEXPORT jfloatArray JNICALL
    Java_com_samsungxr_NativeBoxCollider_getHalfExtents(JNIEnv* env, jclass obj, jlong jcollider)
    {
        BoxCollider* c = reinterpret_cast<BoxCollider*>(jcollider);
        const glm::vec3& v = c->get_half_extents();
        jfloatArray result = env->NewFloatArray(3);

        env->SetFloatArrayRegion(result, 0, 3, glm::value_ptr(v));
        return result;
    }
    }
}
