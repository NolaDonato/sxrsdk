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

#include "mesh_collider.h"

namespace sxr {
extern "C" {


JNIEXPORT jlong JNICALL
Java_com_samsungxr_NativeMeshCollider_ctor(JNIEnv *env,
                                           jclass obj, jboolean useBounds)
{
    return reinterpret_cast<jlong>(new MeshCollider(useBounds));
}

JNIEXPORT jlong JNICALL
Java_com_samsungxr_NativeMeshCollider_ctorMesh(JNIEnv *env, jclass obj, jlong jmesh)
{
    Mesh *mesh = reinterpret_cast<Mesh *>(jmesh);
    return reinterpret_cast<jlong>(new MeshCollider(mesh));
}

JNIEXPORT jlong JNICALL
Java_com_samsungxr_NativeMeshCollider_ctorMeshPicking(JNIEnv *env, jclass obj,
                                                      jlong jmesh, jboolean pickCoordinates)
{
    Mesh *mesh = reinterpret_cast<Mesh *>(jmesh);
    return reinterpret_cast<jlong>(new MeshCollider(mesh, pickCoordinates));
}

JNIEXPORT long JNICALL
Java_com_samsungxr_NativeMeshCollider_getMesh(JNIEnv *env, jclass obj, jlong jmesh_collider)
{
    MeshCollider *meshcollider = reinterpret_cast<MeshCollider *>(jmesh_collider);
    return reinterpret_cast<long>(meshcollider->mesh());
}


JNIEXPORT void JNICALL
Java_com_samsungxr_NativeMeshCollider_setMesh(JNIEnv *env, jclass obj,
                                              jlong jmesh_collider, jlong jmesh)
{
    MeshCollider *meshcollider = reinterpret_cast<MeshCollider *>(jmesh_collider);
    Mesh *mesh = reinterpret_cast<Mesh *>(jmesh);
    meshcollider->set_mesh(mesh);
}

JNIEXPORT void JNICALL
Java_com_samsungxr_NativeMeshCollider_setMeshType(JNIEnv *env, jclass obj,
                                                  jlong jmesh_collider, jint meshtype)
{
    MeshCollider *meshcollider = reinterpret_cast<MeshCollider *>(jmesh_collider);
    meshcollider->set_mesh_type(meshtype);
}
}
}
