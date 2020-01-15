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

#include "bullet_sxr_utils.h"

#include "glm/glm.hpp"
#include "glm/gtc/matrix_inverse.hpp"
#include <glm/gtx/quaternion.hpp>
#include <contrib/glm/gtc/type_ptr.hpp>

#include <BulletCollision/CollisionShapes/btShapeHull.h>
#include <BulletCollision/CollisionShapes/btCapsuleShape.h>
#include <BulletCollision/CollisionShapes/btBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btScaledBvhTriangleMeshShape.h>
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletCollision/CollisionShapes/btPolyhedralConvexShape.h>
#include <BulletCollision/CollisionShapes/btStaticPlaneShape.h>
#include <BulletCollision/CollisionShapes/btEmptyShape.h>
#include <BulletCollision/CollisionShapes/btStridingMeshInterface.h>
#include <BulletCollision/CollisionShapes/btCompoundShape.h>
#include <contrib/glm/gtx/matrix_decompose.hpp>
#include "objects/vertex_buffer.h"
#include "util/sxr_log.h"

namespace sxr {

Mesh* getMeshFromCollider(MeshCollider*);

btCollisionShape* convertCollider2CollisionShape(Collider *collider, btTransform& t)
{
    btQuaternion rot = t.getRotation();
    btVector3    pos = t.getOrigin();

    if (collider == nullptr)
    {
        return nullptr;
    }
    /*
     * If a transform was set on this shape, use a compound shape type to apply it.
     */
    rot.normalize();
    if ((fabs(pos.length()) > 0.0001f) ||
        (fabs(btVector3(rot.x(), rot.y(), rot.z()).length()) > 0.0001f))
    {
       btTransform identity;
       btCompoundShape* cshape;
       btCollisionShape* child;

       identity.setIdentity();
       child = convertCollider2CollisionShape(collider, identity);
       cshape = new btCompoundShape(0, 1);
       cshape->addChildShape(t, child);
       return cshape;
    }
    if (collider->shape_type() == COLLIDER_SHAPE_BOX)
    {
        return convertBoxCollider2CollisionShape(static_cast<BoxCollider *>(collider));
    }
    if (collider->shape_type() == COLLIDER_SHAPE_SPHERE)
    {
        return convertSphereCollider2CollisionShape(static_cast<SphereCollider *>(collider));
    }
    if (collider->shape_type() == COLLIDER_SHAPE_CAPSULE)
    {
        return convertCapsuleCollider2CollisionShape(static_cast<CapsuleCollider *>(collider));
    }
    if (collider->shape_type() == COLLIDER_SHAPE_HULL)
    {
        return convertMeshCollider2ConvexHull(static_cast<MeshCollider*>(collider));
    }
    return convertMeshCollider2CollisionShape(static_cast<MeshCollider*>(collider));
}

btCollisionShape *convertSphereCollider2CollisionShape(SphereCollider *collider)
{
    Node* owner = collider->owner_object();
    Mesh* mesh;
    float radius = collider->get_radius();

    if (radius > 0)
    {
        return new btSphereShape(radius);
    }
    if ((owner->render_data() != nullptr) && ((mesh = owner->render_data()->mesh()) != nullptr))
    {
        BoundingVolume bv = mesh->getBoundingVolume();
        if (bv.radius() > 0)
        {
            return new btSphereShape(bv.radius());
        }
    }
    LOGE("PHYSICS: Sphere collider with zero volume");
    return nullptr;
}

btCollisionShape *convertCapsuleCollider2CollisionShape(CapsuleCollider *collider)
{
    btCollisionShape *shape = NULL;

    switch (collider->getDirection())
    {
        case CAPSULE_DIRECTION_Y:
            shape = new btCapsuleShape(collider->getRadius(), collider->getHeight());
            break;

        case CAPSULE_DIRECTION_X:
            shape = new btCapsuleShapeX(collider->getRadius(), collider->getHeight());
            break;

        case CAPSULE_DIRECTION_Z:
            shape = new btCapsuleShapeZ(collider->getRadius(), collider->getHeight());
            break;
    }
    return shape;
}

btCollisionShape* convertBoxCollider2CollisionShape(BoxCollider *collider)
{
    Node* owner = collider->owner_object();
    btVector3 extents(collider->get_half_extents().x,
                      collider->get_half_extents().y,
                      collider->get_half_extents().z);
    Mesh* mesh;

    if (extents.length() > 0)
    {
        return new btBoxShape(extents);
    }
    if ((owner->render_data() != nullptr) && ((mesh = owner->render_data()->mesh()) != nullptr))
    {
        BoundingVolume bv = mesh->getBoundingVolume();
        if (bv.radius() > 0)
        {
            extents.setX((bv.max_corner().x - bv.min_corner().x) / 2.0f);
            extents.setY((bv.max_corner().y - bv.min_corner().y) / 2.0f);
            extents.setZ((bv.max_corner().z - bv.min_corner().z) / 2.0f);
            return new btBoxShape(extents);
        }
    }
    LOGE("PHYSICS: Box collider with zero volume");
    return nullptr;
}

class MeshInterface : public btStridingMeshInterface
{
private:
    const VertexBuffer* mVertexBuffer;
    const IndexBuffer* mIndexBuffer;

public:
    MeshInterface(const VertexBuffer* vb, const IndexBuffer* ib)
    : mVertexBuffer(vb), mIndexBuffer(ib) { }

    virtual void getLockedVertexIndexBase(unsigned char** vertexbase, int& numverts,  PHY_ScalarType& type, int& stride,
                                          unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype,
                                          int subpart)
    {
        const btStridingMeshInterface* THIS = (const btStridingMeshInterface*) this;
        THIS->getLockedReadOnlyVertexIndexBase((const unsigned char**) vertexbase, numverts, type, stride,
                                               (const unsigned char**) indexbase, indexstride, numfaces, indicestype);
    }

    virtual void getLockedReadOnlyVertexIndexBase(const unsigned char** vertexbase, int& numverts, PHY_ScalarType& type, int& stride,
                                                  const unsigned char** indexbase, int& indexstride, int& numfaces, PHY_ScalarType& indicestype,
                                                  int subpart) const
    {
        *vertexbase = (const unsigned char*) mVertexBuffer->getVertexData();
        numverts = mVertexBuffer->getVertexCount();
        stride = mVertexBuffer->getLayoutSize();
        type = PHY_FLOAT;
        if (mIndexBuffer)
        {
            *indexbase = (const unsigned char*) mIndexBuffer->getIndexData();
            numfaces = mIndexBuffer->getIndexCount() / 3;
            indexstride = mIndexBuffer->getIndexSize();
            if (indexstride == 2)
            {
                indicestype = PHY_SHORT;
            }
            else
            {
                indicestype = PHY_INTEGER;
            }
        }
        else
        {
            *indexbase = nullptr;
            numfaces = 0;
            indexstride = 0;
            indicestype = PHY_INTEGER;
        }
    }

    virtual void unLockVertexBase(int subpart)
    {
        mVertexBuffer->lock();
        if (mIndexBuffer)
        {
            mIndexBuffer->lock();
        }
    }

    virtual void unLockReadOnlyVertexBase(int subpart) const
    {
        mVertexBuffer->unlock();
        if (mIndexBuffer)
        {
            mIndexBuffer->unlock();
        }
    }

    virtual int getNumSubParts() const { return 1; }
    virtual void preallocateVertices(int numverts) { }
    virtual void preallocateIndices(int numindices) { }
};

btCollisionShape *convertMeshCollider2CollisionShape(MeshCollider *collider)
{
    Mesh* mesh = getMeshFromCollider(collider);
    if (mesh == NULL)
    {
        return NULL;
    }
    MeshInterface* imesh = new MeshInterface(mesh->getVertexBuffer(), mesh->getIndexBuffer());
    collider->set_physics_info(imesh);
    btBvhTriangleMeshShape* cshape = new btBvhTriangleMeshShape(imesh, true);
    return new btScaledBvhTriangleMeshShape(cshape, btVector3(1, 1, 1));
}

Mesh* getMeshFromCollider(MeshCollider* collider)
{
    if (collider == NULL)
    {
        return NULL;
    }
    Mesh *mesh = collider->mesh();
    if (mesh == NULL)
    {
        Node *owner = collider->owner_object();
        if (owner == NULL)
        {
            return NULL;
        }
        RenderData *rdata = owner->render_data();
        if (rdata == NULL)
        {
            return NULL;
        }
        mesh = rdata->mesh();
        if (mesh == NULL)
        {
            return NULL;
        }
    }
    return mesh;
}

btCollisionShape *convertMeshCollider2ConvexHull(MeshCollider *collider)
{
    Mesh *mesh = getMeshFromCollider(collider);

    if (mesh == NULL)
    {
        return NULL;
    }
    return createConvexHullShapeFromMesh(mesh);
}

btConvexHullShape* createConvexHullShapeFromMesh(Mesh *mesh)
{
    btConvexHullShape* hull_shape = NULL;
    btConvexHullShape* initial_hull_shape = NULL;
    btShapeHull *hull_shape_optimizer = NULL;

    initial_hull_shape = new btConvexHullShape();
    mesh->getVertexBuffer()->forAllVertices("a_position", [initial_hull_shape](int iter, const float* v)
    {
        btVector3 vertex(v[0], v[1], v[2]);
        initial_hull_shape->addPoint(vertex);
    });

    btScalar margin(initial_hull_shape->getMargin());
    hull_shape_optimizer = new btShapeHull(initial_hull_shape);
    hull_shape_optimizer->buildHull(margin);

    hull_shape = new btConvexHullShape((btScalar*) hull_shape_optimizer->getVertexPointer(),
                                        hull_shape_optimizer->numVertices());
    return hull_shape;
}

btTransform convertTransform2btTransform(Transform *t)
{
    glm::mat4 m;
    if (t->owner_object()->parent())
    {
        m = t->getModelMatrix(false);
    }
    else
    {
        m = t->getLocalModelMatrix();
    }
    return convertTransform2btTransform(m);
}

btTransform convertTransform2btTransform(const glm::mat4& m)
{
    glm::vec4 p(m[3]);
    glm::quat q = glm::quat_cast(m);
    btVector3 pos(p.x, p.y, p.z);
    btQuaternion rot(q.x, q.y, q.z, q.w);
    return btTransform(rot, pos);
}

void convertbtTransformToMatrix(const btTransform& t, glm::mat4 &m)
{
    btQuaternion q(t.getRotation());
    btVector3    p(t.getOrigin());
    glm::quat    rot(q.x(), q.y(), q.z(), q.w());

    m = glm::mat4_cast(rot);
    m[3][0] = p.x();
    m[3][1] = p.y();
    m[3][2] = p.x();
}

void decomposeMatrix(glm::mat4& m, glm::vec3& scale, glm::quat& rotation, glm::vec3& translation)
{
    glm::vec3 skew;
    glm::vec4 perspective;

    glm::decompose(m, scale, rotation, translation, skew, perspective);
}

}
