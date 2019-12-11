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

#include <android/asset_manager.h>
#include <stdio.h>
#include "stdioext.h"

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <Serialize/BulletWorldImporter/btBulletWorldImporter.h>
#include <Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h>
#include <LinearMath/btQuaternion.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofSpringConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include <LinearMath/btSerializer.h>
#include <CommonInterfaces/CommonFileIOInterface.h>
#include <Utils/b3BulletDefaultFileIO.h>

#include "URDFConverter.h"
#include "engine/renderer/renderer.h"
#include "objects/components/mesh_collider.h"
#include "objects/components/box_collider.h"
#include "objects/components/sphere_collider.h"
#include "objects/components/capsule_collider.h"
#include "util/sxr_log.h"
#include "bullet_fileloader.h"
#include "bullet_world.h"
#include "bullet_rigidbody.h"
#include "bullet_joint.h"
#include "bullet_conetwistconstraint.h"
#include "bullet_fixedconstraint.h"
#include "bullet_generic6dofconstraint.h"
#include "bullet_hingeconstraint.h"
#include "bullet_point2pointconstraint.h"
#include "bullet_sliderconstraint.h"
#include "bullet_jointmotor.h"
#include "../../bullet3/include/BulletDynamics/Featherstone/btMultiBodyLink.h"
#include "../../bullet3/include/BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"


namespace sxr
{

    static int android_read(void *cookie, char *buf, int size)
    {
        return AAsset_read((AAsset *) cookie, buf, size);
    }

    static int android_write(void *cookie, const char *buf, int size)
    {
        return EACCES; // can't provide write access to the apk
    }

    static fpos_t android_seek(void *cookie, fpos_t offset, int whence)
    {
        return AAsset_seek((AAsset *) cookie, offset, whence);
    }

    static int android_close(void *cookie)
    {
        AAsset_close((AAsset *) cookie);
        return 0;
    }

    BulletFileLoader::FileIO::FileIO(AAssetManager *am, int fileIOType, const char *pathPrefix)
            : b3BulletDefaultFileIO(fileIOType, pathPrefix),
              mAssetManager(am)
    {
    }

    int BulletFileLoader::FileIO::fileOpen(const char *fileName, const char *mode)
    {
        //search a free slot
        int slot = -1;
        for (int i = 0; i < B3_FILEIO_MAX_FILES; i++)
        {
            if (m_fileHandles[i] == 0)
            {
                slot = i;
                break;
            }
        }
        if (slot >= 0)
        {
            FILE *f = fopen(fileName, mode);
            if (f)
            {
                m_fileHandles[slot] = f;
            }
            else
            {
                slot = -1;
            }
        }
        return slot;
    }


    FILE *BulletFileLoader::FileIO::fopen(const char *fileName, const char *mode)
    {
        if (fileName[0] == '/')
        {
            return ::fopen(fileName, mode);
        }
        if ((fileName[0] == '.') && (fileName[1] == '/'))
        {
            fileName += 2;
        }
        if (mBaseDir.empty())
        {
            const char* p = strrchr(fileName, '/');
            if (p)
            {
                mBaseDir.assign(fileName, p - fileName);
            }
        }
        std::string path(mBaseDir);
        path += fileName;
        AAsset *asset = AAssetManager_open(mAssetManager, path.c_str(), 0);
        if (asset)
        {
            return funopen(asset, android_read, android_write, android_seek, android_close);
        }
        return nullptr;
    }

/**
 * Construct the appropriate subclass of SXRCollider to match the input
 * Bullet collision object. Box, sphere, capsule and mesh colliders
 * are supported.
 * @param collider Bullet collider
 * @return Java SXRCollider object
 */
    jobject BulletFileLoader::createCollider(btCollisionObject *collider)
    {
        jobject o = 0;
        JNIEnv *env;
        btVector3 color(1, 1, 1);
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);

        btCollisionShape *inshape = collider->getCollisionShape();
        createCollisionShape(*env, inshape, o, color);
        collider->setCustomDebugColor(color);
        return o;
    }

    void
    BulletFileLoader::createCollisionShape(JNIEnv &env, btCollisionShape *shape, jobject &javaObj,
                                           btVector3 &debugColor)
    {
        switch (shape->getShapeType())
        {
            case BOX_SHAPE_PROXYTYPE:
            {
                btBoxShape *inbox = dynamic_cast<btBoxShape *>(shape);
                BoxCollider *bc = new BoxCollider();
                btVector3 he(inbox->getHalfExtentsWithoutMargin());

                bc->set_half_extents(he.x(), he.y(), he.z());
                LOGD("PHYSICS LOADER:    box collider dimensions = (%0.3f, %0.3f, %0.3f)",
                     he.x(), he.y(), he.z());
                javaObj = CreateInstance(env, "com/samsungxr/SXRBoxCollider",
                                         "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(),
                                         bc);
                debugColor.setValue(0, 0, 1);
                return;
            }

            case SPHERE_SHAPE_PROXYTYPE:
            {
                btSphereShape *sphere = dynamic_cast<btSphereShape *>(shape);
                SphereCollider *sc = new SphereCollider();
                float radius = sphere->getRadius();
                sc->set_radius(radius);
                LOGD("PHYSICS LOADER:    sphere collider radius = %0.3f", radius);
                javaObj = CreateInstance(env, "com/samsungxr/SXRSphereCollider",
                                         "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(),
                                         sc);
                return;
            }

            case CAPSULE_SHAPE_PROXYTYPE:
            {
                btCapsuleShape *capsule = dynamic_cast<btCapsuleShape *>(shape);
                CapsuleCollider *cc = new CapsuleCollider();
                float r = cc->getRadius();
                float h = cc->getHeight();
                const char *name = capsule->getName();
                char last = name[strlen(name) - 1];

                switch (last)
                {
                    case 'X':cc->setToXDirection();
                        break;

                    case 'Z':cc->setToZDirection();
                        break;
                }
                cc->setHeight(h);
                cc->setRadius(r);
                LOGD("PHYSICS LOADER:    box collider height = %0.3f, radius = %0.3f", h, r);
                javaObj = CreateInstance(env, "com/samsungxr/SXRCapsuleCollider",
                                         "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(),
                                         cc);
                debugColor.setValue(0, 1, 1);
                return;
            }

            case CONVEX_HULL_SHAPE_PROXYTYPE:
            {
                btConvexHullShape *hull = dynamic_cast<btConvexHullShape *>(shape);
                int numVerts = hull->getNumVertices();
                VertexBuffer *vb = Renderer::getInstance()
                        ->createVertexBuffer("float3 a_position", numVerts);
                float *verts = new float[numVerts * 3];
                btVector3 dimensions;

                copyHull(hull, verts, dimensions);
                vb->setFloatVec("a_position", (float *) verts, numVerts * 3, 3);
                delete[] verts;
                Mesh *mesh = new Mesh(*vb);
                MeshCollider *mc = new MeshCollider(mesh);
                LOGD("PHYSICS LOADER:    convex hull collider %d vertices dimensions = (%0.3f, %0.3f, %0.3f)",
                     numVerts, dimensions.x(), dimensions.y(), dimensions.z());
                javaObj = CreateInstance(env, "com/samsungxr/SXRMeshCollider",
                                         "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(),
                                         mc);
                debugColor.setValue(0, 1, 0);
                return;
            }

            case COMPOUND_SHAPE_PROXYTYPE:
            {
                btCompoundShape *cshape = dynamic_cast<btCompoundShape *>(shape);
                btTransform t = mTransformCoords * cshape->getChildTransform(0);

                shape = cshape->getChildShape(0);
                createCollisionShape(env, shape, javaObj, debugColor);
                {
                    const btVector3 &pos = t.getOrigin();
                    LOGD("PHYSICS LOADER:    compound collider pos = (%0.3f, %0.3f, %0.3f)",
                         pos.x(),
                         pos.y(), pos.z());
                }
                return;
            }

            default:
                if (shape->getShapeType() <= CUSTOM_POLYHEDRAL_SHAPE_TYPE)
                {
                    btPolyhedralConvexShape *polyshape = dynamic_cast<btPolyhedralConvexShape *>(shape);
                    const btConvexPolyhedron *poly = polyshape->getConvexPolyhedron();
                    int numVerts = polyshape->getNumVertices();
                    VertexBuffer *vb = Renderer::getInstance()
                            ->createVertexBuffer("float3 a_position", numVerts);
                    float *verts = new float[numVerts * 3];

                    vb->setFloatVec("a_position", (float *) verts, numVerts * 3, 3);
                    delete[] verts;
                    Mesh *mesh = new Mesh(*vb);
                    MeshCollider *mc = new MeshCollider(mesh);
                    LOGD("PHYSICS LOADER:    convex polygon collider %d vertices", numVerts);
                    javaObj = CreateInstance(env, "com/samsungxr/SXRMeshCollider",
                                             "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(),
                                             mc);
                }
                else
                {
                    MeshCollider *mc = new MeshCollider(nullptr);
                    javaObj = CreateInstance(env, "com/samsungxr/SXRMeshCollider",
                                             "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(),
                                             mc);
                }
        }
    }

    btConvexHullShape *BulletFileLoader::copyHull(const btConvexHullShape *input,
                                                  float *outverts, btVector3 &dimensions)
    {
        const btVector3 *inverts = input->getUnscaledPoints();
        int numVerts = input->getNumVertices();
        float *verts = outverts;
        btVector3 boxMin, boxMax;
        btConvexHullShape *outshape = nullptr;

        input->getAabb(btTransform::getIdentity(), boxMin, boxMax);
        for (int i = 0; i < numVerts; ++i)
        {
            btVector3 v = inverts[i];
            *verts++ = v.x();
            *verts++ = v.y();
            *verts++ = v.z();
        }
        dimensions.setX(boxMax.x() - boxMin.x());
        dimensions.setY(boxMax.y() - boxMin.y());
        dimensions.setZ(boxMax.z() - boxMin.z());
        return outshape;

    }

    const char *BulletFileLoader::getNameForPointer(void *ptr)
    {
        if (mBulletImporter)
        {
            return mBulletImporter->getNameForPointer(ptr);
        }
        if (mSerializer)
        {
            return mSerializer->findNameForPointer(ptr);
        }
        return nullptr;
    }

/**
 * Get the name of the scene object this constraint should attach to.
 */
    const char *BulletFileLoader::getConstraintName(PhysicsConstraint *constraint)
    {
        btTypedConstraint *btc = reinterpret_cast<btTypedConstraint *>(constraint->getUnderlying());
        btRigidBody &body = btc->getRigidBodyB();
        return getNameForPointer(&body);
    }

/**
 * Get the Java object that should be body A for the given constraint.
 * This lookup is done by extracting the name of the Bullet rigid body
 * or joint and using it to look up the Java object reference.
 * @param constraint C++ constraint
 * @return Java object which should be bodyA for the constraint
 */
    jobject BulletFileLoader::getConstraintBodyA(PhysicsConstraint *constraint)
    {
        PhysicsCollidable *bodyA = constraint->getBodyA();
        const char *name;

        if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
        {
            BulletRigidBody *bA = static_cast<BulletRigidBody *>(bodyA);
            return getRigidBody(bA->getName());
        }
        else if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
        {
            BulletJoint *bA = static_cast<BulletJoint *>(bodyA);

            return getJoint(bA->getName());
        }
        return 0;
    }

/**
 * Create Java SXRRigidBody and the corresponding C++ BulletRigidBody based on
 * the imported Bullet btRigidBody objects. Upon return it has
 * constructed a map between the name of the rigid body
 * and the SXRRigidBody object. The name of rigid body
 * is also the name of the scene object it should be attached to.
 * If the Bullet rigid body has a collider, a SXRCollider
 * subclass is created corresponding to the Bullet collision shape.
 */
    void BulletFileLoader::createRigidBodies(btBulletWorldImporter &importer)
    {
        JNIEnv *env;
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);
        for (int i = 0; i < importer.getNumRigidBodies(); i++)
        {
            btRigidBody *rb = static_cast<btRigidBody *>(importer.getRigidBodyByIndex(i));
            createRigidBody(*env, rb);
        }
    }

    void BulletFileLoader::createRigidBodies(btDynamicsWorld &world)
    {
        JNIEnv *env;
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);
        btCollisionObjectArray colliders = world.getCollisionObjectArray();
        for (int i = 0; i < colliders.size(); i++)
        {
            btCollisionObject *co = colliders.at(i);
            btRigidBody *rb = dynamic_cast<btRigidBody *>(co);
            if (rb)
            {
                world.removeCollisionObject(rb);
                createRigidBody(*env, rb);
            }
        }
    }

    void BulletFileLoader::createRigidBody(JNIEnv &env, btRigidBody *rb)
    {
        const char *name = getNameForPointer(rb);

        if (name == nullptr)    // we can't import rigid bodies without names
        {
            return;
        }
        if (mNeedRotate)
        {
            btTransform t = mTransformCoords * rb->getWorldTransform();
            rb->setWorldTransform(t);
        }
        BulletRigidBody *nativeBody = new BulletRigidBody(rb);
        jobject javaBody = CreateInstance(env, "com/samsungxr/physics/SXRRigidBody",
                                          "(Lcom/samsungxr/SXRContext;J)V",
                                          mContext.getObject(),
                                          reinterpret_cast<jlong>(nativeBody));
        nativeBody->setName(name);
        SmartLocalRef r(mJavaVM, javaBody);
        std::string s(name);
        mRigidBodies.emplace(s, r);

        jobject javaCollider = createCollider(nativeBody->getRigidBody());
        SmartLocalRef c(mJavaVM, javaCollider);
        mColliders.emplace(s, c);
    }

/**
 * Create Java SXRPhysicsJoint and the corresponding C++ BulletJoint based on
 * the imported Bullet btMultiBody objects. Upon return it has
 * constructed a map between the name of the joint
 * and the SXRPhysicsJoint object. The name of joint
 * is also the name of the scene object it should be attached to.
 */
    void BulletFileLoader::createJoints(btMultiBodyDynamicsWorld &world)
    {
        JNIEnv *env;
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);
        for (int i = mFirstMultiBody; i < world.getNumMultibodies(); i++)
        {
            btMultiBody *mb = world.getMultiBody(i);
            btMultiBodyLinkCollider *btc = mb->getBaseCollider();
            const char *name = getNameForPointer(btc);

            if (name == nullptr)    // cannot import bodies without names
            {
                name = getNameForPointer(mb);
                if (name == nullptr)
                {
                    continue;
                }
            }
            if (mNeedRotate)
            {
                btQuaternion qYtoZ;

                mRotateCoords.getRotation(qYtoZ);
                btQuaternion rot = qYtoZ * mb->getWorldToBaseRot();
                btVector3 pos = mb->getBasePos();
                mb->setWorldToBaseRot(rot);
                mb->setBasePos(rotatePoint(pos));
                btc->setWorldTransform(btTransform(rot, pos));
            }
            {
                const btVector3 &p = mb->getBaseWorldTransform().getOrigin();
                LOGD("PHYSICS LOADER: root joint %s pos = (%0.3f, %0.3f, %0.3f)",
                     name, p.x(), p.y(), p.z());
            }
            BulletJoint *rootJoint = new BulletRootJoint(mb);
            jobject javaJoint = CreateInstance(*env, "com/samsungxr/physics/SXRPhysicsJoint",
                                               "(Lcom/samsungxr/SXRContext;J)V",
                                               mContext.getObject(),
                                               reinterpret_cast<jlong>(rootJoint));
            std::string s(name);
            SmartLocalRef r(mJavaVM, javaJoint);
            auto pair = std::make_pair(s, r);

            rootJoint->setName(name);
            mJoints.emplace(pair);
            world.removeMultiBody(mb);
            if (btc != nullptr)
            {
                jobject javaCollider = createCollider(btc);
                SmartLocalRef c(mJavaVM, javaCollider);

                s = name;
                mColliders.emplace(s, c);
                world.removeCollisionObject(btc);
            }
            for (int i = 0; i < mb->getNumLinks(); ++i)
            {
                btMultibodyLink &link = mb->getLink(i);
                btMultiBodyLinkCollider *collider = link.m_collider;

                name = getNameForPointer(collider);
                if (name == nullptr)   // cannot map joints without names
                {
                    continue;
                }
                {
                    const btVector3 &p = mb->getRVector(i);
                    LOGD("PHYSICS LOADER: joint %s index = %d, Rvector = (%0.3f, %0.3f, %0.3f)",
                         name, i, p.x(), p.y(), p.z());
                }
                BulletJoint *nativeJoint = (BulletJoint *) (link.m_userPtr);
                javaJoint = CreateInstance(*env, "com/samsungxr/physics/SXRPhysicsJoint",
                                           "(Lcom/samsungxr/SXRContext;J)V",
                                           mContext.getObject(),
                                           reinterpret_cast<jlong>(nativeJoint));
                SmartLocalRef j(mJavaVM, javaJoint);

                s = name;
                mJoints.emplace(s, j);
                nativeJoint->setName(name);
                if (collider != nullptr)
                {
                    jobject javaCollider = createCollider(collider);
                    SmartLocalRef r(mJavaVM, javaCollider);

                    s = name;
                    mColliders.emplace(s, r);
                    world.removeCollisionObject(collider);
                }
            }
        }
    }

    btVector3 &BulletFileLoader::rotatePoint(btVector3 &p)
    {
        p = p * mRotateCoords;
        return p;
    }


/*
 * Create a Java SXRPoint2PointConstraint and the C++ PhysicsConstraint
 * based on the input btPoint2PointConstraint
 */
    jobject BulletFileLoader::createP2PConstraint(JNIEnv &env, btPoint2PointConstraint *p2p,
                                                  PhysicsConstraint *&constraint)
    {
        // Constraint userPointer will point to newly created BulletPoint2PointConstraint
        BulletPoint2PointConstraint *bp2p = new BulletPoint2PointConstraint(p2p);

        if (mNeedRotate)
        {
            btVector3 pivot = p2p->getPivotInA();
            p2p->setPivotA(rotatePoint(pivot));
            pivot = p2p->getPivotInB();
            p2p->setPivotB(rotatePoint(pivot));
        }
        constraint = bp2p;
        return CreateInstance(env, "com/samsungxr/physics/SXRPoint2PointConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bp2p));
    }

/*
 * Create a Java SXRPoint2PointConstraint and the C++ PhysicsConstraint
 * based on the input btPoint2PointConstraint
 */
    jobject BulletFileLoader::createP2PConstraint(JNIEnv &env, btMultiBodyPoint2Point *p2p,
                                                  PhysicsConstraint *&constraint)
    {
        // Constraint userPointer will point to newly created BulletPoint2PointConstraint
        BulletPoint2PointConstraint *bp2p = new BulletPoint2PointConstraint(p2p);

        if (mNeedRotate)
        {
            btVector3 pB = p2p->getPivotInB();
            p2p->setPivotInB(rotatePoint(pB));
        }
        constraint = bp2p;
        return CreateInstance(env, "com/samsungxr/physics/SXRPoint2PointConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bp2p));
    }

/*
 * Create a Java SXRHingeConstraint and the C++ PhysicsConstraint
 * based on the input btHingeConstraint
 */
    jobject BulletFileLoader::createHingeConstraint(JNIEnv &env, btHingeConstraint *hg,
                                                    PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btTransform &tA = hg->getAFrame();
            btTransform &tB = hg->getBFrame();

            tA = mTransformCoords * tA;
            tB = mTransformCoords * tB;
        }
        BulletHingeConstraint *bhg = new BulletHingeConstraint(hg);
        constraint = bhg;
        return CreateInstance(env, "com/samsungxr/physics/SXRHingeConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bhg));
    }

/*
 * Create a Java SXRConeTwistConstraint and the C++ PhysicsConstraint
 * based on the input btConeTwistConstraint
 */
    jobject BulletFileLoader::createConeTwistConstraint(JNIEnv &env, btConeTwistConstraint *ct,
                                                        PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btTransform tA = ct->getAFrame();
            btTransform tB = ct->getBFrame();

            tA = mTransformCoords * tA;
            tB = mTransformCoords * tB;
            ct->setFrames(tA, tB);
        }
        BulletConeTwistConstraint *bct = new BulletConeTwistConstraint(ct);
        constraint = bct;
        return CreateInstance(env, "com/samsungxr/physics/SXRConeTwistConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bct));
    }

/*
 * Create a Java SXRGenericConstraint and the C++ PhysicsConstraint
 * based on the input btGeneric6DofConstraint
 */
    jobject BulletFileLoader::createGenericConstraint(JNIEnv &env, btGeneric6DofConstraint *gen,
                                                      PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btTransform tA = gen->getFrameOffsetA();
            btTransform tB = gen->getFrameOffsetB();

            tA = mTransformCoords * tA;
            tB = mTransformCoords * tB;
            gen->setFrames(tA, tB);
        }
        BulletGeneric6dofConstraint *bg = new BulletGeneric6dofConstraint(gen);
        constraint = bg;
        return CreateInstance(env, "com/samsungxr/physics/SXRGenericConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bg));
    }

/*
 * Create a Java SXRGenericConstraint and the C++ PhysicsConstraint
 * based on the input btGeneric6DofConstraint
 */
jobject BulletFileLoader::createGenericConstraint(JNIEnv &env, btGeneric6DofSpringConstraint *gen,
                                                  PhysicsConstraint *&constraint)
{
    if (mNeedRotate)
    {
        btTransform tA = gen->getFrameOffsetA();
        btTransform tB = gen->getFrameOffsetB();

        tA = mTransformCoords * tA;
        tB = mTransformCoords * tB;
        gen->setFrames(tA, tB);
    }
    BulletGeneric6dofConstraint *bg = new BulletGeneric6dofConstraint(gen);
    constraint = bg;
    return CreateInstance(env, "com/samsungxr/physics/SXRGenericConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bg));
}

/*
 * Create a Java SXRGenericConstraint and the C++ PhysicsConstraint
 * based on the input btGeneric6DofSpring2Constraint
 */
    jobject
    BulletFileLoader::createSpringConstraint(JNIEnv &env, btGeneric6DofSpring2Constraint *gen,
                                             PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btTransform tA = gen->getFrameOffsetA();
            btTransform tB = gen->getFrameOffsetB();

            tA = mTransformCoords * tA;
            tB = mTransformCoords * tB;
            gen->setFrames(tA, tB);
        }
        BulletGeneric6dofConstraint *bg = new BulletGeneric6dofConstraint(gen);
        constraint = bg;
        return CreateInstance(env, "com/samsungxr/physics/SXRGenericConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bg));
    }

/*
 * Create a Java SXRFixedConstraint and the C++ PhysicsConstraint
 * based on the input btFixedConstraint
 */
    jobject BulletFileLoader::createFixedConstraint(JNIEnv &env, btFixedConstraint *fix,
                                                    PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btTransform tA = fix->getFrameOffsetA();
            btTransform tB = fix->getFrameOffsetB();

            tA = mTransformCoords * tA;
            tB = mTransformCoords * tB;
            fix->setFrames(tA, tB);
        }
        BulletFixedConstraint *bfix = new BulletFixedConstraint(fix);
        constraint = bfix;
        return CreateInstance(env, "com/samsungxr/physics/SXRFixedConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bfix));
    }

/*
 * Create a Java SXRFixedConstraint and the C++ PhysicsConstraint
 * based on the input btMultiBodyFixedConstraint
 */
    jobject BulletFileLoader::createFixedConstraint(JNIEnv &env, btMultiBodyFixedConstraint *fix,
                                                    PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btMatrix3x3 fA = fix->getFrameInA();
            btMatrix3x3 fB = fix->getFrameInB();
            btVector3 pA = fix->getPivotInA();
            btVector3 pB = fix->getPivotInB();
            float t;

            fA = mRotateCoords * fA;
            fB = mRotateCoords * fB;
            rotatePoint(pA);
            rotatePoint(pB);
            fix->setFrameInA(fA);
            fix->setFrameInB(fB);
            fix->setPivotInA(pA);
            fix->setPivotInB(pB);
        }
        BulletFixedConstraint *bfix = new BulletFixedConstraint(fix);
        constraint = bfix;
        return CreateInstance(env, "com/samsungxr/physics/SXRFixedConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(bfix));
    }

/*
 * Create a Java SXRSliderConstraint and the C++ PhysicsConstraint
 * based on the input btSliderConstraint
 */
    jobject BulletFileLoader::createSliderConstraint(JNIEnv &env, btSliderConstraint *sld,
                                                     PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btTransform tA = sld->getFrameOffsetA();
            btTransform tB = sld->getFrameOffsetB();

            tA = mTransformCoords * tA;
            tB = mTransformCoords * tB;
            sld->setFrames(tA, tB);
        }
        BulletSliderConstraint *c = new BulletSliderConstraint(sld);
        constraint = c;
        return CreateInstance(env, "com/samsungxr/physics/SXRSliderConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(c));
    }

/*
 * Create a Java SXRSliderConstraint and the C++ PhysicsConstraint
 * based on the input btMultiBodySliderConstraint
 */
    jobject BulletFileLoader::createSliderConstraint(JNIEnv &env, btMultiBodySliderConstraint *sld,
                                                     PhysicsConstraint *&constraint)
    {
        if (mNeedRotate)
        {
            btMatrix3x3 fA = sld->getFrameInA();
            btMatrix3x3 fB = sld->getFrameInB();
            btVector3 pA = sld->getPivotInA();
            btVector3 pB = sld->getPivotInB();

            fA = mRotateCoords * fA;
            fB = mRotateCoords * fB;
            rotatePoint(pA);
            rotatePoint(pB);
            sld->setFrameInA(fA);
            sld->setFrameInB(fB);
            sld->setPivotInA(pA);
            sld->setPivotInB(pB);
        }
        BulletSliderConstraint *c = new BulletSliderConstraint(sld);
        constraint = c;
        return CreateInstance(env, "com/samsungxr/physics/SXRSliderConstraint",
                              "(Lcom/samsungxr/SXRContext;J)V",
                              mContext.getObject(), reinterpret_cast<jlong>(c));
    }

/**
 * Create Java and the corresponding C++ constraints based on
 * the imported Bullet constraints. This function only examines
 * constraints between rigid bodies. Upon return it has
 * constructed a map between the name of the rigid body
 * the constraint is attached to and the Java constraint object.
 * The name of rigid body B is also the name of the scene
 * object this constraint should be attached to.
 */
    void BulletFileLoader::createConstraints(btBulletWorldImporter &importer)
    {
        JNIEnv *env;
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);

        for (int i = 0; i < importer.getNumConstraints(); i++)
        {
            btTypedConstraint *constraint = importer.getConstraintByIndex(i);
            createConstraint(*env, constraint);
        }
    }

    void BulletFileLoader::createConstraints(btDynamicsWorld &w)
    {
        JNIEnv *env;
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);

        for (int i = 0; i < w.getNumConstraints(); i++)
        {
            btTypedConstraint *constraint = w.getConstraint(i);
            w.removeConstraint(constraint);
            createConstraint(*env, constraint);
        }
    }

    void BulletFileLoader::createConstraint(JNIEnv &env, btTypedConstraint *constraint)
    {
        btRigidBody &bbA = constraint->getRigidBodyA();
        btRigidBody &bbB = constraint->getRigidBodyB();
        BulletRigidBody *rbA = reinterpret_cast<BulletRigidBody *>(bbA.getUserPointer());
        BulletRigidBody *rbB = reinterpret_cast<BulletRigidBody *>(bbB.getUserPointer());
        const char *nameB = getNameForPointer(&bbB);

        if ((rbA == nullptr) ||
            (rbB == nullptr) ||
            (nameB == nullptr))
        {
            // This constraint has at least one invalid rigid body and then it must to be ignored
            return;
        }
        jobject javaConstraint = nullptr;
        PhysicsConstraint *physCon = nullptr;

        if (constraint->getConstraintType() == btTypedConstraintType::POINT2POINT_CONSTRAINT_TYPE)
        {
            javaConstraint = createP2PConstraint(env,
                                                 dynamic_cast<btPoint2PointConstraint *>(constraint),
                                                 physCon);
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::HINGE_CONSTRAINT_TYPE)
        {
            javaConstraint = createHingeConstraint(env,
                                                   dynamic_cast<btHingeConstraint *>(constraint),
                                                   physCon);
        }
        else if (constraint->getConstraintType() ==
                 btTypedConstraintType::CONETWIST_CONSTRAINT_TYPE)
        {
            javaConstraint = createConeTwistConstraint(env,
                                                       dynamic_cast<btConeTwistConstraint *>(constraint),
                                                       physCon);
        }
        else if (constraint->getConstraintType() ==
                 btTypedConstraintType::D6_CONSTRAINT_TYPE)
        {
            // Blender exports generic constraint as generic spring constraint
            javaConstraint = createGenericConstraint(env,
                                                     dynamic_cast<btGeneric6DofConstraint *>(constraint),
                                                     physCon);
        }
        else if (constraint->getConstraintType() ==
                 btTypedConstraintType::D6_SPRING_CONSTRAINT_TYPE)
        {
            // Blender exports generic constraint as generic spring constraint
            javaConstraint = createGenericConstraint(env,
                                                     dynamic_cast<btGeneric6DofSpringConstraint *>(constraint),
                                                     physCon);
        }
        else if (constraint->getConstraintType() ==
                 btTypedConstraintType::D6_SPRING_2_CONSTRAINT_TYPE)
        {
            // URDF exporter generic constraint as btGeneric6DofSpring2Constraint
            javaConstraint = createSpringConstraint(env,
                                                    dynamic_cast<btGeneric6DofSpring2Constraint *>(constraint),
                                                    physCon);
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::FIXED_CONSTRAINT_TYPE)
        {
            // btFixedConstraint constraint is derived from btGeneric6DofSpring2Constraint and its
            // type is set to D6_SPRING_2_CONSTRAINT_TYPE instead of FIXED_CONSTRAINT_TYPE in
            // Bullet (at least up to) 2.87
            javaConstraint = createFixedConstraint(env,
                                                   dynamic_cast<btFixedConstraint *>(constraint),
                                                   physCon);
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::SLIDER_CONSTRAINT_TYPE)
        {
            javaConstraint = createSliderConstraint(env,
                                                    dynamic_cast<btSliderConstraint *>(constraint),
                                                    physCon);
        }
        if (javaConstraint)
        {
            const char *nameA = getNameForPointer(&bbA);
            jobject bodyA = getRigidBody(nameA);
            mConstraints.emplace(std::string(nameB), SmartLocalRef(mJavaVM, javaConstraint));
            if (bodyA)
            {
                CallVoidMethod(env, javaConstraint, "com/samsungxr/physics/SXRConstraint",
                               "setBodyA",
                               "(Lcom/samsungxr/physics/SXRPhysicsCollidable;)V",
                               env.NewLocalRef(bodyA));
            }
        }
    }

/**
 * Create Java and the corresponding C++ constraints based on
 * the imported Bullet multi body constraints. This function only examines
 * constraints between 2 joints or a joint and a rigid body. Upon return it has
 * constructed a map between the name of the joint or rigid body
 * the constraint is attached to and the Java constraint object.
 * The name of joint B or rigid body B is also the name of the scene
 * object this constraint should be attached to.
 */
    void BulletFileLoader::createMultiBodyConstraints(btMultiBodyDynamicsWorld &world)
    {
        JNIEnv *env;
        mJavaVM.GetEnv((void **) &env, SUPPORTED_JNI_VERSION);
        for (int i = mFirstMultiBody; i < world.getNumMultiBodyConstraints(); ++i)
        {
            btMultiBodyConstraint *c = world.getMultiBodyConstraint(i);
            world.removeMultiBodyConstraint(c);
            createMultiBodyConstraint(*env, c);
        }
    }

    void BulletFileLoader::createMultiBodyConstraint(JNIEnv &env, btMultiBodyConstraint *c)
    {
        btMultiBody *const mbA = c->getMultiBodyA();
        btMultiBody *const mbB = c->getMultiBodyB();
        btRigidBody *rbB = nullptr;
        PhysicsConstraint *physCon = nullptr;
        jobject javaConstraint;
        const char *nameB = nullptr;
        const char *constraintClass = nullptr;

        if (mbA->getUserPointer() == nullptr)
        {
            // This constraint has at least one invalid rigid body so it must to be ignored
            return;
        }
        if (typeid(c) == typeid(btMultiBodyFixedConstraint))
        {
            btMultiBodyFixedConstraint *fc = dynamic_cast<btMultiBodyFixedConstraint *>(c);
            javaConstraint = createFixedConstraint(env, fc, physCon);
            rbB = fc->getRigidBodyB();
        }
        else if (typeid(c) == typeid(btMultiBodySliderConstraint))
        {
            btMultiBodySliderConstraint *sc = dynamic_cast<btMultiBodySliderConstraint *>(c);

            javaConstraint = createSliderConstraint(env, sc, physCon);
            rbB = sc->getRigidBodyB();
        }
        else if (typeid(c) == typeid(btMultiBodyPoint2Point))
        {
            btMultiBodyPoint2Point *p2p = dynamic_cast<btMultiBodyPoint2Point *>(c);
            javaConstraint = createP2PConstraint(env, p2p, physCon);
            rbB = p2p->getRigidBodyB();
        }
        else
        {
            return;
        }
        if (mbB != nullptr)
        {
            nameB = mbB->getBaseName();
        }
        else if (rbB != nullptr)
        {
            nameB = getNameForPointer(rbB);
        }
        if (nameB && javaConstraint)
        {
            mConstraints.emplace(std::string(nameB), SmartLocalRef(mJavaVM, javaConstraint));
            jobject bodyA = getConstraintBodyA(physCon);
            if (bodyA)
            {
                CallVoidMethod(env, javaConstraint, "com/samsungxr/physics/SXRConstraint",
                               "setBodyA",
                               "(Lcom/samsungxr/physics/SXRPhysicsCollidable;)V",
                               env.NewLocalRef(bodyA));
            }
        }
    }

    BulletFileLoader::BulletFileLoader(jobject context, JavaVM &jvm, AAssetManager *am)
            : HybridObject(),
              mFileIO(am),
              mContext(jvm, context),
              mBulletImporter(nullptr),
              mURDFImporter(nullptr),
              mSerializer(nullptr),
              mJavaVM(jvm),
              mFirstMultiBody(0)
    {
    }

/**
 * Free all the resources used by the loader,
 * including the Java global references to the
 * Java objects created during import.
 */
    BulletFileLoader::~BulletFileLoader()
    {
        clear();
    }

/**
 * Parse the Bullet file in the input buffer and construct
 * the corresponding SXR objects: SXRRigidBody, SXRCollider,
 * and SXRConstraint. The physics objects are connected to
 * each other but are not attached to scene
 * objects or a physics world. The Java part of the physics
 * loader attaches the physics objects to the appropriate
 * scene nodes based on name matching.
 * This version of <i>parse</i> will <b>not</b>input Featherstone multi-body
 * hierarchies.
 *
 * @param buffer        input buffer with binary .bullet data
 * @param length        number of bytes in the input buffer
 * @param ignoreUpAxis  true to rotate scene using Y up
 * @return true if parsing succeeded, else false
 */
    bool BulletFileLoader::parse(char *buffer, size_t length, bool ignoreUpAxis)
    {
        bParse::btBulletFile *bullet_file = nullptr;

        try
        {
            bullet_file = new bParse::btBulletFile(buffer, length);
            mBulletImporter = new btBulletWorldImporter(nullptr);
            bool result = mBulletImporter->loadFileFromMemory(bullet_file);
            if (!result)
            {
                clear();
                delete bullet_file;
                LOGE("BULLET: Bullet parse failed");
            }
        }
        catch (...)
        {
            clear();
            LOGE("BULLET: exception during file load");
            delete bullet_file;
            return false;
        }
        if (ignoreUpAxis)
        {
            mNeedRotate = false;
        }
        else if (bullet_file->getFlags() & bParse::FD_DOUBLE_PRECISION)
        {
            btDynamicsWorldDoubleData *ddata = reinterpret_cast<btDynamicsWorldDoubleData *>
                            (bullet_file->m_dynamicsWorldInfo[0]);
            double *gravity = reinterpret_cast<double *>(&ddata->m_gravity);
            mNeedRotate = gravity[2] != 0.0;
            mRotateCoords = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f};
            mTransformCoords.setBasis(mRotateCoords);
            mTransformCoords.setOrigin(btVector3(0, 0, 0));
        }
        else
        {
            btDynamicsWorldFloatData *fdata =
                    reinterpret_cast<btDynamicsWorldFloatData *>(bullet_file
                            ->m_dynamicsWorldInfo[0]);
            float *gravity = reinterpret_cast<float *>(&fdata->m_gravity);
            mNeedRotate = gravity[2] != 0.f;
            mRotateCoords = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f};
            mTransformCoords.setBasis(mRotateCoords);
            mTransformCoords.setOrigin(btVector3(0, 0, 0));
        }

        delete bullet_file;

        createRigidBodies(*mBulletImporter);
        createConstraints(*mBulletImporter);
        return true;
    }

/**
 * Parse the Bullet file in the input buffer and construct
 * the corresponding SXR objects: SXRRigidBody, SXRCollider,
 * SXRPhysicsJoint and SXRConstraint. The physics objects
 * are connected to each other and attached to the
 * input world but are not attached to scene nodes.
 * The Java part of the physics loader attaches the
 * physics objects to the appropriate scene nodes based on name matching.
 * This version of <i>parse</i> will input Featherstone multi-body
 * hierarchies.
 *
 * @param world         Bullet multi-body dynamics world to add objects to.
 * @param buffer        input buffer with binary .bullet data
 * @param length        number of bytes in the input buffer
 * @param ignoreUpAxis  true to rotate scene using Y up
 * @return true if parsing succeeded, else false
 */
    bool BulletFileLoader::parse(BulletWorld *world, char *buffer, size_t length, bool ignoreUpAxis)
    {
        btMultiBodyDynamicsWorld *worldMB = dynamic_cast<btMultiBodyDynamicsWorld*>
                                            (world->getPhysicsWorld());
        bParse::btBulletFile* bullet_file = new bParse::btBulletFile(buffer, length);
        btBulletWorldImporter* importer = new btMultiBodyWorldImporter(worldMB);
        if (worldMB)
        {
            mFirstMultiBody = worldMB->getNumMultibodies();
        }
        else
        {
            return false;
        }
        mBulletImporter = importer;
        mBulletImporter->loadFileFromMemory(bullet_file);
        if (ignoreUpAxis)
        {
            mNeedRotate = false;
        }
        else if (bullet_file->getFlags() & bParse::FD_DOUBLE_PRECISION)
        {
            btDynamicsWorldDoubleData *ddata =
                    reinterpret_cast<btDynamicsWorldDoubleData *>(bullet_file
                            ->m_dynamicsWorldInfo[0]);
            double *gravity = reinterpret_cast<double *>(&ddata->m_gravity);
            mNeedRotate = gravity[2] != 0.0;
            mRotateCoords = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f};
            mTransformCoords.setBasis(mRotateCoords);
            mTransformCoords.setOrigin(btVector3(0, 0, 0));
        }
        else
        {
            btDynamicsWorldFloatData *fdata =
                    reinterpret_cast<btDynamicsWorldFloatData *>(bullet_file
                            ->m_dynamicsWorldInfo[0]);
            float *gravity = reinterpret_cast<float *>(&fdata->m_gravity);
            mNeedRotate = gravity[2] != 0.f;
            mRotateCoords = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f};
            mTransformCoords.setBasis(mRotateCoords);
            mTransformCoords.setOrigin(btVector3(0, 0, 0));
        }

        delete bullet_file;
        createRigidBodies(*importer);
        createConstraints(*importer);

        if (worldMB)
        {
            createJoints(*worldMB);
            createMultiBodyConstraints(*worldMB);
        }
        return true;
    }

/**
 * Parse the Bullet file in the input buffer and construct
 * the corresponding SXR objects: SXRRigidBody, SXRCollider,
 * SXRPhysicsJoint and SXRConstraint. The physics objects
 * are connected to each other and attached to the
 * input world but are not attached to scene nodes.
 * The Java part of the physics loader attaches the
 * physics objects to the appropriate scene nodes based on name matching.
 * This version of <i>parse</i> will input Featherstone multi-body
 * hierarchies.
 *
 * @param world         Bullet multi-body dynamics world to add objects to.
 * @param buffer        input buffer with binary .bullet data
 * @param length        number of bytes in the input buffer
 * @param ignoreUpAxis  true to rotate scene using Y up
 * @return true if parsing succeeded, else false
 */
    bool BulletFileLoader::parseURDF(BulletWorld *world, const char *xmldata, bool ignoreUpAxis, bool multiBody)
    {
        btMultiBodyDynamicsWorld* worldMB = dynamic_cast<btMultiBodyDynamicsWorld *>(world->getPhysicsWorld());
        btDynamicsWorld* bulletWorld = world->getPhysicsWorld();
        const btVector3& gravity = bulletWorld->getGravity();

        if (worldMB && worldMB->getNumMultibodies() > 0)
        {
            mFirstMultiBody = worldMB->getNumMultibodies();
        }
        mSerializer = new btDefaultSerializer();
        mURDFImporter = new URDFConverter(multiBody, &mFileIO);
        worldMB = mURDFImporter->importPhysics(xmldata, worldMB);
//        mURDFImporter->exportPhysics("/storage/emulated/0/temp.bullet", mSerializer);
        mURDFImporter->registerNames(*mSerializer, false);
        bulletWorld->setGravity(gravity);
        mNeedRotate = !ignoreUpAxis;    // URDF is Z up
        if (mNeedRotate)
        {
            mRotateCoords = {1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f};
            mTransformCoords.setBasis(mRotateCoords);
            mTransformCoords.setOrigin(btVector3(0, 0, 0));
        }
        createRigidBodies(*worldMB);
        createConstraints(*worldMB);
        if (bulletWorld == worldMB)
        {
            createJoints(*worldMB);
            createMultiBodyConstraints(*worldMB);
        }
        return true;
    }

/**
 * Export the physics world provided as binary and store in the given file.
 * @param world     physics world to export
 * @param fileName  full path name of file to get exported data.
 * @return true for successful export, false on failure.
 */
    bool BulletFileLoader::exportBullet(BulletWorld *world, const char *fileName)
    {
        FILE *f = fopen(fileName, "wb");

        if (f == NULL)
        {
            return false;
        }
        btSerializer *s = new btDefaultSerializer;
        registerNames(world->getPhysicsWorld(), s);
        world->getPhysicsWorld()->serialize(s);
        fwrite(s->getBufferPointer(), s->getCurrentBufferSize(), 1, f);
        fclose(f);
        delete s;
        return true;
    }

/**
 * Detach all the imported physics objects from the Bullet world
 * (if there is one). Free all the outstanding Java global
 * references to the created objects. If they have not been
 * attached to scene nodes, the Java physics objects will
 * eventually be garbage collected and the Bullet objects
 * will go away as well.
 */
    void BulletFileLoader::clear()
    {
        mConstraints.clear();
        mRigidBodies.clear();
        mJoints.clear();
        if (mBulletImporter)
        {
            delete mBulletImporter;
            mBulletImporter = nullptr;
        }
        if (mURDFImporter)
        {
            delete mURDFImporter;
            mURDFImporter = nullptr;
        }
        if (mSerializer)
        {
            delete mSerializer;
            mSerializer = nullptr;
        }
    }

    void BulletFileLoader::registerNames(btDynamicsWorld* world, btSerializer* s)
    {
        btCollisionObjectArray colliders = world->getCollisionObjectArray();
        for (int i = 0; i < colliders.size(); i++)
        {
            btCollisionObject* bulletCollider = colliders.at(i);
            PhysicsCollidable* nativeCollider = (PhysicsCollidable*) bulletCollider->getUserPointer();

            if (nativeCollider)
            {
                const char* name = nativeCollider->getName();

                if (name)
                {
                    s->registerNameForPointer(bulletCollider, name);
                }
            }
        }
    }

/**
 * Get the Java SXRRigidBody associated with the input name.
 * @param name name of rigid body to find.
 * @return SXRRigidBody object or null if not found.
 */
    jobject BulletFileLoader::getRigidBody(const char* name)
    {
        std::string s(name);
        const SmartLocalRef& r = mRigidBodies[s];
        JNIEnv* env;

        mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
        return env->NewLocalRef(r.getObject());
    }

/**
 * Get the Java SXRPhysicsJoint associated with the input name.
 * @param name name of joint to find.
 * @return SXRPhysicsJoint object or null if not found.
 */
    jobject BulletFileLoader::getJoint(const char* name)
    {
        std::string s(name);
        const SmartLocalRef& r = mJoints[s];
        JNIEnv* env;

        mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
        return env->NewLocalRef(r.getObject());
    }

/**
 * Get the Java SXRCollider associated with the input name.
 * @param name name of collidable body for this collider.
 * @return SXRRCollider object for named body.
 */
    jobject BulletFileLoader::getCollider(const char* name)
    {
        std::string s(name);
        const SmartLocalRef& r = mColliders[s];
        JNIEnv* env;

        mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
        return env->NewLocalRef(r.getObject());
    }

/**
 * Create a Java array containing all the SXRRigidBody
 * physics objects constructed from the Bullet file.
 *
 * @return rigid body array
 */
    jobjectArray BulletFileLoader::getRigidBodies()
    {
        int n = mRigidBodies.size();
        JNIEnv* env;

        mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
        jclass clazz = env->FindClass("com/samsungxr/physics/SXRRigidBody");
        jobjectArray array = env->NewObjectArray(n, clazz, nullptr);
        int i = 0;

        for (const auto& entry : mRigidBodies)
        {
            jobject o = entry.second.getObject();
            if (o)
            {
                env->SetObjectArrayElement(array, i, env->NewLocalRef(o));
            }
            ++i;
        }
        return array;
    }

/**
 * Create a Java array containing all the SXRPhysicsJoint
 * objects constructed from the Bullet file.
 *
 * @return joint array
 */
    jobjectArray BulletFileLoader::getJoints()
    {
        int n = mJoints.size();
        JNIEnv* env;

        mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
        jclass clazz = env->FindClass("com/samsungxr/physics/SXRPhysicsJoint");
        jobjectArray array = env->NewObjectArray(n, clazz, nullptr);
        int i = 0;

        for (const auto& entry : mJoints)
        {
            jobject o = entry.second.getObject();
            if (o)
            {
                env->SetObjectArrayElement(array, i, env->NewLocalRef(o));
            }
            ++i;
        }
        return array;
    }

/**
 * Create a Java array containing all the SXRPConstraint
 * objects constructed from the Bullet file.
 *
 * @return constraint array
 */
    jobjectArray BulletFileLoader::getConstraints()
    {
        int n = mConstraints.size();
        JNIEnv* env;

        mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
        jclass clazz = env->FindClass("com/samsungxr/physics/SXRConstraint");
        jobjectArray array = env->NewObjectArray(n, clazz, nullptr);
        int i = 0;

        for (const auto& entry : mConstraints)
        {
            jobject o = entry.second.getObject();
            if (o)
            {
                env->SetObjectArrayElement(array, i, env->NewLocalRef(o));
            }
            ++i;
        }
        return array;
    }

}
