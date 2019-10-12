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

#include <btBulletCollisionCommon.h>
#include <btBulletDynamicsCommon.h>
#include <Serialize/BulletWorldImporter/btMultiBodyWorldImporter.h>
#include <LinearMath/btQuaternion.h>
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodySliderConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <Serialize/BulletFileLoader/btBulletFile.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btRigidBody.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>

#include "engine/renderer/renderer.h"
#include "objects/components/mesh_collider.h"
#include "objects/components/box_collider.h"
#include "objects/components/sphere_collider.h"
#include "objects/components/capsule_collider.h"
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

static btMatrix3x3 matrixInvIdty(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);
static btTransform transformInvIdty(matrixInvIdty);


namespace sxr {

jobject BulletFileLoader::createCollider(btCollisionObject* collider)
{
    jobject o = 0;

    btCollisionShape* shape = collider->getCollisionShape();
    if (mNeedRotate)
    {
        btTransform t = collider->getWorldTransform();
        collider->setWorldTransform(t * transformInvIdty);
    }
    switch (shape->getShapeType())
    {
        case BOX_SHAPE_PROXYTYPE:
        {
            btBoxShape* box = dynamic_cast<btBoxShape*>(shape);
            BoxCollider* bc = new BoxCollider();
            btVector3 he(box->getHalfExtentsWithoutMargin());
            bc->set_half_extents(he.x(), he.y(), he.z());
            o = CreateInstance(mEnv, "com/samsungxr/physics/SXRBoxCollider",
                               "void(Lcom/samsungxr/SXRContext;J)", mContext.getObject(), bc);
        }

        case SPHERE_SHAPE_PROXYTYPE:
        {
            btSphereShape* sphere = dynamic_cast<btSphereShape*>(shape);
            SphereCollider* sc = new SphereCollider();
            float radius = sphere->getRadius();
            sc->set_radius(radius);
            o = CreateInstance(mEnv, "com/samsungxr/physics/SXRSphereCollider",
                               "void(Lcom/samsungxr/SXRContext;J)", mContext.getObject(), sc);
        }

        case CAPSULE_SHAPE_PROXYTYPE:
        {
            btCapsuleShape* capsule = dynamic_cast<btCapsuleShape*>(shape);
            CapsuleCollider* cc = new CapsuleCollider();
            float r = cc->getRadius();
            float h = cc->getHeight();
            const char* name = capsule->getName();
            char last = name[strlen(name) - 1];

            if (last == 'X')
            {
                cc->setToXDirection();
            }
            else if (last == 'Z')
            {
                cc->setToZDirection();
            }
            cc->setHeight(h);
            cc->setRadius(r);
            o = CreateInstance(mEnv, "com/samsungxr/physics/SXRCapsuleCollider",
                              "void(Lcom/samsungxr/SXRContext;J)", mContext.getObject(), cc);
        }

        case CONVEX_HULL_SHAPE_PROXYTYPE:
        {
            btConvexHullShape* hull = dynamic_cast<btConvexHullShape *>(shape);
            int numVerts = hull->getNumVertices();
            const btVector3* verts = hull->getUnscaledPoints();
            VertexBuffer* vb = Renderer::getInstance()->createVertexBuffer("float3 a_position", numVerts);
            IndexBuffer* ib = Renderer::getInstance()->createIndexBuffer(2, numVerts);
            short* indices = (short*) alloca(numVerts * 2);
            for (int i = 0; i < numVerts; ++i)
            {
                indices[i] = i;
            }
            vb->setFloatVec("a_position", (float*) verts, numVerts * 3, 3);
            Mesh* mesh = new Mesh(*vb);
            mesh->setIndexBuffer(ib);
            MeshCollider* mc = new MeshCollider(mesh);
            o = CreateInstance(mEnv, "com/samsungxr/physics/SXRMeshCollider",
                               "void(Lcom/samsungxr/SXRContext;J)", mContext.getObject(), mc);

        }
    }
    return o;
}

const char* BulletFileLoader::getRigidBodyName(BulletRigidBody* body)
{
    return mImporter->getNameForPointer(body);
}

const char* BulletFileLoader::getJointName(BulletJoint* joint)
{
    btMultiBody* mb = joint->getMultiBody();
    int i = joint->getJointIndex();

    if (i < 0)
    {
        return mb->getBaseName();
    }
    btMultibodyLink& link = mb->getLink(i);
    return link.m_jointName;
}

const char* BulletFileLoader::getConstraintName(PhysicsConstraint* constraint)
{
    return mImporter->getNameForPointer(constraint);
}

jobject BulletFileLoader::getConstraintBodyA(PhysicsConstraint* constraint)
{
    PhysicsCollidable* bodyA = constraint->getBodyA();
    const char* name;

    if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
    {
        BulletRigidBody* bA = static_cast<BulletRigidBody*>(bodyA);
        name = mImporter->getNameForPointer(bA->getRigidBody());
        return getRigidBody(name);
    }
    else if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
    {
        BulletJoint* bA = static_cast<BulletJoint*>(bodyA);
        btMultiBody* mb = bA->getMultiBody();

        if (bA->getJointIndex() < 0)
        {
            name = mb->getBaseName();
        }
        else
        {
            btMultibodyLink& l = mb->getLink(bA->getJointIndex());
            name = l.m_jointName;
        }
        return getJoint(name);
    }
    return 0;
}


void BulletFileLoader::createRigidBodies()
{
    for (int i = 0; i < mImporter->getNumRigidBodies(); i++)
    {
        btRigidBody *rb = static_cast<btRigidBody*>(mImporter->getRigidBodyByIndex(i));
        const char* name = mImporter->getNameForPointer(rb);

        if (name == nullptr)    // we can't import rigid bodies without names
        {
            continue;
        }
        BulletRigidBody* nativeBody = new BulletRigidBody(rb);
        jobject javaBody = CreateInstance(mEnv, "com/samsungxr/physics/SXRRigidBody",
                                          "void(Lcom/samsungxr/SXRContext;J)",
                                          mContext.getObject(), reinterpret_cast<jlong>(nativeBody));
        SmartLocalRef r(mEnv, javaBody);
        std::string s(name);
        mRigidBodies.emplace(s, r);
        jobject javaCollider = createCollider(nativeBody->getRigidBody());
        SmartLocalRef c(mEnv, javaCollider);

        s = name;
        mColliders.emplace(s, c);
    }
}

void BulletFileLoader::createJoints()
{
    for (int i = mFirstMultiBody; i < mWorld->getNumMultibodies(); i++)
    {
        btMultiBody* mb = mWorld->getMultiBody(i);
        const char* name = mb->getBaseName();

        if (name == nullptr)    // cannot import bodies without names
        {
            continue;
        }
        if (mNeedRotate)
        {
            btQuaternion rot = mb->getWorldToBaseRot();
            btVector3    pos = mb->getBasePos();
            btTransform  t(rot, pos);
            t *= transformInvIdty;
            mb->setBaseWorldTransform(t);
        }
        const BulletJoint* nativeJoint = new BulletRootJoint(mb);
        btMultiBodyLinkCollider* btc = mb->getBaseCollider();
        jobject javaJoint = CreateInstance(mEnv, "com/samsungxr/physics/SXRPhysicsJoint",
                                          "void(Lcom/samsungxr/SXRContext;J)",
                                          mContext.getObject(), reinterpret_cast<jlong>(nativeJoint));
        std::string s(name);
        SmartLocalRef r(mEnv, javaJoint);
        auto pair = std::make_pair(s, r);
        mJoints.emplace(pair);
        if (btc != nullptr)
        {
            jobject javaCollider = createCollider(btc);
            SmartLocalRef c(mEnv, javaCollider);

            s = name;
            mColliders.emplace(s, c);
        }
        for (int i = 0; i < mb->getNumLinks(); ++i)
        {
            btMultibodyLink& link = mb->getLink(i);
            const char* jointName = link.m_jointName;

            if (jointName == nullptr)   // cannot map joints without names
            {
                continue;
            }
            if (mNeedRotate)
            {
                btTransform t = link.m_cachedWorldTransform;
                t *= transformInvIdty;
                link.m_cachedWorldTransform = t;
            }
            nativeJoint = reinterpret_cast<const BulletJoint*>(link.m_userPtr);
            javaJoint = CreateInstance(mEnv, "com/samsungxr/physics/SXRPhysicsJoint",
                                       "void(Lcom/samsungxr/SXRContext;J)",
                                       mContext.getObject(), reinterpret_cast<jlong>(nativeJoint));
            SmartLocalRef j(mEnv, javaJoint);

            s = name;
            mJoints.emplace(s, j);
            btc = mb->getBaseCollider();
            if (btc != nullptr)
            {
                jobject javaCollider = createCollider(btc);
                SmartLocalRef r(mEnv, javaCollider);

                s = name;
                mColliders.emplace(s, r);
            }
        };
    }
}

jobject BulletFileLoader::createP2PConstraint(btPoint2PointConstraint* p2p)
{
    // Constraint userPointer will point to newly created BulletPoint2PointConstraint
    BulletPoint2PointConstraint* bp2p = new BulletPoint2PointConstraint(p2p);

    if (mNeedRotate)
    {
        // Adapting pivot to SXRf coordinates system
        btVector3 pivot = p2p->getPivotInA();
        float t = pivot.getZ();
        pivot.setZ(-pivot.getY());
        pivot.setY(t);
        p2p->setPivotA(pivot);

        pivot = p2p->getPivotInB();
        t = pivot.getZ();
        pivot.setZ(-pivot.getY());
        pivot.setY(t);
        p2p->setPivotB(pivot);
    }
    return CreateInstance(mEnv, "com/samsungxr/physics/SXRPoint2PointConstraint",
                          "void(Lcom/samsungxr/SXRContext;J)",
                          mContext.getObject(), reinterpret_cast<jlong>(bp2p));
}

jobject BulletFileLoader::createHingeConstraint(btHingeConstraint* hg)
{
    BulletHingeConstraint *bhg = new BulletHingeConstraint(hg);

    if (mNeedRotate)
    {
        btTransform t = hg->getAFrame();

        hg->getAFrame().mult(transformInvIdty, t);
        t = hg->getBFrame();
        hg->getBFrame().mult(transformInvIdty, t);
    }
    return CreateInstance(mEnv, "com/samsungxr/physics/SXRHingeConstraint",
                          "void(Lcom/samsungxr/SXRContext;J)",
                          mContext.getObject(), reinterpret_cast<jlong>(bhg));
}

jobject BulletFileLoader::createConeTwistConstraint(btConeTwistConstraint* ct)
{
    BulletConeTwistConstraint *bct = new BulletConeTwistConstraint(ct);

    if (mNeedRotate)
    {
        btTransform tA = ct->getAFrame();
        btTransform tB = ct->getBFrame();
        btTransform t = tA;

        tA.mult(transformInvIdty, t);
        t = tB;
        tB.mult(transformInvIdty, t);
        ct->setFrames(tA, tB);
    }
    return CreateInstance(mEnv, "com/samsungxr/physics/SXRConeTwistConstraint",
                          "void(Lcom/samsungxr/SXRContext;J)",
                          mContext.getObject(), reinterpret_cast<jlong>(bct));
}

jobject BulletFileLoader::createGenericConstraint(btGeneric6DofConstraint* gen)
{
    BulletGeneric6dofConstraint *bct = new BulletGeneric6dofConstraint(gen);

    if (mNeedRotate)
    {
        btTransform tA = gen->getFrameOffsetA();
        btTransform tB = gen->getFrameOffsetB();
        btTransform t = tA;

        tA.mult(transformInvIdty, t);
        t = tB;
        tB.mult(transformInvIdty, t);
        gen->setFrames(tA, tB);
    }
    return CreateInstance(mEnv, "com/samsungxr/physics/SXRGenericConstraint",
                          "void(Lcom/samsungxr/SXRContext;J)",
                          mContext.getObject(), reinterpret_cast<jlong>(bct));
}

jobject BulletFileLoader::createFixedConstraint(btFixedConstraint* fix)
{
    BulletFixedConstraint *bfix = new BulletFixedConstraint(fix);

    if (mNeedRotate)
    {
        btTransform tA = fix->getFrameOffsetA();
        btTransform tB = fix->getFrameOffsetB();

        btTransform t = tA;
        tA.mult(transformInvIdty, t);

        t = tB;
        tB.mult(transformInvIdty, t);

        fix->setFrames(tA, tB);
    }
    return CreateInstance(mEnv, "com/samsungxr/physics/SXRFixedConstraint",
                          "void(Lcom/samsungxr/SXRContext;J)",
                          mContext.getObject(), reinterpret_cast<jlong>(bfix));
}

jobject BulletFileLoader::createSliderConstraint(btSliderConstraint* sld)
{
    BulletSliderConstraint* c = new BulletSliderConstraint(sld);
    return CreateInstance(mEnv, "com/samsungxr/physics/SXRSliderConstraint",
                          "void(Lcom/samsungxr/SXRContext;J)",
                          mContext.getObject(), reinterpret_cast<jlong>(c));
}

void BulletFileLoader::createConstraints()
{
    for (int i = 0; i < mImporter->getNumConstraints(); i++)
    {
        btTypedConstraint *constraint = mImporter->getConstraintByIndex(i);
        btRigidBody const *rbA = &constraint->getRigidBodyA();
        btRigidBody const *rbB = &constraint->getRigidBodyB();
        jobject javaConstraint;
        const char* name = mImporter->getNameForPointer(rbB);

        if (rbA->getUserPointer() == nullptr || rbB->getUserPointer() == nullptr)
        {
            // This constraint has at least one invalid rigid body and then it must to be ignored
            continue;
        }

        if (constraint->getConstraintType() == btTypedConstraintType::POINT2POINT_CONSTRAINT_TYPE)
        {
            javaConstraint = createP2PConstraint(static_cast<btPoint2PointConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::HINGE_CONSTRAINT_TYPE)
        {
            javaConstraint = createHingeConstraint(static_cast<btHingeConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::CONETWIST_CONSTRAINT_TYPE)
        {
            javaConstraint =
                    createConeTwistConstraint(static_cast<btConeTwistConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::D6_CONSTRAINT_TYPE ||
                 constraint->getConstraintType() == btTypedConstraintType::D6_SPRING_CONSTRAINT_TYPE)
        {
            // Blender exports generic constraint as generic spring constraint
            javaConstraint =
                    createGenericConstraint(static_cast<btGeneric6DofConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::FIXED_CONSTRAINT_TYPE ||
                 constraint->getConstraintType() == btTypedConstraintType::D6_SPRING_2_CONSTRAINT_TYPE)
        {
            // btFixedConstraint constraint is derived from btGeneric6DofSpring2Constraint and its
            // type is set to D6_SPRING_2_CONSTRAINT_TYPE instead of FIXED_CONSTRAINT_TYPE in
            // Bullet (at least up to) 2.87
            javaConstraint = createFixedConstraint(static_cast<btFixedConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::SLIDER_CONSTRAINT_TYPE)
        {
            javaConstraint = createSliderConstraint(static_cast<btSliderConstraint*>(constraint));
        }
        if (name && javaConstraint)
        {
            mConstraints.emplace(std::string(name), SmartLocalRef(mEnv, javaConstraint));
        }
    }
}

    void BulletFileLoader::createMultiBodyConstraints()
    {
        for (int i = mFirstMultiBody; i < mWorld->getNumMultiBodyConstraints(); ++i)
        {
            btMultiBodyConstraint* c = mWorld->getMultiBodyConstraint(i);
            btMultiBody* const mbA = c->getMultiBodyA();
            btMultiBody* const mbB = c->getMultiBodyB();
            btRigidBody* rbB = nullptr;
            PhysicsConstraint* nativeConstraint;
            jobject javaConstraint;
            const char* nameB = nullptr;
            PhysicsCollidable* bodyB;

            if (mbA->getUserPointer() == nullptr)
            {
                // This constraint has at least one invalid rigid body and then it must to be ignored
                continue;
            }
            if (typeid(c) == typeid(btMultiBodyFixedConstraint))
            {
                btMultiBodyFixedConstraint* fc = dynamic_cast<btMultiBodyFixedConstraint*>(c);
                BulletFixedConstraint* bfc = new BulletFixedConstraint(fc);
                javaConstraint =  CreateInstance(mEnv, "com/samsungxr/physics/SXRFixedConstraint",
                                                "void(Lcom/samsungxr/SXRContext;J)",
                                                 mContext.getObject(), reinterpret_cast<long>(bfc));
                rbB = fc->getRigidBodyB();
            }
            else if (typeid(c) == typeid(btMultiBodySliderConstraint))
            {
                btMultiBodySliderConstraint* sc = dynamic_cast<btMultiBodySliderConstraint*>(c);
                BulletSliderConstraint* bsc = new BulletSliderConstraint(sc);
                javaConstraint =  CreateInstance(mEnv, "com/samsungxr/physics/SXRSliderConstraint",
                                                 "void(Lcom/samsungxr/SXRContext;J)",
                                                 mContext.getObject(), reinterpret_cast<long>(bsc));
                rbB = sc->getRigidBodyB();
            }
            else if (typeid(c) == typeid(btMultiBodyPoint2Point))
            {
                btMultiBodyPoint2Point* p2p = dynamic_cast<btMultiBodyPoint2Point*>(c);
                BulletPoint2PointConstraint* bp2p = new BulletPoint2PointConstraint(p2p);
                javaConstraint =  CreateInstance(mEnv, "com/samsungxr/physics/SXRPoint2PointConstraint",
                                                 "void(Lcom/samsungxr/SXRContext;J)",
                                                 mContext.getObject(), reinterpret_cast<long>(bp2p));
                rbB = p2p->getRigidBodyB();
            }
            if (mbB != nullptr)
            {
                nameB = mbB->getBaseName();
            }
            else if (rbB != nullptr)
            {
                nameB = mImporter->getNameForPointer(rbB);
            }
            if (nameB && javaConstraint)
            {
                mConstraints.emplace(std::string(nameB), SmartLocalRef(mEnv, javaConstraint));
            }
        }
    }

BulletFileLoader::BulletFileLoader(jobject context, JNIEnv* env, char *buffer, size_t length, bool ignoreUpAxis)
:   mEnv(env),
    mContext(env, context),
    mFirstMultiBody(0)
{
    bParse::btBulletFile *bullet_file = new bParse::btBulletFile(buffer, length);

    mImporter = new btBulletWorldImporter(nullptr);
    mImporter->loadFileFromMemory(bullet_file);
    mWorld = nullptr;
    if (ignoreUpAxis)
    {
        mNeedRotate = false;
    }
    else if (bullet_file->getFlags() & bParse::FD_DOUBLE_PRECISION)
    {
        btDynamicsWorldDoubleData* ddata =
                reinterpret_cast<btDynamicsWorldDoubleData*>(bullet_file->m_dynamicsWorldInfo[0]);
        double *gravity = reinterpret_cast<double *>(&ddata->m_gravity);
        mNeedRotate = gravity[2] != 0.0;
    }
    else
    {
        btDynamicsWorldFloatData* fdata =
                reinterpret_cast<btDynamicsWorldFloatData*>(bullet_file->m_dynamicsWorldInfo[0]);
        float *gravity = reinterpret_cast<float*>(&fdata->m_gravity);
        mNeedRotate = gravity[2] != 0.f;
    }

    delete bullet_file;

    createRigidBodies();
    createConstraints();
}

BulletFileLoader::BulletFileLoader(jobject context, JNIEnv* env, btMultiBodyDynamicsWorld* world, char *buffer, size_t length, bool ignoreUpAxis)
:   mEnv(env),
    mContext(env, context)
{
    bParse::btBulletFile *bullet_file = new bParse::btBulletFile(buffer, length);
    btMultiBodyWorldImporter* importer = new btMultiBodyWorldImporter(world);
    mWorld = world;
    mImporter = importer;
    mImporter->loadFileFromMemory(bullet_file);

    if (ignoreUpAxis)
    {
        mNeedRotate = false;
    }
    else if (bullet_file->getFlags() & bParse::FD_DOUBLE_PRECISION)
    {
        btDynamicsWorldDoubleData* ddata =
                reinterpret_cast<btDynamicsWorldDoubleData*>(bullet_file->m_dynamicsWorldInfo[0]);
        double *gravity = reinterpret_cast<double *>(&ddata->m_gravity);
        mNeedRotate = gravity[2] != 0.0;
    }
    else
    {
        btDynamicsWorldFloatData* fdata =
                reinterpret_cast<btDynamicsWorldFloatData*>(bullet_file->m_dynamicsWorldInfo[0]);
        float *gravity = reinterpret_cast<float*>(&fdata->m_gravity);
        mNeedRotate = gravity[2] != 0.f;
    }
    if (world->getNumMultibodies() > 0)
    {
        mFirstMultiBody = world->getNumMultibodies();
        importer->convertAllObjects(bullet_file);
    }
    delete bullet_file;

    createRigidBodies();
    createJoints();
    createConstraints();
}

BulletFileLoader::~BulletFileLoader()
{
    int i;

    for (i = 0; i < mImporter->getNumConstraints(); i++)
    {
        btTypedConstraint *constraint = mImporter->getConstraintByIndex(i);
        PhysicsConstraint *phcons = static_cast<PhysicsConstraint*>(constraint->getUserConstraintPtr());
        if (nullptr != phcons && ((PhysicsConstraint*)-1) != phcons)
        {
            // Constraint is valid, but was it attached?
            if (nullptr != phcons->owner_object())
            {
                continue;
            }
            else
            {
                delete phcons;
            }
        }
        else
        {
            delete constraint;
        }
    }

    for (i = 0; i < mImporter->getNumRigidBodies(); i++) {
        btRigidBody *rb = static_cast<btRigidBody*>(mImporter->getRigidBodyByIndex(i));
        BulletRigidBody *brb = static_cast<BulletRigidBody*>(rb->getUserPointer());
        if (nullptr != brb)
        {
            // Rigid body is valid, but was it attached?
            if (nullptr != brb->owner_object())
            {
                continue;
            }
            else
            {
                delete brb;
            }
        }
        else
        {
            delete rb;
        }
    }

    for (i = 0; i < mWorld->getNumMultibodies(); i++) {
        btMultiBody* mb = mWorld->getMultiBody(i);
        BulletRootJoint* rootJoint = reinterpret_cast<BulletRootJoint*>(mb->getUserPointer());
        if (nullptr != rootJoint)
        {
            // Rigid body is valid, but was it attached?
            if (nullptr != rootJoint->owner_object())
            {
                continue;
            }
            else
            {
                delete rootJoint;
            }
        }
        else
        {
            delete rootJoint;
        }
    }

    delete mImporter;
}

jobject BulletFileLoader::getRigidBody(const char* name)
{
    std::string s(name);
    const SmartLocalRef& r = mRigidBodies[s];
    return r.getObject();
}

jobject BulletFileLoader::getJoint(const char* name)
{
    std::string s(name);
    const SmartLocalRef& r = mJoints[s];
    return r.getObject();
}

jobject BulletFileLoader::getCollider(const char* name)
{
    std::string s(name);
    const SmartLocalRef& r = mColliders[s];
    return r.getObject();
}

jobject BulletFileLoader::getConstraint(const char* name)
{
    std::string s(name);
    const SmartLocalRef& r = mConstraints[s];
    return r.getObject();
}

jobjectArray BulletFileLoader::getRigidBodies()
{
    int n = mRigidBodies.size();
    jclass clazz = mEnv->FindClass("com/samsungxr/physics/SXRRigidBody");
    jobjectArray array = mEnv->NewObjectArray(n, clazz, nullptr);
    int i = 0;

    for (const auto& entry : mRigidBodies)
    {
        const SmartLocalRef& r = entry.second;
        mEnv->SetObjectArrayElement(array, i++, r.getObject());
    }
    return array;
}

jobjectArray BulletFileLoader::getJoints()
{
    int n = mRigidBodies.size();
    jclass clazz = mEnv->FindClass("com/samsungxr/physics/SXRPhysicsJoint");
    jobjectArray array = mEnv->NewObjectArray(n, clazz, nullptr);
    int i = 0;

    for (const auto& entry : mJoints)
    {
        const SmartLocalRef& r = entry.second;
        mEnv->SetObjectArrayElement(array, i++, r.getObject());
    }
    return array;
}

jobjectArray BulletFileLoader::getConstraints()
{
    int n = mRigidBodies.size();
    jclass clazz = mEnv->FindClass("com/samsungxr/physics/SXRConstraint");
    jobjectArray array = mEnv->NewObjectArray(n, clazz, nullptr);
    int i = 0;

    for (const auto& entry : mConstraints)
    {
        const SmartLocalRef& r = entry.second;
        mEnv->SetObjectArrayElement(array, i++, r.getObject());
    }
    return array;
}

}
