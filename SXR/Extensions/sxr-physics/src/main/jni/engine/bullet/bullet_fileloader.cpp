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
#include <BulletDynamics/ConstraintSolver/btConstraintSolver.h>
#include <BulletDynamics/ConstraintSolver/btConeTwistConstraint.h>
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h>
#include <BulletDynamics/ConstraintSolver/btHingeConstraint.h>
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/ConstraintSolver/btSliderConstraint.h>
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

static btMatrix3x3 matrixInvIdty(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);
static btTransform transformInvIdty(matrixInvIdty);


namespace sxr {


BulletFileLoader::Collidable::Collidable(JNIEnv* env, jobject context, BulletRigidBody* body)
{
    jobject b =  createInstance(env, "com/samsungxr/physics/SXRRigidBody", "void(Lcom/samsungxr/SXRContext;J)", (jvalue*) body);
    Body = SmartLocalRef(env, b);
    makeCollider(env, context, body->getRigidBody()->getCollisionShape());
}

BulletFileLoader::Collidable::Collidable(JNIEnv* env, jobject context, BulletJoint* joint)
{
    btCollisionShape* shape;
    btMultiBody* mb = joint->getMultiBody();

    Body = SmartLocalRef(env, joint);
    if (joint->getParent() == nullptr)
    {
        shape = mb->getBaseCollider()->getCollisionShape();
    }
    else
    {
        btMultibodyLink& link = mb->getLink(joint->getJointIndex());
        shape = link.m_collider->getCollisionShape();
    }
    makeCollider(env, context, shape);
}


jobject BulletFileLoader::Collidable::createInstance(JNIEnv* env, const char* className, const char* signature, ...)
{
    va_list args;

    jclass clazz = GetGlobalClassReference(*env, className);

    if (NULL == clazz)
    {
        return 0;
    }
    jmethodID ctr_id = env->GetMethodID(clazz, "<init>", signature);

    if (NULL == ctr_id)
    {
        return 0;
    }
    va_start(args, ctr_id);
    return env->NewObjectV(clazz, ctr_id, args);
}

void BulletFileLoader::Collidable::makeCollider(JNIEnv* env, jobject context, btCollisionShape* shape)
{
    jobject o = 0;
    switch (shape->getShapeType())
    {
        case BOX_SHAPE_PROXYTYPE:
        {
            btBoxShape* box = dynamic_cast<btBoxShape*>(shape);
            BoxCollider* bc = new BoxCollider();
            btVector3 he(box->getHalfExtentsWithoutMargin());
            bc->set_half_extents(he.x(), he.y(), he.z());
            o = createInstance(env, "com/samsungxr/physics/SXRBoxCollider",
                               "void(Lcom/samsungxr/SXRContext;J)", context, bc);
        }

        case SPHERE_SHAPE_PROXYTYPE:
        {
            btSphereShape* sphere = dynamic_cast<btSphereShape*>(shape);
            SphereCollider* sc = new SphereCollider();
            float radius = sphere->getRadius();
            sc->set_radius(radius);
            o = createInstance(env, "com/samsungxr/physics/SXRSphereCollider",
                               "void(Lcom/samsungxr/SXRContext;J)", context, sc);
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
            o = createInstance(env, "com/samsungxr/physics/SXRCapsuleCollider",
                              "void(Lcom/samsungxr/SXRContext;J)", context, cc);
        }

        case CONVEX_HULL_SHAPE_PROXYTYPE:
        {
            btConvexHullShape* hull = dynamic_cast<btConvexHullShape *>(shape);
            int numVerts = hull->getNumVertices();
            const btVector3* verts = hull->getUnscaledPoints();
            VertexBuffer* vb = Renderer::getInstance()->createVertexBuffer("float3 a_position", numVerts);
            IndexBuffer* ib = Renderer::getInstance()->createIndexBuffer(2, numVerts);
            short* indices = alloca(numVerts * 2);
            for (int i = 0; i < numVerts; ++i)
            {
                indices[i] = i;
            }
            vb->setFloatVec("a_position", (float*) verts, numVerts * 3, 3);
            Mesh* mesh = new Mesh(*vb);
            mesh->setIndexBuffer(ib);
            MeshCollider* mc = new MeshCollider(mesh);
            o = createInstance(env, "com/samsungxr/physics/SXRMeshCollider",
                               "void(Lcom/samsungxr/SXRContext;J)", context, mc);

        }
    }
    CollisionShape = SmartLocalRef(env, o);
}

void BulletFileLoader::createBulletRigidBodies()
{
    for (int i = 0; i < mImporter->getNumRigidBodies(); i++)
    {
        btRigidBody *rb = static_cast<btRigidBody*>(mImporter->getRigidBodyByIndex(i));

        if (nullptr == mImporter->getNameForPointer(rb))
        {
            // A rigid body has no name.
            continue;
        }

        // btRigidBody userPointer will point to the newly created BulletRigidBody
        BulletRigidBody* brb = new BulletRigidBody(rb);
        Collidable entry(mEnv, mContext.getObject(), brb);
        mRigidBodies.push_back(entry);
    }
}

void BulletFileLoader::createBulletMultiBodies()
{
    for (int i = mFirstMultiBody; i < mWorld->getNumMultibodies(); i++)
    {
        btMultiBody* mb = mWorld->getMultiBody(i);

        if (nullptr == mImporter->getNameForPointer(mb))
        {
            // A multi body has no name.
            continue;
        }

        // btRigidBody userPointer will point to the newly created BulletRigidBody
        BulletRootJoint* rootJoint = new BulletRootJoint(mb);
        Collidable entry(mEnv, mContext.getObject(), rootJoint);
        mJoints.push_back(entry);
        for (int i = 0; i < mb->getNumLinks(); ++i)
        {
            btMultibodyLink& link = mb->getLink(i);
            BulletJoint* joint = reinterpret_cast<BulletJoint*>(link.m_userPtr);
            Collidable entry(mEnv, mContext.getObject(), joint);
            mJoints.push_back(entry);
        };
    }
}

void BulletFileLoader::createBulletP2pConstraint(btPoint2PointConstraint *p2p)
{
    // Constraint userPointer will point to newly created BulletPoint2PointConstraint
    BulletPoint2PointConstraint *bp2p = new BulletPoint2PointConstraint(p2p);

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
}

void BulletFileLoader::createBulletHingeConstraint(btHingeConstraint *hg)
{
    BulletHingeConstraint *bhg = new BulletHingeConstraint(hg);

    if (mNeedRotate)
    {
        btTransform t = hg->getAFrame();
        hg->getAFrame().mult(transformInvIdty, t);

        t = hg->getBFrame();
        hg->getBFrame().mult(transformInvIdty, t);
    }
}

void BulletFileLoader::createBulletConeTwistConstraint(btConeTwistConstraint *ct)
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
}

void BulletFileLoader::createBulletGenericConstraint(btGeneric6DofConstraint *gen)
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
}

void BulletFileLoader::createBulletFixedConstraint(btFixedConstraint *fix)
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
}

void BulletFileLoader::createBulletSliderConstraint(btSliderConstraint *sld)
{
    BulletSliderConstraint *bsld = new BulletSliderConstraint(sld);
}

void BulletFileLoader::createBulletConstraints()
{
    for (int i = 0; i < mImporter->getNumConstraints(); i++)
    {
        btTypedConstraint *constraint = mImporter->getConstraintByIndex(i);

        btRigidBody const *rbA = &constraint->getRigidBodyA();
        btRigidBody const *rbB = &constraint->getRigidBodyB();

        if (rbA->getUserPointer() == nullptr || rbB->getUserPointer() == nullptr)
        {
            // This constraint has at least one invalid rigid body and then it must to be ignored
            continue;
        }

        if (constraint->getConstraintType() == btTypedConstraintType::POINT2POINT_CONSTRAINT_TYPE)
        {
            createBulletP2pConstraint(static_cast<btPoint2PointConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::HINGE_CONSTRAINT_TYPE)
        {
            createBulletHingeConstraint(static_cast<btHingeConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::CONETWIST_CONSTRAINT_TYPE)
        {
            createBulletConeTwistConstraint(static_cast<btConeTwistConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::D6_CONSTRAINT_TYPE ||
                 constraint->getConstraintType() == btTypedConstraintType::D6_SPRING_CONSTRAINT_TYPE)
        {
            // Blender exports generic constraint as generic spring constraint
            createBulletGenericConstraint(static_cast<btGeneric6DofConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::FIXED_CONSTRAINT_TYPE ||
                 constraint->getConstraintType() == btTypedConstraintType::D6_SPRING_2_CONSTRAINT_TYPE)
        {
            // btFixedConstraint constraint is derived from btGeneric6DofSpring2Constraint and its
            // type is set to D6_SPRING_2_CONSTRAINT_TYPE instead of FIXED_CONSTRAINT_TYPE in
            // Bullet (at least up to) 2.87
            createBulletFixedConstraint(static_cast<btFixedConstraint*>(constraint));
        }
        else if (constraint->getConstraintType() == btTypedConstraintType::SLIDER_CONSTRAINT_TYPE)
        {
            createBulletSliderConstraint(static_cast<btSliderConstraint*>(constraint));
        }
    }
}

BulletFileLoader::BulletFileLoader(jobject context, JNIEnv* env, char *buffer, size_t length, bool ignoreUpAxis) :
    PhysicsLoader(buffer, length, ignoreUpAxis),
    mCurrRigidBody(0),
    mCurrConstraint(0),
    mCurrJoint(0),
    mEnv(env),
    mContext(env, context),
    mFirstMultiBody(0),
    mCurrCollider(nullptr)
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

    createBulletRigidBodies();
    createBulletConstraints();
}

BulletFileLoader::BulletFileLoader(jobject context, JNIEnv* env, btMultiBodyDynamicsWorld* world, char *buffer, size_t length, bool ignoreUpAxis)
:   PhysicsLoader(buffer, length, ignoreUpAxis),
    mCurrRigidBody(0),
    mCurrConstraint(0),
    mCurrJoint(0),
    mEnv(env),
    mContext(env, context),
    mCurrCollider(nullptr)
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

    createBulletRigidBodies();
    createBulletMultiBodies();
    createBulletConstraints();
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

PhysicsRigidBody* BulletFileLoader::getNextRigidBody()
{
    if (mCurrRigidBody < mJoints.size())
    {
        Collidable& entry = mJoints[mCurrRigidBody++];
        mCurrCollider = entry.CollisionShape;
        return static_cast<PhysicsRigidBody*>(entry.Body);
    }
    return nullptr;
}

PhysicsJoint* BulletFileLoader::getNextJoint()
{
    if (mCurrJoint < mJoints.size())
    {
        Collidable& entry = mJoints[mCurrJoint++];
        mCurrCollider = entry.CollisionShape;
        return static_cast<PhysicsJoint*>(entry.Body);
    }
    return nullptr;
}

const char* BulletFileLoader::getRigidBodyName(PhysicsRigidBody *body) const
{
    btRigidBody *rb = reinterpret_cast<BulletRigidBody*>(body)->getRigidBody();

    return mImporter->getNameForPointer(rb);
}

const char* BulletFileLoader::getJointName(PhysicsJoint *joint) const
{
    btMultiBody* mb  = reinterpret_cast<BulletJoint*>(joint)->getMultiBody();

    if (joint->getParent() == nullptr)
    {
        return mb->getBaseName();
    }
    else
    {
        btMultibodyLink& link = mb->getLink(joint->getJointIndex());
        return link.m_jointName;
    }
}

PhysicsConstraint* BulletFileLoader::getNextConstraint()
{
    PhysicsConstraint *ret = nullptr;

    mCurrCollider = nullptr;
    while (mCurrConstraint < mImporter->getNumConstraints())
    {
        btTypedConstraint *constraint = mImporter->getConstraintByIndex(mCurrConstraint);
        ++mCurrConstraint;

        if (nullptr != constraint->getUserConstraintPtr() && ((void*)-1) != constraint->getUserConstraintPtr())
        {
            ret = static_cast<PhysicsConstraint *>(constraint->getUserConstraintPtr());
            break;
        }
    }

    return ret;
}

PhysicsRigidBody* BulletFileLoader::getConstraintBodyA(PhysicsConstraint *constraint)
{
    btTypedConstraint *btc = static_cast<btTypedConstraint*>(constraint->getUnderlying());
    btRigidBody *rbA = &btc->getRigidBodyA();
    return static_cast<PhysicsRigidBody*>(rbA->getUserPointer());
}

PhysicsRigidBody* BulletFileLoader::getConstraintBodyB(PhysicsConstraint *constraint)
{
    btTypedConstraint *btc = static_cast<btTypedConstraint*>(constraint->getUnderlying());
    btRigidBody *rbB = &btc->getRigidBodyB();
    return static_cast<PhysicsRigidBody*>(rbB->getUserPointer());
}


}
