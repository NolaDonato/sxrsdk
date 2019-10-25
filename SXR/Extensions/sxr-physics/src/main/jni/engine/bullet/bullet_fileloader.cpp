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
#include <Serialize/BulletWorldImporter/btBulletWorldImporter.h>
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
#include <LinearMath/btSerializer.h>

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

static btMatrix3x3 matrixInvIdty(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, -1.0f, 0.0f);
static btTransform transformInvIdty(matrixInvIdty);


namespace sxr {

/**
 * Construct the appropriate subclass of SXRCollider to match the input
 * Bullet collision object. Box, sphere, capsule and mesh colliders
 * are supported.
 * @param collider Bullet collider
 * @return Java SXRCollider object
 */
jobject BulletFileLoader::createCollider(btCollisionObject* collider)
{
    jobject o = 0;
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);

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
            o = CreateInstance(*env, "com/samsungxr/SXRBoxCollider",
                               "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(), bc);
            break;
        }

        case SPHERE_SHAPE_PROXYTYPE:
        {
            btSphereShape* sphere = dynamic_cast<btSphereShape*>(shape);
            SphereCollider* sc = new SphereCollider();
            float radius = sphere->getRadius();
            sc->set_radius(radius);
            o = CreateInstance(*env, "com/samsungxr/SXRSphereCollider",
                               "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(), sc);
            break;
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
            o = CreateInstance(*env, "com/samsungxr/SXRCapsuleCollider",
                              "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(), cc);
            break;
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
            o = CreateInstance(*env, "com/samsungxr/SXRMeshCollider",
                               "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(), mc);
            break;
        }

        default:
        {
            MeshCollider* mc = new MeshCollider(nullptr);
            o = CreateInstance(*env, "com/samsungxr/SXRMeshCollider",
                               "(Lcom/samsungxr/SXRContext;J)V", mContext.getObject(), mc);
            break;
        }
    }
    return o;
}

const char* BulletFileLoader::getNameForPointer(void* ptr)
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
const char* BulletFileLoader::getConstraintName(PhysicsConstraint* constraint)
{
    btTypedConstraint* btc = reinterpret_cast<btTypedConstraint*>(constraint->getUnderlying());
    btRigidBody& body = btc->getRigidBodyB();
    return getNameForPointer(&body);
}

/**
 * Get the Java object that should be body A for the given constraint.
 * This lookup is done by extracting the name of the Bullet rigid body
 * or joint and using it to look up the Java object reference.
 * @param constraint C++ constraint
 * @return Java object which should be bodyA for the constraint
 */
jobject BulletFileLoader::getConstraintBodyA(PhysicsConstraint* constraint)
{
    PhysicsCollidable* bodyA = constraint->getBodyA();
    const char* name;

    if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
    {
        BulletRigidBody* bA = static_cast<BulletRigidBody*>(bodyA);
        return getRigidBody(bA->getName());
    }
    else if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
    {
        BulletJoint* bA = static_cast<BulletJoint*>(bodyA);

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
void BulletFileLoader::createRigidBodies(btBulletWorldImporter& importer)
{
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
    for (int i = 0; i < importer.getNumRigidBodies(); i++)
    {
        btRigidBody *rb = static_cast<btRigidBody*>(importer.getRigidBodyByIndex(i));
        createRigidBody(*env, rb);
    }
}

void BulletFileLoader::createRigidBodies(btMultiBodyDynamicsWorld& world)
{
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
    btCollisionObjectArray colliders = world.getCollisionObjectArray();
    for (int i = 0; i < colliders.size(); i++)
    {
        btCollisionObject* co = colliders.at(i);
        btRigidBody* rb = dynamic_cast<btRigidBody*>(co);
        if (rb)
        {
            createRigidBody(*env, rb);
        }
    }
}

void BulletFileLoader::createRigidBody(JNIEnv& env, btRigidBody* rb)
{
    const char* name = getNameForPointer(rb);

    if (name == nullptr)    // we can't import rigid bodies without names
    {
        return;
    }
    BulletRigidBody* nativeBody = new BulletRigidBody(rb);
    jobject javaBody = CreateInstance(env, "com/samsungxr/physics/SXRRigidBody",
                                      "(Lcom/samsungxr/SXRContext;J)V",
                                      mContext.getObject(), reinterpret_cast<jlong>(nativeBody));
    nativeBody->setName(name);
    SmartLocalRef r(mJavaVM, javaBody);
    std::string s(name);
    mRigidBodies.emplace(s, r);

    if (mNeedRotate)
    {
        btTransform& t = rb->getWorldTransform();
        t *= transformInvIdty;
        rb->setWorldTransform(t);
    }
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
void BulletFileLoader::createJoints(btMultiBodyDynamicsWorld& world)
{
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
    for (int i = mFirstMultiBody; i < world.getNumMultibodies(); i++)
    {
        btMultiBody* mb = world.getMultiBody(i);
        btMultiBodyLinkCollider* btc = mb->getBaseCollider();
        const char* name = getNameForPointer(mb);

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
        BulletJoint* rootJoint = new BulletRootJoint(mb);
        jobject javaJoint = CreateInstance(*env, "com/samsungxr/physics/SXRPhysicsJoint",
                                          "(Lcom/samsungxr/SXRContext;J)V",
                                          mContext.getObject(), reinterpret_cast<jlong>(rootJoint));
        std::string s(name);
        SmartLocalRef r(mJavaVM, javaJoint);
        auto pair = std::make_pair(s, r);

        rootJoint->setName(name);
        mJoints.emplace(pair);
        if (btc != nullptr)
        {
            jobject javaCollider = createCollider(btc);
            SmartLocalRef c(mJavaVM, javaCollider);

            s = name;
            mColliders.emplace(s, c);
        }
        for (int i = 0; i < mb->getNumLinks(); ++i)
        {
            btMultibodyLink& link = mb->getLink(i);
            btMultiBodyLinkCollider* collider = link.m_collider;

            name = getNameForPointer(collider);
            if (name == nullptr)   // cannot map joints without names
            {
                continue;
            }
            if (mNeedRotate)
            {
                btTransform t = link.m_cachedWorldTransform;
                t *= transformInvIdty;
                link.m_cachedWorldTransform = t;
                t = collider->getWorldTransform();
                t *= transformInvIdty;
                collider->setWorldTransform(t);
            }
            BulletJoint* nativeJoint = (BulletJoint*) (link.m_userPtr);
            javaJoint = CreateInstance(*env, "com/samsungxr/physics/SXRPhysicsJoint",
                                       "(Lcom/samsungxr/SXRContext;J)V",
                                       mContext.getObject(), reinterpret_cast<jlong>(nativeJoint));
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
            }
        };
    }
}

/*
 * Create a Java SXRPoint2PointConstraint and the C++ PhysicsConstraint
 * based on the input btPoint2PointConstraint
 */
jobject BulletFileLoader::createP2PConstraint(JNIEnv& env, btPoint2PointConstraint* p2p, PhysicsConstraint*& constraint)
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
    constraint = bp2p;
    return CreateInstance(env, "com/samsungxr/physics/SXRPoint2PointConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bp2p));
}

/*
 * Create a Java SXRHingeConstraint and the C++ PhysicsConstraint
 * based on the input btHingeConstraint
 */
jobject BulletFileLoader::createHingeConstraint(JNIEnv& env, btHingeConstraint* hg, PhysicsConstraint*& constraint)
{
    BulletHingeConstraint *bhg = new BulletHingeConstraint(hg);

    if (mNeedRotate)
    {
        btTransform t = hg->getAFrame();

        hg->getAFrame().mult(transformInvIdty, t);
        t = hg->getBFrame();
        hg->getBFrame().mult(transformInvIdty, t);
    }
    constraint = bhg;
    return CreateInstance(env, "com/samsungxr/physics/SXRHingeConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bhg));
}

/*
 * Create a Java SXRConeTwistConstraint and the C++ PhysicsConstraint
 * based on the input btConeTwistConstraint
 */
jobject BulletFileLoader::createConeTwistConstraint(JNIEnv& env, btConeTwistConstraint* ct, PhysicsConstraint*& constraint)
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
    constraint = bct;
    return CreateInstance(env, "com/samsungxr/physics/SXRConeTwistConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bct));
}

/*
 * Create a Java SXRGenericConstraint and the C++ PhysicsConstraint
 * based on the input btGeneric6DofConstraint
 */
jobject BulletFileLoader::createGenericConstraint(JNIEnv& env, btGeneric6DofConstraint* gen, PhysicsConstraint*& constraint)
{
    BulletGeneric6dofConstraint *bg = new BulletGeneric6dofConstraint(gen);

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
    constraint = bg;
    return CreateInstance(env, "com/samsungxr/physics/SXRGenericConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bg));
}

/*
 * Create a Java SXRGenericConstraint and the C++ PhysicsConstraint
 * based on the input btGeneric6DofSpring2Constraint
 */
jobject BulletFileLoader::createSpringConstraint(JNIEnv& env, btGeneric6DofSpring2Constraint* gen, PhysicsConstraint*& constraint)
{
    BulletGeneric6dofConstraint *bg = new BulletGeneric6dofConstraint(gen);

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
    constraint = bg;
    return CreateInstance(env, "com/samsungxr/physics/SXRGenericConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bg));
}

/*
 * Create a Java SXRFixedConstraint and the C++ PhysicsConstraint
 * based on the input btFixedConstraint
 */
jobject BulletFileLoader::createFixedConstraint(JNIEnv& env, btFixedConstraint* fix, PhysicsConstraint*& constraint)
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
    constraint = bfix;
    return CreateInstance(env, "com/samsungxr/physics/SXRFixedConstraint",
                          "(Lcom/samsungxr/SXRContext;J)V",
                          mContext.getObject(), reinterpret_cast<jlong>(bfix));
}

/*
 * Create a Java SXRSliderConstraint and the C++ PhysicsConstraint
 * based on the input btSliderConstraint
 */
jobject BulletFileLoader::createSliderConstraint(JNIEnv& env, btSliderConstraint* sld, PhysicsConstraint*& constraint)
{
    BulletSliderConstraint* c = new BulletSliderConstraint(sld);
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
void BulletFileLoader::createConstraints(btMultiBodyWorldImporter& importer)
{
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);

    for (int i = 0; i < importer.getNumConstraints(); i++)
    {
        btTypedConstraint *constraint = importer.getConstraintByIndex(i);
        createConstraint(*env, constraint);
    }
}

void BulletFileLoader::createConstraints(btMultiBodyDynamicsWorld& w)
{
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);

    for (int i = 0; i < w.getNumConstraints(); i++)
    {
        btTypedConstraint *constraint = w.getConstraint(i);
        createConstraint(*env, constraint);
    }
}

void BulletFileLoader::createConstraint(JNIEnv& env, btTypedConstraint* constraint)
{
    btRigidBody& bbA = constraint->getRigidBodyA();
    btRigidBody& bbB = constraint->getRigidBodyB();
    BulletRigidBody* rbA = reinterpret_cast<BulletRigidBody*>(bbA.getUserPointer());
    BulletRigidBody* rbB = reinterpret_cast<BulletRigidBody*>(bbB.getUserPointer());
    const char* nameB = getNameForPointer(&bbB);

    if ((rbA == nullptr) ||
        (rbB == nullptr) ||
        (nameB == nullptr))
    {
        // This constraint has at least one invalid rigid body and then it must to be ignored
        return;
    }
    jobject javaConstraint = nullptr;
    PhysicsConstraint* physCon = nullptr;

    if (constraint->getConstraintType() == btTypedConstraintType::POINT2POINT_CONSTRAINT_TYPE)
    {
        javaConstraint = createP2PConstraint(env, dynamic_cast<btPoint2PointConstraint*>(constraint), physCon);
    }
    else if (constraint->getConstraintType() == btTypedConstraintType::HINGE_CONSTRAINT_TYPE)
    {
        javaConstraint = createHingeConstraint(env, dynamic_cast<btHingeConstraint*>(constraint), physCon);
    }
    else if (constraint->getConstraintType() == btTypedConstraintType::CONETWIST_CONSTRAINT_TYPE)
    {
        javaConstraint = createConeTwistConstraint(env, dynamic_cast<btConeTwistConstraint*>(constraint), physCon);
    }
    else if (constraint->getConstraintType() == btTypedConstraintType::D6_CONSTRAINT_TYPE ||
             constraint->getConstraintType() == btTypedConstraintType::D6_SPRING_CONSTRAINT_TYPE)
    {
        // Blender exports generic constraint as generic spring constraint
        javaConstraint = createGenericConstraint(env, dynamic_cast<btGeneric6DofConstraint*>(constraint), physCon);
    }
    else if (constraint->getConstraintType() == btTypedConstraintType::D6_SPRING_2_CONSTRAINT_TYPE)
    {
        // URDF exporter generic constraint as btGeneric6DofSpring2Constraint
        javaConstraint = createSpringConstraint(env, dynamic_cast<btGeneric6DofSpring2Constraint*>(constraint), physCon);
    }
    else if (constraint->getConstraintType() == btTypedConstraintType::FIXED_CONSTRAINT_TYPE)
    {
        // btFixedConstraint constraint is derived from btGeneric6DofSpring2Constraint and its
        // type is set to D6_SPRING_2_CONSTRAINT_TYPE instead of FIXED_CONSTRAINT_TYPE in
        // Bullet (at least up to) 2.87
        javaConstraint = createFixedConstraint(env, dynamic_cast<btFixedConstraint*>(constraint), physCon);
    }
    else if (constraint->getConstraintType() == btTypedConstraintType::SLIDER_CONSTRAINT_TYPE)
    {
        javaConstraint = createSliderConstraint(env, dynamic_cast<btSliderConstraint*>(constraint), physCon);
    }
    if (javaConstraint)
    {
        const char* nameA = getNameForPointer(&bbA);
        jobject bodyA = getRigidBody(nameA);
        mConstraints.emplace(std::string(nameB), SmartLocalRef(mJavaVM, javaConstraint));
        if (bodyA)
        {
            CallVoidMethod(env, javaConstraint, "com/samsungxr/physics/SXRConstraint", "setBodyA",
                           "(Lcom/samsungxr/physics/SXRPhysicsCollidable;)V",  env.NewLocalRef(bodyA));
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
void BulletFileLoader::createMultiBodyConstraints(btMultiBodyDynamicsWorld& world)
{
    JNIEnv* env;
    mJavaVM.GetEnv((void**) &env, SUPPORTED_JNI_VERSION);
    for (int i = mFirstMultiBody; i < world.getNumMultiBodyConstraints(); ++i)
    {
        btMultiBodyConstraint* c = world.getMultiBodyConstraint(i);
        createMultiBodyConstraint(*env, c);
    }
}

void BulletFileLoader::createMultiBodyConstraint(JNIEnv& env, btMultiBodyConstraint* c)
{
    btMultiBody* const mbA = c->getMultiBodyA();
    btMultiBody* const mbB = c->getMultiBodyB();
    btRigidBody* rbB = nullptr;
    PhysicsConstraint* physCon = nullptr;
    jobject javaConstraint;
    const char* nameB = nullptr;
    const char* constraintClass = nullptr;

    if (mbA->getUserPointer() == nullptr)
    {
        // This constraint has at least one invalid rigid body and then it must to be ignored
        return;
    }
    if (typeid(c) == typeid(btMultiBodyFixedConstraint))
    {
        btMultiBodyFixedConstraint* fc = dynamic_cast<btMultiBodyFixedConstraint*>(c);
        physCon = new BulletFixedConstraint(fc);
        constraintClass = "com/samsungxr/physics/SXRFixedConstraint";
        rbB = fc->getRigidBodyB();
    }
    else if (typeid(c) == typeid(btMultiBodySliderConstraint))
    {
        btMultiBodySliderConstraint* sc = dynamic_cast<btMultiBodySliderConstraint*>(c);
        physCon = new BulletSliderConstraint(sc);
        constraintClass = "com/samsungxr/physics/SXRSliderConstraint";
        rbB = sc->getRigidBodyB();
    }
    else if (typeid(c) == typeid(btMultiBodyPoint2Point))
    {
        btMultiBodyPoint2Point* p2p = dynamic_cast<btMultiBodyPoint2Point*>(c);
        physCon = new BulletPoint2PointConstraint(p2p);
        constraintClass = "com/samsungxr/physics/SXRPoint2PointConstraint";
        rbB = p2p->getRigidBodyB();
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
        javaConstraint =  CreateInstance(env, constraintClass,
                                         "(Lcom/samsungxr/SXRContext;J)V",
                                         mContext.getObject(), reinterpret_cast<long>(physCon));

        mConstraints.emplace(std::string(nameB), SmartLocalRef(mJavaVM, javaConstraint));
        jobject bodyA = getConstraintBodyA(physCon);
        if (bodyA)
        {
            CallVoidMethod(env, javaConstraint, "com/samsungxr/physics/SXRConstraint", "setBodyA",
                           "(Lcom/samsungxr/physics/SXRPhysicsCollidable;)V",  env.NewLocalRef(bodyA));
        }
    }
}

BulletFileLoader::BulletFileLoader(jobject context, JavaVM& jvm)
:   HybridObject(),
    mContext(jvm, context),
    mBulletImporter(nullptr),
    mURDFImporter(nullptr),
    mSerializer(nullptr),
    mJavaVM(jvm),
    mFirstMultiBody(0),
    mWorld(nullptr)
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
    bParse::btBulletFile* bullet_file = nullptr;

    try
    {
        bullet_file = new bParse::btBulletFile(buffer, length);
        mBulletImporter = new btMultiBodyWorldImporter(nullptr);
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
bool BulletFileLoader::parse(BulletWorld* world, char *buffer, size_t length, bool ignoreUpAxis)
{
    btMultiBodyDynamicsWorld* bulletWorld = dynamic_cast<btMultiBodyDynamicsWorld*>(world->getPhysicsWorld());

    if (bulletWorld == nullptr)
    {
        LOGE("BULLET: Input BulletWorld does not support multibody, cannot import");
        return false;
    }
    bParse::btBulletFile* bullet_file = new bParse::btBulletFile(buffer, length);
    btMultiBodyWorldImporter* importer = new btMultiBodyWorldImporter(bulletWorld);
    mWorld = bulletWorld;
    mBulletImporter = importer;
    mBulletImporter->loadFileFromMemory(bullet_file);

    if (ignoreUpAxis)
    {
        mNeedRotate = false;
    }
    else if (bullet_file->getFlags() & bParse::FD_DOUBLE_PRECISION)
    {
        btDynamicsWorldDoubleData* ddata =
                reinterpret_cast<btDynamicsWorldDoubleData*>(bullet_file->m_dynamicsWorldInfo[0]);
        double* gravity = reinterpret_cast<double*>(&ddata->m_gravity);
        mNeedRotate = gravity[2] != 0.0;
    }
    else
    {
        btDynamicsWorldFloatData* fdata =
                reinterpret_cast<btDynamicsWorldFloatData*>(bullet_file->m_dynamicsWorldInfo[0]);
        float* gravity = reinterpret_cast<float*>(&fdata->m_gravity);
        mNeedRotate = gravity[2] != 0.f;
    }
    if (bulletWorld->getNumMultibodies() > 0)
    {
        mFirstMultiBody = bulletWorld->getNumMultibodies();
        importer->convertAllObjects(bullet_file);
    }
    delete bullet_file;

    createRigidBodies(*importer);
    createJoints(*bulletWorld);
    createConstraints(*importer);
    createMultiBodyConstraints(*bulletWorld);
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
bool BulletFileLoader::parseURDF(BulletWorld* world, const char* xmldata, bool ignoreUpAxis)
{
    btMultiBodyDynamicsWorld* bulletWorld = dynamic_cast<btMultiBodyDynamicsWorld*>(world->getPhysicsWorld());

    if (bulletWorld == nullptr)
    {
        LOGE("BULLET: Input BulletWorld does not support multibody, cannot import");
        return false;
    }
    if (bulletWorld->getNumMultibodies() > 0)
    {
        mFirstMultiBody = bulletWorld->getNumMultibodies();
    }
    mSerializer = new btDefaultSerializer();
    mURDFImporter = new URDFConverter(world->isMultiBody());
    mURDFImporter->importPhysics(xmldata, bulletWorld);
    mURDFImporter->registerNames(*mSerializer);
    mWorld = world->getPhysicsWorld();
    const btVector3& gravity = mWorld->getGravity();

    if (ignoreUpAxis)
    {
        mNeedRotate = false;
    }
    else
    {
        mNeedRotate = gravity.y() != 0.0;
    }
    createRigidBodies(*bulletWorld);
    createJoints(*bulletWorld);
    createConstraints(*bulletWorld);
    createMultiBodyConstraints(*bulletWorld);
    delete mURDFImporter;
    mURDFImporter = nullptr;
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
 * Get the Java SXRConstraint associated with the input name.
 * @param name name of rigid body B or joint B for this constraint.
 * @return SXRRConstraint object for named body.
 */
jobject BulletFileLoader::getConstraint(const char* name)
{
    std::string s(name);
    const SmartLocalRef& r = mConstraints[s];
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
