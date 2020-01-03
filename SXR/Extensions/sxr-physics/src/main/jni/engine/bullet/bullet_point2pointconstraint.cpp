//
// Created by Juliana Figueira on 5/9/17.
//
#include "bullet_sxr_utils.h"
#include "bullet_point2pointconstraint.h"
#include <BulletDynamics/ConstraintSolver/btPoint2PointConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyPoint2Point.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>
#include "bullet_joint.h"
#include "bullet_rigidbody.h"
#include "bullet_world.h"

static const char tag[] = "PHYSICS";

namespace sxr {

    BulletPoint2PointConstraint::BulletPoint2PointConstraint(PhysicsCollidable* bodyA,
            const glm::vec3& pivotA, const glm::vec3& pivotB)
    {
        mConstraint =  nullptr;
        mMBConstraint = nullptr;
        mBodyA = static_cast<BulletRigidBody*>(bodyA);
        mBreakingImpulse = SIMD_INFINITY;
        mPivotA = pivotA;
        mPivotB = pivotB;
    };

    // This constructor is only used when loading physics from bullet file
    BulletPoint2PointConstraint::BulletPoint2PointConstraint(btPoint2PointConstraint* constraint)
    {
        mConstraint = constraint;
        mMBConstraint = nullptr;
        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletPoint2PointConstraint::BulletPoint2PointConstraint(btMultiBodyPoint2Point* constraint)
    {
        btVector3 pA(constraint->getPivotInA());
        btVector3 pB(constraint->getPivotInB());
        btMultiBody* mbA = constraint->getMultiBodyA();

        mMBConstraint = constraint;
        mBodyA = reinterpret_cast<PhysicsCollidable*>(mbA->getUserPointer());
        mPivotA = glm::vec3(pA.x(), pA.y(), pA.z());
        mPivotB = glm::vec3(pB.x(), pB.y(), pB.z());
        mBreakingImpulse = SIMD_INFINITY;
    }

    BulletPoint2PointConstraint::~BulletPoint2PointConstraint()
    {
        if (mConstraint)
        {
            delete mConstraint;
        }
        if (mMBConstraint)
        {
            delete mMBConstraint;
        }
    };

    void BulletPoint2PointConstraint::setBreakingImpulse(float impulse)
    {
        if (mMBConstraint)
        {
            mMBConstraint->setMaxAppliedImpulse(impulse);
        }
        else if (mConstraint)
        {
            mConstraint->setBreakingImpulseThreshold(impulse);
        }
        mBreakingImpulse = impulse;
    }

    float BulletPoint2PointConstraint::getBreakingImpulse() const
    {
        return mBreakingImpulse;
    }

    void BulletPoint2PointConstraint::setParentBody(PhysicsCollidable* body)
    {
        PhysicsCollidable* bodyA = mBodyA;
        BulletRigidBody* rb = static_cast<BulletRigidBody*>(bodyA);
        BulletWorld* bw;
        btDynamicsWorld* dw = rb->getPhysicsWorld();

        if (body == bodyA)
        {
            return;
        }
        if (mConstraint)
        {
            if (dw)
            {
                bw = static_cast<BulletWorld*>(dw->getWorldUserInfo());
                dw->removeConstraint(mConstraint);
            }
            delete mConstraint;
            mConstraint = nullptr;
            mBodyA = body;
            sync(bw);
        }
        else
        {
            mBodyA = body;
        }
    }

    void BulletPoint2PointConstraint::sync(PhysicsWorld *world)
    {
        if ((mConstraint != nullptr) || (mMBConstraint != nullptr))
        {
            return;
        }
        btVector3 pA(mPivotA.x, mPivotA.y, mPivotA.z);
        btVector3 pB(mPivotB.x, mPivotB.y, mPivotB.z);
        BulletRigidBody *bodyB = static_cast<BulletRigidBody *>
        (owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY));

        if (bodyB)
        {
            if (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
            {
                btRigidBody *rbB = bodyB->getRigidBody();
                btRigidBody *rbA = static_cast<BulletRigidBody *>(mBodyA)->getRigidBody();
                btPoint2PointConstraint *constraint = new btPoint2PointConstraint(*rbA, *rbB, pA,
                                                                                  pB);
                constraint->setBreakingImpulseThreshold(mBreakingImpulse);
                mConstraint = constraint;
            }
            else if (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
            {
                btRigidBody *rbB = bodyB->getRigidBody();
                BulletJoint *jointA = static_cast<BulletJoint *>(mBodyA);
                btMultiBody *mbA = jointA->getMultiBody();
                btMultiBodyPoint2Point *constraint = new btMultiBodyPoint2Point(
                        mbA, jointA->getJointIndex(),
                        rbB, pA, pB);
                mMBConstraint = constraint;
                constraint->setMaxAppliedImpulse(mBreakingImpulse);
            }
            return;
        }
        BulletJoint *jointB = static_cast<BulletJoint *>
        (owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_JOINT));
        if (jointB)
        {
            btMultiBody *mbB = jointB->getMultiBody();

            if (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
            {
                BulletJoint *jointA = static_cast<BulletJoint *>(mBodyA);
                btMultiBody *mbA = jointA->getMultiBody();
                btMultiBodyPoint2Point *constraint = new btMultiBodyPoint2Point(
                        mbA, jointA->getJointIndex(),
                        mbB, jointB->getJointIndex(),
                        pA, pB);
                constraint->setMaxAppliedImpulse(mBreakingImpulse);
                mMBConstraint = constraint;
            }
            else if (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
            {
                btRigidBody *rbA = static_cast<BulletRigidBody *>(mBodyA)->getRigidBody();
                btMultiBody *mbB = jointB->getMultiBody();
                btMultiBodyPoint2Point *constraint = new btMultiBodyPoint2Point(
                        mbB, jointB->getJointIndex(),
                        rbA, pB, pA);
                constraint->setMaxAppliedImpulse(mBreakingImpulse);
                mMBConstraint = constraint;
            }
        }
    }

    void BulletPoint2PointConstraint::addToWorld(PhysicsWorld* w)
    {
        BulletWorld* bw = static_cast<BulletWorld*>(w);

        if (w->isMultiBody() && mMBConstraint)
        {
            btMultiBodyDynamicsWorld* world = dynamic_cast<btMultiBodyDynamicsWorld*>(bw->getPhysicsWorld());
            if (world)
            {
                world->addMultiBodyConstraint(mMBConstraint);
            }
        }
        else if (mConstraint)
        {
            bw->getPhysicsWorld()->addConstraint(mConstraint, true);
        }
    }

    void BulletPoint2PointConstraint::removeFromWorld(PhysicsWorld* w)
    {
        BulletWorld* bw = static_cast<BulletWorld*>(w);

        if (w->isMultiBody() && mMBConstraint)
        {
            btMultiBodyDynamicsWorld* world = dynamic_cast<btMultiBodyDynamicsWorld*>(bw->getPhysicsWorld());
            if (world)
            {
                world->removeMultiBodyConstraint(mMBConstraint);
            }
        }
        else if (mConstraint)
        {
            bw->getPhysicsWorld()->removeConstraint(mConstraint);
        }
    }

}


