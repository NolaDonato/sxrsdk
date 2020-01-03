//
// Created by Juliana Figueira on 5/9/17.
//
#include "bullet_joint.h"
#include "bullet_rigidbody.h"
#include "bullet_fixedconstraint.h"
#include "bullet_sxr_utils.h"
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyFixedConstraint.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>

static const char tag[] = "PHYSICS";

namespace sxr
{

    BulletFixedConstraint::BulletFixedConstraint(PhysicsCollidable* bodyA)
    {
        mConstraint = nullptr;
        mMBConstraint = nullptr;
        mBodyA = bodyA;
        mBreakingImpulse = SIMD_INFINITY;
    }

    BulletFixedConstraint::BulletFixedConstraint(btFixedConstraint* constraint)
    {
        mConstraint = constraint;
        mBodyA = static_cast<BulletRigidBody*>(constraint->getRigidBodyA().getUserPointer());
        constraint->setUserConstraintPtr(this);
    }

    BulletFixedConstraint::BulletFixedConstraint(btMultiBodyFixedConstraint* constraint)
    {
        btVector3 pA(constraint->getPivotInA());
        btVector3 pB(constraint->getPivotInB());

        mMBConstraint = constraint;
        mConstraint = nullptr;
         mBodyA = static_cast<BulletJoint*>(constraint->getMultiBodyA()->getUserPointer());
        mBreakingImpulse = SIMD_INFINITY;
        mPivotA = glm::vec3(pA.x(), pA.y(), pA.z());
        mPivotB = glm::vec3(pB.x(), pB.y(), pB.z());
    }

    BulletFixedConstraint::~BulletFixedConstraint()
    {
        if (mConstraint)
        {
            delete mConstraint;
        }
        if (mMBConstraint)
        {
            delete mMBConstraint;
        }
    }

    void BulletFixedConstraint::setParentBody(PhysicsCollidable* body)
    {
        PhysicsCollidable* bodyA = mBodyA;
        btDynamicsWorld* dw;
        BulletWorld* bw;

        if (body == bodyA)
        {
            return;
        }

        if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
        {
            BulletRigidBody* rb = static_cast<BulletRigidBody*>(bodyA);
            dw = rb->getPhysicsWorld();

        }
        else if (bodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
        {
            BulletJoint* j = static_cast<BulletJoint*>(bodyA);
            dw = j->getPhysicsWorld();
        }
        if (dw)
        {
            bw = static_cast<BulletWorld*>(dw->getWorldUserInfo());
        }

        if (mConstraint)
        {
            dw->removeConstraint(mConstraint);
            delete mConstraint;
            mConstraint = nullptr;
        }
        else if (mMBConstraint)
        {
            btMultiBodyDynamicsWorld* mbdw = dynamic_cast<btMultiBodyDynamicsWorld*>(dw);
            if (mbdw)
            {
                mbdw->removeMultiBodyConstraint(mMBConstraint);
                delete mMBConstraint;
                mMBConstraint = nullptr;
            }
        }
        else
        {
            mBodyA = body;
        }
        mBodyA = body;
        sync(bw);
    }

    void BulletFixedConstraint::setBreakingImpulse(float impulse)
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

    float BulletFixedConstraint::getBreakingImpulse() const
    {
        return mBreakingImpulse;
    }

    void BulletFixedConstraint::sync(PhysicsWorld *world)
    {
        if ((mConstraint != nullptr) || (mMBConstraint != nullptr))
        {
            return;
        }
        btTransform  worldFrameA = convertTransform2btTransform(mBodyA->owner_object()->transform());
        btTransform  worldFrameB = convertTransform2btTransform(owner_object()->transform());
        BulletRigidBody* bodyB = static_cast<BulletRigidBody*>
                                    (owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY));
        btVector3    pA(mPivotA.x, mPivotA.y, mPivotA.z);
        btVector3    pB(mPivotB.x, mPivotB.y, mPivotB.z);
        btTransform  frameA = worldFrameB.inverse() * worldFrameA;
        btTransform  frameB = worldFrameA.inverse() * worldFrameB;

        frameA.setOrigin(pA);
        frameB.setOrigin(pB);
        if (bodyB)
        {
            if ((bodyB != mBodyA) && (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY))
            {
                btRigidBody* rbB = bodyB->getRigidBody();
                btRigidBody* rbA = static_cast<BulletRigidBody*>(mBodyA)->getRigidBody();
                btFixedConstraint* constraint = new btFixedConstraint(*rbA, *rbB, frameA, frameB);
                constraint->setBreakingImpulseThreshold(mBreakingImpulse);
                mConstraint = constraint;
                return;
            }
            else if (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT)
            {
                btRigidBody* rbB = bodyB->getRigidBody();
                BulletJoint* jointA = static_cast<BulletJoint*>(mBodyA);
                btMultiBody* mbA = jointA->getMultiBody();
                btMultiBodyFixedConstraint* constraint = new btMultiBodyFixedConstraint(
                        mbA, jointA->getJointIndex(),
                        rbB, pA, pB,
                        frameA.getBasis(), frameB.getBasis());
                mMBConstraint = constraint;
                constraint->setMaxAppliedImpulse(mBreakingImpulse);
                return;
            }
        }
        BulletJoint* jointB = static_cast<BulletJoint*>
                                (owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_JOINT));
        if (jointB)
        {
            btMultiBody *mbB = jointB->getMultiBody();

            if ((jointB != mBodyA) && (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_JOINT))
            {
                BulletJoint *jointA = static_cast<BulletJoint *>(mBodyA);
                btMultiBody *mbA = jointA->getMultiBody();
                btMultiBodyFixedConstraint *constraint = new btMultiBodyFixedConstraint(
                        mbA, jointA->getJointIndex(),
                        mbB, jointB->getJointIndex(),
                        pA, pB,
                        frameA.getBasis(), frameB.getBasis());
                constraint->setMaxAppliedImpulse(mBreakingImpulse);
                mMBConstraint = constraint;
            }
            else if (mBodyA->getType() == COMPONENT_TYPE_PHYSICS_RIGID_BODY)
            {
                btRigidBody *rbA = static_cast<BulletRigidBody *>(mBodyA)->getRigidBody();
                btMultiBody *mbB = jointB->getMultiBody();
                btMultiBodyFixedConstraint *constraint = new btMultiBodyFixedConstraint(
                        mbB, jointB->getJointIndex(),
                        rbA, pB, pA,
                        frameB.getBasis(), frameA.getBasis());
                constraint->setMaxAppliedImpulse(mBreakingImpulse);
                mMBConstraint = constraint;
            }
        }
    }

    void BulletFixedConstraint::addToWorld(PhysicsWorld* w)
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

    void BulletFixedConstraint::removeFromWorld(PhysicsWorld* w)
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