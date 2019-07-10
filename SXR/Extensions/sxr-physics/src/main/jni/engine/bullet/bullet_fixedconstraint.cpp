//
// Created by Juliana Figueira on 5/9/17.
//
#include "bullet_joint.h"
#include "bullet_rigidbody.h"
#include "bullet_fixedconstraint.h"
#include <BulletDynamics/ConstraintSolver/btFixedConstraint.h>

static const char tag[] = "PHYSICS";

namespace sxr {

BulletFixedConstraint::BulletFixedConstraint(PhysicsRigidBody* rigidBodyB)
{
    mFixedConstraint = 0;
    mRigidBodyB = reinterpret_cast<BulletRigidBody*>(rigidBodyB);
    mBreakingImpulse = SIMD_INFINITY;
}

BulletFixedConstraint::BulletFixedConstraint(btFixedConstraint *constraint)
{
    mFixedConstraint = constraint;
    mRigidBodyB = static_cast<BulletRigidBody*>(constraint->getRigidBodyB().getUserPointer());
    constraint->setUserConstraintPtr(this);
}

BulletFixedConstraint::~BulletFixedConstraint() {
    if (0 != mFixedConstraint) {
        delete mFixedConstraint;
    }
}

void BulletFixedConstraint::setBreakingImpulse(float impulse)
{
    if (0 != mFixedConstraint)
    {
        mFixedConstraint->setBreakingImpulseThreshold(impulse);
    }
    else
    {
        mBreakingImpulse = impulse;
    }
}

float BulletFixedConstraint::getBreakingImpulse() const
{
    if (0 != mFixedConstraint)
    {
        return mFixedConstraint->getBreakingImpulseThreshold();
    }
    else
    {
        return mBreakingImpulse;
    }
}

void BulletFixedConstraint::updateConstructionInfo()
{
    if (mFixedConstraint != nullptr)
    {
        return;
    }
    BulletRigidBody* bodyA = (BulletRigidBody*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_RIGID_BODY);

    if (bodyA)
    {
        btRigidBody *rbA = bodyA->getRigidBody();
        btRigidBody *rbB = mRigidBodyB->getRigidBody();
        mFixedConstraint = new btFixedConstraint(*rbA, *rbB,
                                                 rbB->getWorldTransform(),
                                                 rbA->getWorldTransform());
        mFixedConstraint->setBreakingImpulseThreshold(mBreakingImpulse);
    }
    else
    {
        BulletJoint* jointA = (BulletJoint*) owner_object()->getComponent(COMPONENT_TYPE_PHYSICS_JOINT);
        if (jointA)
        {
            btMultibodyLink* link = jointA->getLink();
            BulletJoint* jointB = (BulletJoint*) mRigidBodyB;
            btTransform transA;
            btTransform transB;

            jointA->getWorldTransform(transA);
            jointB->getWorldTransform(transB);
            btVector3 posA(transA.getOrigin());
            btVector3 posB(transB.getOrigin());

            link->m_jointType = btMultibodyLink::eRevolute;
            link->m_dVector = posB.normalize();
            link->m_eVector = posA.normalize();
            link->m_dofCount = 0;
        }
    }
}
}