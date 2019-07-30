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
#include <algorithm>
#include <glm/mat4x4.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "bullet_world.h"
#include "bullet_rigidbody.h"
#include "bullet_joint.h"

#include "bullet_sxr_utils.h"

#include <BulletCollision/CollisionDispatch/btDefaultCollisionConfiguration.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/BroadphaseCollision/btDbvtBroadphase.h>
#include <BulletDynamics/Dynamics/btDiscreteDynamicsWorld.h>
#include <BulletDynamics/Dynamics/btDynamicsWorld.h>
#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <BulletDynamics/Featherstone/btMultiBodyLinkCollider.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyConstraintSolver.h>
#include <BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h>

#include <android/log.h>
#include <contrib/glm/ext.hpp>

namespace sxr {

BulletWorld::BulletWorld(bool isMultiBody)
{
    initialize(isMultiBody);
}

BulletWorld::~BulletWorld() {
    finalize();
}

bool BulletWorld::isMultiBody() { return mIsMultiBody; }

void BulletWorld::initialize(bool isMultiBody)
{
    mIsMultiBody = isMultiBody;

    // Default setup for memory, collision setup.
    mCollisionConfiguration = new btDefaultCollisionConfiguration();

    /// Default collision dispatcher.
    mDispatcher = new btCollisionDispatcher(mCollisionConfiguration);

    ///btDbvtBroadphase is a good general purpose broadphase. You can also try out btAxis3Sweep.
    mOverlappingPairCache = new btDbvtBroadphase();

    ///the default constraint solver. For parallel processing you can use a different solver (see Extras/BulletMultiThreaded)
    if (isMultiBody)
    {
        mSolver = new btMultiBodyConstraintSolver();
        mPhysicsWorld = new btMultiBodyDynamicsWorld(mDispatcher,
                mOverlappingPairCache,
                (btMultiBodyConstraintSolver*) mSolver,
                mCollisionConfiguration);
        mPhysicsWorld->getSolverInfo().m_globalCfm = 1e-3;
    }
    else
    {
        mSolver = new btSequentialImpulseConstraintSolver();
        mPhysicsWorld = new btDiscreteDynamicsWorld(mDispatcher,
                mOverlappingPairCache,
                mSolver,
                mCollisionConfiguration);
    }
    mPhysicsWorld->setGravity(btVector3(0, -10, 0));
    mDraggingConstraint = nullptr;
}

void BulletWorld::finalize()
{
    for (int i = mPhysicsWorld->getNumCollisionObjects() - 1; i >= 0; i--)
    {
        btCollisionObject *obj = mPhysicsWorld->getCollisionObjectArray()[i];
        if (obj)
        {
            mPhysicsWorld->removeCollisionObject(obj);
            delete obj;
        }
    }

    if (isMultiBody())
    {
        btMultiBodyDynamicsWorld* world = static_cast<btMultiBodyDynamicsWorld*>(mPhysicsWorld);
        for (int i = 0; i < world->getNumMultibodies(); ++i)
        {
            btMultiBody* mb = world->getMultiBody(i);
            world->removeMultiBody(mb);
        }
    }
    if (nullptr != mDraggingConstraint)
    {
        delete mDraggingConstraint;
    }

    //delete dynamics world
    delete mPhysicsWorld;

    //delete solver
    delete mSolver;

    //delete broadphase
    delete mOverlappingPairCache;

    //delete dispatcher
    delete mDispatcher;

    delete mCollisionConfiguration;
}

btDynamicsWorld* BulletWorld::getPhysicsWorld() const {
    return mPhysicsWorld;
}

int BulletWorld::getUpdated(std::vector<PhysicsCollidable*>& updated)
{
    int n = mBodiesChanged.size();

    if (n > 0)
    {
        copy(mBodiesChanged.begin(), mBodiesChanged.end(), back_inserter(updated));
        mBodiesChanged.clear();
    }
    return n;
}

void BulletWorld::markUpdated(PhysicsCollidable* body)
{
    mBodiesChanged.push_back(body);
}

void BulletWorld::addConstraint(PhysicsConstraint *constraint)
{
    constraint->updateConstructionInfo(this);
    Node* owner = constraint->owner_object();
    PhysicsJoint* joint = static_cast<PhysicsJoint*>(owner->getComponent(COMPONENT_TYPE_PHYSICS_JOINT));

    if (joint == nullptr)
    {
        btTypedConstraint *_constr = reinterpret_cast<btTypedConstraint *>(constraint->getUnderlying());
        mPhysicsWorld->addConstraint(_constr);
    }
}

void BulletWorld::removeConstraint(PhysicsConstraint *constraint)
{
    Node* owner = constraint->owner_object();
    PhysicsJoint* joint = static_cast<PhysicsJoint*>(owner->getComponent(COMPONENT_TYPE_PHYSICS_JOINT));

    if (joint == nullptr)
    {
        mPhysicsWorld->removeConstraint(
                reinterpret_cast<btTypedConstraint *>(constraint->getUnderlying()));
    }
}

void BulletWorld::startDrag(Node *pivot_obj, PhysicsRigidBody *target,
                            float relx, float rely, float relz)
{
    btRigidBody* rb = reinterpret_cast<BulletRigidBody*>(target)->getRigidBody();
    mActivationState = rb->getActivationState();
    rb->setActivationState(DISABLE_DEACTIVATION);

    mDraggingConstraint = new btPoint2PointConstraint(*rb, btVector3(relx, rely, relz));

    mPhysicsWorld->addConstraint(mDraggingConstraint, true);
    mDraggingConstraint->m_setting.m_impulseClamp = 30.f;
    mDraggingConstraint->m_setting.m_tau = 0.001f;
    mPivotObject = pivot_obj;
}

void BulletWorld::stopDrag()
{
    btRigidBody* rb = &mDraggingConstraint->getRigidBodyA();
    rb->forceActivationState(mActivationState);
    rb->activate();

    mPhysicsWorld->removeConstraint(mDraggingConstraint);
    delete mDraggingConstraint;
    mDraggingConstraint = nullptr;
}

void BulletWorld::addRigidBody(PhysicsRigidBody *body)
{
    BulletRigidBody* rb = static_cast<BulletRigidBody *>(body);
    body->updateConstructionInfo(this);
    mPhysicsWorld->addRigidBody(rb->getRigidBody());
    rb->mWorld = this;
}

void BulletWorld::addRigidBody(PhysicsRigidBody *body, int collisiontype, int collidesWith)
{
    BulletRigidBody* rb = static_cast<BulletRigidBody *>(body);
    body->updateConstructionInfo(this);
    mPhysicsWorld->addRigidBody(rb->getRigidBody(), collidesWith, collisiontype);
    rb->mWorld = this;
}

void BulletWorld::removeRigidBody(PhysicsRigidBody *body)
{
    mPhysicsWorld->removeRigidBody((static_cast<BulletRigidBody *>(body))->getRigidBody());
}

void BulletWorld::addJoint(PhysicsJoint *joint)
{
    if (isMultiBody())
    {
        joint->updateConstructionInfo(this);
    }
}

void BulletWorld::removeJoint(PhysicsJoint *body)
{
    if (isMultiBody() && (body->getBoneID() == 0))
    {
        btMultiBodyDynamicsWorld* world = static_cast<btMultiBodyDynamicsWorld*>(mPhysicsWorld);
        btMultiBody* mb = static_cast<BulletJoint*>(body)->getMultiBody();
        world->removeMultiBody(mb);
    }
}

void BulletWorld::step(float timeStep, int maxSubSteps)
{
    if (mDraggingConstraint != nullptr)
    {
        auto matrixB = mPivotObject->transform()->getModelMatrix(true);
        mDraggingConstraint->setPivotB(btVector3(matrixB[3][0], matrixB[3][1], matrixB[3][2]));
    }
    if (mIsMultiBody)
    {
        setPhysicsTransforms();
        mPhysicsWorld->stepSimulation(timeStep, maxSubSteps);
        getPhysicsTransforms();
    }
    else
    {
        mPhysicsWorld->stepSimulation(timeStep, maxSubSteps);
    }
}

void BulletWorld::setPhysicsTransforms()
{
    btMultiBodyDynamicsWorld* world = static_cast<btMultiBodyDynamicsWorld*>(mPhysicsWorld);
    for (int i = 0; i < world->getNumMultibodies(); ++i)
    {
        btMultiBody* mb = world->getMultiBody(i);
        const BulletJoint* joint = static_cast<const BulletJoint*>(mb->getUserPointer());
        btTransform t;

        if (!joint->isReady())
        {
            return;
        }
        joint->getWorldTransform(t);
        for (int j = 0; j < mb->getNumLinks(); ++j)
        {
            btMultibodyLink& link = mb->getLink(j);
            const void* p = link.m_userPtr;
            joint = static_cast<const BulletJoint*>(p);
            joint->getWorldTransform(t);
        }
    }
}

void BulletWorld::getPhysicsTransforms()
{
    btMultiBodyDynamicsWorld* world = static_cast<btMultiBodyDynamicsWorld*>(mPhysicsWorld);
    for (int i = 0; i < world->getNumMultibodies(); ++i)
    {
        btMultiBody* mb = world->getMultiBody(i);
        BulletJoint* joint = static_cast<BulletJoint*>(mb->getUserPointer());

        if (!joint->isReady())
        {
            return;
        }
        joint->setWorldTransform(mb->getBaseWorldTransform());
        for (int j = 0; j < mb->getNumLinks(); ++j)
        {
            btMultibodyLink& link = mb->getLink(j);
            btMultiBodyLinkCollider* collider = link.m_collider;
            const void* p = link.m_userPtr;
            joint = (BulletJoint*) p;
            const btTransform& t = collider->getWorldTransform();
            joint->setWorldTransform(t);
        }
    }
}

/**
 * Returns by reference the list of new and ceased collisions
 *  that will be the objects of ONENTER and ONEXIT events.
 */
void BulletWorld::listCollisions(std::list <ContactPoint> &contactPoints)
{

/*
 * Creates a list of all the current collisions on the World
 * */
    std::map<std::pair<long,long>, ContactPoint> currCollisions;
    int numManifolds = mPhysicsWorld->getDispatcher()->getNumManifolds();
    btPersistentManifold *contactManifold;

    for (int i = 0; i < numManifolds; i++) {
        ContactPoint contactPt;

        contactManifold = mPhysicsWorld->getDispatcher()->
                getManifoldByIndexInternal(i);

        contactPt.body0 = (BulletRigidBody *) (contactManifold->getBody0()->getUserPointer());
        contactPt.body1 = (BulletRigidBody *) (contactManifold->getBody1()->getUserPointer());
        contactPt.normal[0] = contactManifold->getContactPoint(0).m_normalWorldOnB.getX();
        contactPt.normal[1] = contactManifold->getContactPoint(0).m_normalWorldOnB.getY();
        contactPt.normal[2] = contactManifold->getContactPoint(0).m_normalWorldOnB.getZ();
        contactPt.distance = contactManifold->getContactPoint(0).getDistance();
        contactPt.isHit = true;

        std::pair<long, long> collisionPair((long)contactPt.body0, (long)contactPt.body1);
        std::pair<std::pair<long, long>, ContactPoint> newPair(collisionPair, contactPt);
        currCollisions.insert(newPair);

        /*
         * If one of these current collisions is not on the list with all the previous
         * collision, then it should be on the return list, because it is an onEnter event
         * */
        auto it = prevCollisions.find(collisionPair);
        if ( it == prevCollisions.end())
        {
            contactPoints.push_front(contactPt);
        } 
        contactManifold = 0;
    }

    /*
     * After going through all the current list, go through all the previous collisions list,
     * if one of its collisions is not on the current collision list, then it should be
     * on the return list, because it is an onExit event
     * */
    for (auto it = prevCollisions.begin(); it != prevCollisions.end(); ++it)
    {
        if (currCollisions.find(it->first) == currCollisions.end())
        {
            ContactPoint cp = it->second;
            cp.isHit = false;
            contactPoints.push_front(cp);
        }
    }

/*
 * Save all the current collisions on the previous collisions list for the next iteration
 * */
    prevCollisions.clear();
    prevCollisions.swap(currCollisions);

}


void BulletWorld::setGravity(float x, float y, float z)
{
    mPhysicsWorld->setGravity(btVector3(x, y, z));
}

void BulletWorld::setGravity(glm::vec3 gravity)
{
    mPhysicsWorld->setGravity(btVector3(gravity.x, gravity.y, gravity.z));
}

const glm::vec3& BulletWorld::getGravity() const
{
    btVector3 g = mPhysicsWorld->getGravity();

    mGravity.x = g.getX();
    mGravity.y = g.getY();
    mGravity.z = g.getZ();
    return mGravity;
}

}

