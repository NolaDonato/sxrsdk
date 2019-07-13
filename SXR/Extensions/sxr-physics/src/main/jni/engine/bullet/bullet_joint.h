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

#ifndef BULLET_JOINT_H_
#define BULLET_JOINT_H_

#include "../physics_joint.h"

#include <BulletDynamics/Featherstone/btMultiBody.h>
#include <BulletDynamics/Featherstone/btMultiBodyLink.h>
#include <LinearMath/btMotionState.h>

class btDynamicsWorld;
class BulletWorld;

namespace sxr {
class Node;

class BulletJoint : public PhysicsJoint
{
 public:
    BulletJoint(float mass, int numBones);

    BulletJoint(BulletJoint* parent, int boneID, float mass);

    BulletJoint(btMultiBody* multibody);

    BulletJoint(btMultibodyLink* link);

    virtual ~BulletJoint();

    btMultiBody* getMultiBody() const { return mMultiBody; }

    btMultibodyLink* getLink() const { return mLink; }

    virtual void setMass(float mass);

    virtual float getMass() const;

    float getFriction() const { return mLink ? mLink->m_jointFriction : 0; }

    void setFriction(float f);

    int getBoneID() { return mBoneID; }

    virtual void getWorldTransform(btTransform &worldTrans) const;

    virtual void setWorldTransform(const btTransform &worldTrans);

    virtual void updateConstructionInfo();

private:
    void finalize();
    void updateCollisionShapeLocalScaling();

private:
    btMultiBody*             mMultiBody;
    btMultibodyLink*         mLink;
    btMultiBodyLinkCollider* mCollider;
    BulletWorld*             mWorld;
    int                      mBoneID;
    std::string              mName;
    friend class BulletWorld;
};

}

#endif /* BULLET_JOINT_H_ */
