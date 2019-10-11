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

#ifndef EXTENSIONS_BULLET_FILELOADER_H
#define EXTENSIONS_BULLET_FILELOADER_H

#include "../physics_loader.h"
#include "util/jni_utils.h"

class btMultiBodyWorldImporter;
class btMultiBodyDynamicsWorld;
class btCollisionShape;

namespace sxr
{
class PhysicsCollidable;
class Collider;
class BulletRigidBody;
class BulletJoint;

class BulletFileLoader : public PhysicsLoader
{
public:
    BulletFileLoader(jobject context, JNIEnv* env, char *buffer, size_t length, bool ignoreUpAxis);

    BulletFileLoader(jobject context, JNIEnv* env, btMultiBodyDynamicsWorld* world, char *buffer, size_t length, bool ignoreUpAxis);

    virtual ~BulletFileLoader();

    PhysicsRigidBody* getNextRigidBody();

    PhysicsJoint* getNextJoint();

    Collider* getCollider();

    const char* getRigidBodyName(PhysicsRigidBody *body) const;

    const char* getJointName(PhysicsJoint *joint) const;

    PhysicsConstraint* getNextConstraint();

    PhysicsRigidBody* getConstraintBodyA(PhysicsConstraint *constraint);

    PhysicsRigidBody* getConstraintBodyB(PhysicsConstraint *constraint);

private:
    struct Collidable
    {
        SmartLocalRef Body;
        SmartLocalRef CollisionShape;

        Collidable(JNIEnv* env, jobject context, BulletRigidBody* body);
        Collidable(JNIEnv* env, jobject context, BulletJoint* joint);
        void      makeCollider(JNIEnv* env, jobject context, btCollisionShape*);
        jobject   createInstance(JNIEnv* env, const char* className, const char* signature, ...);
    };
    void createBulletRigidBodies();
    void createBulletMultiBodies();
    void createBulletP2pConstraint(btPoint2PointConstraint *p2p);
    void createBulletHingeConstraint(btHingeConstraint *hg);
    void createBulletConeTwistConstraint(btConeTwistConstraint *ct);
    void createBulletFixedConstraint(btFixedConstraint *fix);
    void createBulletSliderConstraint(btSliderConstraint *sld);
    void createBulletGenericConstraint(btGeneric6DofConstraint *gen);
    void createBulletConstraints();

    btMultiBodyDynamicsWorld* mWorld;
    btBulletWorldImporter*  mImporter;
    std::vector<Collidable> mRigidBodies;
    std::vector<Collidable> mJoints;
    JNIEnv*                 mEnv;
    SmartLocalRef           mContext;

    bool mNeedRotate;
    int mCurrRigidBody;
    int mCurrJoint;
    int mCurrConstraint;
    int mFirstMultiBody;
    Collider* mCurrCollider;
};

}

#endif //EXTENSIONS_BULLET_FILELOADER_H
