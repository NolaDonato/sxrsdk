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

#include "util/jni_utils.h"
#include <unordered_map>
#include <vector>

class btBulletWorldImporter;
class btMultiBodyWorldImporter;
class btMultiBodyDynamicsWorld;
class btMultiBodyFixedConstraint;
class btGeneric6DofConstraint;
class btPoint2PointConstraint;
class btSliderConstraint;
class btConeTwistConstraint;
class btHingeConstraint;
class btFixedConstraint;
class btCollisionObject;

namespace std
{
    template <>
    struct hash<sxr::SmartLocalRef>
    {
        size_t operator()(const sxr::SmartLocalRef& k) const
        {
            long l = reinterpret_cast<long>(k.getObject());
            return std::hash<long>{}(l);
        }
    };
}

namespace sxr
{
class PhysicsCollidable;
class PhysicsConstraint;
class Collider;
class BulletRigidBody;
class BulletJoint;

class BulletFileLoader : public HybridObject
{
public:
    BulletFileLoader(jobject context, JNIEnv* env);

    bool parse(btMultiBodyDynamicsWorld* world, char *buffer, size_t length, bool ignoreUpAxis);

    bool parse(char *buffer, size_t length, bool ignoreUpAxis);

    void clear();

    virtual ~BulletFileLoader();

    const char* getRigidBodyName(BulletRigidBody* body);

    const char* getJointName(BulletJoint* joint);

    const char* getConstraintName(PhysicsConstraint* constraint);

    jobject getRigidBody(const char* name);

    jobject getJoint(const char* name);

    jobject getCollider(const char* name);

    jobject getConstraint(const char* name);

    jobjectArray getRigidBodies();

    jobjectArray getJoints();

    jobjectArray getConstraints();

    jobject getConstraintBodyA(PhysicsConstraint*);

private:
    jobject    createP2PConstraint(btPoint2PointConstraint* p2p);
    jobject    createHingeConstraint(btHingeConstraint* hg);
    jobject    createConeTwistConstraint(btConeTwistConstraint* ct);
    jobject    createFixedConstraint(btFixedConstraint* fix);
    jobject    createSliderConstraint(btSliderConstraint* sld);
    jobject    createGenericConstraint(btGeneric6DofConstraint* gen);
    void       createRigidBodies();
    void       createJoints();
    void       createConstraints();
    void       createMultiBodyConstraints();
    jobject    createCollider(btCollisionObject*);

    std::unordered_map<std::string, SmartLocalRef>  mRigidBodies;
    std::unordered_map<std::string, SmartLocalRef>  mJoints;
    std::unordered_map<std::string, SmartLocalRef>  mColliders;
    std::unordered_map<std::string, SmartLocalRef>  mConstraints;
    btMultiBodyDynamicsWorld*   mWorld;
    btBulletWorldImporter*      mImporter;
    JNIEnv*                     mEnv;
    SmartLocalRef               mContext;
    bool                        mNeedRotate;
    int                         mFirstMultiBody;
};

}

#endif //EXTENSIONS_BULLET_FILELOADER_H
