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
#include <Utils/b3BulletDefaultFileIO.h>

class btBulletWorldImporter;
class btMultiBodyWorldImporter;
class btDynamicsWorld;
class btMultiBodyDynamicsWorld;
class btMultibodyLink;
class btTypedConstraint;
class btMultiBodyConstraint;
class btCollisionShape;
class btMultiBodyFixedConstraint;
class btGeneric6DofConstraint;
class btGeneric6DofSpring2Constraint;
class btPoint2PointConstraint;
class btSliderConstraint;
class btConeTwistConstraint;
class btHingeConstraint;
class btFixedConstraint;
class btCollisionObject;
class btConvexPolyhedron;
class btConvexHullShape;
class btRigidBody;
class btSerializer;
class btVector3;
class btQuaternion;
class URDFConverter;

namespace std
{
    template <>
    struct hash<sxr::SmartGlobalRef>
    {
        size_t operator()(const sxr::SmartGlobalRef& k) const
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
class BulletWorld;
class BulletRigidBody;
class BulletJoint;


/**
 * Imports a binary .bullet file and constructs the SXR
 * Java physics components from it:
 * btRigidBody          SXRRigidBody
 * btCollisionShape     SXRCollider
 * btMultiBody          SXRPhysicsJoint
 * btTypedConstraint    SXRConstraint
 * These objects are connected to each other upon return
 * but are not connected to nodes in the scene.
 *
 * The C++ loader maintain maps between the name of
 * the scene node and the physics components that
 * should be attached to it. They are used by the
 * Java part of the physics loader to attach the physics
 * components to the right nodes.
 */
class BulletFileLoader : public HybridObject
{
protected:
    struct FileIO : b3BulletDefaultFileIO
    {
        AAssetManager* mAssetManager;

        FileIO(AAssetManager* am, int fileIOType = 0, const char* pathPrefix = nullptr);
        virtual int fileOpen(const char* fileName, const char* mode);

    private:
        FILE* fopen(const char* fileName, const char* mode);
    };

public:
    BulletFileLoader(jobject context, JavaVM& jvm, AAssetManager* am);

    bool parse(BulletWorld* world, char *buffer, size_t length, bool ignoreUpAxis);

    bool parse(char *buffer, size_t length, bool ignoreUpAxis);

    bool parseURDF(BulletWorld* world, const char* xmldata, bool ignoreUpAxis);

    void clear();

    virtual ~BulletFileLoader();

    const char* getConstraintName(PhysicsConstraint* constraint);

    jobject getRigidBody(const char* name);

    jobject getJoint(const char* name);

    jobject getCollider(const char* name);

    jobject getConstraint(const char* name);

    jobjectArray getRigidBodies();

    jobjectArray getJoints();

    jobjectArray getConstraints();

private:
    jobject     getConstraintBodyA(PhysicsConstraint*);
    jobject     createP2PConstraint(JNIEnv& env, btPoint2PointConstraint* , PhysicsConstraint*& constraint);
    jobject     createHingeConstraint(JNIEnv& env, btHingeConstraint* hg, PhysicsConstraint*& constraint);
    jobject     createConeTwistConstraint(JNIEnv& env, btConeTwistConstraint* ct, PhysicsConstraint*& constraint);
    jobject     createFixedConstraint(JNIEnv& env, btFixedConstraint* fix, PhysicsConstraint*& constraint);
    jobject     createSliderConstraint(JNIEnv& env, btSliderConstraint* sld, PhysicsConstraint*& constraint);
    jobject     createGenericConstraint(JNIEnv& env, btGeneric6DofConstraint* gen, PhysicsConstraint*& constraint);
    jobject     createSpringConstraint(JNIEnv& env, btGeneric6DofSpring2Constraint* gen, PhysicsConstraint*& constraint);
    void        createRigidBodies(btBulletWorldImporter&);
    void        createRigidBodies(btDynamicsWorld&);
    void        createRigidBody(JNIEnv& env, btRigidBody* rb);
    void        createJoints(btMultiBodyDynamicsWorld&);
    void        createConstraints(btBulletWorldImporter&);
    void        createConstraints(btDynamicsWorld&);
    void        createMultiBodyConstraints(btMultiBodyDynamicsWorld&);
    jobject     createCollider(btCollisionObject*);
    void        createConstraint(JNIEnv& env, btTypedConstraint* constraint);
    void        createMultiBodyConstraint(JNIEnv& env, btMultiBodyConstraint* c);
    const char* getNameForPointer(void* p);
    void        rotateLink(btMultibodyLink& link);
    btVector3&  rotatePoint(btVector3& p);
    btQuaternion&       rotateQuat(btQuaternion& q);
    btConvexPolyhedron* rotatePoly(const btConvexPolyhedron* input, btVector3* outVerts);
    btCollisionShape*   createCollisionShape(JNIEnv& env, btCollisionShape* shape, jobject& obj);
    btConvexHullShape* copyHull(const btConvexHullShape *input, btVector3 *outVerts);

    std::unordered_map<std::string, SmartGlobalRef>  mRigidBodies;
    std::unordered_map<std::string, SmartGlobalRef>  mJoints;
    std::unordered_map<std::string, SmartGlobalRef>  mColliders;
    std::unordered_map<std::string, SmartGlobalRef>  mConstraints;
    btSerializer*               mSerializer;
    btBulletWorldImporter*      mBulletImporter;
    URDFConverter*              mURDFImporter;
    JavaVM&                     mJavaVM;
    SmartGlobalRef              mContext;
    bool                        mNeedRotate;
    int                         mFirstMultiBody;
    FileIO                      mFileIO;
};

}

#endif //EXTENSIONS_BULLET_FILELOADER_H
