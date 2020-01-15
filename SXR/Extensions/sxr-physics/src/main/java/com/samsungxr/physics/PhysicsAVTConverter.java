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

package com.samsungxr.physics;

import android.provider.ContactsContract;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRCapsuleCollider;
import com.samsungxr.SXRCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.animation.SXRPose;
import com.samsungxr.animation.SXRSkeleton;
import com.samsungxr.utility.Log;

import org.joml.Matrix4f;
import org.joml.Vector2f;
import org.joml.Vector3f;
import org.joml.Vector4f;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * Loader for Avatar physics files which describe a multibody articulated hierarchy.
 */
public class PhysicsAVTConverter extends SXRPhysicsLoader
{
    private final Map<String, JSONObject> mTargetBones = new HashMap<String, JSONObject>();
    private final String TAG = "AVT";

    private SXRNode mRoot;
    private String mAttachBoneName;
    private SXRSkeleton mSkeleton;
    private float mAngularDamping;
    private float mLinearDamping;
    private SXRPhysicsContent mWorld;
    private String mFileName;
    private int mAttachBoneIndex;
    private ArrayList<SXRRigidBody> mBodies = new ArrayList<>();
    private ArrayList<SXRPhysicsJoint> mJoints = new ArrayList<>();
    private ArrayList<SXRCollider> mColliders = new ArrayList<>();
    private ArrayList<SXRConstraint> mConstraints = new ArrayList<>();
    private Vector3f mWorldOffset = new Vector3f();
    private int mSimType = SXRRigidBody.DYNAMIC;
    private Vector3f mAngularLimits;
    private Vector3f mAngularSpringStiffness;
    private Vector3f mAngularSpringDamping;
    private int mCollisionGroup = SXRCollisionMatrix.DEFAULT_GROUP;

    public PhysicsAVTConverter(SXRContext ctx)
    {
        super(ctx);
        mRoot = null;
        mAttachBoneName = null;
        mWorld = null;
    }

    protected void applyLoaderProperties(Map<String, Object> loaderProperties)
    {
        Object o = loaderProperties.get("AngularSpringDamping");

        mAttachBoneName = (String) loaderProperties.get("AttachBone");
        mAngularSpringDamping = (o != null) ? (Vector3f) o :
                                 new Vector3f(0, 0, 0);
        o = loaderProperties.get("AngularSpringStiffness");
        mAngularSpringStiffness = (o != null) ? (Vector3f) o :
                                  new Vector3f(0, 0, 0);
        o = loaderProperties.get("AngularLimits");
        mAngularLimits = (o != null) ? (Vector3f) o :
                          new Vector3f((float) Math.PI, (float) Math.PI / 2, (float) Math.PI);
        o = loaderProperties.get("CollisionGroup");
        mCollisionGroup = (o != null) ? (int) o : SXRCollisionMatrix.DEFAULT_GROUP;
        o = loaderProperties.get("SimulationType");
        mSimType = (o != null) ? (int) o : SXRRigidBody.DYNAMIC;
        o = loaderProperties.get("Skeleton");
        mSkeleton = (o != null) ? (SXRSkeleton) o : null;
    }

    public SXRPhysicsContent loadPhysics(SXRScene scene, SXRAndroidResource resource, Map<String, Object> loaderProperties)
    {
        mFileName = resource.getResourcePath();

        if (!mFileName.endsWith(".avt"))
        {
            return super.loadPhysics(scene, resource, loaderProperties);
        }

        String inputData = toString(resource);
        if (inputData != null && !inputData.isEmpty())
        {
            mRoot = scene.getRoot();
            mWorld = (SXRWorld) mRoot.getComponent(SXRWorld.getComponentType());
            applyLoaderProperties(loaderProperties);
            SXRPhysicsContent world = parse(inputData);
            mAttachBoneName = null;
            if (world != null)
            {
                getSXRContext().getEventManager().sendEvent(this,
                                                            IPhysicsLoaderEvents.class,
                                                            "onPhysicsLoaded",
                                                            world, mSkeleton, mFileName);
                return mWorld;
            }
         }
        getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                    "onLoadError",
                                                    mWorld, mFileName,
                                                    "Cannot open physics file");
        return null;
    }

    public void loadPhysics(SXRWorld world, SXRAndroidResource resource, Map<String, Object> loaderProperties)
    {
        mRoot = world.getOwnerObject();
        mFileName = resource.getResourcePath();
        if (!mFileName.endsWith(".avt"))
        {
            super.loadPhysics(world, resource, loaderProperties);
            return;
        }
        String inputData = toString(resource);

        if (inputData != null && !inputData.isEmpty())
        {
            if (mRoot == null)
            {
                mRoot = new SXRNode(getSXRContext());
                mRoot.setName(mFileName);
                mRoot.attachComponent(world);
            }
            mWorld = world;
            applyLoaderProperties(loaderProperties);
            SXRPhysicsContent content = parse(inputData);
            if (content != null)
            {
                getSXRContext().getEventManager().sendEvent(this,
                                                            IPhysicsLoaderEvents.class,
                                                            "onPhysicsLoaded",
                                                            world, mSkeleton, mFileName);
                return;
            }
        }
        getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                    "onLoadError",
                                                    world, mFileName,
                                                    "Cannot open physics file");
    }

    private SXRPhysicsContent parse(String inputData)
    {
        Log.e(TAG, "loading physics file");
        try
        {
            JSONObject start = new JSONObject(inputData);
            mAttachBoneIndex = -1;
            mWorldOffset.set(0, 0, 0);
            if ((mSkeleton != null) && (mAttachBoneName != null))
            {
                mAttachBoneIndex = mSkeleton.getBoneIndex(mAttachBoneName);
                if (mAttachBoneIndex < 0)
                {
                    mAttachBoneIndex = mSkeleton.getNumBones();
                }
            }
            if (mIsMultiBody)
            {
                parseMultiBodyPhysics(start);
            }
            else
            {
                parsePhysics(start);
            }
            Log.e(TAG, "loading done");
            return mWorld;
        }
        catch (JSONException ex)
        {
            ex.printStackTrace();
            return null;
        }
    }

    private void parsePhysics(JSONObject root) throws JSONException
    {
        JSONObject multibody = root.getJSONObject("Multi Body").getJSONObject("value");
        JSONObject basebone = multibody.getJSONObject("Base Bone");
        JSONArray childbones = multibody.getJSONObject("Child Bones").getJSONArray("property_value");

        mTargetBones.clear();
        mTargetBones.put(basebone.getString("Name"), basebone);
        mAngularDamping = (float) multibody.optDouble("Angular Damping", 0.0f);
        mLinearDamping = (float) multibody.optDouble("Linear Damping", 0.0f);
        parseSkeleton(basebone, childbones);
        makeRoom(mSkeleton.getNumBones());
        if (mAttachBoneIndex < 0)
        {
            parseRigidBody(basebone);
        }
        for (int i = 0; i < childbones.length(); ++i)
        {
            JSONObject link = childbones.getJSONObject(i).getJSONObject("value");
            String nodeName = link.getString("Target Bone");
            int boneIndex = mSkeleton.getBoneIndex(nodeName);

            if ((boneIndex < 0) || (boneIndex >= mAttachBoneIndex))
            {
                parseRigidBody(link);
            }
        }
        attachBodies();
    }

    private void attachBodies()
    {
        if (mAttachBoneIndex < 0)
        {
            mAttachBoneIndex = 0;
        }
        int boneoptions = (mSimType == SXRRigidBody.DYNAMIC) ? SXRSkeleton.BONE_PHYSICS :
                           SXRSkeleton.BONE_ANIMATE;

        for (int i = mAttachBoneIndex; i < mSkeleton.getNumBones(); ++i)
        {
            SXRNode node = mSkeleton.getBone(i);
            SXRPhysicsJoint joint = (SXRPhysicsJoint) node.getComponent(SXRPhysicsJoint.getComponentType());
            SXRConstraint constraint = mConstraints.get(i);
            SXRRigidBody oldBody = (SXRRigidBody) node.getComponent(SXRRigidBody.getComponentType());
            SXRRigidBody body = mBodies.get(i);
            SXRCollider collider = mColliders.get(i);
            boolean isNewBone = isAttachedTo(mSkeleton, i, mAttachBoneIndex);

            if (collider != null)
            {
                node.attachComponent(collider);
            }
            if (body != null)
            {
                mSkeleton.setBoneOptions(i, boneoptions);
                if ((joint != null) && joint.getOwnerObject() != null)
                {
                    joint.removeJointAt(joint.getJointIndex());
                }
                node.attachComponent(body);
            }
            if (constraint != null)
            {
                node.detachComponent(SXRConstraint.getComponentType());
                node.attachComponent(constraint);
            }
            else if ((mAttachBoneIndex > 0) && (mSkeleton.getParentBoneIndex(i) == mAttachBoneIndex))
            {
                body = (SXRRigidBody) mSkeleton.getBone(mAttachBoneIndex).getComponent(SXRRigidBody.getComponentType());
                constraint = new SXRFixedConstraint(getSXRContext(), body);
                node.attachComponent(constraint);
            }
        }
    }

    boolean isAttachedTo(SXRSkeleton skel, int boneIndex, int attachIndex)
    {
        int parentIndex = 0;
        do
        {
            parentIndex = skel.getParentBoneIndex(boneIndex);
            if (parentIndex == attachIndex)
            {
                return true;
            }
            boneIndex = parentIndex;
        }
        while (parentIndex > 0);
        return false;
    }

    private SXRSkeleton parseSkeleton(JSONObject basebone, JSONArray bonelist) throws JSONException
    {
        int        maxInputBones = bonelist.length() + 1;
        int        numInputBones = 0;
        int        firstNewBone = 0;
        int        attachBone = -1;
        SXRContext ctx = getSXRContext();
        String     bonenames[] = new String[maxInputBones];
        int        boneparents[] = new int[maxInputBones];
        SXRNode    boneNodes[] = new SXRNode[maxInputBones];

        if (mSkeleton != null)
        {
            attachBone = mSkeleton.getBoneIndex(mAttachBoneName);
        }
        boneparents[0] = -1;
        mTargetBones.clear();

        for (int i = 0; i < maxInputBones; ++i)
        {
            JSONObject bone = (i == 0) ? basebone : bonelist.getJSONObject(i - 1).getJSONObject("value");
            String     parentName = bone.optString("Parent", null);
            String     targetBone = bone.getString("Target Bone");
            JSONObject parent = (parentName != null) ? mTargetBones.get(parentName) : null;
            SXRNode    node;
            SXRNode    parentNode  = null;
            int        parentIndex = -1;

            if (parent != null)
            {
                parentName = parent.getString("Target Bone");
                for (int j = 0; j < i; ++j)
                {
                    if (parentName.equals(bonenames[j]))
                    {
                        parentIndex = j;
                    }
                }
                if (parentIndex >= 0)
                {
                    parentNode = boneNodes[parentIndex];
                }
            }
            bonenames[i] = targetBone;
            boneparents[i] = parentIndex;
            mTargetBones.put(bone.getString("Name"), bone);
            if (mSkeleton != null)
            {
                int boneIndex = mSkeleton.getBoneIndex(targetBone);

                if (parentNode == null)
                {
                    int pi = mSkeleton.getBoneIndex(parentName);
                    parentNode = mSkeleton.getBone(pi);
                }
                if (boneIndex >= 0)
                {
                    if (targetBone.equals(mAttachBoneName))
                    {
                        attachBone = i;
                        if (parentIndex <= attachBone)
                        {
                            boneparents[i] = -1;
                        }
                        boneNodes[i] = mSkeleton.getBone(boneIndex);
                    }
                    if (boneIndex <= attachBone)
                    {
                        continue;
                    }
                    firstNewBone = numInputBones;
                }
            }
            node = new SXRNode(ctx);
            boneNodes[i] = node;
            node.setName(targetBone);
            if (parentNode != null)
            {
                parentNode.addChildObject(node);
            }
            ++numInputBones;
        }
        if (numInputBones > 0)
        {
            bonenames = Arrays.copyOfRange(bonenames, 0, numInputBones);
            boneNodes = Arrays.copyOfRange(boneNodes, 0, numInputBones);
            boneparents = Arrays.copyOfRange(boneparents, 0, numInputBones);
            SXRSkeleton skel = new SXRSkeleton(ctx, boneparents);

            for (int i = 0; i < numInputBones; ++i)
            {
                skel.setBoneName(i, bonenames[i]);
                skel.setBone(i, boneNodes[i]);
                int boneoptions = (mSimType == SXRRigidBody.DYNAMIC) ? SXRSkeleton.BONE_PHYSICS : SXRSkeleton.BONE_ANIMATE;

                skel.setBoneOptions(i, boneoptions);
            }
            if (mSkeleton == null)
            {
                mSkeleton = skel;
                mSkeleton.getBone(0).attachComponent(skel);
                mRoot.addChildObject(mSkeleton.getBone(0));
                mWorldOffset.set(0, 0, 0);
                updateSkeletonPose(mSkeleton, 0, numInputBones, basebone, bonelist);
            }
            else
            {
                JSONObject bone = bonelist.getJSONObject(attachBone - 1).getJSONObject("value");;
                Matrix4f worldMtx = mSkeleton.getBone(attachBone).getTransform().getModelMatrix4f();
                Vector3f newPos = getPosition(bone);

                worldMtx.getTranslation(mWorldOffset);
                mWorldOffset.sub(newPos);
                updateSkeletonPose(skel, firstNewBone, numInputBones, basebone, bonelist);
                mSkeleton.merge(skel, mAttachBoneName);
            }
        }
        else
        {
            updateSkeletonPose(mSkeleton, 0, numInputBones, basebone, bonelist);
        }
        return mSkeleton;
    }

    private void updateSkeletonPose(SXRSkeleton skel, int firstBoneAdded, int numbones, JSONObject basebone, JSONArray bonelist) throws JSONException
    {
        SXRPose worldPose = new SXRPose(skel.getPose());
        Matrix4f worldMtx = new Matrix4f();

        for (int i = firstBoneAdded; i < numbones; ++i)
        {
            JSONObject bone = (i == 0) ? basebone : bonelist.getJSONObject(i - 1).getJSONObject("value");
            JSONObject xform = bone.getJSONObject("Transform");
            JSONObject orientation = xform.getJSONObject("Orientation");
            Vector3f worldPos = getPosition(bone);

            worldPos.add(mWorldOffset, worldPos);
            worldMtx.translationRotate(worldPos.x, worldPos.y, worldPos.z,
                                       (float) orientation.getDouble("X"),
                                       (float) orientation.getDouble("Y"),
                                       (float) orientation.getDouble("Z"),
                                       (float) orientation.getDouble("W"));
            worldPose.setWorldMatrix(i, worldMtx);
        }
        worldPose.sync();
        skel.setPose(worldPose);
        skel.poseToBones();
    }

    private SXRPhysicsJoint parseMultiBodyPhysics(JSONObject root) throws JSONException
    {
        JSONObject multibody = root.getJSONObject("Multi Body").getJSONObject("value");
        JSONObject basebone = multibody.getJSONObject("Base Bone");
        JSONArray childbones = multibody.getJSONObject("Child Bones").getJSONArray("property_value");
        float mass = (float) basebone.getDouble("Mass");
        SXRPhysicsJoint rootJoint = null;
        List<JSONObject> newJoints = new ArrayList<JSONObject>();
        JSONObject link = basebone;
        int i = -1;
        int nextJointIndex = 0;
        boolean extendingSkel = false;
        float maximpulse = (float) multibody.optDouble("Max Applied Impulse", 1000.0f);
        float maxcoordvel = (float) multibody.optDouble("Max Coord Vel", 100.0f);

        mAngularDamping = (float) multibody.optDouble("Angular Damping", 0.0f);
        mLinearDamping = (float) multibody.optDouble("Linear Damping", 0.0f);
        if ((mSkeleton != null) && (mAttachBoneName != null))
        {
            synchronized (mSkeleton)
            {
                nextJointIndex = mSkeleton.getNumBones();
                parseSkeleton(basebone, childbones);
            }
        }
        else
        {
            parseSkeleton(basebone, childbones);
        }
        makeRoom(mSkeleton.getNumBones());
        while (true)
        {
            String nodeName = link.getString("Target Bone");
            int boneID = mSkeleton.getBoneIndex(nodeName);

            if ((boneID < 0) && (newJoints.size() > 0))
            {
                throw new IllegalArgumentException("AVT file skeleton missing bone " +
                        nodeName + " referenced by MultiBody physics");
            }
            newJoints.add(link);
            if (++i >= childbones.length())
            {
                break;
            }
            link = childbones.getJSONObject(i).getJSONObject("value");
        }
        if (nextJointIndex == 0)
        {
            String name = basebone.getString("Target Bone");
            Vector3f scale = new Vector3f(1, 1, 1);

            rootJoint = new SXRPhysicsJoint(mSkeleton, mass, newJoints.size(), mCollisionGroup);
            rootJoint.setFriction((float) basebone.getJSONObject("Physic Material").getDouble("Friction"));
            rootJoint.setName(name);
            rootJoint.setLinearDamping(mLinearDamping);
            rootJoint.setAngularDamping(mAngularDamping);
            rootJoint.setMaxAppliedImpulse(maximpulse);
            rootJoint.setMaxCoordVelocity(maxcoordvel);
            mJoints.set(0, rootJoint);
            Log.e(TAG, "creating root joint %s mass = %3f", basebone.getString("Name"), mass);
            parseCollider(link, 0, name, scale);
            rootJoint.setScale(scale);
        }
        else
        {
            extendingSkel = true;
            rootJoint = (SXRPhysicsJoint) mSkeleton.getBone(0).getComponent(SXRPhysicsJoint.getComponentType());
        }
        for (int j = 1; j < newJoints.size(); ++j)
        {
            parseJoint(newJoints.get(j), extendingSkel);
        }
        attachJoints();
        rootJoint.getSkeleton();
        return rootJoint;
    }

    private void makeRoom(int n)
    {
        mBodies.clear();
        mJoints.clear();
        mConstraints.clear();
        mColliders.clear();
        mBodies.ensureCapacity(n);
        mJoints.ensureCapacity(n);
        mConstraints.ensureCapacity(n);
        mColliders.ensureCapacity(n);
        for (int i = 0; i < n; ++i)
        {
            mBodies.add(null);
            mJoints.add(null);
            mConstraints.add(null);
            mColliders.add(null);
        }
    }

    private void attachJoints()
    {
        if (mAttachBoneIndex < 0)
        {
            mAttachBoneIndex = 0;
        }
        SXRNode root = mSkeleton.getBone(mAttachBoneIndex);
        SXRCollider collider = mColliders.get(mAttachBoneIndex);
        SXRPhysicsJoint joint = mJoints.get(mAttachBoneIndex);
        int boneoptions = (mSimType == SXRRigidBody.DYNAMIC) ? SXRSkeleton.BONE_PHYSICS :
                           SXRSkeleton.BONE_ANIMATE;

        if (collider != null)
        {
            root.attachComponent(collider);
        }
        if (joint != null)
        {
            root.attachComponent(joint);
        }
        for (int i = mAttachBoneIndex + 1; i < mSkeleton.getNumBones(); ++i)
        {
            joint = mJoints.get(i);
            if (joint != null)
            {
                SXRNode node = mSkeleton.getBone(i);
                collider = mColliders.get(i);

                if (collider != null)
                {
                    node.attachComponent(collider);
                }
                mSkeleton.setBoneOptions(i, boneoptions);
                node.attachComponent(joint);
            }
        }
    }

    private SXRCollider parseCollider(JSONObject link, int boneIndex, String targetBone, Vector3f scale) throws JSONException
    {
        SXRContext ctx = mRoot.getSXRContext();
        JSONObject colliderRoot = link.getJSONObject("Collider").getJSONObject("value");
        JSONArray colliders = colliderRoot.getJSONObject("Child Colliders").getJSONArray("property_value");
        String name = colliderRoot.optString("Name");

        for (int i = 0; i < colliders.length(); ++i)
        {
            JSONObject  child = colliders.getJSONObject(i);
            JSONObject  trans = child.getJSONObject("Transform");
            JSONObject  c = child.getJSONObject("Collider");
            String      type = c.optString("type");
            JSONObject  s = trans.optJSONObject("Scale");
            SXRNode     owner = mSkeleton.getBone(mSkeleton.getBoneIndex(targetBone));
            SXRCollider collider;

            if (owner == null)
            {
                throw new IllegalArgumentException("Target bone " + targetBone + " referenced in AVT file not found in scene");
            }
            collider = (SXRCollider) owner.getComponent(SXRCollider.getComponentType());

            if (s != null)
            {
                scale.set((float) s.getDouble("X"),
                          (float) s.getDouble("Y"),
                          (float) s.getDouble("Z"));
            }
            c = c.getJSONObject("value");
            if (type.equals("dmCapsuleCollider"))
            {
                SXRCapsuleCollider capsule = null;
                String direction = c.getString("Direction");
                float radius = (float) c.getDouble("Radius");
                float height = ((float) (c.getDouble("Half Height") * 2) + radius);
                Vector3f dimensions = new Vector3f();

                if ((collider == null) || !(collider instanceof SXRCapsuleCollider))
                {
                    capsule = new SXRCapsuleCollider(ctx);
                    mColliders.set(boneIndex, capsule);
                }
                else if (collider instanceof SXRCapsuleCollider)
                {
                    capsule = (SXRCapsuleCollider) collider;
                }
                else
                {
                    owner.detachComponent(SXRCollider.getComponentType());
                }
                capsule.setRadius(radius);
                if (direction.equals("X"))
                {
                    capsule.setDirection(SXRCapsuleCollider.CapsuleDirection.X_AXIS);
                    dimensions.x = height + radius;
                    dimensions.y = dimensions.z = radius;
                }
                else if (direction.equals("Y"))
                {
                    dimensions.y = height + radius;
                    dimensions.x = dimensions.z = radius;
                    capsule.setDirection(SXRCapsuleCollider.CapsuleDirection.Y_AXIS);
                }
                else if (direction.equals("Z"))
                {
                    dimensions.z = height + radius;
                    dimensions.x = dimensions.y = radius;
                    capsule.setDirection(SXRCapsuleCollider.CapsuleDirection.Z_AXIS);
                }
                capsule.setHeight(height);
                capsule.setRadius(radius);
                Log.e(TAG, "capsule collider %s height %f radius %f %s axis",
                      name, height, radius, direction);
                collider = capsule;
            }
            else if (type.equals("dmBoxCollider"))
            {
                SXRBoxCollider box = null;
                JSONObject size = c.getJSONObject("Half Size");
                float x = (float) size.getDouble("X");
                float y = (float) size.getDouble("Y");
                float z = (float) size.getDouble("Z");

                if ((collider == null) || !(collider instanceof SXRCapsuleCollider))
                {
                    box = new SXRBoxCollider(ctx);
                    mColliders.set(boneIndex, box);
                }
                else if (collider instanceof SXRBoxCollider)
                {
                    box = (SXRBoxCollider) collider;
                }
                else
                {
                    owner.detachComponent(SXRCollider.getComponentType());
                }
                Log.e(TAG, "box collider %s extents  %f, %f, %f",
                      name, x, y, z);
                box.setHalfExtents(x, y, z);
                collider = box;
            }
            else if (type.equals("dmSphereCollider"))
            {
                SXRSphereCollider sphere = null;
                float radius = (float) c.getDouble("Radius");

                if ((collider == null) || !(collider instanceof SXRSphereCollider))
                {
                    sphere = new SXRSphereCollider(ctx);
                    mColliders.set(boneIndex, sphere);
                }
                else if (collider instanceof SXRSphereCollider)
                {
                    sphere = (SXRSphereCollider) collider;
                }
                else
                {
                    owner.detachComponent(SXRCollider.getComponentType());
                }
                Log.e(TAG, "sphere collider %s radius %f", name, radius);
                sphere.setRadius(radius);
                collider = sphere;
            }
            else
            {
                throw new JSONException(type + " is an unknown collider type");
            }
            return collider;
        }
        return null;
    }

    private SXRRigidBody findParentBody(String parentName) throws JSONException
    {
        if (parentName == null)
        {
            return null;
        }
        JSONObject parent = mTargetBones.get(parentName);
        if (parent != null)
        {
            parentName = parent.getString("Target Bone");
            int boneIndex = mSkeleton.getBoneIndex(parentName);
            SXRNode node = mSkeleton.getBone(boneIndex);
            SXRRigidBody body = (SXRRigidBody) node.getComponent(SXRRigidBody.getComponentType());

            if (body != null)
            {
                return body;
            }
            return mBodies.get(boneIndex);
        }
        Log.e(TAG, "Cannot find bone %s referenced by AVT file", parentName);
        return null;
    }

    private Vector3f getPosition(JSONObject link) throws JSONException
    {
        JSONObject trans = link.getJSONObject("Transform");
        JSONObject pos = trans.getJSONObject("Position");

        return new Vector3f((float) pos.getDouble("X"),
                            (float) pos.getDouble("Y"),
                            (float) pos.getDouble("Z"));
    }

    private SXRPhysicsJoint parseJoint(JSONObject link, boolean extendingSkeleton) throws JSONException
    {
        String nodeName = link.getString("Target Bone");
        int jointIndex = mSkeleton.getBoneIndex(nodeName);
        SXRNode node = mSkeleton.getBone(jointIndex);
        float[] pivotB = new float[] { 0, 0, 0 };
        JSONObject piv = link.optJSONObject("Pivot Pos.");
        SXRPhysicsJoint joint = (SXRPhysicsJoint) node.getComponent(SXRPhysicsJoint.getComponentType());
        float friction = (float) link.getJSONObject("Physic Material").getDouble("Friction");
        Vector3f scale = new Vector3f(1, 1, 1);
        SXRPhysicsJoint parentJoint = getParentJoint(link);
        Vector3f worldPos = getPosition(link);

        if (piv != null)
        {
            pivotB[0] = (float) (piv.getDouble("X") - worldPos.x);
            pivotB[1] = (float) (piv.getDouble("Y") - worldPos.y);
            pivotB[2] = (float) (piv.getDouble("Z") - worldPos.z);
        }
        JSONObject v = link.getJSONObject("Axis A");
        Vector3f axisA = new Vector3f((float) v.getDouble("X"),
                                      (float) v.getDouble("Y"),
                                      (float) v.getDouble("Z"));

        parseCollider(link, jointIndex, nodeName, scale);
        if (joint == null)
        {
            joint = createJoint(link, jointIndex, parentJoint, extendingSkeleton);
            if (joint == null)
            {
                return null;
            }

            joint.setPivot(pivotB[0], pivotB[1], pivotB[2]);
            joint.setAxis(axisA.x, axisA.y, axisA.z);
            joint.setFriction(friction);
            joint.setScale(scale);
        }
        else if (jointIndex > mAttachBoneIndex)
        {
            Log.e(TAG, "updating joint %s pivotB(%3f, %3f, %3f) axis(%3f, %3f, %3f)",
                  link.getString("Name"),
                  pivotB[0], pivotB[1], pivotB[2],
                  axisA.x, axisA.y, axisA.z);
            joint.setPivot(pivotB[0], pivotB[1], pivotB[2]);
            joint.setAxis(axisA.x, axisA.y, axisA.z);
            joint.setFriction(friction);
            joint.setScale(scale);
       }
        else if (jointIndex == mAttachBoneIndex)
        {
            joint.setScale(scale);
        }
        return joint;
    }

    private int findJointType(String type) throws JSONException
    {
        if (type.equals("ball"))
        {
            return SXRPhysicsJoint.SPHERICAL;
        }
        if (type.equals("universal"))  // TODO: figure out universal joint
        {
            return SXRPhysicsJoint.SPHERICAL;
        }
        if (type.equals("hinge"))
        {
            return SXRPhysicsJoint.REVOLUTE;
        }
        else if (type.equals("fixed"))
        {
            return SXRPhysicsJoint.FIXED;
        }
        else
        {
            throw new JSONException(type + " is an unknown joint type");
        }
    }

    SXRPhysicsJoint getParentJoint(JSONObject link) throws JSONException
    {
        String parentName = link.optString("Parent", null);
        JSONObject parent = mTargetBones.get(parentName);
        int parentIndex = mSkeleton.getBoneIndex(parent.getString("Target Bone"));
        SXRPhysicsJoint parentJoint = mJoints.get(parentIndex);

        if (parentJoint == null)
        {
            SXRNode node = mSkeleton.getBone(parentIndex);

            parentJoint = (SXRPhysicsJoint) node.getComponent(SXRPhysicsJoint.getComponentType());
            if (parentJoint == null)
            {
                Log.e(TAG, "Parent %s not found", parentName);
                return null;
            }
        }
        return parentJoint;
    }

    private SXRPhysicsJoint createJoint(JSONObject link, int jointIndex, SXRPhysicsJoint parentJoint, boolean addToBody) throws JSONException
    {
        String type = link.getString("Joint Type");
        int jointType = findJointType(type);
        String parentName = link.optString("Parent", null);
        float mass = (float) link.getDouble("Mass");

        if (parentJoint == null)
        {
            return null;
        }
        Log.e(TAG, "creating joint %s parent = %s, mass = %3f",
              link.getString("Name"), parentName, mass);
        SXRPhysicsJoint joint = new SXRPhysicsJoint(parentJoint, jointType, jointIndex, mass, mCollisionGroup);
        joint.setName(link.getString("Target Bone"));
        if (addToBody)
        {
            parentJoint.addJoint(joint);
        }
        mJoints.set(jointIndex, joint);
        return joint;
    }

    private SXRRigidBody parseRigidBody(JSONObject link) throws JSONException
    {
        String nodeName = link.getString("Target Bone");
        int boneIndex = mSkeleton.getBoneIndex(nodeName);
        int attachIndex = mSkeleton.getBoneIndex(mAttachBoneName);
        SXRNode node = mSkeleton.getBone(boneIndex);
        Vector3f scale = new Vector3f(1, 1, 1);

        if (node == null)
        {
            Log.e(TAG,"Cannot find bone " + nodeName + " referenced by AVT file");
            return null;
        }

        SXRContext ctx = getSXRContext();
        float mass = (mSimType != SXRRigidBody.STATIC) ? (float) link.getDouble("Mass") : 0;
        SXRRigidBody parentBody = findParentBody(link.optString("Parent", null));
        SXRRigidBody body = (SXRRigidBody) node.getComponent(SXRRigidBody.getComponentType());
        JSONObject props = link.getJSONObject("Physic Material");

        if (body == null)
        {
            body = new SXRRigidBody(ctx, mass, mCollisionGroup);
            mBodies.set(boneIndex, body);
            body.setSimulationType(mSimType);
        }
        else if (isAttachedTo(mSkeleton, boneIndex, attachIndex))
        {
            body.setSimulationType(mSimType);
        }
        body.setFriction((float) props.getDouble("Friction"));
        body.setDamping(mLinearDamping, mAngularDamping);
        Log.e(TAG, "creating rigid body %s, mass = %3f", link.getString("Name"), mass);

        if ((parentBody == null) && (mSkeleton == null))
        {
            mSkeleton = (SXRSkeleton) node.getComponent(SXRSkeleton.getComponentType());
        }
        if (parseCollider(link, boneIndex, nodeName, scale) == null)
        {
            return null;
        }
        body.setScale(new Vector3f(scale.x, scale.y, scale.z));
        parseConstraint(link, parentBody, node, boneIndex);
        return body;
    }

    SXRConstraint parseConstraint(JSONObject link, SXRRigidBody parentBody, SXRNode node, int boneIndex) throws JSONException
    {
        if (parentBody == null)
        {
            return null;
        }
        String        name = link.getString("Name");
        JSONObject    pivot = link.optJSONObject("Pivot Pos.");
        String        parentName = link.optString("Parent", null);
        Vector3f      pivotB = new Vector3f(0, 0, 0);
        String        type = link.getString("Joint Type");
        JSONArray     dofdata = link.getJSONArray("DOF Data");
        Matrix4f      worldMtx = node.getTransform().getModelMatrix4f();
        Vector3f      worldPos = getPosition(link);
        SXRConstraint constraint;
        JSONObject    v;

        node.detachComponent(SXRConstraint.getComponentType());
        if (pivot != null)
        {
            pivotB.set((float) (pivot.getDouble("X") - worldPos.x),
                       (float) (pivot.getDouble("Y") - worldPos.y),
                       (float) (pivot.getDouble("Z") - worldPos.z));
        }
        JSONObject parentLink = mTargetBones.get(parentName);
        Vector3f pivotA = new Vector3f(0, 0, 0);
        worldPos = getPosition(parentLink);
        if (pivot != null)
        {
            pivotA.set((float) (pivot.getDouble("X") - worldPos.x),
                       (float) (pivot.getDouble("Y") - worldPos.y),
                       (float) (pivot.getDouble("Z") - worldPos.z));
        }
        if (type.equals("ball"))
        {
            SXRGenericConstraint ball;
            JSONObject dof0 = dofdata.getJSONObject(0);
            JSONObject dof1 = dofdata.getJSONObject(1);
            JSONObject dof2 = dofdata.getJSONObject(2);
            Vector2f limits0 = getLimits(dof0, mAngularLimits.x);
            Vector2f limits1 = getLimits(dof1, mAngularLimits.y);
            Vector2f limits2 = getLimits(dof2, mAngularLimits.z);
            Vector3f stiffness = new Vector3f(getValue(dof0,"springStiffness",  mAngularSpringStiffness.x),
                                              getValue(dof1,"springStiffness",  mAngularSpringStiffness.y),
                                              getValue(dof2,"springStiffness",  mAngularSpringStiffness.z));
            Vector3f damping = new Vector3f(getValue(dof0,"springDamping",  mAngularSpringDamping.x),
                                            getValue(dof1,"springDamping",  mAngularSpringDamping.y),
                                            getValue(dof2,"springDamping",  mAngularSpringDamping.z));

            ball = new SXRGenericConstraint(getSXRContext(), parentBody, pivotA, pivotB);
            mConstraints.set(boneIndex, ball);
            ball.setAngularLowerLimits(limits0.x, limits1.x, limits2.x);
            ball.setAngularUpperLimits(limits0.y, limits1.y, limits2.y);
            ball.setLinearLowerLimits(0, 0, 0);
            ball.setLinearUpperLimits(0, 0, 0);
            ball.setAngularStiffness(stiffness.x, stiffness.y, stiffness.z);
            ball.setAngularDamping(damping.x, damping.y, damping.z);
            constraint = ball;
        }
        else if (type.equals("universal"))
        {
            SXRUniversalConstraint ball;
            v = link.getJSONObject("Axis A");
            Vector4f aa = new Vector4f(
                    (float) v.getDouble("X"),
                    (float) v.getDouble("Y"),
                    (float) v.getDouble("Z"),
                    0);
            v = link.getJSONObject("Axis B");
            Vector4f ab = new Vector4f(
                    (float) v.getDouble("X"),
                    (float) v.getDouble("Y"),
                    (float) v.getDouble("Z"),
                    0);
            worldMtx.transform(aa);
            worldMtx.transform(ab);
            Vector3f axisA = new Vector3f(aa.x, aa.y, aa.z);
            Vector3f axisB = new Vector3f(ab.x, ab.y, ab.z);
            axisA.normalize();
            axisB.normalize();

            ball = new SXRUniversalConstraint(getSXRContext(), parentBody, pivotB, axisA, axisB);
            mConstraints.set(boneIndex, ball);
            JSONObject dof0 = dofdata.getJSONObject(0);
            JSONObject dof1 = dofdata.getJSONObject(1);
            Vector2f limits0 = getLimits(dof0, mAngularLimits.x);
            Vector2f limits1 = getLimits(dof1, mAngularLimits.y);

            ball.setAngularLowerLimits(limits0.x, limits1.x, 0);
            ball.setAngularUpperLimits(limits0.y, limits1.y, 0);
            constraint = ball;
        }
        else if (type.equals("hinge"))
        {
            v = link.optJSONObject("Axis A");
            Vector3f axisA = new Vector3f(
                    (float) v.getDouble("X"),
                    (float) v.getDouble("Y"),
                    (float) v.getDouble("Z"));
            SXRHingeConstraint hinge = null;
            JSONObject dof = dofdata.getJSONObject(0);
            Vector2f limits = getLimits(dof, mAngularLimits.x);

            hinge = new SXRHingeConstraint(getSXRContext(), parentBody,
                                          pivotA, pivotB, axisA);
            mConstraints.set(boneIndex, hinge);
            if (limits != null)
            {
                hinge.setLimits(limits.x, limits.y);
            }
            constraint = hinge;
        }
        else if (type.equals("fixed"))
        {
            SXRFixedConstraint fixed = new SXRFixedConstraint(getSXRContext(), parentBody);

            mConstraints.set(boneIndex, fixed);
            constraint = fixed;
        }
        else
        {
            throw new JSONException(type + " is an unknown constraint type");
        }
        Log.e(TAG, "%s constraint between %s pivotA(%3f, %3f, %3f) and %s pivotB(%3f, %3f, %3f)",
              type, parentName,
              pivotA.x, pivotA.y, pivotA.z,
              name, pivotB.x, pivotB.y, pivotB.z);
        return constraint;
    }

    Vector2f getLimits(JSONObject dof, float d) throws JSONException
    {
        float low = (float) Math.toRadians(dof.getDouble("limitLow"));
        float hi = (float) Math.toRadians(dof.getDouble("limitHigh"));
        Vector2f limits = new Vector2f(-d, d);

        if (low != hi)
        {
            limits.x = Math.max(low, -d);
            limits.y = Math.min(hi, d);
        }
        else if (d == 0)
        {
            return null;
        }
        return limits;
    }

    float getValue(JSONObject obj, String name, float d) throws JSONException
    {
        if (!obj.has(name))
        {
            return d;
        }
        float v = (float) Math.toRadians(obj.getDouble(name));
        if (v == 0)
        {
            return d;
        }
        return v;
    }
}

