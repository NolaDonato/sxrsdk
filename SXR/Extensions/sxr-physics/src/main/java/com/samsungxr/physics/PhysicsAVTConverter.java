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
import org.joml.Vector3f;
import org.joml.Vector4f;
import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

import java.io.FileOutputStream;
import java.io.IOException;
import java.io.InputStream;
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

    public PhysicsAVTConverter(SXRContext ctx)
    {
        super(ctx);
        mRoot = null;
        mAttachBoneName = null;
        mWorld = null;
    }

    public SXRPhysicsContent loadPhysics(SXRScene scene, SXRAndroidResource resource, boolean ignoreUpAxis)
    {
        mFileName = resource.getResourceFilename().toLowerCase();
        if (!mFileName.endsWith(".avt"))
        {
            return super.loadPhysics(scene, resource, ignoreUpAxis);
        }

        String inputData = toString(resource);
        if (inputData != null && !inputData.isEmpty())
        {
            mRoot = scene.getRoot();
            mWorld = (SXRWorld) mRoot.getComponent(SXRWorld.getComponentType());
            SXRPhysicsContent world = parse(inputData);
            mAttachBoneName = null;
            SXRSkeleton skel = mSkeleton;
            mSkeleton = null;
            if (world != null)
            {
                getSXRContext().getEventManager().sendEvent(this,
                                                            IPhysicsLoaderEvents.class,
                                                            "onPhysicsLoaded",
                                                            world, skel, mFileName);
                return mWorld;
            }
         }
        getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                    "onLoadError", mFileName,
                                                    "Cannot open physics file");
        return null;
    }

    public void loadPhysics(SXRWorld world, SXRAndroidResource resource, boolean ignoreUpAxis)
    {
        mRoot = world.getOwnerObject();
        mFileName = resource.getResourceFilename().toLowerCase();
        if (!mFileName.endsWith(".avt"))
        {
            super.loadPhysics(world, resource, ignoreUpAxis);
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
            SXRPhysicsContent content = parse(inputData);
            SXRSkeleton skel = mSkeleton;
            mSkeleton = null;
            mAttachBoneName = null;
            if (content != null)
            {
                getSXRContext().getEventManager().sendEvent(this,
                                                            IPhysicsLoaderEvents.class,
                                                            "onPhysicsLoaded",
                                                            world, skel, mFileName);
                return;
            }
        }
        getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                    "onLoadError", mFileName,
                                                    "Cannot open physics file");
    }


    /**
     * Loads physics components from a JSON file describing the avatar and associates
     * them to the skeleton provided
     * <p>
     * Avatar files describe physics for articulated bodies.
     * Each file has a skeleton and joints with collision geometries.
     * The contents of the AVT is not added to the current scene.
     * Instead it is imported and contained in a {@link SXRPhysicsContent}
     * object (like a physics world but it cannot simulate, just a container).
     * @param skel        {@link SXRSkeleton} to use
     * @param attachBone  name of bone in skeleton to associate with the root joint in physics.
     * @param resource    source of physics content to import
     */
    public SXRPhysicsContent loadPhysics(SXRAndroidResource resource, SXRSkeleton skel, String attachBone)
    {
        mSkeleton = skel;
        mAttachBoneName = attachBone;
        return loadPhysics(resource, false);
    }

    /**
     * Loads physics components from a JSON file describing the avatar and associates
     * them to the skeleton provided
     * <p>
     * Avatar files describe physics for articulated bodies.
     * Each file has a skeleton and joints with collision geometries.
     * The contents of the AVT is not added to the current scene.
     * Instead it is imported and contained in a {@link SXRPhysicsContent}
     * object (like a physics world but it cannot simulate, just a container).
     * @param skel        {@link SXRSkeleton} to use
     * @param attachBone  name of bone in skeleton to associate with the root joint in physics.
     * @param resource    source of physics content to import
     */
    public SXRPhysicsContent loadPhysics(SXRWorld world, SXRAndroidResource resource, SXRSkeleton skel, String attachBone)
    {
        mSkeleton = skel;
        mAttachBoneName = attachBone;
        loadPhysics(world, resource, false);
        return world;
    }

    public boolean convertToBullet(String infile, String outfile, boolean isMultibody)
    {
        SXRAndroidResource resource = toAndroidResource(getSXRContext(), infile);
        SXRPhysicsContent content = loadPhysics(resource, false);
        if (content == null)
        {
            return false;
        }
        SXRNode physicsRoot = new SXRNode(getSXRContext());
        SXRWorld bulletWorld = new SXRWorld(physicsRoot, isMultibody);
        bulletWorld.merge(content);
        return exportPhysics(bulletWorld, outfile);
    }

    private SXRPhysicsContent parse(String inputData)
    {
        Log.e(TAG, "loading physics file");
        try
        {
            JSONObject start = new JSONObject(inputData);
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
            return null;
        }
    }

    private void parsePhysics(JSONObject root) throws JSONException
    {
        JSONObject multibody = root.getJSONObject("Multi Body").getJSONObject("value");
        JSONObject basebone = multibody.getJSONObject("Base Bone");
        JSONArray childbones = multibody.getJSONObject("Child Bones").getJSONArray("property_value");
        int firstBoneIndex = -1;

        mTargetBones.clear();
        mTargetBones.put(basebone.getString("Name"), basebone);
        mAngularDamping = (float) multibody.optDouble("Angular Damping", 0.0f);
        mLinearDamping = (float) multibody.optDouble("Linear Damping", 0.0f);
        if (mSkeleton != null)
        {
            if (mAttachBoneName != null)
            {
                firstBoneIndex = mSkeleton.getBoneIndex(mAttachBoneName);
                if (firstBoneIndex < 0)
                {
                    firstBoneIndex = mSkeleton.getNumBones();
                }
            }
        }
        parseSkeleton(basebone, childbones);
        if (firstBoneIndex < 0)
        {
            parseRigidBody(basebone);
        }
        for (int i = 0; i < childbones.length(); ++i)
        {
            JSONObject link = childbones.getJSONObject(i).getJSONObject("value");
            String nodeName = link.getString("Target Bone");
            int boneIndex = mSkeleton.getBoneIndex(nodeName);

            if ((boneIndex < 0) || (boneIndex > firstBoneIndex))
            {
                parseRigidBody(link);
            }
        }
    }

    private SXRSkeleton parseSkeleton(JSONObject basebone, JSONArray bonelist) throws JSONException
    {
        int maxInputBones = bonelist.length() + 1;
        String bonenames[] = new String[maxInputBones];
        int boneparents[] = new int[maxInputBones];
        SXRNode boneNodes[] = new SXRNode[maxInputBones];
        int numInputBones = 0;
        int firstBoneAdded = -1;
        int attachBoneIndex = 0;
        SXRContext ctx = getSXRContext();

        boneparents[0] = -1;
        mTargetBones.clear();

        for (int i = 0; i < maxInputBones; ++i)
        {
            JSONObject bone = (i == 0) ? basebone : bonelist.getJSONObject(i - 1).getJSONObject("value");
            String parentName = bone.optString("Parent", "");
            String targetBone = bone.getString("Target Bone");
            JSONObject parent = mTargetBones.get(parentName);
            SXRNode node = null;

            mTargetBones.put(bone.getString("Name"), bone);
            /*
             * Skip all bones which are parents of the attachment point.
             * These will presumably be in the skeleton we attach to.
             */
            if ((mAttachBoneName != null) &&
                (numInputBones == 0))
            {
                if (targetBone.equals(mAttachBoneName))
                {
                    attachBoneIndex = i;
                }
                else
                {
                    continue;
                }
            }
            if (mSkeleton != null)
            {
                int boneIndex = mSkeleton.getBoneIndex(targetBone);
                if (mSkeleton.getBone(boneIndex) != null)
                {
                    ++numInputBones;
                    continue;
                }
            }
            node = new SXRNode(ctx);
            boneparents[numInputBones] = -1;
            parentName = (parent != null) ? parent.getString("Target Bone") : "";
            bonenames[numInputBones] = targetBone;
            boneNodes[numInputBones] = node;
            node.setName(bonenames[numInputBones]);
            if (firstBoneAdded < 0)
            {
                firstBoneAdded = i;
            }
            if (!parentName.isEmpty())
            {
                for (int j = 0; j < i; ++j)
                {
                    if (parentName.equals(bonenames[j]))
                    {
                        SXRNode p = boneNodes[j];
                        boneparents[numInputBones] = j;

                        if (p != null)
                        {
                            p.addChildObject(boneNodes[numInputBones]);
                        }
                        break;
                    }
                }
            }
            ++numInputBones;
        }
        if (firstBoneAdded >= 0)
        {
            if (numInputBones < maxInputBones)
            {
                bonenames = Arrays.copyOfRange(bonenames, 0, numInputBones);
                boneNodes = Arrays.copyOfRange(boneNodes, 0, numInputBones);
                boneparents = Arrays.copyOfRange(boneparents, 0, numInputBones);
            }
            SXRSkeleton skel = new SXRSkeleton(ctx, boneparents);

            for (int i = 0; i < numInputBones; ++i)
            {
                skel.setBoneName(i, bonenames[i]);
                skel.setBone(i, boneNodes[i]);
            }
            updateSkeletonPose(skel, numInputBones, basebone, bonelist, attachBoneIndex);
            if (mSkeleton == null)
            {
                mSkeleton = skel;
                mSkeleton.getBone(0).attachComponent(skel);
                mRoot.addChildObject(mSkeleton.getBone(0));
            }
            else
            {
                mSkeleton.merge(skel, mAttachBoneName);
            }
        }
        else
        {
            updateSkeletonPose(mSkeleton, numInputBones, basebone, bonelist, attachBoneIndex);
        }
        return mSkeleton;
    }

    private void updateSkeletonPose(SXRSkeleton skel, int numInputBones, JSONObject basebone, JSONArray bonelist, int attachBoneIndex) throws JSONException
    {
        SXRPose worldPose = new SXRPose(skel);
        Matrix4f worldMtx = new Matrix4f();

        for (int i = attachBoneIndex; i < numInputBones; ++i)
        {
            JSONObject bone = (i == 0) ? basebone : bonelist.getJSONObject(i - 1).getJSONObject("value");
            JSONObject xform = bone.getJSONObject("Transform");
            JSONObject position = xform.getJSONObject("Position");
            JSONObject orientation = xform.getJSONObject("Orientation");

            worldMtx.translationRotate((float) position.getDouble("X"),
                    (float) position.getDouble("Y"),
                    (float) position.getDouble("Z"),
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
        int firstBoneIndex = 0;
        SXRPhysicsJoint rootJoint = null;
        List<JSONObject> newJoints = new ArrayList<JSONObject>();
        JSONObject link = basebone;
        int i = -1;
        int jointIndex = 0;

        if ((mSkeleton != null) && (mAttachBoneName != null))
        {
            jointIndex = mSkeleton.getNumBones();
        }
        parseSkeleton(basebone, childbones);
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
        if (jointIndex == 0)
        {
            rootJoint = new SXRPhysicsJoint(mSkeleton, mass, newJoints.size(), SXRCollisionMatrix.DEFAULT_GROUP);
            rootJoint.setFriction((float) link.getJSONObject("Physic Material").getDouble("Friction"));
            rootJoint.setBoneIndex(0);
            parseCollider(link, newJoints.get(0).getString("Target Bone"));
            mSkeleton.getBone(0).attachComponent(rootJoint);
        }
        else
        {
            int jointsAdded = mSkeleton.getNumBones() - jointIndex;
            rootJoint = (SXRPhysicsJoint) mSkeleton.getBone(0).getComponent(SXRPhysicsJoint.getComponentType());
            if (jointsAdded > 0)
            {
                rootJoint.setNumJoints(jointIndex + jointsAdded);
            }
        }
        for (int j = 1; j < newJoints.size(); ++j)
        {
            parseJoint(newJoints.get(j), ++jointIndex);
        }
        rootJoint.getSkeleton();
        return rootJoint;
    }

    private SXRCollider parseCollider(JSONObject link, String targetBone) throws JSONException
    {
        SXRContext ctx = mRoot.getSXRContext();
        JSONObject colliderRoot = link.getJSONObject("Collider").getJSONObject("value");
        JSONArray colliders = colliderRoot.getJSONObject("Child Colliders").getJSONArray("property_value");

        for (int i = 0; i < colliders.length(); ++i)
        {
            JSONObject c = colliders.getJSONObject(i).getJSONObject("Collider");
            JSONObject xform = colliders.getJSONObject(i).getJSONObject("Transform");
            String type = c.optString("type");
            SXRNode owner = mSkeleton.getBone(mSkeleton.getBoneIndex(targetBone));
            SXRCollider collider;

            if (owner == null)
            {
                throw new IllegalArgumentException("Target bone " + targetBone + " referenced in AVT file not found in scene");
            }
            owner.detachComponent(SXRCollider.getComponentType());
            JSONObject scale = xform.getJSONObject("Scale");
            Vector3f s = new Vector3f(1, 1, 1);

            c = c.getJSONObject("value");
            s.set((float) scale.getDouble("X"),
                  (float) scale.getDouble("Y"),
                  (float) scale.getDouble("Z"));
            if (type.equals("dmCapsuleCollider"))
            {
                SXRCapsuleCollider capsule = new SXRCapsuleCollider(ctx);
                String direction = c.getString("Direction");
                float radius = (float) c.getDouble("Radius");
                float height = ((float) (c.getDouble("Half Height") * 2) + radius);
                Vector3f dimensions = new Vector3f();

                capsule.setRadius(radius);
                if (direction.equals("X"))
                {
                    radius *= s.z;
                    height *= s.x;
                    capsule.setDirection(SXRCapsuleCollider.CapsuleDirection.X_AXIS);
                    dimensions.x = height + radius;
                    dimensions.y = dimensions.z = radius;
                }
                else if (direction.equals("Y"))
                {
                    height *= s.y;
                    radius *= s.x;
                    dimensions.y = height + radius;
                    dimensions.x = dimensions.z = radius;
                    capsule.setDirection(SXRCapsuleCollider.CapsuleDirection.Y_AXIS);
                }
                else if (direction.equals("Z"))
                {
                    height *= s.z;
                    radius *= s.x;
                    dimensions.z = height + radius;
                    dimensions.x = dimensions.y = radius;
                    capsule.setDirection(SXRCapsuleCollider.CapsuleDirection.Z_AXIS);
                }
                capsule.setHeight(height);
                capsule.setRadius(radius);
                Log.e(TAG, "capsule collider %s height %f radius %f %s axis",
                      colliderRoot.getString("Name"), height, radius, direction);
                collider = capsule;
            }
            else if (type.equals("dmBoxCollider"))
            {
                SXRBoxCollider box = new SXRBoxCollider(ctx);
                JSONObject size = c.getJSONObject("Half Size");
                float x = (float) size.getDouble("X") * s.x;
                float y = (float) size.getDouble("Y") * s.y;
                float z = (float) size.getDouble("Z") * s.z;

                Log.e(TAG, "box collider %s extents  %f, %f, %f",
                      colliderRoot.getString("Name"), x, y, z);
                box.setHalfExtents(x, y, z);
                collider = box;
            }
            else if (type.equals("dmSphereCollider"))
            {
                SXRSphereCollider sphere = new SXRSphereCollider(ctx);
                float radius = (float) c.getDouble("Radius");

                Log.e(TAG, "sphere collider %s radius %f", colliderRoot.getString("Name"), radius);
                sphere.setRadius(radius);
                collider = sphere;
            }
            else
            {
                throw new JSONException(type + " is an unknown collider type");
            }
            owner.attachComponent(collider);
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
        if (parent == null)
        {
            return null;
        }
        String nodeName = parent.getString("Target Bone");
        SXRNode parentNode = mSkeleton.getBone(mSkeleton.getBoneIndex(nodeName));

        if (parentNode != null)
        {
            SXRRigidBody body = (SXRRigidBody) parentNode.getComponent(SXRRigidBody.getComponentType());
            if (body != null)
            {
                return body;
            }
        }
        Log.e(TAG, "Cannot find bone %s referenced by AVT file", nodeName);
        return null;
    }

    private SXRPhysicsJoint parseJoint(JSONObject link, int jointIndex) throws JSONException
    {
        String nodeName = link.getString("Target Bone");
        int boneID = mSkeleton.getBoneIndex(nodeName);
        SXRNode node = mSkeleton.getBone(boneID);
        JSONArray dofdata = link.getJSONArray("DOF Data");
        float[] pivotB = new float[] { 0, 0, 0 };
        Vector3f axis = null;
        JSONObject trans = link.getJSONObject("Transform");
        JSONObject pos = trans.getJSONObject("Position");
        JSONObject piv = link.optJSONObject("Pivot Pos.");
        SXRPhysicsJoint joint = (SXRPhysicsJoint) node.getComponent(SXRPhysicsJoint.getComponentType());

        if (piv != null)
        {
            pivotB[0] = (float) (piv.getDouble("X") - pos.getDouble("X"));
            pivotB[1] = (float) (piv.getDouble("Y") - pos.getDouble("Y"));
            pivotB[2] = (float) (piv.getDouble("Z") - pos.getDouble("Z"));
        }
        JSONObject v = link.getJSONObject("Axis A");
        axis = new Vector3f((float) v.getDouble("X"),
                (float) v.getDouble("Y"),
                (float) v.getDouble("Z"));
        parseCollider(link, nodeName);
        if (joint == null)
        {
            joint = createJoint(link, jointIndex);
            if (joint == null)
            {
                return null;
            }
            joint.setPivot(pivotB[0], pivotB[1], pivotB[2]);
            joint.setAxis(axis.x, axis.y, axis.z);
            joint.setFriction((float) link.getJSONObject("Physic Material").getDouble("Friction"));
            node.attachComponent(joint);
        }
        else
        {
            Log.e(TAG, "updating joint %s", link.getString("Name"));
            joint.setPivot(pivotB[0], pivotB[1], pivotB[2]);
            joint.setAxis(axis.x, axis.y, axis.z);
            joint.setFriction((float) link.getJSONObject("Physic Material").getDouble("Friction"));
            joint.sync(SXRPhysicsCollidable.SYNC_ALL);
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

    private SXRPhysicsJoint createJoint(JSONObject link, int jointIndex) throws JSONException
    {
        String type = link.getString("Joint Type");
        int jointType = findJointType(type);
        String parentName = link.optString("Parent", null);
        float mass = (float) link.getDouble("Mass");
        SXRPhysicsJoint parentJoint = null;
        JSONObject parent = mTargetBones.get(parentName);
        int boneIndex = mSkeleton.getBoneIndex(parent.getString("Target Bone"));
        SXRNode parentNode = mSkeleton.getBone(boneIndex);

        if (parentNode != null)
        {
            parentJoint = (SXRPhysicsJoint) parentNode.getComponent(SXRPhysicsJoint.getComponentType());
            if (parentJoint == null)
            {
                parentJoint = parseJoint(parent, boneIndex);
            }
        }
        if (parentJoint == null)
        {
            Log.e(TAG, "Parent %s not found", parentName);
            return null;
        }
        Log.e(TAG, "creating joint %s parent = %s", link.getString("Name"), parentName);
        SXRPhysicsJoint joint = new SXRPhysicsJoint(parentJoint, jointType, jointIndex, mass, SXRCollisionMatrix.DEFAULT_GROUP);
        joint.setBoneIndex(jointIndex);
        return joint;
    }

    private SXRRigidBody parseRigidBody(JSONObject link) throws JSONException
    {
        String nodeName = link.getString("Target Bone");
        SXRNode node = mSkeleton.getBone(mSkeleton.getBoneIndex(nodeName));
        SXRContext ctx = getSXRContext();

        if (node == null)
        {
            Log.e(TAG,"Cannot find bone " + nodeName + " referenced by AVT file");
            return null;
        }
        String name = link.getString("Name");
        String parentName = link.optString("Parent", null);
        float mass = (float) link.getDouble("Mass");
        SXRRigidBody parentBody = findParentBody(parentName);
        SXRRigidBody body = new SXRRigidBody(ctx, mass, SXRCollisionMatrix.DEFAULT_GROUP);
        JSONObject props = link.getJSONObject("Physic Material");
        JSONObject v;
        float PIover2 = (float) Math.PI / 2;
        SXRPhysicsJoint joint = (SXRPhysicsJoint) node.getComponent(SXRPhysicsJoint.getComponentType());

        body.setFriction((float) props.getDouble("Friction"));
        body.setDamping(mLinearDamping, mAngularDamping);
        node.detachComponent(SXRRigidBody.getComponentType());
        if (joint != null)
        {
            joint.disable();
        }
        mTargetBones.put(name, link);
        if (parentBody == null)
        {
            SXRSkeleton skel = (SXRSkeleton) node.getComponent(SXRSkeleton.getComponentType());
            if (mWorld.isMultiBody() && (skel != null))
            {
                mSkeleton = skel;
            }
            if (parseCollider(link, nodeName) == null)
            {
                return null;
            }
            node.attachComponent(body);
            return body;
        }
        String type = link.getString("Joint Type");
        SXRConstraint constraint;
        JSONArray dofdata = link.getJSONArray("DOF Data");
        if (parseCollider(link, nodeName) == null)
        {
            return null;
        }
        node.attachComponent(body);
        JSONObject trans = link.getJSONObject("Transform");
        JSONObject pos = trans.getJSONObject("Position");
        JSONObject pivot = link.optJSONObject("Pivot Pos.");
        Vector3f pivotB = new Vector3f(0, 0, 0);
        Matrix4f worldMtx = node.getTransform().getModelMatrix4f();

        if (pivot != null)
        {
            pivotB.set((float) (pivot.getDouble("X") - pos.getDouble("X")),
                       (float) (pivot.getDouble("Y") - pos.getDouble("Y")),
                       (float) (pivot.getDouble("Z") - pos.getDouble("Z")));
        }
        JSONObject parentLink = mTargetBones.get(parentName);
        Vector3f pivotA = new Vector3f(0, 0, 0);
        trans = parentLink.getJSONObject("Transform");
        pos = trans.getJSONObject("Position");
        if (pivot != null)
        {
            pivotA.set((float) (pivot.getDouble("X") - pos.getDouble("X")),
                       (float) (pivot.getDouble("Y") - pos.getDouble("Y")),
                       (float) (pivot.getDouble("Z") - pos.getDouble("Z")));
        }
        if (type.equals("ball"))
        {
            if (true)
            {
                SXRGenericConstraint ball = new SXRGenericConstraint(ctx, parentBody, pivotA, pivotB);
                ball.setAngularLowerLimits(-PIover2, -PIover2, -PIover2);
                ball.setAngularUpperLimits(PIover2, PIover2, PIover2);
                ball.setLinearLowerLimits(0, 0, 0);
                ball.setLinearUpperLimits(0, 0, 0);
                constraint = ball;
            }
            else
            {
                SXRPoint2PointConstraint ball = new SXRPoint2PointConstraint(ctx, parentBody, pivotA, pivotB);
                constraint = ball;
            }
            Log.e(TAG, "Ball joint between %s and %s (%f, %f, %f)",
                  parentName, name, pivotB.x, pivotB.y, pivotB.z);
        }
        else if (type.equals("universal"))
        {
            if (pivot != null)
            {
                pivotB.set((float) (pivot.getDouble("X")),
                           (float) (pivot.getDouble("Y")),
                           (float) (pivot.getDouble("Z")));
            }
            else
            {
                pivotB.set(0, 0, 0);
            }
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

            SXRUniversalConstraint ball = new SXRUniversalConstraint(ctx, parentBody, pivotB, axisA, axisB);
            JSONObject dof0 = dofdata.getJSONObject(0);
            JSONObject dof1 = dofdata.getJSONObject(1);
            float lz = dof0.getBoolean("useLimit") ? (float) Math.toRadians(dof0.getDouble("limitLow")) : -PIover2;
            float ly = dof1.getBoolean("useLimit") ? (float) Math.toRadians(dof1.getDouble("limitLow")) : -PIover2;

            ball.setAngularLowerLimits(0, ly, lz);
            lz = dof0.getBoolean("useLimit") ? (float) Math.toRadians(dof0.getDouble("limitHigh")) : PIover2;
            ly = dof1.getBoolean("useLimit") ? (float) Math.toRadians(dof1.getDouble("limitHigh")) : PIover2;
            ball.setAngularUpperLimits(0, ly, lz);
            constraint = ball;
            Log.e(TAG, "Universal joint between %s and %s (%f, %f, %f)",
                  parentName, name, pivotB.x, pivotB.y, pivotB.z);
        }
        else if (type.equals("hinge"))
        {
            v = link.optJSONObject("Axis A");
            Vector3f axisA = new Vector3f(
                        (float) v.getDouble("X"),
                        (float) v.getDouble("Y"),
                        (float) v.getDouble("Z"));
            SXRHingeConstraint hinge = new SXRHingeConstraint(ctx, parentBody,
                                                              pivotA, pivotB, axisA);
            JSONObject dof = dofdata.getJSONObject(0);

            if (dof.getBoolean("useLimit"))
            {
                hinge.setLimits((float) Math.toRadians(dof.getDouble("limitLow")), (float) Math.toRadians(dof.getDouble("limitHigh")));
            }
            else
            {
                hinge.setLimits(-PIover2, PIover2);
            }
            constraint = hinge;
            Log.e(TAG, "Hinge joint between %s and %s (%f, %f, %f)",
                  parentName, name, pivotB.x, pivotB.y, pivotB.z);
        }
        else if (type.equals("fixed"))
        {
           SXRFixedConstraint fixed = new SXRFixedConstraint(ctx, parentBody);
            constraint = fixed;
            Log.e(TAG, "Fixed joint between %s and %s", parentName, name);
        }
        else
        {
            throw new JSONException(type + " is an unknown constraint type");
        }
        if (link.has("Breaking Reaction Impulse"))
        {
            constraint.setBreakingImpulse((float) link.getDouble("Breaking Reaction Impulse"));
        }
        node.attachComponent(constraint);
        return body;
    }

}

