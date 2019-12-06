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

import android.content.res.AssetManager;

import com.samsungxr.IEventReceiver;
import com.samsungxr.IEvents;
import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXREventReceiver;
import com.samsungxr.SXRHybridObject;
import com.samsungxr.SXRMeshCollider;
import com.samsungxr.SXRResourceVolume;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.animation.SXRSkeleton;
import com.samsungxr.utility.Log;

import org.joml.Vector3f;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

public class SXRPhysicsLoader extends SXRHybridObject implements IEventReceiver
{
    static protected final String TAG = SXRPhysicsLoader.class.getSimpleName();
    protected boolean mIsMultiBody = false;
    protected String mErrors = "";
    protected SXREventReceiver mListeners;
    protected Map<String, Object> mDefaultProperties = new HashMap<String, Object>();

    static
    {
        System.loadLibrary("BulletFileLoader");
        System.loadLibrary("BulletWorldImporter");
    }

    public interface IPhysicsLoaderEvents extends IEvents
    {
        /**
         * Called after a physics file is loaded.
         * @param  world    {@link }SXRPhysicsContent} containing the physics objects loaded.
         *                  May be null if the load failed.
         * @param skel      {@link SXRSkeleton} created from physics, may be null.
         * @param filename  Name of file or resource loaded.
         */
        public void onPhysicsLoaded(SXRPhysicsContent world, SXRSkeleton skel, String filename);

        /**
         * Called if a physics file fails to load.
         * @param filename  Name of file or resource loaded.
         * @param errors    Errors during loading, null if load was successful.
         */
        public void onLoadError(SXRPhysicsContent world, String filename, String errors);
    }

    public SXRPhysicsLoader(SXRContext ctx)
    {
        super(ctx, NativeBulletLoader.ctor(ctx, ctx.getActivity().getApplicationContext().getResources().getAssets()));
        mListeners = new SXREventReceiver(this);
    }

    public SXRPhysicsLoader(SXRContext ctx, AssetManager assetMgr)
    {
        super(ctx, NativeBulletLoader.ctor(ctx, assetMgr));
        mListeners = new SXREventReceiver(this);
    }

    public SXREventReceiver getEventReceiver() { return mListeners; }

    public boolean isMultiBody() { return mIsMultiBody; }

    public void setMultiBody(boolean flag)
    {
        mIsMultiBody = flag;
    }

    /**
     * Loads a physics content file.
     * <p>
     * Bullet (.bullet) physics files and Universal Robot
     * Description Format (.urdf) are supported.
     * Physics files contain only physics components,
     * rigid bodies, joints, constraints and colliders.
     * There are no nodes or meshes. The imported physics
     * components are added to the nodes with the same
     * name in the scene provided.
     *
     * @param fileName Name of file containing physics content.
     * @param scene    The scene containing the objects to attach physics components.
     */
    public void loadPhysics(SXRScene scene, String fileName)
    {
        loadPhysics(scene, fileName, new HashMap<String, Object>());
    }

    /**
     * Loads a physics content file.
     * <p>
     * Bullet (.bullet) physics files and Universal Robot
     * Description Format (.urdf) are supported.
     * Physics files contain only physics components,
     * rigid bodies, joints, constraints and colliders.
     * There are no nodes or meshes. The imported physics
     * components are added to the nodes with the same
     * name in the scene provided.
     *
     * @param fileName         Name of file containing physics content.
     * @param scene            The scene containing the objects to attach physics components.
     * @param loaderProperties Default values for physics properties:
     *                         ignoreupaxis:           assume Y axis is up if true, else set from physics content
     *                         attachbone:             name of skeleton bone to attach new physics to
     *                         CollisionGroup:         integer collision group for new bodies / joints
     *                         AngularLimits:          angular limits in radians for generic constraints
     *                         AngularSpringStiffness: angular spring stiffness for generic constraints
     *                         AngularSpringDamping:   angular spring damping for generic constraints
     */
    public void loadPhysics(SXRScene scene, String fileName, Map<String, Object> loaderProperties)
    {
        SXRAndroidResource resource = toAndroidResource(scene.getSXRContext(), fileName);
        loadPhysics(scene, resource, loaderProperties);
    }

    /**
     * Loads a physics content file.
     * <p>
     * Bullet (.bullet) physics files and Universal Robot
     * Description Format (.urdf) are supported.
     * Physics files contain only physics components,
     * rigid bodies, joints, constraints and colliders.
     * There are no nodes or meshes. The imported physics
     * components are added to the nodes with the same
     * name in the scene provided.
     *
     * @param resource {@link SXRAndroidResource} referencing the file containing physics content.
     * @param scene    The scene containing the objects to attach physics components.
     * @param loaderProperties Default values for physics properties:
     *                         ignoreupaxis:           assume Y axis is up if true, else set from physics content
     *                         attachbone:             name of skeleton bone to attach new physics to
     *                         CollisionGroup:         integer collision group for new bodies / joints
     *                         AngularLimits:          angular limits in radians for generic constraints
     *                         AngularSpringStiffness: angular spring stiffness for generic constraints
     *                         AngularSpringDamping:   angular spring damping for generic constraints
     */
    public SXRPhysicsContent loadPhysics(SXRScene scene, SXRAndroidResource resource, Map<String, Object> loaderProperties)
    {
        String fname = resource.getResourceFilename().toLowerCase();
        SXRWorld world = (SXRWorld) scene.getRoot().getComponent(SXRWorld.getComponentType());

        if (world == null)
        {
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError",
                                                        world, fname,
                                                        "To load physics files, you must have a physics world attached to the scene");
            return null;
        }
        if (fname.endsWith(".urdf"))
        {
            loadPhysics(world, resource, loaderProperties);
            return world;
        }
        else if (fname.endsWith(".bullet"))
        {
            byte[] inputData = toByteArray(resource);

            if (inputData == null || inputData.length == 0)
            {
                getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                            "onLoadError",
                                                            world, fname,
                                                            "Cannot open physics file");
                return null;
            }
            loadBulletFile(inputData, scene.getRoot(), loaderProperties);
            return world;
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError",
                                                        world, fname,
                                                        "Unknown physics file type, must be .bullet or .urdf");
            return null;
        }
    }

    /**
     * Loads a physics content file.
     * <p>
     * Bullet (.bullet) physics files and Universal Robot
     * Description Format (.urdf) are supported.
     * This function can import Featherstone multi-body hierarchies.
     * It throws an exception if the input world does not support multi-body.
     * <p>
     * Physics files contain only physics components,
     * rigid bodies, joints, constraints and colliders.
     * There are no nodes or meshes. The imported physics
     * components are added to the nodes with the same
     * name in the scene provided.
     * <p>
     * When a multi-body hierarchy is imported, a {@link SXRSkeleton} is created
     * from the hierarchy and attached to the root joint. A corresponding
     * hierarchy of {@SXRNode} objects is also constructed and attached
     * to the owner of the root joint.
     * @param resource {@link SXRAndroidResource} referencing the file containing physics content.
     * @param world    The physics world to attach physics components.
     *                 This world should be attached to the root of the
     *                 node hierarchy to attach physics to.
     * @param loaderProperties Default values for physics properties:
     *                         ignoreupaxis:           assume Y axis is up if true, else set from physics content
     *                         attachbone:             name of skeleton bone to attach new physics to
     *                         CollisionGroup:         integer collision group for new bodies / joints
     *                         AngularLimits:          angular limits in radians for generic constraints
     *                         AngularSpringStiffness: angular spring stiffness for generic constraints
     *                         AngularSpringDamping:   angular spring damping for generic constraints
     */
    public void loadPhysics(SXRWorld world, SXRAndroidResource resource, Map<String, Object> loaderProperties)
    {
        SXRNode root = world.getOwnerObject();
        String fname = resource.getResourceFilename().toLowerCase();

        if (root == null)
        {
            root = new SXRNode(world.getSXRContext());
            root.setName(resource.getResourceFilename());
            root.attachComponent(world);
        }
        if (fname.endsWith(".urdf"))
        {
            String urdfXML = SXRPhysicsLoader.toString(resource);
            if (urdfXML == null)
            {
                getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                            "onLoadError",
                                                            world, fname,
                                                            "Cannot parse URDF file");
                return;
            }
            loadURDFFile(world, urdfXML, loaderProperties);
        }
        else if (fname.endsWith(".bullet"))
        {
            byte[] inputData = toByteArray(resource);

            if (inputData == null || inputData.length == 0)
            {
                getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                            "onLoadError",
                                                            world, fname,
                                                            "Cannot open physics file");
                return;
            }
            loadBulletFile(world, inputData, root, loaderProperties);
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError",
                                                        world, fname,
                                                        "Unknown physics file type, must be .bullet or .urdf");
        }
    }

    /**
     * Loads a physics content file.
     * <p>
     * Bullet (.bullet) physics files and Universal Robot
     * Description Format (.urdf) are supported.
     * This function can import Featherstone multi-body hierarchies.
     * It throws an exception if the input world does not support multi-body.
     * <p>
     * Physics files contain only physics components,
     * rigid bodies, joints, constraints and colliders.
     * There are no nodes or meshes. This function constructs a new
     * physics world and creates a new node for each rigid body imported.
     * <p>
     * When a multi-body hierarchy is imported, a {@link SXRSkeleton} is created
     * from the hierarchy and attached to the root joint. A corresponding
     * hierarchy of {@SXRNode} objects is also constructed and attached
     * to the owner of the root joint.
     * @param resource {@link SXRAndroidResource} referencing the file containing physics content.
     * @param loaderProperties Default values for physics properties:
     *                         ignoreupaxis:           assume Y axis is up if true, else set from physics content
     *                         attachbone:             name of skeleton bone to attach new physics to
     *                         CollisionGroup:         integer collision group for new bodies / joints
     *                         AngularLimits:          angular limits in radians for generic constraints
     *                         AngularSpringStiffness: angular spring stiffness for generic constraints
     *                         AngularSpringDamping:   angular spring damping for generic constraints
     */
    public SXRPhysicsContent loadPhysics(SXRAndroidResource resource, Map<String, Object> loaderProperties)
    {
        SXRNode root = new SXRNode(getSXRContext());
        SXRWorld world = new SXRWorld(root, mIsMultiBody);

        root.setName(resource.getResourceFilename());
        loadPhysics(world, resource, loaderProperties);
        return world;
    }

    /***
     * Export the given physics world in Bullet binary format.
     * @param world     physics world to export
     * @param fileName  name of file to get physics world
     * @return true if export successful, false if not
     */
    public boolean exportPhysics(SXRWorld world, String fileName)
    {
        return NativeBulletLoader.exportBullet(getNative(), world.getNative(), fileName);
    }

    private void loadBulletFile(byte[] inputData, SXRNode sceneRoot, Map<String, Object> loaderProperties)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        long loader = getNative();
        boolean result;
        SXRWorld world = (SXRWorld) sceneRoot.getComponent(SXRWorld.getComponentType());
        boolean ignoreUpAxis = false;

        if ((loaderProperties != null) && loaderProperties.containsKey("IgnoreUpAxis"))
        {
            ignoreUpAxis = (boolean) loaderProperties.get("IgnoreUpAxis");
        }
        if (mIsMultiBody)
        {
            if (!world.isMultiBody())
            {
                throw new UnsupportedOperationException("Cannot load multibody content into a world that is not multibody");
            }
            result = NativeBulletLoader.parseMB(loader, world.getNative(), inputData, inputData.length, ignoreUpAxis);
        }
        else
        {
            result = NativeBulletLoader.parse(loader, inputData, inputData.length, ignoreUpAxis);
        }
        if (!result)
        {
            NativeBulletLoader.clear(loader);
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError",
                                                        world, sceneRoot.getName(),
                                                        "Failed to parse bullet file");
            return;
        }
        /*
         * attach physics components to scene objects.
         */
        SXRSkeleton skel = attachPhysics(sceneRoot, loaderProperties);
        NativeBulletLoader.clear(loader);
        if (!mErrors.isEmpty())
        {
            String errors = mErrors;
            mErrors = "";
            getSXRContext().getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                        "onLoadError",
                                                        world, sceneRoot.getName(), errors);
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onPhysicsLoaded", world, skel, sceneRoot.getName());
        }
    }

    private void loadBulletFile(final SXRWorld world, byte[] inputData, final SXRNode sceneRoot, final Map<String, Object> loaderProperties)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        boolean result;
        boolean ignoreUpAxis = false;

        if ((loaderProperties != null) && loaderProperties.containsKey("IgnoreUpAxis"))
        {
            ignoreUpAxis = (boolean) loaderProperties.get("IgnoreUpAxis");
        }
        if (mIsMultiBody)
        {
            if (!world.isMultiBody())
            {
                throw new UnsupportedOperationException("Cannot load multibody content into a world that is not multibody");
            }
            result = NativeBulletLoader.parseMB(loader, world.getNative(), inputData, inputData.length, ignoreUpAxis);
        }
        else
        {
            result = NativeBulletLoader.parse(loader, inputData, inputData.length, ignoreUpAxis);
        }
        if (!result)
        {
            NativeBulletLoader.clear(loader);
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError",
                                                        world, sceneRoot.getName(),
                                                        "Failed to parse bullet file");
            return;
        }
        /*
         * attach physics components to scene objects.
         */
        world.run(new Runnable()
        {
            public void run()
            {
                SXRSkeleton skel = attachPhysics(sceneRoot, loaderProperties);
                NativeBulletLoader.clear(loader);
                if (!mErrors.isEmpty())
                {
                    String errors = mErrors;
                    mErrors = "";
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onLoadError",
                                                    world, sceneRoot.getName(), errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onPhysicsLoaded",
                                                    world, skel, sceneRoot.getName());
                }
            }
        });
    }

    void loadURDFFile(final SXRWorld world, String xmlData, final Map<String, Object> loaderProperties)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        final SXRNode sceneRoot = world.getOwnerObject();
        boolean ignoreUpAxis = false;

        if ((loaderProperties != null) && loaderProperties.containsKey("IgnoreUpAxis"))
        {
            ignoreUpAxis = (boolean) loaderProperties.get("IgnoreUpAxis");
        }

        boolean result = NativeBulletLoader.parseURDF(loader, world.getNative(), xmlData, ignoreUpAxis, mIsMultiBody);

        if (!result)
        {
            NativeBulletLoader.clear(loader);
            ctx.getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                             "onLoadError",
                                            world, sceneRoot.getName(),
                                             "Failed to parse URDF file");
            return;
        }
        /*
         * attach physics components to scene objects.
         */
        world.run(new Runnable()
        {
            public void run()
            {
                SXRSkeleton skel = attachPhysics(sceneRoot, loaderProperties);
                NativeBulletLoader.clear(loader);
                if (!mErrors.isEmpty())
                {
                    String errors = mErrors;
                    mErrors = "";
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onLoadError",
                                                    world, sceneRoot.getName(), errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onPhysicsLoaded",
                                                    world, skel, sceneRoot.getName());
                }
            }
        });
    }

    /**
     * Attach the SXR physics components to the corresponding scene
     * nodes based on name matching.
     * @param sceneRoot root of scene hierarchy to add physics to
     */
    private SXRSkeleton attachPhysics(SXRNode sceneRoot, Map<String, Object> loaderProperties)
    {
        SXRContext ctx = sceneRoot.getSXRContext();
        long loader = getNative();

        /*
         * Attach imported SXRRigidBody objects to the corresponding
         * scene nodes based on name matching. If a collider exists
         * for the body, attach it to the scene object too.
         */
        SXRRigidBody[] bodies = NativeBulletLoader.getRigidBodies(loader);
        for (SXRRigidBody body : bodies)
        {
            String name = body.getName();
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (sceneObject == null)
            {
                sceneObject = new SXRNode(getSXRContext());
                sceneObject.setName(name);
                sceneRoot.addChildObject(sceneObject);
                Log.w("PHYSICS LOADER", "Didn't find node for rigid body " + name);
            }
            else
            {
                SXRPhysicsJoint joint = (SXRPhysicsJoint) sceneObject.getComponent(SXRPhysicsJoint.getComponentType());
                if (joint != null)
                {
                    joint.removeJointAt(joint.getJointIndex());
                }
            }
            if (sceneObject.getComponent(SXRCollider.getComponentType()) == null)
            {
                SXRCollider collider = NativeBulletLoader.getCollider(loader, name);
                if (collider == null)
                {
                    collider = new SXRMeshCollider(ctx, true);
                }
                sceneObject.attachComponent(collider);
            }
            if (loaderProperties != null)
            {
                int collisionGroup = (int) loaderProperties.get("CollisionGroup");
                if (collisionGroup != 0)
                {
//                body.setCollisionGroup(collisionGroup);
                }
            }
            sceneObject.attachComponent(body);
        }

        /*
         * Attach imported SXRRPhysicsJoint objects to the corresponding
         * scene nodes based on name matching. If a collider exists
         * for the joint, attach it to the scene object too.
         */
        SXRPhysicsJoint[] jointsLoaded = NativeBulletLoader.getJoints(loader);
        SXRPhysicsJoint[] skelJoints = new SXRPhysicsJoint[jointsLoaded.length];
        SXRSkeleton skel = null;

        for (SXRPhysicsJoint joint : jointsLoaded)
        {
            String name = joint.getName();

            if (sceneRoot.getNodeByName(name) == null)
            {
                Log.w("PHYSICS LOADER","Didn't find node for joint " + name);
            }
            if (loaderProperties != null)
            {
                int collisionGroup = (int) loaderProperties.get("CollisionGroup");
                if (collisionGroup != 0)
                {
//                joint.setCollisionGroup(collisionGroup);
                }
            }
            skelJoints[joint.getJointIndex()] = joint;
        }

        if (skelJoints.length != 0)
        {
            SXRPhysicsJoint rootJoint = skelJoints[0];
            skel = (rootJoint != null) ? rootJoint.getSkeleton() : null;

            if (skel != null)
            {
                addJointsToSkeleton(skelJoints, skel, sceneRoot);
            }
        }
        /*
         * Attach imported SXRConstraint objects to the corresponding
         * scene nodes based on name matching. The Java constraints
         * are already attached to bodyA, The scene object and its
         * rigid body / joint is bodyB.
         */
        SXRConstraint[] constraints = NativeBulletLoader.getConstraints(loader);
        for (SXRConstraint constraint : constraints)
        {
            String name = NativeBulletLoader.getConstraintName(loader, constraint.getNative());
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (sceneObject == null)
            {
                mErrors += "Didn't find node for constraint '" + name + "'\n";
                continue;
            }
            if (loaderProperties != null)
            {
                float f = (float) loaderProperties.get("BreakingImpulse");

                if (f != 0)
                {
                    constraint.setBreakingImpulse(f);
                }
                if (constraint instanceof SXRGenericConstraint)
                {
                    SXRGenericConstraint gc = (SXRGenericConstraint) constraint;
                    Vector3f v = (Vector3f) loaderProperties.get("AngularLimits");
                    if (v != null)
                    {
                        gc.setAngularLowerLimits(-v.x, -v.y, -v.z);
                        gc.setAngularUpperLimits(v.x, v.y, v.z);
                    }
                    v = (Vector3f) loaderProperties.get("AngularSpringStiffness");
                    if (v != null)
                    {
                        gc.setAngularStiffness(v.x, v.y, v.z);
                    }
                    v = (Vector3f) loaderProperties.get("AngularSpringDamping");
                    if (v != null)
                    {
                        gc.setAngularDamping(v.x, v.y, v.z);
                    }
                }
            }
            sceneObject.attachComponent(constraint);
        }
        return skel;
    }

    protected SXRSkeleton addJointsToSkeleton(SXRPhysicsJoint[] skelJoints, SXRSkeleton skel, SXRNode root)
    {
        for (int i = 0; i < skel.getNumBones(); ++i)
        {
            String boneName = skel.getBoneName(i);
            SXRNode bone = root.getNodeByName(boneName);
            int parentIndex = skel.getParentBoneIndex(i);
            SXRCollider collider = null;
            SXRPhysicsJoint joint = skelJoints[i];

            if (bone == null)
            {
                bone = new SXRNode(getSXRContext());
                bone.setName(boneName);
                skel.setBone(i, bone);
                if (parentIndex >= 0)
                {
                    SXRNode parentBone = skel.getBone(parentIndex);

                    if (parentBone != null)
                    {
                        parentBone.addChildObject(bone);
                    }
                    else
                    {
                        skel.getBone(0).addChildObject(bone);
                    }
                }
                else
                {
                    root.addChildObject(bone);
                }
            }
            if (bone.getComponent(SXRCollider.getComponentType()) == null)
            {
                collider = NativeBulletLoader.getCollider(getNative(), boneName);
            }
            if (collider == null)
            {
                collider = new SXRMeshCollider(getSXRContext(), true);
            }
            bone.attachComponent(collider);
            bone.attachComponent(joint);
        }
        return skel;
    }

    protected static byte[] toByteArray(SXRAndroidResource resource)
    {
        try
        {
            resource.openStream();
            InputStream is = resource.getStream();
            ByteArrayOutputStream baos = new ByteArrayOutputStream();
            byte[] buffer = new byte[1024];
            for (int read; (read = is.read(buffer, 0, buffer.length)) != -1; )
            {
                baos.write(buffer, 0, read);
            }
            baos.flush();
            resource.closeStream();
            return baos.toByteArray();
        }
        catch (IOException ex)
        {
            return null;
        }
    }

    protected static String toString(SXRAndroidResource resource)
    {
        try
        {
            InputStream stream = resource.getStream();
            byte[] bytes = new byte[stream.available()];
            stream.read(bytes);
            stream.close();
            return new String(bytes);
        }
        catch (IOException ex)
        {
            return null;
        }
    }

    protected static SXRAndroidResource toAndroidResource(SXRContext context, String fileName)
    {
        try
        {
            SXRResourceVolume resVol = new SXRResourceVolume(context, fileName);

            final int i = fileName.lastIndexOf("/");
            if (i > 0)
            {
                fileName = fileName.substring(i + 1);
            }
            return resVol.openResource(fileName);
        }
        catch (IOException ex)
        {
            return null;
        }
    }
}

class NativeBulletLoader
{
    static native long ctor(SXRContext jcontext, AssetManager jassetManager);

    static native boolean parse(long loader, byte[] bytes, int len, boolean ignoreUpAxis);

    static native boolean parseMB(long loader, long world, byte[] bytes, int len, boolean ignoreUpAxis);

    static native boolean parseURDF(long loader, long world, String xmldata, boolean ignoreUpAxis, boolean multibody);

    static native boolean exportBullet(long loader, long world, String filename);

    static native void clear(long loader);

    static native SXRCollider getCollider(long loader, String name);

    static native SXRRigidBody[] getRigidBodies(long loader);

    static native SXRPhysicsJoint[] getJoints(long loader);

    static native SXRConstraint[] getConstraints(long loader);

    static native String getConstraintName(long loader, long nativeConstraint);
}
