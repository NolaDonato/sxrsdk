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

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.List;

public class SXRPhysicsLoader extends SXRHybridObject implements IEventReceiver
{
    static private final String TAG = SXRPhysicsLoader.class.getSimpleName();
    protected boolean mCreateNodes = false;
    protected boolean mIsMultiBody = false;
    protected String mErrors = null;
    private SXREventReceiver mListeners;

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
         * @param filename  Name of file or resource loaded.
         */
        public void onPhysicsLoaded(SXRPhysicsContent world, SXRSkeleton skel, String filename);

        /**
         * Called if a physics file fails to load.
         * @param filename  Name of file or resource loaded.
         * @param errors    Errors during loading, null if load was successful.
         */
        public void onLoadError(String filename, String errors);
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
        loadPhysics(scene, fileName, false);
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
     * @param ignoreUpAxis Assume Y axis is up if true, down if false.
     */
    public void loadPhysics(SXRScene scene, String fileName, boolean ignoreUpAxis)
    {
        SXRAndroidResource resource = toAndroidResource(scene.getSXRContext(), fileName);
        loadPhysics(scene, resource, ignoreUpAxis);
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
     * @param ignoreUpAxis Assume Y axis is up if true, down if false.
     */
    public SXRPhysicsContent loadPhysics(SXRScene scene, SXRAndroidResource resource, boolean ignoreUpAxis)
    {
        String fname = resource.getResourceFilename().toLowerCase();
        SXRWorld world = (SXRWorld) scene.getRoot().getComponent(SXRWorld.getComponentType());

        mCreateNodes = false;
        if (world == null)
        {
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError", fname,
                                                        "To load physics files, you must have a physics world attached to the scene");
            return null;
        }
        if (fname.endsWith(".urdf"))
        {
            return loadPhysics(resource, ignoreUpAxis);
        }
        else if (fname.endsWith(".bullet"))
        {
            byte[] inputData = toByteArray(resource);

            if (inputData == null || inputData.length == 0)
            {
                getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                            "onLoadError", fname,
                                                            "Cannot open physics file");
                return null;
            }
            loadBulletFile(inputData, scene.getRoot(), ignoreUpAxis);
            return world;
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError", fname,
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
     * @param ignoreUpAxis Assume Y axis is up if true, down if false.
     */
    public void loadPhysics(SXRWorld world, SXRAndroidResource resource, boolean ignoreUpAxis)
    {
        SXRNode root = world.getOwnerObject();
        String fname = resource.getResourceFilename().toLowerCase();

        if (root == null)
        {
            root = new SXRNode(world.getSXRContext());
            root.setName(resource.getResourceFilename());
            root.attachComponent(world);
        }
        if (root.getChildrenCount() == 0)
        {
            mCreateNodes = true;
        }
        if (fname.endsWith(".urdf"))
        {
            String urdfXML = SXRPhysicsLoader.toString(resource);
            if (urdfXML == null)
            {
                getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                            "onLoadError", fname,
                                                            "Cannot parse URDF file");
                return;
            }
            loadURDFFile(world, urdfXML, ignoreUpAxis);
        }
        else if (fname.endsWith(".bullet"))
        {
            byte[] inputData = toByteArray(resource);

            if (inputData == null || inputData.length == 0)
            {
                getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                            "onLoadError", fname,
                                                            "Cannot open physics file");
                return;
            }
            loadBulletFile(world, inputData, root, ignoreUpAxis);
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError", fname,
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
     * @param ignoreUpAxis Assume Y axis is up if true, down if false.
     */
    public SXRPhysicsContent loadPhysics(SXRAndroidResource resource, boolean ignoreUpAxis)
    {
        SXRNode root = new SXRNode(getSXRContext());
        SXRWorld world = new SXRWorld(root, mIsMultiBody);

        mCreateNodes = true;
        root.setName(resource.getResourceFilename());
        loadPhysics(world, resource, ignoreUpAxis);
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

    private void loadBulletFile(byte[] inputData, SXRNode sceneRoot, boolean ignoreUpAxis)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        long loader = getNative();
        boolean result = NativeBulletLoader.parse(loader, inputData, inputData.length, ignoreUpAxis);

        if (!result)
        {
            NativeBulletLoader.clear(loader);
            mCreateNodes = false;
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                                        "onLoadError", sceneRoot.getName(),
                                                        "Failed to parse bullet file");
            return;
        }
        /*
         * attach physics components to scene objects.
         */
        SXRSkeleton skel = attachPhysics(sceneRoot);
        NativeBulletLoader.clear(loader);
        mCreateNodes = false;
        if (mErrors != null)
        {
            String errors = mErrors;
            mErrors = null;
            getSXRContext().getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onLoadError", sceneRoot.getName(), errors);
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onPhysicsLoaded", null, skel, sceneRoot.getName());
        }
    }

    private void loadBulletFile(final SXRWorld world, byte[] inputData, final SXRNode sceneRoot, boolean ignoreUpAxis)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        boolean result;

        if (mIsMultiBody)
        {
            result = NativeBulletLoader.parseMB(loader, world.getNative(), inputData, inputData.length, ignoreUpAxis);
        }
        else
        {
            result = NativeBulletLoader.parse(loader, inputData, inputData.length, ignoreUpAxis);
        }
        if (!result)
        {
            NativeBulletLoader.clear(loader);
            mCreateNodes = false;
            getSXRContext().getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                            "onLoadError", sceneRoot.getName(),
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
                SXRSkeleton skel = attachPhysics(sceneRoot);
                NativeBulletLoader.clear(loader);
                mCreateNodes = false;
                if (mErrors != null)
                {
                    String errors = mErrors;
                    mErrors = null;
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onLoadError", sceneRoot.getName(), errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onPhysicsLoaded", world, skel, sceneRoot.getName());
                }
            }
        });
    }

    void loadURDFFile(final SXRWorld world, String xmlData, boolean ignoreUpAxis)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        final SXRNode sceneRoot = world.getOwnerObject();
        boolean result = NativeBulletLoader.parseURDF(loader, world.getNative(), xmlData, ignoreUpAxis, mIsMultiBody);

        if (!result)
        {
            NativeBulletLoader.clear(loader);
            mCreateNodes = false;
            ctx.getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                             "onLoadError", sceneRoot.getName(),
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
                SXRSkeleton skel = attachPhysics(sceneRoot);
                NativeBulletLoader.clear(loader);
                mCreateNodes = false;
                if (mErrors != null)
                {
                    String errors = mErrors;
                    mErrors = null;
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onLoadError", sceneRoot.getName(), errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onPhysicsLoaded", world, skel, sceneRoot.getName());
                }
            }
        });
    }

    /**
     * Attach the SXR physics components to the corresponding scene
     * nodes based on name matching.
     * @param sceneRoot root of scene hierarchy to add physics to
     */
    private SXRSkeleton attachPhysics(SXRNode sceneRoot)
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
                if (mCreateNodes)
                {
                    sceneObject = new SXRNode(getSXRContext());
                    sceneObject.setName(name);
                    sceneRoot.addChildObject(sceneObject);
                }
                else
                {
                    mErrors += "Didn't find node for rigid body '" + name + "'\n";
                    continue;
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
            sceneObject.attachComponent(body);
        }

        /*
         * Attach imported SXRRPhysicsJoint objects to the corresponding
         * scene nodes based on name matching. If a collider exists
         * for the joint, attach it to the scene object too.
         */
        SXRPhysicsJoint[] joints = NativeBulletLoader.getJoints(loader);
        SXRPhysicsJoint rootJoint = null;
        for (SXRPhysicsJoint joint : joints)
        {
            String name = joint.getName();
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (sceneObject == null)
            {
                if (mCreateNodes)
                {
                    sceneObject = new SXRNode(getSXRContext());
                    sceneObject.setName(name);
                    sceneRoot.addChildObject(sceneObject);
                }
                else
                {
                    mErrors += "Didn't find node for joint '" + name + "'\n";
                    continue;
                }
            }
            if (joint.getJointIndex() <= 0)
            {
                rootJoint = joint;
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
            sceneObject.attachComponent(joint);
        }

        SXRSkeleton skel = rootJoint.getSkeleton();

        if (mCreateNodes)
        {
            for (int i = 0; i < skel.getNumBones(); ++i)
            {
                String boneName = skel.getBoneName(i);
                SXRNode bone = sceneRoot.getNodeByName(boneName);
                int parentIndex = skel.getParentBoneIndex(i);

                if (bone != null)
                {
                    skel.setBone(i, bone);
                    if (parentIndex >= 0)
                    {
                        SXRNode parentBone = skel.getBone(parentIndex);

                        if (parentBone != null)
                        {
                            sceneRoot.removeChildObject(bone);
                            parentBone.addChildObject(bone);
                        }
                    }
                }
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
            sceneObject.attachComponent(constraint);
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
