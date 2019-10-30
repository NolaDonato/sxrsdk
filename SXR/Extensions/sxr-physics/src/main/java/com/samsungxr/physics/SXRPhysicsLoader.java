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
    private boolean mCreateNodes = false;
    private SXREventReceiver mListeners;

    public interface IPhysicsLoaderEvents extends IEvents
    {
        /**
         * Called after a physics file is loaded.
         * @param  world    {@link }SXRPhysicsContent} containing the physics objects loaded.
         *                  May be null if the load failed.
         * @param filename  Name of file or resource loaded.
         */
        public void onPhysicsLoaded(SXRPhysicsContent world, String filename);

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

    public SXREventReceiver getEventReceiver() { return mListeners; }


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
    public void loadPhysics(SXRScene scene, String fileName) throws IOException
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
    public void loadPhysics(SXRScene scene, String fileName, boolean ignoreUpAxis) throws IOException
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
    public SXRPhysicsContent loadPhysics(SXRScene scene, SXRAndroidResource resource, boolean ignoreUpAxis) throws IOException
    {
        String fname = resource.getResourceFilename().toLowerCase();
        SXRWorld world = (SXRWorld) scene.getRoot().getComponent(SXRWorld.getComponentType());

        mCreateNodes = false;
        if (world == null)
        {
            throw new IOException("To load physics files, you must have a physics world attached to the scene");
        }
        if (fname.endsWith(".urdf"))
        {
            return loadPhysics(resource, false);
        }
        else if (fname.endsWith(".bullet"))
        {
            byte[] inputData = toByteArray(resource);

            if (inputData == null || inputData.length == 0)
            {
                throw new IOException("Failed to load physics file " + resource.getResourceFilename());
            }
            loadBulletFile(inputData, scene.getRoot(), ignoreUpAxis);
            return world;
        }
        else
        {
            throw new IOException("Unknown physics file type, must be .bullet or .urdf");
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
    public void loadPhysics(SXRWorld world, SXRAndroidResource resource, boolean ignoreUpAxis) throws IOException
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
            mCreateNodes = mCreateNodes || !world.isMultiBody();
            loadURDFFile(world, urdfXML, ignoreUpAxis);
        }
        else if (fname.endsWith(".bullet"))
        {
            byte[] inputData = toByteArray(resource);

            if (inputData == null || inputData.length == 0)
            {
                throw new IOException("Failed to load physics file " + resource.getResourceFilename());
            }
            loadBulletFile(world, inputData, root, ignoreUpAxis);
        }
        else
        {
            throw new IOException("Unknown physics file type, must be .bullet or .urdf");
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
    public SXRPhysicsContent loadPhysics(SXRAndroidResource resource, boolean ignoreUpAxis) throws IOException
    {
        SXRNode root = new SXRNode(getSXRContext());
        SXRWorld world = new SXRWorld(root, true);

        mCreateNodes = true;
        root.setName(resource.getResourceFilename());
        loadPhysics(world, resource, ignoreUpAxis);
        return world;
    }

    /**
     * Loads a skeleton and physics components from a JSON file describing the avatar.
     * <p>
     * Avatar files describe physics for articulated bodies.
     * Each file has a skeleton and joints with collision geometries.
     * @param resource    {@link SXRAndroidResource} containing the physics components.
     * @param isMultibody If true, use {@link SXRPhysicsJoint} and Bullet multibody support,
     *                    otherwise use {@link SXRRigidBody} and discrete dynamics simulation.
     */
    public SXRPhysicsContent loadAvatarFile(SXRAndroidResource resource, boolean isMultibody) throws IOException
    {
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + resource.getResourceFilename());
        }
        PhysicsAVTLoader loader = new PhysicsAVTLoader(getSXRContext(), isMultibody);
        return loader.parse(inputData);
    }

    /**
     * Loads physics components from a JSON file describing the avatar and associates
     * them to the skeleton provided
     * <p>
     * Avatar files describe physics for articulated bodies.
     * Each file has a skeleton and joints with collision geometries.
     * @param skel        {@link SXRSkeleton} to use
     * @param attachBone  name of bone in skeleton to associate with the root joint in physics.
     * @param resource    {@link SXRAndroidResource} containing the physics components.
     * @param isMultibody If true, use {@link SXRPhysicsJoint} and Bullet multibody support,
     *                    otherwise use {@link SXRRigidBody} and discrete dynamics simulation.
     */
    public SXRPhysicsContent loadAvatarFile(SXRSkeleton skel, String attachBone, SXRAndroidResource resource, boolean isMultibody) throws IOException
    {
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + resource.getResourceFilename());
        }
        PhysicsAVTLoader loader = new PhysicsAVTLoader(skel, attachBone, isMultibody);
        return loader.parse(inputData);
    }

    /**
     * Loads a skeleton and physics components from a JSON file describing the avatar.
     * <p>
     * Avatar files describe physics for articulated bodies.
     * Each file has a skeleton and joints with collision geometries.
     * The contents of the AVT is not added to the current scene.
     * Instead it is imported and contained in a {@link SXRPhysicsContent}
     * object (like a physics world but it cannot simulate, just a container).
     * @param fileName    Physics settings file name.
     * @param isMultibody If true, use {@link SXRPhysicsJoint} and Bullet multibody support,
     *                    otherwise use {@link SXRRigidBody} and discrete dynamics simulation.
     */
    public SXRPhysicsContent loadAvatarFile(String fileName, boolean isMultibody) throws IOException
    {
        SXRContext ctx = getSXRContext();
        SXRAndroidResource resource = toAndroidResource(ctx, fileName);
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + fileName);
        }
        PhysicsAVTLoader loader = new PhysicsAVTLoader(ctx, isMultibody);
        return loader.parse(inputData);
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
     * @param fileName    Physics settings file name.
     * @param isMultibody If true, use {@link SXRPhysicsJoint} and Bullet multibody support,
     *                    otherwise use {@link SXRRigidBody} and discrete dynamics simulation.
     */
    public SXRPhysicsContent loadAvatarFile(SXRSkeleton skel, String fileName, String attachBone, boolean isMultibody) throws IOException
    {
        SXRAndroidResource resource = toAndroidResource(skel.getSXRContext(), fileName);
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + fileName);
        }
        PhysicsAVTLoader loader = new PhysicsAVTLoader(skel, attachBone, isMultibody);
        return loader.parse(inputData);
    }

    private void loadBulletFile(byte[] inputData, SXRNode sceneRoot, boolean ignoreUpAxis) throws IOException
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
            throw new IOException("Failed to parse bullet file");
        }
        /*
         * attach physics components to scene objects.
         */
        attachPhysics(sceneRoot);
        NativeBulletLoader.clear(loader);
    }

    private void loadBulletFile(final SXRWorld world, byte[] inputData, final SXRNode sceneRoot, boolean ignoreUpAxis) throws IOException
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        boolean result;

        if (world.isMultiBody())
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
            getSXRContext().getEventManager().sendEvent(world, IPhysicsLoaderEvents.class,
                                            "onLoadError", sceneRoot.getName(),
                                                        "Failed to parse bullet file");
            throw new IOException("Failed to parse bullet file");
        }
        /*
         * attach physics components to scene objects.
         */
        world.run(new Runnable()
        {
            public void run()
            {
                String errors = attachPhysics(sceneRoot);
                NativeBulletLoader.clear(loader);
                mCreateNodes = false;
                if (errors != null)
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onLoadError", sceneRoot.getName(), errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onPhysicsLoaded", world, sceneRoot.getName());
                }
            }
        });
    }

    void loadURDFFile(final SXRWorld world, String xmlData, boolean ignoreUpAxis) throws IOException
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        final SXRNode sceneRoot = world.getOwnerObject();
        boolean result = NativeBulletLoader.parseURDF(loader, world.getNative(), xmlData, ignoreUpAxis);

        if (!result)
        {
            NativeBulletLoader.clear(loader);
            mCreateNodes = false;
            ctx.getEventManager().sendEvent(world, IPhysicsLoaderEvents.class,
                                             "onLoadError", sceneRoot.getName(),
                                             "Failed to parse URDF file");
            throw new IOException("Failed to parse URDF file");
        }
        /*
         * attach physics components to scene objects.
         */
        world.run(new Runnable()
        {
            public void run()
            {
                String errors = attachPhysics(sceneRoot);
                NativeBulletLoader.clear(loader);
                mCreateNodes = false;
                if (errors != null)
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onLoadError", sceneRoot.getName(), errors);
                 }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class, "onPhysicsLoaded", world, sceneRoot.getName());
                }
            }
        });
    }

    /**
     * Attach the SXR physics components to the corresponding scene
     * nodes based on name matching.
     * @param sceneRoot root of scene hierarchy to add physics to
     */
    private String attachPhysics(SXRNode sceneRoot)
    {
        SXRContext ctx = sceneRoot.getSXRContext();
        long loader = getNative();
        String errors = "";
        List<SXRPhysicsJoint> rootJoints = new ArrayList<SXRPhysicsJoint>();

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
                    errors += "Didn't find node for rigid body '" + name + "'\n";
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
                    errors += "Didn't find node for joint '" + name + "'\n";
                    continue;
                }
            }
            if (joint.getJointIndex() <= 0)
            {
                rootJoints.add(joint);
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

        for (SXRPhysicsJoint joint : rootJoints)
        {
            SXRSkeleton skel = joint.getSkeleton();

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
                errors += "Didn't find node for constraint '" + name + "'\n";
                continue;
            }
            sceneObject.attachComponent(constraint);
        }
        return errors.isEmpty() ? null : errors;
    }

    private static byte[] toByteArray(SXRAndroidResource resource) throws IOException
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
        return  baos.toByteArray();
    }

    private static String toString(SXRAndroidResource resource) throws IOException
    {
        InputStream stream = resource.getStream();
        byte[] bytes = new byte[stream.available()];
        stream.read(bytes);
        stream.close();
        return new String(bytes);
    }

    private static SXRAndroidResource toAndroidResource(SXRContext context, String fileName) throws IOException {
        SXRResourceVolume resVol = new SXRResourceVolume(context, fileName);

        final int i = fileName.lastIndexOf("/");
        if (i > 0) {
            fileName = fileName.substring(i + 1);
        }

        return resVol.openResource(fileName);
    }
}

class NativeBulletLoader
{
    static native long ctor(SXRContext jcontext, AssetManager assetManager);

    static native boolean parse(long loader, byte[] bytes, int len, boolean ignoreUpAxis);

    static native boolean parseMB(long loader, long world, byte[] bytes, int len, boolean ignoreUpAxis);

    static native boolean parseURDF(long loader, long world, String xmldata, boolean ignoreUpAxis);

    static native void clear(long loader);

    static native SXRCollider getCollider(long loader, String name);

    static native SXRRigidBody[] getRigidBodies(long loader);

    static native SXRPhysicsJoint[] getJoints(long loader);

    static native SXRConstraint[] getConstraints(long loader);

    static native String getConstraintName(long loader, long nativeConstraint);
}
