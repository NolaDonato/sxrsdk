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

import android.util.ArrayMap;
import android.util.Log;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRCollider;
import com.samsungxr.SXRComponentGroup;
import com.samsungxr.SXRContext;
import com.samsungxr.SXRHybridObject;
import com.samsungxr.SXRMeshCollider;
import com.samsungxr.SXRResourceVolume;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.animation.SXRSkeleton;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;

public class SXRPhysicsLoader extends SXRHybridObject
{
    static private final String TAG = SXRPhysicsLoader.class.getSimpleName();

    static {
        System.loadLibrary("sxr-physics");
    }

    public SXRPhysicsLoader(SXRContext ctx)
    {
        super(ctx, NativeBulletLoader.ctor(ctx));
    }

    /**
     * Loads a Bullet physics content file.
     * <p>
     * The Bullet binary files only contain physics, there
     * are no nodes or meshes. The rigid bodies and constraints
     * from the Bullet file are added to the nodes in the
     * given scene.
     *
     * @param fileName Name of file containing physics content.
     * @param scene    The scene containing the objects to attach physics components.
     */
    public void loadBulletFile(SXRScene scene, String fileName) throws IOException
    {
        SXRAndroidResource resource = toAndroidResource(scene.getSXRContext(), fileName);
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + fileName);
        }
        loadBulletFile(inputData, scene.getRoot(), false);
    }

    /**
     * Loads a Bullet physics content file.
     * <p>
     * The Bullet binary files only contain physics, there
     * are no nodes or meshes. The rigid bodies and constraints
     * from the Bullet file are added to the nodes in the
     * given scene.
     *
     * @param fileName Name of file containing physics content.
     * @param scene    The scene containing the objects to attach physics components.
     */
    public void loadBulletFile(SXRScene scene, String fileName, boolean ignoreUpAxis) throws IOException
    {
        SXRAndroidResource resource = toAndroidResource(scene.getSXRContext(), fileName);
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + fileName);
        }
        loadBulletFile(inputData, scene.getRoot(), ignoreUpAxis);
    }

    /**
     * Loads a Bullet physics content file.
     * <p>
     * The Bullet binary files only contain physics, there
     * are no nodes or meshes. The rigid bodies and constraints
     * from the Bullet file are added to the nodes in the given scene.
     * >p?
     * This function will not import Featherstone multi-body hierarchies.
     * Use {@link #loadBulletFile(SXRWorld, SXRAndroidResource)} if your
     * file contains these Bullet objects.
     *
     * @param resource {@link SXRAndroidResource} containing the physics content..
     * @param scene    The scene containing the objects to attach physics components.
     */
    public void loadBulletFile(SXRScene scene, SXRAndroidResource resource) throws IOException
    {
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + resource.getResourceFilename());
        }
        loadBulletFile(inputData, scene.getRoot(), false);
    }

    /**
     * Loads a Bullet physics content file.
     * <p>
     * The Bullet binary files only contain physics, there
     * are no nodes or meshes. The rigid bodies and constraints
     * from the Bullet file are added to the nodes in the given scene.
     * <p>
     * This function can import Featherstone multi-body hierarchies.
     * It throws an exception if the input world does not support multi-body.
     *
     * @param resource {@link SXRAndroidResource} containing the physics content..
     * @param world    The physics world to attach physics components.
     *                 This world should be attached to the root of the
     *                 node hierarchy to attach physics to.
     */
    public void loadBulletFile(SXRWorld world, SXRAndroidResource resource) throws IOException
    {
        SXRNode root = world.getOwnerObject();

        if (!world.isMultiBody())
        {
            throw new IllegalArgumentException("The physics world must support multi-body dynamics to use this function");
        }
        if (root == null)
        {
            root = new SXRNode(world.getSXRContext());
            root.setName(resource.getResourceFilename());
            root.attachComponent(world);
        }
        byte[] inputData = toByteArray(resource);

        if (inputData == null || inputData.length == 0)
        {
            throw new IOException("Failed to load physics file " + resource.getResourceFilename());
        }
        loadBulletFile(world, inputData, root, false);
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

    private void loadBulletFile(SXRWorld world, byte[] inputData, SXRNode sceneRoot, boolean ignoreUpAxis) throws IOException
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        long loader = getNative();
        boolean result = NativeBulletLoader.parseMB(loader, world.getNative(), inputData, inputData.length, ignoreUpAxis);

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

    /**
     * Attach the SXR physics components to the corresponding scene
     * nodes based on name matching.
     * @param sceneRoot root of scene hierarchy to add physics to
     */
    private void attachPhysics(SXRNode sceneRoot)
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
            String name = NativeBulletLoader.getRigidBodyName(loader, body.getNative());
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (sceneObject == null)
            {
                Log.w(TAG, "Didn't find node for rigid body '" + name + "'");
                continue;
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
            String name = NativeBulletLoader.getJointName(loader, joint.getNative());
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (sceneObject == null)
            {
                Log.w(TAG, "Didn't find node for joint '" + name + "'");
                continue;
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
                Log.w(TAG, "Didn't find node for constraint '" + name + "'");
                continue;
            }
            sceneObject.attachComponent(constraint);
        }
    }

    private static byte[] toByteArray(SXRAndroidResource resource) throws IOException {
        resource.openStream();
        InputStream is = resource.getStream();
        ByteArrayOutputStream baos = new ByteArrayOutputStream();
        byte[] buffer = new byte[1024];
        for (int read; (read = is.read(buffer, 0, buffer.length)) != -1; ) {
            baos.write(buffer, 0, read);
        }
        baos.flush();
        resource.closeStream();
        return  baos.toByteArray();
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
    static native long ctor(SXRContext jcontext);

    static native boolean parse(long loader, byte[] bytes, int len, boolean ignoreUpAxis);

    static native boolean parseMB(long loader, long world, byte[] bytes, int len, boolean ignoreUpAxis);

    static native void clear(long loader);

    static native SXRCollider getCollider(long loader, String name);

    static native SXRRigidBody[] getRigidBodies(long loader);

    static native SXRPhysicsJoint[] getJoints(long loader);

    static native SXRConstraint[] getConstraints(long loader);

    static native String getRigidBodyName(long loader, long nativeBody);

    static native String getJointName(long loader, long nativeJoint);

    static native String getConstraintName(long loader, long nativeConstraint);
}
