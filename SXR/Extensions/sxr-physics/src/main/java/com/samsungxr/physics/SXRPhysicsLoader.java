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
import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRCapsuleCollider;
import com.samsungxr.SXRCollider;
import com.samsungxr.SXRContext;
import com.samsungxr.SXREventReceiver;
import com.samsungxr.SXRHybridObject;
import com.samsungxr.SXRMeshCollider;
import com.samsungxr.SXRResourceVolume;
import com.samsungxr.SXRScene;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.SXRTransform;
import com.samsungxr.animation.SXRSkeleton;
import com.samsungxr.utility.FileNameUtils;
import com.samsungxr.utility.Log;

import org.joml.Vector3f;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
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
        String fname = resource.getResourcePath();
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
            loadBulletFile(inputData, scene.getRoot(), loaderProperties, fname);
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
        String fname = resource.getResourcePath();

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
            loadURDFFile(world, urdfXML, loaderProperties, fname);
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
            loadBulletFile(world, inputData, root, loaderProperties, fname);
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
    public boolean exportAsBullet(SXRWorld world, String fileName)
    {
        return NativeBulletLoader.exportBullet(getNative(), world.getNative(), fileName);
    }

    /***
     * Export the given physics world as URDF (ASCII XML format).
     * <p>
     * URDF requires a connected hierarchy of joints. SXR represents this
     * as a {@SXRSkeleton skeleton} with bone nodes which have {@link SXRRigidBody rigid bodies}
     * or {@link SXRPhysicsJoint joints} attached. The skeleton describes the hierarchy
     * and the physics information is contained in the physics components.
     * @param skeleton     {@link SXRSkeleton skeleton} with {@link SXRRigidBody rigid bodies}
     *                      or {@link SXRPhysicsJoint joints} attached to the bone nodes.
     * @param fileName      name of file to get physics world
     * @return true if export successful, false if not
     */
    public boolean exportPhysics(SXRSkeleton skeleton, String fileName)
    {
        URDFExporter exporter = new URDFExporter();
        return exporter.exportAsURDF(skeleton, fileName);
    }

    private void loadBulletFile(byte[] inputData, SXRNode sceneRoot, Map<String, Object> loaderProperties, final String fname)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        long loader = getNative();
        boolean result;
        SXRWorld world = (SXRWorld) sceneRoot.getComponent(SXRWorld.getComponentType());
        boolean ignoreUpAxis = false;
        SXRSkeleton destSkel = null;

        if (loaderProperties != null)
        {
            if (loaderProperties.containsKey("IgnoreUpAxis"))
            {
                ignoreUpAxis = (boolean) loaderProperties.get("IgnoreUpAxis");
            }
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
                                                        world, fname,
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
                                                        world, fname, errors);
        }
        else
        {
            getSXRContext().getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                        "onPhysicsLoaded", world,
                                                         skel, fname);
        }
    }

    private void loadBulletFile(final SXRWorld world, byte[] inputData, final SXRNode sceneRoot, final Map<String, Object> loaderProperties, final String fname)
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
                                                        world, fname,
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
                                                    world, fname, errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onPhysicsLoaded",
                                                    world, skel, fname);
                }
            }
        });
    }

    void loadURDFFile(final SXRWorld world, String xmlData, final Map<String, Object> loaderProperties, final String fname)
    {
        /*
         * Import the Bullet binary file and construct the Java physics objects.
         * They are attached to each other but not to nodes in the scene.
         */
        final long loader = getNative();
        final SXRContext ctx = getSXRContext();
        final SXRNode sceneRoot = world.getOwnerObject();
        boolean ignoreUpAxis = false;
        final Map<String, Object> lprops;

        if (loaderProperties != null)
        {
            lprops = loaderProperties;
            if (lprops.containsKey("IgnoreUpAxis"))
            {
                ignoreUpAxis = (boolean) lprops.get("IgnoreUpAxis");
            }
            lprops.put("LinkRigidBodies", true);
        }
        else
        {
            lprops = new HashMap<String, Object>();
            lprops.put("LinkRigidBodies", true);
        }

        boolean result = NativeBulletLoader.parseURDF(loader, world.getNative(), xmlData, ignoreUpAxis, mIsMultiBody);

        if (!result)
        {
            NativeBulletLoader.clear(loader);
            ctx.getEventManager().sendEvent(this, IPhysicsLoaderEvents.class,
                                             "onLoadError",
                                            world, fname,
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
                SXRSkeleton skel = attachPhysics(sceneRoot, lprops);
                NativeBulletLoader.clear(loader);
                if (!mErrors.isEmpty())
                {
                    String errors = mErrors;
                    mErrors = "";
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onLoadError",
                                                    world, fname, errors);
                }
                else
                {
                    ctx.getEventManager().sendEvent(SXRPhysicsLoader.this, IPhysicsLoaderEvents.class,
                                                    "onPhysicsLoaded",
                                                    world, skel, fname);
                }
            }
        });
    }

    /**
     * Attach the SXR physics components to the given hierarchy.
     * If {@link SXRPhysicsJoint joints} are encountered,
     * they are used to construct a {@link SXRSkeleton skeleton}
     * describing the joint hierarchy. If only rigid bodies are encountered,
     * the loader attempts to attach these to scene nodes based on
     * name matching. If the loader properties have "LinkRigidBodies"
     * set to true, a skeleton is constructed from the rigid bodies
     * and their constraints, if possible.
     * <p>
     * The physics skeleton may be merged with another existing skeleton.
     * The destination skeleton is designated in the loader properties
     * as the value of the "Skeleton" key. The name of the bone to start
     * merging at is the "AttachBone" loader property. If both are provided,
     * In this case thes skeletons are merged. Rigid bodies and joints are removed
     * from the source skeleton and attached to the destination skeleton nodes.
     *
     * @param sceneRoot        root of scene hierarchy to add physics to
     * @param loaderProperties map containing loader properties and their values.
     * @returns {@link SXRSkeleton} constructed from physics components
     */
    protected SXRSkeleton attachPhysics(SXRNode sceneRoot, Map<String, Object> loaderProperties)
    {
        long loader = getNative();
        boolean linkRigidBodies = false;
        SXRRigidBody[] bodies = NativeBulletLoader.getRigidBodies(loader);
        SXRPhysicsJoint[] joints = NativeBulletLoader.getJoints(loader);
        SXRConstraint[] constraints = NativeBulletLoader.getConstraints(loader);
        SXRSkeleton srcSkel = null;
        SXRSkeleton destSkel = null;
        int attachBoneIndex = -1;
        int simtype = SXRRigidBody.DYNAMIC;
        SXRNode physicsRoot = sceneRoot;

        if (loaderProperties != null)
        {
            if (loaderProperties.containsKey("LinkRigidBodies"))
            {
                linkRigidBodies = (boolean) loaderProperties.get("LinkRigidBodies");
            }
            if (loaderProperties.containsKey("Skeleton"))
            {
                destSkel = (SXRSkeleton) loaderProperties.get("Skeleton");
                if ((destSkel != null) && loaderProperties.containsKey("AttachBone"))
                {
                    String boneName = (String) loaderProperties.get("AttachBone");
                    if (boneName != null)
                    {
                        attachBoneIndex = destSkel.getBoneIndex(boneName);
                    }
                }
            }
            if (loaderProperties.containsKey("SimulationType"))
            {
                simtype = (int) loaderProperties.get("SimulationType");
            }
        }
        if (linkRigidBodies ||(joints.length > 0))
        {
            physicsRoot = new SXRNode(getSXRContext());
            sceneRoot.addChildObject(physicsRoot);
        }
        if (joints.length > 0)
        {
            srcSkel = attachJoints(physicsRoot, joints);
        }
        if (bodies.length > 0)
        {
            if (linkRigidBodies)
            {
                physicsRoot = new SXRNode(getSXRContext());
                sceneRoot.addChildObject(physicsRoot);
                attachRigidBodies(physicsRoot, bodies, loaderProperties);
                attachConstraints(physicsRoot, constraints, loaderProperties);
                if (srcSkel == null)
                {
                    srcSkel = linkRigidBodies(bodies, sceneRoot, simtype);
                }
                if (destSkel != null)
                {
                    mergeBodies(destSkel, srcSkel, attachBoneIndex);
                    return destSkel;
                }
            }
            else
            {
                attachRigidBodies(physicsRoot, bodies, loaderProperties);
                attachConstraints(physicsRoot, constraints, loaderProperties);
            }
            return srcSkel;
        }
        else if ((srcSkel != null) && (destSkel != null))
        {
            if (mergeJoints(destSkel, srcSkel, attachBoneIndex))
            {
                return destSkel;
            }
            return null;
        }
        return srcSkel;
    }

    /*
     * Attach imported SXRRigidBody objects to the corresponding
     * scene nodes based on name matching. If a collider exists
     * for the body, attach it to the scene object too.
     */
    protected void attachRigidBodies(SXRNode sceneRoot, SXRRigidBody[] bodies, Map<String, Object> loaderProperties)
    {
        int simtype = SXRRigidBody.DYNAMIC;
        boolean linkRigidBodies = false;

        if (loaderProperties != null)
        {
            if (loaderProperties.containsKey("SimulationType"))
            {
                simtype = (int) loaderProperties.get("SimulationType");
            }
            if (loaderProperties.containsKey("LinkRigidBodies"))
            {
                linkRigidBodies = (boolean) loaderProperties.get("LinkRigidBodies");
            }
        }
        for (SXRRigidBody body : bodies)
        {
            String name = body.getName();
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (simtype != SXRRigidBody.DYNAMIC)
            {
                body.setSimulationType(simtype);
            }
            if (linkRigidBodies)
            {
                sceneObject = new SXRNode(getSXRContext());
                sceneObject.setName(name);
                sceneRoot.addChildObject(sceneObject);
                Log.w("PHYSICS LOADER", "Creating node for rigid body " + name);
            }
            else if (sceneObject == null)
            {
                Log.w("PHYSICS LOADER", "Didn't find node for joint " + name);
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
                SXRCollider collider = NativeBulletLoader.getCollider(getNative(), name);
                if (collider == null)
                {
                    collider = new SXRMeshCollider(getSXRContext(), true);
                }
                collider.setPickDistance(-1);
                sceneObject.attachComponent(collider);
            }
            sceneObject.attachComponent(body);
        }
    }

    /*
     * Attach imported SXRConstraint objects to the corresponding
     * scene nodes based on name matching. The Java constraints
     * are already attached to bodyA, The scene object and its
     * rigid body / joint is bodyB.
     */
    protected void attachConstraints(SXRNode sceneRoot, SXRConstraint[] constraints, Map<String, Object> loaderProperties)
    {
        Vector3f angularLimits = null;
        Vector3f springStiffness = null;
        Vector3f springDamping = null;
        float breakingImpulse = 0;

        if (loaderProperties != null)
        {
            if (loaderProperties.containsKey("AngularLimits"))
            {
                angularLimits = (Vector3f) loaderProperties.get("AngularLimits");
            }
            if (loaderProperties.containsKey("AngularSpringStiffness"))
            {
                springStiffness = (Vector3f) loaderProperties.get("AngularSpringStiffness");
            }
            if (loaderProperties.containsKey("AngularSpringDamping"))
            {
                springDamping = (Vector3f) loaderProperties.get("AngularSpringDamping");
            }
        }
        for (SXRConstraint constraint : constraints)
        {
            String name = NativeBulletLoader.getConstraintName(getNative(), constraint.getNative());
            SXRNode sceneObject = sceneRoot.getNodeByName(name);

            if (sceneObject == null)
            {
                mErrors += "Didn't find node for constraint '" + name + "'\n";
                continue;
            }
            if (breakingImpulse != 0)
            {
                constraint.setBreakingImpulse(breakingImpulse);
            }
            if ((loaderProperties != null) && (constraint instanceof SXRGenericConstraint))
            {
                SXRGenericConstraint gc = (SXRGenericConstraint) constraint;

                if (angularLimits != null)
                {
                    gc.setAngularLowerLimits(-angularLimits.x, -angularLimits.y, -angularLimits.z);
                    gc.setAngularUpperLimits(angularLimits.x, angularLimits.y, angularLimits.z);
                }
                if (springStiffness != null)
                {
                    gc.setAngularStiffness(springStiffness.x, springStiffness.y, springStiffness.z);
                }
                if (springDamping != null)
                {
                    gc.setAngularDamping(springDamping.x, springDamping.y, springDamping.z);
                }
            }
            sceneObject.attachComponent(constraint);
        }
    }

    /*
     * Attach imported SXRRPhysicsJoint objects to the corresponding
     * scene nodes based on name matching. If a collider exists
     * for the joint, attach it to the scene object too.
     */
    protected SXRSkeleton attachJoints(SXRNode sceneRoot, SXRPhysicsJoint[] joints)
    {
        SXRPhysicsJoint[] skelJoints = new SXRPhysicsJoint[joints.length];
        SXRSkeleton skel = null;

        for (SXRPhysicsJoint joint : joints)
        {
            String name = joint.getName();

            if (sceneRoot.getNodeByName(name) == null)
            {
                Log.w("PHYSICS LOADER", "Didn't find node for joint " + name);
            }
            skelJoints[joint.getJointIndex()] = joint;
        }
        if (skelJoints.length == 0)
        {
            return null;
        }
        SXRPhysicsJoint rootJoint = skelJoints[0];

        if (rootJoint == null)
        {
            return null;
        }
        skel = rootJoint.getSkeleton();
        if (skel == null)
        {
            return null;
        }
        for (int i = 0; i < skel.getNumBones(); ++i)
        {
            String boneName = skel.getBoneName(i);
            SXRNode bone = sceneRoot.getNodeByName(boneName);
            int parentIndex = skel.getParentBoneIndex(i);
            SXRCollider collider = null;
            SXRPhysicsCollidable joint = skelJoints[i];

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
                    sceneRoot.addChildObject(bone);
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
            collider.setPickDistance(-1);
            bone.attachComponent(collider);
            bone.attachComponent(joint);
        }
        return skel;
    }

    protected SXRSkeleton linkRigidBodies(SXRRigidBody[] bodies, SXRNode sceneRoot, int boneoptions)
    {
        SXRNode rootNode = null;

        for (SXRRigidBody body : bodies)
        {
            SXRNode owner = body.getOwnerObject();
            SXRConstraint c = (SXRConstraint) body.getOwnerObject().getComponent(SXRConstraint.getComponentType());

            if (c == null)
            {
                continue;
            }
            SXRPhysicsCollidable parentBody = c.getBodyA();
            SXRNode parentOwner = parentBody.getOwnerObject();

            c = (SXRConstraint) parentOwner.getComponent(SXRConstraint.getComponentType());
            if (owner.getParent() == sceneRoot)
            {
                sceneRoot.removeChildObject(owner);
                parentOwner.addChildObject(owner);
            }
            if (c == null)
            {
                rootNode = parentOwner;
            }
        }
        final List<SXRNode> nodes = new ArrayList<SXRNode>();
        final List<String> boneNames = new ArrayList<String>();

        rootNode.forAllDescendants(new SXRNode.SceneVisitor()
        {
            @Override
            public boolean visit(SXRNode obj)
            {
                nodes.add(obj);
                boneNames.add(obj.getName());
                return true;
            }
        });

        int numbones = nodes.size();
        SXRSkeleton skel = new SXRSkeleton(rootNode, boneNames);

        skel.setBone(0, rootNode);
        for (int i = 1; i < numbones; ++i)
        {
            skel.setBoneOptions(i, boneoptions);
            skel.setBone(i, nodes.get(i));
        }
        return skel;
    }

    /**
     * Merges the rigid bodies from one skeleton into another.
     * <p>
     * Bones in the destination skeleton above the attachment point
     * are unchanged. Bones which are common between the two skeletons
     * will get the rigid bodies and constraints from the source skeleton.
     * Components of the same type in the destination skeleton
     * will be removed first. Source constraints are updated
     * to reflect new rigid body parents.
     * @param destSkel   destination skeleton
     * @param srcSkel    source skeleton
     * @param attachBone bone index in destination skeleton where
     *                   source skeleton should be merge
     */
    protected void mergeBodies(SXRSkeleton destSkel, SXRSkeleton srcSkel, int attachBone)
    {
        int numSrcBones = srcSkel.getNumBones();

        for (int i = 0; i < numSrcBones; ++i)
        {
            String srcName = srcSkel.getBoneName(i);
            SXRNode srcBone = srcSkel.getBone(i);
            SXRRigidBody srcBody = (SXRRigidBody) srcBone.getComponent(SXRRigidBody.getComponentType());
            SXRConstraint srcConstraint = (SXRConstraint) srcBone.getComponent(SXRConstraint.getComponentType());
            int destIndex = destSkel.getBoneIndex(srcName);
            SXRNode destParent = null;
            SXRRigidBody destBody = null;
            SXRRigidBody parentBody = null;

            /*
             * Common bone between the two skeletons below the attach point.
             * Copy source collidable into the corresponding destination
             * component if it exists. Otherwise attach the source collidable
             * to the destination bone.
             */
            if (destIndex >= 0)
            {
                if (destIndex < attachBone)
                {
                    continue;
                }
                SXRNode destBone = destSkel.getBone(destIndex);
                if (destBone == null)
                {
                    Log.w("PHYSICS LOADER", "Skipping bone node " + srcName + " missing in hierarchy");
                    continue;
                }
                destParent = destBone.getParent();
                destBody = (SXRRigidBody) destBone.getComponent(SXRRigidBody.getComponentType());
                parentBody = (SXRRigidBody) destParent.getComponent(SXRRigidBody.getComponentType());

                SXRPhysicsJoint joint = (SXRPhysicsJoint) destBone.getComponent(SXRPhysicsJoint.getComponentType());
                if ((joint != null) && joint.getOwnerObject() != null)
                {
                    joint.removeJointAt(joint.getJointIndex());
                }
                srcBone.detachComponent(SXRRigidBody.getComponentType());
                srcBone.detachComponent(SXRPhysicsJoint.getComponentType());
                if (destBody != null)
                {
                    destBody.copy(srcBody);
                }
                else
                {
                    destBone.attachComponent(srcBody);
                }
                /*
                 * Update the source constraint parent body to be the
                 * rigid body in the destination. Attach the constraint
                 * to the destination skeleton bone.
                 */
                if ((srcConstraint != null) && (srcConstraint.getBodyA() != destBody))
                {
                    destBone.detachComponent(SXRConstraint.getComponentType());
                    srcBone.detachComponent(SXRConstraint.getComponentType());
                    if (parentBody != null)
                    {
                        srcConstraint.setParentBody(parentBody);
                    }
                }
            }
            /**
             * This is not a common bone. It is present in the source
             * skeleton but not the destination. Try to obtain
             * the destination parent bone from the source parent
             * bone name. If the parent is found, update the
             * source constraint to refer to the collidable
             * in the destination.
             */
            else
            {
                int srcParentIndex = srcSkel.getParentBoneIndex(i);

                if (srcParentIndex < 0)
                {
                    continue;
                }
                String parentBoneName = srcSkel.getBoneName(srcParentIndex);
                int destParentIndex = destSkel.getBoneIndex(parentBoneName);

                if (destParentIndex < 0)
                {
                    continue;
                }
                destParent = destSkel.getBone(destParentIndex);
                if (destParent != null)
                {
                    parentBody = (SXRRigidBody) destParent.getComponent(SXRRigidBody.getComponentType());
                    if (srcConstraint == null)
                    {
                        if (parentBody != null)
                        {
                            SXRConstraint constraint = new SXRFixedConstraint(getSXRContext(), parentBody);
                            srcBone.attachComponent(constraint);
                        }
                    }
                    else if (srcConstraint.getBodyA() != destBody)
                    {
                        srcConstraint.setParentBody(parentBody);
                    }
                }
            }
        }
        destSkel.merge(srcSkel, destSkel.getBoneName(attachBone));
    }

    /**
     * Merges the joints from one skeleton into another.
     * <p>
     * Bones in the destination skeleton above the attachment point
     * are unchanged. Bones which are common between the two skeletons
     * will get the joints from the source skeleton.
     * Components of the same type in the destination skeleton
     * will be removed first.
     * @param destSkel   destination skeleton
     * @param srcSkel    source skeleton
     * @param attachBone bone index in destination skeleton where
     *                   source skeleton should be merge
     */
    protected boolean mergeJoints(SXRSkeleton destSkel, SXRSkeleton srcSkel, int attachBone)
    {
        int numSrcBones = srcSkel.getNumBones();
        SXRPhysicsJoint destRootJoint = (SXRPhysicsJoint) destSkel.getBone(0).
                                            getComponent(SXRPhysicsJoint.getComponentType());

        if (destRootJoint == null)
        {
            Log.w("PHYSICS LOADER","Source skeleton has no root joint, merge failed");
            return false;
        }
        for (int i = 0; i < numSrcBones; ++i)
        {
            String srcName = srcSkel.getBoneName(i);
            SXRNode srcBone = srcSkel.getBone(i);
            SXRPhysicsJoint srcJoint = (SXRPhysicsJoint) srcBone.getComponent(SXRPhysicsJoint.getComponentType());
            int destIndex = destSkel.getBoneIndex(srcName);
            SXRNode destParent = null;
            SXRNode srcParent = null;
            SXRPhysicsJoint destJoint = null;
            SXRPhysicsJoint parentJoint = null;

            /*
             * Common bone between the two skeletons below the attach point.
             * Copy source joint into the corresponding destination
             * component if it exists. Otherwise remove the source
             * joint from its hierarchy, add it to the joint and
             * attach the source collidable to the destination bone.
             */
            if (destIndex >= 0)
            {
                if (destIndex < attachBone)
                {
                    continue;
                }
                SXRNode destBone = destSkel.getBone(destIndex);
                if (destBone == null)
                {
                    Log.w("PHYSICS LOADER", "Skipping bone node " + srcName + " missing in hierarchy");
                    continue;
                }
                destJoint = (SXRPhysicsJoint) destBone.getComponent(SXRPhysicsJoint.getComponentType());

                srcBone.detachComponent(SXRRigidBody.getComponentType());
                srcBone.detachComponent(SXRPhysicsJoint.getComponentType());
                if (srcJoint.getJointIndex() > 0)
                {
                    srcJoint.removeJointAt(srcJoint.getJointIndex());
                }
                if (destJoint != null)
                {
                    destJoint.copy(srcJoint);
                }
                else
                {
                    destRootJoint.addJoint(srcJoint);
                    destBone.attachComponent(srcJoint);
                }
            }
            /**
             * This is not a common bone. It is present in the source
             * skeleton but not the destination. Try to obtain
             * the destination parent bone from the source parent
             * bone name. If the parent is found, update the
             * source constraint to refer to the collidable
             * in the destination.
             */
            else
            {
                int srcParentIndex = srcSkel.getParentBoneIndex(i);

                if (srcParentIndex < 0)
                {
                    continue;
                }
                String parentBoneName = srcSkel.getBoneName(srcParentIndex);
                int destParentIndex = destSkel.getBoneIndex(parentBoneName);

                if (destParentIndex >= attachBone)
                {
                    if (srcJoint.getJointIndex() > 0)
                    {
                        srcJoint.removeJointAt(srcJoint.getJointIndex());
                    }
                    destRootJoint.removeJointAt(parentJoint.getJointIndex());
                    destRootJoint.addJoint(srcJoint);
                }
             }
        }
        destSkel.merge(srcSkel, destSkel.getBoneName(attachBone));
        return true;
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
