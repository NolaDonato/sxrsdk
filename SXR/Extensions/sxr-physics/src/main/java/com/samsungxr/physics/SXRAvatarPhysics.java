package com.samsungxr.physics;

import com.samsungxr.SXRAndroidResource;
import com.samsungxr.SXRBehavior;
import com.samsungxr.SXRHybridObject;
import com.samsungxr.SXRNode;
import com.samsungxr.animation.SXRAnimation;
import com.samsungxr.animation.SXRAnimationEngine;
import com.samsungxr.animation.SXRAnimator;
import com.samsungxr.animation.SXRAvatar;
import com.samsungxr.animation.SXRPoseMapper;
import com.samsungxr.animation.SXRSkeleton;
import com.samsungxr.utility.FileNameUtils;

import org.joml.Vector3f;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import static com.samsungxr.animation.SXRRepeatMode.REPEATED;

public class SXRAvatarPhysics extends SXRBehavior implements SXRPhysicsLoader.IPhysicsLoaderEvents
{
    static private long TYPE_AVATAR_PHYSICS = newComponentType(SXRAnimator.class);
    protected SXRAvatar mAvatar;
    protected SXRWorld mPhysicsWorld;
    protected SXRSkeleton mPhysicsSkel = null;
    protected SXRNode mPhysicsRoot = null;
    protected SXRPhysicsLoader mPhysicsLoader;
    protected SXRPoseMapper mPhysicsToAvatar = null;
    protected final Map<String, Map<String, Object>> mPhysicsProperties = new HashMap<String, Map<String, Object>>();

    public SXRAvatarPhysics(SXRAvatar avatar, SXRWorld physicsWorld)
    {
        this(avatar, physicsWorld, new SXRPhysicsLoader(avatar.getSXRContext()));
    }

    public SXRAvatarPhysics(SXRAvatar avatar, SXRWorld physicsWorld, SXRPhysicsLoader loader)
    {
        super(avatar.getSXRContext(), 0L);
        mAvatar = avatar;
        mPhysicsWorld = physicsWorld;
        mPhysicsLoader = loader;
        mPhysicsLoader.getEventReceiver().addListener(this);
        mAvatar.getEventReceiver().addListener(mAvatarEventHandler);
        mPhysicsWorld.getEventReceiver().addListener(this);
    }

    protected SXRAvatarPhysics(SXRAvatar avatar, SXRWorld physicsWorld, SXRPhysicsLoader loader, long nativePtr)
    {
        super(avatar.getSXRContext(), nativePtr);
        mAvatar = avatar;
        mPhysicsWorld = physicsWorld;
        mPhysicsLoader = loader;
        mAvatar.getEventReceiver().addListener(this);
        if (loader != null)
        {
            mPhysicsLoader.getEventReceiver().addListener(this);
        }
        if (physicsWorld != null)
        {
            mPhysicsWorld.getEventReceiver().addListener(this);
        }
    }

    public static long getComponentType() { return TYPE_AVATAR_PHYSICS; }

    public SXRPhysicsLoader getPhysicsLoader()
    {
        return mPhysicsLoader;
    }

    public SXRSkeleton getPhysicsSkeleton()
    {
        return mPhysicsSkel;
    }

    public boolean isRunning()
    {
        return mPhysicsWorld.isEnabled();
    }

    public void setPhysicsProperties(String filename, Map<String, Object> properties)
    {
        String fname = FileNameUtils.getFilename(filename).toLowerCase();
        if (properties == null)
        {
            mPhysicsProperties.remove(fname);
        }
        else
        {
            mPhysicsProperties.put(fname, properties);
        }
    }

    public Map<String, Object> getPhysicsProperties(String filename)
    {
        return mPhysicsProperties.get(FileNameUtils.getFilename(filename).toLowerCase());
    }

    public void start()
    {
        if (mPhysicsToAvatar == null)
        {
            mPhysicsToAvatar = new PhysicsRetargeter();
            mPhysicsToAvatar.setName("PhysicsToAvatar");
            mPhysicsToAvatar.setRepeatMode(REPEATED);
        }
        mAvatar.getSkeleton().enable();
        SXRAnimationEngine.getInstance(mAvatar.getSXRContext()).start(mPhysicsToAvatar);
        if (mPhysicsWorld != null)
        {
            mPhysicsWorld.enable();
        }
        mPhysicsSkel.enable();
    }

    public void stop()
    {
        if (mPhysicsWorld != null)
        {
            mPhysicsWorld.disable();
        }
        mPhysicsSkel.disable();
        SXRAnimationEngine.getInstance(mAvatar.getSXRContext()).stop(mPhysicsToAvatar);
    }

    @Override
    public void onPhysicsLoaded(SXRPhysicsContent world, SXRSkeleton skel, String filename)
    {
        Map<String, Object> physicsProps = getPhysicsProperties(filename);
        String attachBone = (physicsProps != null) ? (String) physicsProps.get("AttachBone") : null;

        setPhysicsProperties(filename, null);
        if (mPhysicsSkel == null)
        {
            mPhysicsRoot = world.getOwnerObject();
            if ((mPhysicsRoot != null) && (mAvatar.getSkeleton() != null) && (skel != null))
            {
                mPhysicsSkel = skel;
                mPhysicsSkel.poseFromBones();
                mPhysicsSkel.disable();
                if (mPhysicsWorld != world)
                {
                    mAvatar.getModel().addChildObject(mPhysicsRoot);
                    mPhysicsWorld.merge(world);
                }
            }
        }
        else if (attachBone != null)
        {
            int attachIndex1 = mPhysicsSkel.getBoneIndex(attachBone);
            int attachIndex2 = skel.getBoneIndex(attachBone);
            final SXRNode attachNode1 = mPhysicsSkel.getBone(attachIndex1);
            final SXRNode attachNode2 = skel.getBone(attachIndex2);
            SXRPhysicsJoint attachJoint1 = (SXRPhysicsJoint) ((attachNode1 != null) ?
                attachNode1.getComponent(SXRPhysicsJoint.getComponentType()) : null);
            SXRPhysicsJoint attachJoint2 = (SXRPhysicsJoint) ((attachNode2 != null) ?
                attachNode2.getComponent(SXRPhysicsJoint.getComponentType()) : null);

            skel.poseFromBones();
            if (mPhysicsWorld != world)
            {
                if ((attachJoint1 != null) && (attachJoint2 != null))
                {
                    attachJoint1.merge(mPhysicsSkel, skel);
                    mPhysicsWorld.merge(world);
                    return;
                }
                else
                {
                    if (skel != null)
                    {
                        mPhysicsSkel.merge(skel, attachBone);
                    }
                    mPhysicsWorld.merge(world);
                }
            }
            SXRRigidBody attachBody1 = (SXRRigidBody) ((attachNode1 != null) ?
                                        attachNode1.getComponent(SXRRigidBody.getComponentType()) : 0);
            SXRRigidBody attachBody2 = (SXRRigidBody) ((attachNode2 != null) ?
                                        attachNode2.getComponent(SXRRigidBody.getComponentType()) : 0);
            final SXRPhysicsCollidable bodyA = ((attachBody1 != attachBody2) && (attachBody1 != null)) ?
                                                attachBody1 : attachJoint1;
            final SXRPhysicsCollidable bodyB = ((attachBody1 != attachBody2) && (attachBody2 != null)) ?
                                                attachBody2 : attachJoint2;

            if ((bodyA != bodyB) &&
                ((bodyA != null) || (bodyB != null)))
            {
                mPhysicsWorld.run(new Runnable()
                {
                    @Override
                    public void run()
                    {
                        SXRFixedConstraint constraint = new SXRFixedConstraint(mAvatar.getSXRContext(), bodyB);
                        attachNode1.attachComponent(constraint);
                    }
                });
            }
        }
        else if (mPhysicsWorld != world)
        {
            if (skel != null)
            {
                mPhysicsSkel.merge(skel, attachBone);
            }
            mPhysicsWorld.merge(world);
        }
    }

    @Override
    public void onLoadError(SXRPhysicsContent world, String filename, String errors) { }

    protected SXRAvatar.IAvatarEvents mAvatarEventHandler = new SXRAvatar.IAvatarEvents()
    {
        @Override
        public void onAvatarLoaded(SXRAvatar avatar, SXRNode avatarRoot, String filePath, String errors)
        {
            String physicsfile = avatar.getProperty("physics");

            physicsfile = (physicsfile != null) ? physicsfile : avatar.getProperty("avt");
            avatar.getSkeleton().disable();
            if (physicsfile != null)
            {
                SXRAndroidResource res;
                Map<String, Object> physicsProps = getPhysicsProperties(physicsfile);

                if (physicsProps == null)
                {
                    physicsProps = initPhysicsProperties(avatar, "avatar");
                }
                if (physicsProps != null)
                {
                    setPhysicsProperties(physicsfile, physicsProps);
                }
                try
                {
                    if (physicsfile.startsWith("/"))
                    {
                        res = new SXRAndroidResource(physicsfile);
                    }
                    else
                    {
                        res = new SXRAndroidResource(mAvatar.getSXRContext(), physicsfile);
                    }
                    mPhysicsLoader.loadPhysics(mPhysicsWorld, res, physicsProps);
                }
                catch (IOException ex)
                {
                    ex.printStackTrace();
                }
            }
        }

        @Override
        public void onModelLoaded(SXRAvatar avatar, SXRNode modelRoot, String filePath, String errors)
        {
            String modelType = avatar.findModelType(modelRoot);
            String physicsfile;
            SXRAndroidResource res;

            if (modelType == null)
            {
                return;
            }
            physicsfile = avatar.getModelProperty(modelType, "physics");
            physicsfile = (physicsfile != null) ? physicsfile : avatar.getModelProperty(modelType, "avt");
            if (physicsfile == null)
            {
                return;
            }
            try
            {
                if (physicsfile.startsWith("/"))
                {
                    res = new SXRAndroidResource(physicsfile);
                }
                else
                {
                    res = new SXRAndroidResource(mAvatar.getSXRContext(), physicsfile);
                }
                Map<String, Object> physicsProps = getPhysicsProperties(physicsfile);

                if (physicsProps == null)
                {
                    physicsProps = initPhysicsProperties(avatar, modelType);
                }
                if (physicsProps != null)
                {
                    physicsProps.put("Skeleton", mPhysicsSkel);
                    setPhysicsProperties(physicsfile, physicsProps);
                }
                synchronized (mPhysicsSkel)
                {
                    mPhysicsLoader.loadPhysics(mPhysicsWorld, res, physicsProps);
                }
            }
            catch (IOException ex) { }
        }

        @Override
        public void onAnimationLoaded(SXRAvatar avatar, SXRAnimator animator, String filePath, String errors)
        {
            if (animator == null)
            {
                return;
            }
            synchronized (animator)
            {
                if (animator.getAnimationCount() > 1)
                {
                    SXRAnimation anim = animator.getAnimation(1);
                    if (anim instanceof SXRPoseMapper)
                    {
                        SXRPoseMapper animToAvatar = (SXRPoseMapper) anim;
                        SXRPoseMapper animToPhysics = new SXRPoseMapper(mPhysicsSkel,
                                                                        animToAvatar
                                                                            .getSourceSkeleton(),
                                                                        anim.getDuration());

                        animToPhysics.setName(anim.getName() + ".ToPhysics");
                        animToPhysics.setBoneOptions(SXRSkeleton.BONE_ANIMATE);
                        animToPhysics.setBoneMap(avatar.getProperty("bonemap"));

                        animator.removeAnimation(animToAvatar);
                        animator.addAnimation(animToPhysics);
                    }
                }
            }
        }

        @Override
        public void onAnimationStarted(SXRAvatar avatar, SXRAnimator animator)
        {
            if (!isRunning())
            {
                start();
            }
        }

        @Override
        public void onAnimationFinished(SXRAvatar avatar, SXRAnimator animator) { }
    };

    protected  Map<String, Object> initPhysicsProperties(SXRAvatar avatar, String modelType)
    {
        Map<String, Object> physicsProps = new HashMap<>();
        String attachbone = avatar.getModelProperty(modelType, "attachbone");
        String s;

        if (attachbone != null)
        {
            physicsProps.put("AttachBone", attachbone);
        }
        s = avatar.getModelProperty(modelType, "angularspringdamping");
        if (s != null)
        {
            float angularspringdamping = Float.parseFloat(s);
            physicsProps.put("AngularSpringDamping",
                    new Vector3f(angularspringdamping, angularspringdamping, angularspringdamping));
        }
        s = avatar.getModelProperty(modelType, "angularspringstiffness");
        if (s != null)
        {
            float angularspringstiffness = Float.parseFloat(s);
            physicsProps.put("AngularSpringStiffness",
                    new Vector3f(angularspringstiffness, angularspringstiffness, angularspringstiffness));
        }
        s = avatar.getModelProperty(modelType, "angularlimits");
        if (s != null)
        {
            float angularlimits = Float.parseFloat(s);
            physicsProps.put("AngularLimits",
                    new Vector3f(angularlimits, angularlimits, angularlimits));
        }
        s = avatar.getModelProperty(modelType, "collisiongroup");
        if (s != null)
        {
            int collisiongroup = Integer.parseInt(s);
            physicsProps.put("CollisionGroup", collisiongroup);
        }
        s = avatar.getModelProperty(modelType, "simulationtype");
        if (s != null)
        {
            int simulationType = Integer.parseInt(s);
            physicsProps.put("SimulationType", simulationType);
        }
        else
        {
            physicsProps.put("SimulationType", SXRRigidBody.DYNAMIC);
        }
        return physicsProps;
    }

    protected class PhysicsRetargeter extends SXRPoseMapper
    {
        public PhysicsRetargeter()
        {
            super(mAvatar.getSkeleton(), mPhysicsSkel, 100);
        }

        public void animate(float time)
        {
            if (mPhysicsSkel != null)
            {
                mPhysicsSkel.poseFromBones();
                super.animate(time);
            }
        }
    }
};
