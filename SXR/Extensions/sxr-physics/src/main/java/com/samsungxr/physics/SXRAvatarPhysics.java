package com.samsungxr.physics;

import com.samsungxr.SXRAndroidResource;
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

public class SXRAvatarPhysics implements SXRPhysicsLoader.IPhysicsLoaderEvents, SXRAvatar.IAvatarEvents
{
    protected SXRAvatar mAvatar;
    protected SXRWorld mPhysicsWorld;
    protected SXRSkeleton mPhysicsSkel = null;
    protected SXRNode mPhysicsRoot = null;
    protected SXRPhysicsLoader mPhysicsLoader;
    protected SXRPoseMapper mPhysicsToAvatar = null;
    protected final Map<String, Map<String, Object>> mPhysicsProperties = new HashMap<String, Map<String, Object>>();

    public SXRAvatarPhysics(SXRAvatar avatar, SXRWorld physicsWorld)
    {
        mAvatar = avatar;
        mPhysicsWorld = physicsWorld;
        mPhysicsLoader = new SXRPhysicsLoader(mAvatar.getSXRContext());
        mPhysicsLoader.setMultiBody(false);
        mPhysicsLoader.getEventReceiver().addListener(this);
        mAvatar.getEventReceiver().addListener(this);
        mPhysicsWorld.getEventReceiver().addListener(this);
    }

    public SXRAvatarPhysics(SXRAvatar avatar, SXRWorld physicsWorld, SXRPhysicsLoader loader)
    {
        mAvatar = avatar;
        mPhysicsWorld = physicsWorld;
        mPhysicsLoader = loader;
        mPhysicsLoader.getEventReceiver().addListener(this);
        mAvatar.getEventReceiver().addListener(this);
        mPhysicsWorld.getEventReceiver().addListener(this);
    }

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
        if (properties == null)
        {
            mPhysicsProperties.remove(filename);
        }
        else
        {
            mPhysicsProperties.put(filename, properties);
        }
    }

    public void start()
    {
        if (mPhysicsToAvatar == null)
        {
            mPhysicsToAvatar = new SXRPoseMapper(mAvatar.getSkeleton(), mPhysicsSkel, 100000);
            mPhysicsToAvatar.setName("PhysicsToAvatar");
            mPhysicsSkel.enable();
            SXRAnimationEngine.getInstance(mAvatar.getSXRContext()).start(mPhysicsToAvatar);
        }
        mPhysicsWorld.enable();
    }

    public void stop()
    {
        mPhysicsWorld.disable();
        SXRAnimationEngine.getInstance(mAvatar.getSXRContext()).stop(mPhysicsToAvatar);
    }

    @Override
    public void onPhysicsLoaded(SXRPhysicsContent world, SXRSkeleton skel, String filename)
    {
        Map<String, Object> loaderProps = mPhysicsProperties.get(filename.toLowerCase());
        String attachBone = (String) loaderProps.get("AttachBone");
        if (mPhysicsSkel == null)
        {
            mPhysicsRoot = world.getOwnerObject();
            mPhysicsProperties.remove(filename);
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

            mPhysicsProperties.remove(filename);
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
                mPhysicsSkel.merge(skel, null);
            }
            mPhysicsWorld.merge(world);
        }
    }

    @Override
    public void onLoadError(SXRPhysicsContent world, String filename, String errors) { }

    @Override
    public void onAvatarLoaded(SXRAvatar avatar, SXRNode avatarRoot, String filePath, String errors)
    {
        String physicsfile = avatar.getProperty("urdf");
        physicsfile = (physicsfile != null) ? physicsfile : avatar.getProperty("bullet");
        physicsfile = (physicsfile != null) ? physicsfile : avatar.getProperty("avt");

        if (physicsfile != null)
        {
            SXRAndroidResource res;

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
                mPhysicsLoader.loadPhysics(res, null);
            }
            catch (IOException ex)
            {
                return;
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
        physicsfile = avatar.getModelProperty(modelType, "urdf");
        physicsfile = (physicsfile != null) ? physicsfile : avatar.getModelProperty(modelType, "bullet");
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
            Map<String, Object> loaderProps = new HashMap<String, Object>();
            String s;
            String attachbone = avatar.getModelProperty(modelType, "attachbone");

            if (attachbone != null)
            {
                loaderProps.put("AttachBone", attachbone);
            }
            loaderProps.put("Skeleton", mPhysicsSkel);
            s = avatar.getModelProperty(modelType, "angularspringdamping");
            if (s != null)
            {
                float angularspringdamping = Float.parseFloat(s);
                loaderProps.put("AngularSpringDamping",
                                new Vector3f(angularspringdamping, angularspringdamping, angularspringdamping));
            }
            s = avatar.getModelProperty(modelType, "angularspringstiffness");
            if (s != null)
            {
                float angularspringstiffness = Float.parseFloat(s);
                loaderProps.put("AngularSpringStiffness",
                                new Vector3f(angularspringstiffness, angularspringstiffness, angularspringstiffness));
            }
            s = avatar.getModelProperty(modelType, "angularlimits");
            if (s != null)
            {
                float angularlimits = Float.parseFloat(s);
                loaderProps.put("AngularLimits",
                                new Vector3f(angularlimits, angularlimits, angularlimits));
            }
            s = avatar.getModelProperty(modelType, "collisiongroup");
            if (s != null)
            {
                int collisiongroup = Integer.parseInt(s);
                loaderProps.put("CollisionGroup", collisiongroup);
            }
            s = avatar.getModelProperty(modelType, "simulationtype");
            if (s != null)
            {
                int simulationType = Integer.parseInt(s);
                loaderProps.put("SimulationType", simulationType);
            }
            mPhysicsProperties.put(FileNameUtils.getBaseName(physicsfile).toLowerCase(), loaderProps);

//            mPhysicsLoader.loadPhysics(res, loaderProps);
        }
        catch (IOException ex) { }
    }

    @Override
    public void onAnimationStarted(SXRAvatar avatar, SXRAnimator animator) { }

    @Override
    public void onAnimationFinished(SXRAvatar avatar, SXRAnimator animator) { }

    @Override
    public void onAnimationLoaded(SXRAvatar avatar, SXRAnimator animator, String filePath, String errors)
    {
        if (animator.getAnimationCount() > 1)
        {
            SXRAnimation anim = animator.getAnimation(1);
            if (anim instanceof SXRPoseMapper)
            {
                SXRPoseMapper animToAvatar = (SXRPoseMapper) anim;
                SXRPoseMapper animToPhysics = new SXRPoseMapper(mPhysicsSkel, animToAvatar.getSourceSkeleton(), anim.getDuration());

                animToPhysics.setName(anim.getName() + ".ToPhysics");
                animToPhysics.setBoneOptions(SXRSkeleton.BONE_ANIMATE);
                animator.removeAnimation(animToAvatar);
                animator.addAnimation(animToPhysics);
            }
        }
    }
};
