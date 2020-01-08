package com.samsungxr.physics;

import com.samsungxr.SXRBoxCollider;
import com.samsungxr.SXRCapsuleCollider;
import com.samsungxr.SXRCollider;
import com.samsungxr.SXRMeshCollider;
import com.samsungxr.SXRNode;
import com.samsungxr.SXRSphereCollider;
import com.samsungxr.SXRTransform;
import com.samsungxr.animation.SXRSkeleton;
import com.samsungxr.utility.FileNameUtils;

import org.joml.Vector3f;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

class URDFExporter
{
    public URDFExporter() { }

    public boolean exportAsURDF(SXRSkeleton skel, String fileName)
    {
        String basename = FileNameUtils.getFilename(fileName);
        String xmldata;

        basename = basename.substring(0, basename.lastIndexOf('.'));
        xmldata = convertToURDF(skel, basename);
        if (xmldata != null)
        {
            File file = new File(fileName);
            return writePhysicsFile(file, xmldata);
        }
        return false;
    }

    public String convertToURDF(SXRSkeleton skel, String name)
    {
        String xml = "<robot name = \"" + name +"\">\n";

        for (int i = 0; i < skel.getNumBones(); ++i)
        {
            SXRNode         bone = skel.getBone(i);
            SXRNode         parentBone = null;
            int             parentIndex = skel.getParentBoneIndex(i);
            SXRPhysicsJoint joint;
            SXRRigidBody    body;

            if (bone == null)
            {
                throw new IllegalArgumentException("node for bone " + skel.getBoneName(i) + " missing");
            }
            if (parentIndex >= 0)
            {
                parentBone = skel.getBone(parentIndex);
            }
            joint = (SXRPhysicsJoint) bone.getComponent(SXRPhysicsJoint.getComponentType());
            if (joint != null)
            {
                xml += convertToURDF(joint, parentBone);
                continue;
            }
            body = (SXRRigidBody) bone.getComponent(SXRRigidBody.getComponentType());
            if (body != null)
            {
                xml += convertToURDF(body, parentBone);
                continue;
            }
            throw new IllegalArgumentException("joint or rigid body for bone " + skel.getBoneName(i) + " missing");
        }
        xml += "</robot>";
        return xml;
    }

    protected String convertToURDF(SXRPhysicsJoint joint, SXRNode parentNode)
    {
        String name = joint.getName();
        SXRCollider collider = joint.getOwnerObject().getCollider();
        String xml;
        String type;
        float[] scale = joint.getScale();
        SXRTransform childLocalTrans = joint.getOwnerObject().getTransform();
        Vector3f origin = new Vector3f(childLocalTrans.getPositionX(),
                childLocalTrans.getPositionY(),
                childLocalTrans.getPositionZ());
        Vector3f eulerRot = new Vector3f(childLocalTrans.getRotationRoll(),
                childLocalTrans.getRotationPitch(),
                childLocalTrans.getRotationYaw());

        if (collider == null)
        {
            throw new IllegalArgumentException("collider for joint " + name + " missing");
        }
        xml = "<link name=\"" + name + "\">\n";
        xml += "    <interial>\n;";
        xml += "        <mass value=\"" + ((Float) joint.getMass()).toString() + "\" />\n";
        // TODO inertia matrix
        xml += "   </interial>\n";
        xml += convertToURDF(collider, scale[1]);
        xml += "</link>\n";
        switch (joint.getJointType())
        {
            case SXRPhysicsJoint.FIXED: type = "fixed"; break;
            case SXRPhysicsJoint.PRISMATIC: type = "prismatic"; break;
            case SXRPhysicsJoint.REVOLUTE: type = "revolute"; break;
            case SXRPhysicsJoint.SPHERICAL: type = "floating"; break;
            default: type = "unknown"; break;
        }
        if (!type.equals("unknown"))
        {
            float[] v = joint.getAxis();
            float friction = joint.getFriction();
            float damping = joint.getAngularDamping();

            xml += "<joint name=\"" + name + "\" type=\"" + type + "\" >\n";
            if (parentNode == null)
            {
                xml += "    <parent link=\"" + parentNode.getName() + "\" />\n";
            }
            xml += "    <child link=\"" + name + "\"/>\n";
            xml += String.format("    axis xyz=\"%f %f %f\"/>", v[0], v[1], v[2]);
            if ((friction != 0) || (damping != 0))
            {
                xml += String.format("<dynamics friction=\"%f\" damping=\"%f\" />\n", friction, damping);
            }
            xml += String.format("    <origin xyz=\"%f %f %f\" rpy=\"%f %f %f\" />\n", origin.x, origin.y, origin.z, eulerRot.x, eulerRot.y, eulerRot.z);
            xml += "</joint>\n";
        }
        return xml;
    }

    protected String convertToURDF(SXRRigidBody body, SXRNode parentNode)
    {
        String name = body.getName();
        SXRCollider collider = body.getOwnerObject().getCollider();
        SXRConstraint constraint = (SXRConstraint) body.getOwnerObject().getComponent(SXRConstraint.getComponentType());
        String xml;
        float[] scale = body.getScale();
        float friction = body.getFriction();
        float[] damping = body.getDamping();

        if (collider == null)
        {
            throw new IllegalArgumentException("collider for body " + name + " missing");
        }
        xml = "<link name=\"" + name + "\">\n";
        xml += "   <interial>\n";
        xml += "       <mass value=\"" + ((Float) body.getMass()).toString() + "\" />\n";
        // TODO inertia matrix
        xml += "   </interial>\n";
        xml += convertToURDF(collider, scale[1]);
        xml += "</link>\n";
        if (parentNode == null)
        {
            return xml;
        }
        if (constraint != null)
        {
            xml += "<joint name=\"joint_" + name + "\" type=\"" + getJointType(constraint) + "\" >\n";
            xml += "    <parent link=\"" + parentNode.getName() + "\" />\n";
            xml += "    <child link=\"" + name + "\" />\n";
            if ((friction != 0) || (damping != null))
            {
                xml += String.format("    <dynamics friction=\"%f\" damping=\"%f\" />\n", friction, damping[0]);
            }
            xml += convertToURDF(constraint);
            xml += "</joint>\n";
        }
        return xml;
    }

    protected String convertToURDF(SXRCollider collider, float scale)
    {
        String xml = "    <collision>\n        <geometry>\n";

        if (collider instanceof SXRSphereCollider)
        {
            SXRSphereCollider c = (SXRSphereCollider) collider;

            xml += String.format("            <sphere radius=\"%f\" />\n", scale * c.getRadius());
        }
        else if (collider instanceof SXRBoxCollider)
        {
            SXRBoxCollider c = (SXRBoxCollider) collider;
            float[] size = c.getHalfExtents();

            xml += String.format("           <box size=\"%f %f %f\" />\n",
                    size[0] * 2 * scale, size[1] * 2 * scale, size[2] * 2 * scale);
        }
        else if (collider instanceof SXRCapsuleCollider)
        {
            SXRCapsuleCollider c = (SXRCapsuleCollider) collider;
            float r = c.getRadius() * scale;
            float h = c.getHeight() * scale;

            xml += String.format("            <cylinder radius=\"%f\" length=\"%f\" />\n", r, h);
        }
        else if (collider instanceof SXRMeshCollider)
        {
            SXRMeshCollider c = (SXRMeshCollider) collider;

            xml += String.format("            <mesh filename=\"ownernode\" scale=\"%f\" />\n", scale);
        }
        xml += "        </geometry>\n    </collision>\n";
        return xml;
    }

    protected String convertToURDF(SXRConstraint constraint)
    {
        SXRNode child = constraint.getOwnerObject();
        SXRTransform childLocalTrans = child.getTransform();
        Vector3f origin = new Vector3f(childLocalTrans.getPositionX(),
                childLocalTrans.getPositionY(),
                childLocalTrans.getPositionZ());
        Vector3f eulerRot = new Vector3f(childLocalTrans.getRotationRoll(),
                childLocalTrans.getRotationPitch(),
                childLocalTrans.getRotationYaw());

        String xml = String.format("    <origin xyz=\"%f %f %f\" rpy=\"%f %f %f\" />\n",
                origin.x, origin.y, origin.z,
                eulerRot.x, eulerRot.y, eulerRot.z);
        if (constraint instanceof SXRSliderConstraint)
        {
            SXRSliderConstraint c = (SXRSliderConstraint) constraint;
            Vector3f axis = new Vector3f();
            float lowerLimit = c.getAngularLowerLimit();
            float upperLimit = c.getAngularUpperLimit();

            if (lowerLimit != upperLimit)
            {
                xml += String.format("    <limit lower=\"%f\" upper=\"%f\" />\n", lowerLimit, upperLimit);
            }

            origin.normalize(axis);
            xml += String.format("    <axis xyz=\"%f %f %f\" />\n",axis.x, axis.y, axis.z);
        }
        else if (constraint instanceof SXRHingeConstraint)
        {
            SXRHingeConstraint c = (SXRHingeConstraint) constraint;
            float[] axis = c.getAxis();
            float lowerLimit = c.getLowerLimit();
            float upperLimit = c.getUpperLimit();

            if (lowerLimit != upperLimit)
            {
                xml += String.format("    <limit lower=\"%f\" upper=\"%f\" />\n", lowerLimit, upperLimit);
            }
            xml += String.format("    <axis xyz=\"%f %f %f\" />\n", axis[0], axis[1], axis[2]);
        }
        return xml;
    }

    protected String getJointType(SXRConstraint constraint)
    {
        if (constraint instanceof SXRHingeConstraint)
        {
            return "revolute";
        }
        else if (constraint instanceof SXRFixedConstraint)
        {
            return "fixed";
        }
        else if (constraint instanceof SXRSliderConstraint)
        {
            return "prismatic";
        }
        return "floating";
    }

    protected boolean writePhysicsFile(File file, String data)
    {
        try
        {
            if (!file.exists())
            {
                file.createNewFile();
            }
            FileWriter writer = new FileWriter(file);
            writer.append(data);
            writer.flush();
            writer.close();
            return true;
        }
        catch (IOException e)
        {
            return false;
        }
    }

}
