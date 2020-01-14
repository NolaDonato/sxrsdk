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

import org.joml.Matrix4f;
import org.joml.Quaternionf;
import org.joml.Vector3f;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

class URDFExporter
{
    protected String mFirstBoneName = null;

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

    public boolean exportAsURDF(SXRSkeleton skel, String fileName, String firstBoneName)
    {
        String basename = FileNameUtils.getFilename(fileName);
        String xmldata;

        mFirstBoneName = firstBoneName;
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
        int firstBoneIndex = 0;
        int lastBoneIndex = skel.getNumBones() - 1;

        if (mFirstBoneName != null)
        {
            firstBoneIndex = skel.getBoneIndex(mFirstBoneName);
            if (firstBoneIndex < 0)
            {
                firstBoneIndex = 0;
            }
        }
        for (int i = firstBoneIndex; i <= lastBoneIndex; ++i)
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
                if ((i > firstBoneIndex) && (parentIndex < firstBoneIndex))
                {
                    break;
                }
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
        SXRTransform transChild = joint.getOwnerObject().getTransform();
        Matrix4f m = transChild.getLocalModelMatrix4f();
        Vector3f origin = new Vector3f();
        Quaternionf rot = new Quaternionf();
        Vector3f scale = new Vector3f();
        Vector3f euler = new Vector3f();

        m.getScale(scale);
        m.m00(m.m00() / scale.x);
        m.m11(m.m11() / scale.y);
        m.m22(m.m22() / scale.z);
        m.rotate((float) (Math.PI / 2), 1, 0, 0);
        m.getTranslation(origin);
        m.getNormalizedRotation(rot);
        rot.getEulerAnglesXYZ(euler);
        if (collider == null)
        {
            throw new IllegalArgumentException("collider for joint " + name + " missing");
        }
        xml = "<link name=\"" + name + "\">\n";
        xml += "    <interial>\n;";
        xml += "        <mass value=\"" + ((Float) joint.getMass()).toString() + "\" />\n";
        // TODO inertia matrix
        xml += "   </interial>\n";
        xml += convertToURDF(collider, joint.getColliderTransform());
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
            xml += String.format("    axis xyz=\"%f %f %f\"/>", v[0], v[2], v[1]);
            if ((friction != 0) || (damping != 0))
            {
                xml += String.format("<dynamics friction=\"%f\" damping=\"%f\" />\n", friction, damping);
            }
            xml += String.format("    <origin xyz=\"%f %f %f\" rpy=\"%f %f %f\" />\n",
                                 origin.x, origin.z, origin.y, euler.z, euler.x, euler.y);
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
        float friction = body.getFriction();
        float[] damping = body.getDamping();
        Vector3f scale = new Vector3f();
        SXRTransform transChild = body.getOwnerObject().getTransform();
        Matrix4f m = transChild.getLocalModelMatrix4f();

        m.getScale(scale);
        m.m00(m.m00() / scale.x);
        m.m11(m.m11() / scale.y);
        m.m22(m.m22() / scale.z);
        m.rotate((float) (Math.PI / 2), 1, 0, 0);

        if (collider == null)
        {
            throw new IllegalArgumentException("collider for body " + name + " missing");
        }
        xml = "<link name=\"" + name + "\">\n";
        xml += "   <interial>\n";
        xml += "       <mass value=\"" + ((Float) body.getMass()).toString() + "\" />\n";
        // TODO inertia matrix
        xml += "   </interial>\n";
        xml += convertToURDF(collider, body.getColliderTransform());
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
            xml += convertToURDF(constraint, m);
            xml += "</joint>\n";
        }
        return xml;
    }

    protected String convertToURDF(SXRCollider collider, float[] colliderTransform)
    {
        String xml = "    <collision>\n";
        Quaternionf rot = new Quaternionf();
        Vector3f euler = new Vector3f(0, 0, 0);
        Vector3f scale = new Vector3f();
        Vector3f offset = new Vector3f();
        Matrix4f m = new Matrix4f();

        m.set(colliderTransform);
        m.getScale(scale);
        m.m00(m.m00() / scale.x);
        m.m11(m.m11() / scale.y);
        m.m22(m.m22() / scale.z);
        m.rotate((float) (Math.PI / 2), 1, 0, 0);
        m.getTranslation(offset);
        m.getNormalizedRotation(rot);

        xml += "        <geometry>\n";
        if (collider instanceof SXRSphereCollider)
        {
            SXRSphereCollider c = (SXRSphereCollider) collider;
            float radius = c.getRadius();

            xml += String.format("            <sphere radius=\"%f\" />\n", scale.y * radius);
        }
        else if (collider instanceof SXRBoxCollider)
        {
            SXRBoxCollider c = (SXRBoxCollider) collider;
            float[] size = c.getHalfExtents();

            xml += String.format("           <box size=\"%f %f %f\" />\n",
                    size[0] * 2 * scale.x, size[1] * 2 * scale.z, size[2] * 2 * scale.y);
        }
        else if (collider instanceof SXRCapsuleCollider)
        {
            SXRCapsuleCollider c = (SXRCapsuleCollider) collider;
            float r = c.getRadius();
            float h = c.getHeight();
            Vector3f size = new Vector3f();

            if (c.getDirection() == SXRCapsuleCollider.CapsuleDirection.X_AXIS)
            {
                r *= scale.y;
                h *= scale.x;
                size.x = h;
                size.z = size.y = r;
//                offset.z = h / 2;
//                rot.rotateAxis((float) -(Math.PI / 2), 0, 1, 0);
            }
            else if (c.getDirection() == SXRCapsuleCollider.CapsuleDirection.Z_AXIS)
            {
                r *= scale.x;
                h *= scale.z;
                size.z = h;
                size.x = size.y = r;
//                offset.z = h / 2;
            }
            else
            {
                r *= scale.x;
                h *= scale.y;
                size.y = h;
                size.x = size.z = r;
//                offset.z = h / 2;
//                rot.rotateAxis((float) -(Math.PI / 2), 1, 0, 0);
           }
            xml += String.format("           <box size=\"%f %f %f\" />\n", size.x, size.y, size.z);

//            xml += String.format("            <capsule radius=\"%f\" length=\"%f\" />\n", r, h);
        }
        else if (collider instanceof SXRMeshCollider)
        {
            SXRMeshCollider c = (SXRMeshCollider) collider;

            xml += String.format("            <mesh filename=\"ownernode\" scale=\"%f\" />\n", scale.y);
        }
        rot.getEulerAnglesXYZ(euler);
        xml += "        </geometry>\n";
        xml += String.format("        <origin xyz=\"%f %f %f\" rpy=\"%f %f %f\" />\n",
                             offset.x, offset.y, offset.z, euler.z, euler.x, euler.y);
        xml += "    </collision>\n";

        return xml;
    }

    protected String convertToURDF(SXRConstraint constraint, Matrix4f m)
    {
        SXRNode child = constraint.getOwnerObject();
        Vector3f origin = new Vector3f();
        Vector3f euler = new Vector3f();
        Quaternionf q = new Quaternionf();

        m.getTranslation(origin);
        m.getNormalizedRotation(q);
        q.getEulerAnglesXYZ(euler);

        String xml = "";
        xml = String.format("    <origin xyz=\"%f %f %f\" rpy=\"%f %f %f\" />\n",
                            origin.x, origin.y, origin.z,
                            Math.toRadians(euler.z),
                            Math.toRadians(euler.x),
                            Math.toRadians(euler.y));
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
            xml += String.format("    <axis xyz=\"%f %f %f\" />\n", axis.x, axis.y, axis.z);
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
            xml += String.format("    <axis xyz=\"%f %f %f\" />\n", axis[0], axis[2], axis[1]);
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
            if (file.exists())
            {
                file.delete();
            }
            file.createNewFile();
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
