#include <cstddef>
#include <iostream>
#include "URDFConverter.h"
#include "BulletDynamics/Featherstone/btMultiBodyLinkCollider.h"
#include "BulletDynamics/Featherstone/btMultiBodyDynamicsWorld.h"
#include "BulletUrdfImporter.h"
#include "CommonInterfaces/CommonGUIHelperInterface.h"
#include "URDF2Bullet.h"
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include "BulletCollision/CollisionShapes/btCompoundShape.h"
#include "btBulletDynamicsCommon.h"
#include "MultiBodyCreationInterface.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofConstraint.h"
#include "BulletDynamics/ConstraintSolver/btGeneric6DofSpring2Constraint.h"
#include "URDFJointTypes.h"

struct GenericConstraintUserInfo
{
	int m_urdfIndex;
	int m_urdfJointType;
	btVector3 m_jointAxisInJointSpace;
	int m_jointAxisIndex;
	btScalar m_lowerJointLimit;
	btScalar m_upperJointLimit;
};

MultiBodyCreator::MultiBodyCreator(const URDFImporterInterface& u2b) : m_urdfImport(u2b)
{
	m_bodies.resize(u2b.getNumAllocatedCollisionShapes());
}

const std::string& MultiBodyCreator::registerNameForPointer(int urdfIndex, btCollisionObject* ptr, const std::string& name)
{
	if (m_bodies.size() <= urdfIndex)
	{
		m_bodies.resize(urdfIndex + 1);
		m_names.resize(urdfIndex + 1);
	}
	m_bodies[urdfIndex] = ptr;
	m_names[urdfIndex] = name;
	return m_names[urdfIndex];
}

btMultiBody* MultiBodyCreator::allocateMultiBody(int urdfLinkIndex, int totalNumJoints,
												 btScalar mass, const btVector3& localInertiaDiagonal,
												 bool isFixedBase, bool canSleep)
{
	m_mb2urdfLink.resize(totalNumJoints + 1, -2);
	m_bulletMultiBody = new btMultiBody(totalNumJoints, mass, localInertiaDiagonal, isFixedBase, canSleep);
	return m_bulletMultiBody;
}

btRigidBody* MultiBodyCreator::allocateRigidBody(int urdfLinkIndex, btScalar mass,
												 const btVector3& localInertiaDiagonal,
												 const btTransform& initialWorldTrans,
												 class btCollisionShape* colShape)
{
	btRigidBody::btRigidBodyConstructionInfo rbci(mass, 0, colShape, localInertiaDiagonal);
	rbci.m_startWorldTransform = initialWorldTrans;
	btRigidBody* body = new btRigidBody(rbci);
	if (m_rigidBody == 0)
	{
		m_rigidBody = body;
	}
	std::string name = m_urdfImport.getJointName(urdfLinkIndex);
	registerNameForPointer(urdfLinkIndex, body, name);
	return body;
}

btMultiBodyLinkCollider* MultiBodyCreator::allocateMultiBodyLinkCollider(int urdfLinkIndex,
																		 int mbLinkIndex, btMultiBody* multiBody)
{
	btMultiBodyLinkCollider* mbCol = new btMultiBodyLinkCollider(multiBody, mbLinkIndex);
	std::string name = m_urdfImport.getJointName(urdfLinkIndex);
	registerNameForPointer(urdfLinkIndex, mbCol, name);
	return mbCol;
}

btGeneric6DofSpring2Constraint* MultiBodyCreator::allocateGeneric6DofSpring2Constraint(int urdfLinkIndex,
																					   btRigidBody& rbA /*parent*/,
																					   btRigidBody& rbB,
																					   const btTransform& offsetInA,
																					   const btTransform& offsetInB,
																					   int rotateOrder)
{
	return new btGeneric6DofSpring2Constraint(rbA, rbB, offsetInA, offsetInB, (RotateOrder)rotateOrder);
}

btGeneric6DofSpring2Constraint* MultiBodyCreator::createPrismaticJoint(int urdfLinkIndex,
																	   btRigidBody& rbA /*parent*/,
																	   btRigidBody& rbB,
																	   const btTransform& offsetInA,
																	   const btTransform& offsetInB,
																	   const btVector3& jointAxisInJointSpace,
																	   btScalar jointLowerLimit, btScalar jointUpperLimit)
{
	int rotateOrder = 0;
	btGeneric6DofSpring2Constraint* dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB,
																				offsetInA, offsetInB, rotateOrder);
	//todo(erwincoumans) for now, we only support principle axis along X, Y or Z
	int principleAxis = jointAxisInJointSpace.closestAxis();

	GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
	userInfo->m_jointAxisInJointSpace = jointAxisInJointSpace;
	userInfo->m_jointAxisIndex = principleAxis;

	userInfo->m_urdfJointType = URDFPrismaticJoint;
	userInfo->m_lowerJointLimit = jointLowerLimit;
	userInfo->m_upperJointLimit = jointUpperLimit;
	userInfo->m_urdfIndex = urdfLinkIndex;
	dof6->setUserConstraintPtr(userInfo);

	switch (principleAxis)
	{
		case 0:
		{
			dof6->setLinearLowerLimit(btVector3(jointLowerLimit, 0, 0));
			dof6->setLinearUpperLimit(btVector3(jointUpperLimit, 0, 0));
			break;
		}
		case 1:
		{
			dof6->setLinearLowerLimit(btVector3(0, jointLowerLimit, 0));
			dof6->setLinearUpperLimit(btVector3(0, jointUpperLimit, 0));
			break;
		}
		case 2:
		default:
		{
			dof6->setLinearLowerLimit(btVector3(0, 0, jointLowerLimit));
			dof6->setLinearUpperLimit(btVector3(0, 0, jointUpperLimit));
		}
	};

	dof6->setAngularLowerLimit(btVector3(0, 0, 0));
	dof6->setAngularUpperLimit(btVector3(0, 0, 0));
	m_6DofConstraints.push_back(dof6);
	return dof6;
}

btGeneric6DofSpring2Constraint* MultiBodyCreator::createRevoluteJoint(int urdfLinkIndex,
																	  btRigidBody& rbA /*parent*/,
																	  btRigidBody& rbB,
																	  const btTransform& offsetInA,
																	  const btTransform& offsetInB,
																	  const btVector3& jointAxisInJointSpace,
																	  btScalar jointLowerLimit, btScalar jointUpperLimit)
{
	btGeneric6DofSpring2Constraint* dof6 = 0;

	//only handle principle axis at the moment,
	//@todo(erwincoumans) orient the constraint for non-principal axis
	int principleAxis = jointAxisInJointSpace.closestAxis();
	switch (principleAxis)
	{
		case 0:
		{
			dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, RO_ZYX);
			dof6->setLinearLowerLimit(btVector3(0, 0, 0));
			dof6->setLinearUpperLimit(btVector3(0, 0, 0));

			dof6->setAngularLowerLimit(btVector3(jointLowerLimit, 0, 0));
			dof6->setAngularUpperLimit(btVector3(jointUpperLimit, 0, 0));

			break;
		}
		case 1:
		{
			dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, RO_XZY);
			dof6->setLinearLowerLimit(btVector3(0, 0, 0));
			dof6->setLinearUpperLimit(btVector3(0, 0, 0));

			dof6->setAngularLowerLimit(btVector3(0, jointLowerLimit, 0));
			dof6->setAngularUpperLimit(btVector3(0, jointUpperLimit, 0));
			break;
		}
		case 2:
		default:
		{
			dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB, RO_XYZ);
			dof6->setLinearLowerLimit(btVector3(0, 0, 0));
			dof6->setLinearUpperLimit(btVector3(0, 0, 0));

			dof6->setAngularLowerLimit(btVector3(0, 0, jointLowerLimit));
			dof6->setAngularUpperLimit(btVector3(0, 0, jointUpperLimit));
		}
	};

	GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
	userInfo->m_jointAxisInJointSpace = jointAxisInJointSpace;
	userInfo->m_jointAxisIndex = 3 + principleAxis;

	if (jointLowerLimit > jointUpperLimit)
	{
		userInfo->m_urdfJointType = URDFContinuousJoint;
	}
	else
	{
		userInfo->m_urdfJointType = URDFRevoluteJoint;
		userInfo->m_lowerJointLimit = jointLowerLimit;
		userInfo->m_upperJointLimit = jointUpperLimit;
	}
	userInfo->m_urdfIndex = urdfLinkIndex;
	dof6->setUserConstraintPtr(userInfo);
	m_6DofConstraints.push_back(dof6);
	return dof6;
}

btGeneric6DofSpring2Constraint* MultiBodyCreator::createFixedJoint(int urdfLinkIndex,
																   btRigidBody& rbA /*parent*/,
																   btRigidBody& rbB,
																   const btTransform& offsetInA,
																   const btTransform& offsetInB)
{
	btGeneric6DofSpring2Constraint* dof6 = allocateGeneric6DofSpring2Constraint(urdfLinkIndex, rbA, rbB, offsetInA, offsetInB);

	GenericConstraintUserInfo* userInfo = new GenericConstraintUserInfo;
	userInfo->m_urdfIndex = urdfLinkIndex;
	userInfo->m_urdfJointType = URDFFixedJoint;

	dof6->setUserConstraintPtr(userInfo);

	dof6->setLinearLowerLimit(btVector3(0, 0, 0));
	dof6->setLinearUpperLimit(btVector3(0, 0, 0));

	dof6->setAngularLowerLimit(btVector3(0, 0, 0));
	dof6->setAngularUpperLimit(btVector3(0, 0, 0));
	m_6DofConstraints.push_back(dof6);
	return dof6;
}

void MultiBodyCreator::addLinkMapping(int urdfLinkIndex, int mbLinkIndex)
{
	if (m_mb2urdfLink.size() < (mbLinkIndex + 1))
	{
		m_mb2urdfLink.resize((mbLinkIndex + 1), -2);
	}
	m_mb2urdfLink[mbLinkIndex] = urdfLinkIndex;
}

void MultiBodyCreator::registerNames(btSerializer& s, bool exporting)
{
	int index = 0;
	for (auto it = m_bodies.begin(); it < m_bodies.end(); ++it)
	{
		btCollisionObject* body = *it;
		const char* name = m_names[index].c_str();

		s.registerNameForPointer(body, name);
		btMultiBodyLinkCollider* mbc = dynamic_cast<btMultiBodyLinkCollider*>(body);
		if (mbc)
		{
			btMultiBody* mb = mbc->m_multiBody;
			if (mbc->m_link < 0)
			{
				mb->setBaseName(name);
			}
			else
			{
				mb->getLink(mbc->m_link).m_jointName = name;
            }
			if (exporting)
			{
				s.registerNameForPointer(name, name);
			}
		}
		++index;
	}
}


URDFConverter::URDFConverter(bool isMultiBody, CommonFileIOInterface* fileIO)
	: CommonMultiBodyBase(NULL),
	  m_fileIO(fileIO),
	  m_dynamicsWorld(NULL),
	  m_ownWorld(false),
	  m_useMultiBody(isMultiBody)
{
}

URDFConverter::~URDFConverter()
{
	if (m_dynamicsWorld && m_ownWorld)
	{
		delete m_dynamicsWorld;
	}
}

btMultiBodyDynamicsWorld* URDFConverter::importPhysics(const char* urdfXMLData, btMultiBodyDynamicsWorld* world)
{
	m_urdfData = urdfXMLData;
	m_dynamicsWorld = world;
	initPhysics();
	return m_dynamicsWorld;
}

void URDFConverter::initPhysics()
{
	DummyGUIHelper gui;
	BulletURDFImporter u2b(&gui, 0, m_fileIO, 1, 0);
	bool loadOk = u2b.loadURDFString(m_urdfData.c_str());

	if (loadOk)
	{
		//printTree(u2b,u2b.getRootLinkIndex());
		//u2b.printTree();
		m_creator = new MultiBodyCreator(u2b);

		btTransform identityTrans;
		identityTrans.setIdentity();
		if (m_dynamicsWorld == NULL)
		{
			createWorld();
			m_ownWorld = true;
		}
		ConvertURDF2Bullet(u2b, *m_creator, identityTrans, m_dynamicsWorld, m_useMultiBody, u2b.getPathPrefix());
	}
}

void URDFConverter::registerNames(btSerializer& s, bool exporting = false)
{
	m_creator->registerNames(s, exporting);
}

bool URDFConverter::exportPhysics(const char* bulletFile, btSerializer* serializer)
{
	FILE* f = fopen(bulletFile, "wb");

	if (f == NULL)
	{
		return false;
	}
	btSerializer* s = serializer ? serializer : new btDefaultSerializer;
	registerNames(*s, true);
	m_dynamicsWorld->serialize(s);
	fwrite(s->getBufferPointer(), s->getCurrentBufferSize(), 1, f);
	fclose(f);
	if (s != serializer)
    {
	    delete s;
    }
	return true;
}

btMultiBodyDynamicsWorld* URDFConverter::createWorld()
{
	btDefaultCollisionConfiguration* colconfig = new btDefaultCollisionConfiguration();
	MyOverlapFilterCallback2* filterCallback = new MyOverlapFilterCallback2();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(colconfig);
	btHashedOverlappingPairCache* pairCache = new btHashedOverlappingPairCache();

	pairCache->setOverlapFilterCallback(filterCallback);
	btDbvtBroadphase* broadPhase = new btDbvtBroadphase(pairCache);  //btSimpleBroadphase();
	btMultiBodyConstraintSolver* solver = new btMultiBodyConstraintSolver;
	m_dynamicsWorld = new btMultiBodyDynamicsWorld(dispatcher, broadPhase, m_solver, colconfig);
	m_dynamicsWorld->setGravity(btVector3(0, -9.81, 0));
	return m_dynamicsWorld;
}

