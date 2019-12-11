#ifndef URDF_CONVERTER_H
#define URDF_CONVERTER_H

#include <string>
#include <vector>
#include "CommonInterfaces/CommonFileIOInterface.h"
#include "CommonInterfaces/CommonMultiBodyBase.h"
#include "MultiBodyCreationInterface.h"

class btMultiBodyDynamicsWorld;
class btSerializer;
class btRigidBody;
class btTransform;
class btVector3;
class btCollisionObject;
class btMultiBodyLinkCollider;
class btGeneric6DofSpring2Constraint;
class URDFImporterInterface;


class MultiBodyCreator : public MultiBodyCreationInterface
{
	const URDFImporterInterface& m_urdfImport;
	std::vector<btCollisionObject*> m_bodies;
	std::vector<std::string> m_names;
	btMultiBody* m_bulletMultiBody;
	btRigidBody* m_rigidBody;
	btAlignedObjectArray<int> m_mb2urdfLink;
	btAlignedObjectArray<btGeneric6DofSpring2Constraint*> m_6DofConstraints;

public:
	MultiBodyCreator(const URDFImporterInterface& u2b);
	void registerNames(btSerializer& s, bool exporting = false);
	btRigidBody* allocateRigidBody(int urdfLinkIndex, btScalar mass,
								   const btVector3& localInertiaDiagonal,
								   const btTransform& initialWorldTrans,
								   class btCollisionShape* colShape);
	btMultiBody* allocateMultiBody(int urdfLinkIndex, int totalNumJoints,
								   btScalar mass, const btVector3& localInertiaDiagonal,
								   bool isFixedBase, bool canSleep);
	btMultiBodyLinkCollider* allocateMultiBodyLinkCollider(int /*urdfLinkIndex*/,
														   int mbLinkIndex, btMultiBody* multiBody);
	btGeneric6DofSpring2Constraint* allocateGeneric6DofSpring2Constraint(int urdfLinkIndex,
																		 btRigidBody& rbA /*parent*/,
																		 btRigidBody& rbB,
																		 const btTransform& offsetInA,
																		 const btTransform& offsetInB,
																		 int rotateOrder = 0);
	btGeneric6DofSpring2Constraint* createPrismaticJoint(int urdfLinkIndex,
														 btRigidBody& rbA /*parent*/,
														 btRigidBody& rbB,
														 const btTransform& offsetInA,
														 const btTransform& offsetInB,
														 const btVector3& jointAxisInJointSpace,
														 btScalar jointLowerLimit, btScalar jointUpperLimit);
	btGeneric6DofSpring2Constraint* createRevoluteJoint(int urdfLinkIndex,
														btRigidBody& rbA /*parent*/,
														btRigidBody& rbB,
														const btTransform& offsetInA,
														const btTransform& offsetInB,
														const btVector3& jointAxisInJointSpace,
														btScalar jointLowerLimit, btScalar jointUpperLimit);
	btGeneric6DofSpring2Constraint* createFixedJoint(int urdfLinkIndex,
													 btRigidBody& rbA /*parent*/,
													 btRigidBody& rbB,
													 const btTransform& offsetInA,
													 const btTransform& offsetInB);
	void addLinkMapping(int urdfLinkIndex, int mbLinkIndex);
	void createRigidBodyGraphicsInstance(int linkIndex, btRigidBody* body, const btVector3& colorRgba, int graphicsIndex) {}

	void createRigidBodyGraphicsInstance2(int linkIndex, class btRigidBody* body,
										  const btVector3& colorRgba, const btVector3& specularColor, int graphicsIndex) {}

	void createCollisionObjectGraphicsInstance(int linkIndex, class btCollisionObject* colObj, const btVector3& colorRgba) {}

	void createCollisionObjectGraphicsInstance2(int linkIndex, class btCollisionObject* col,
												const btVector4& colorRgba, const btVector3& specularColor) {}

	btMultiBody* getBulletMultiBody() { return m_bulletMultiBody; }

private:
	const std::string& registerNameForPointer(int urdfIndex, btCollisionObject* ptr, const std::string& name);
};

class URDFConverter : public CommonMultiBodyBase
{
	std::string m_urdfData;
	bool m_useMultiBody;
	bool m_ownWorld;
	CommonFileIOInterface* m_fileIO;
	btMultiBodyDynamicsWorld* m_dynamicsWorld;
	MultiBodyCreator* m_creator;

public:
	URDFConverter(bool isMultiBody, CommonFileIOInterface* fileIO);
	~URDFConverter();

	btMultiBodyDynamicsWorld* importPhysics(const char* urdfData, btMultiBodyDynamicsWorld* world);
	bool exportPhysics(const char* bulletFileName, btSerializer* serializer = nullptr);
	void initPhysics();
	void registerNames(btSerializer& s, bool exporting);

private:
	btMultiBodyDynamicsWorld* createWorld();
};
#endif  //URDF_CONVERTER_H
