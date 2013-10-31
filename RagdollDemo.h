/*
Bullet Continuous Collision Detection and Physics Library
RagdollDemo
Copyright (c) 2007 Starbreeze Studios

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.

Written by: Marten Svanfeldt
*/

#ifndef RAGDOLLDEMO_H
#define RAGDOLLDEMO_H

#include "GlutDemoApplication.h"
#include "LinearMath/btAlignedObjectArray.h"
class btBroadphaseInterface;
class btCollisionShape;
class btOverlappingPairCache;
class btCollisionDispatcher;
class btConstraintSolver;
struct btCollisionAlgorithmCreateFunc;
class btDefaultCollisionConfiguration;

class RagdollDemo : public GlutDemoApplication
{

	btAlignedObjectArray<class RagDoll*> m_ragdolls;

	//keep the collision shapes, for deletion/cleanup
	btAlignedObjectArray<btCollisionShape*>	m_collisionShapes;

	btBroadphaseInterface*	m_broadphase;

	btCollisionDispatcher*	m_dispatcher;

	btConstraintSolver*	m_solver;

	btDefaultCollisionConfiguration* m_collisionConfiguration;

public:
	btCollisionShape* spheres_shape[16];
	btRigidBody* spheres_body[16];
	btCollisionShape* windball_shape;
	btRigidBody* windball_body;
	bool isFloating[16];
	int maxTimeStep;

	int timeStep;
	int windDirection;
	btRigidBody* flag_pole;

	btCollisionShape* flag_shape;
	btRigidBody*	flag_body;
	//btHingeConstraint* flagjoint;


	bool pause;
	
	btRigidBody* body[19];
	btCollisionShape* geom[19];


	void initPhysics();

	void exitPhysics();

	virtual ~RagdollDemo()
	{
		exitPhysics();
	}

	void spawnRagdoll(const btVector3& startOffset);

	virtual void clientMoveAndDisplay();

	virtual void displayCallback();

	virtual void keyboardCallback(unsigned char key, int x, int y);

	static DemoApplication* Create()
	{
		RagdollDemo* demo = new RagdollDemo();
		demo->myinit();
		demo->initPhysics();
		return demo;
	}

	void CreateBox(int index, double x, double y, double z, double length, double width, double height);
	void CreateCylinder(int index,double x, double y, double z,double radius);
	btRigidBody* CreateCylinder2(double x, double y, double z,double radius, double length, double eulerX, double eulerY, double eulerZ);

	void levitateBall(int i);
	void ResetGravity(int i);
	void dropBall(int i);

	btVector3 PointWorldToLocal(btRigidBody* body, btVector3& p);
	btVector3 AxisWorldToLocal(btRigidBody* body, btVector3& a);

	void CreateSpheres();
	void CreateSphereBox();

	void CreateHinge(btRigidBody* bodyA, btRigidBody* bodyB, const btVector3& axisInA, const btVector3& axisInB,
		const btVector3& pivotInA, const btVector3& pivotInB);
};


#endif
