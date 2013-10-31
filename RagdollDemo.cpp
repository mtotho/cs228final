 /*
Bullet Continuous Collision Detection and Physics Library
Ragdoll Demo
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

#define CONSTRAINT_DEBUG_SIZE 0.2f
int ct = 0;

#include "btBulletDynamicsCommon.h"
#include "GlutStuff.h"
#include "GL_ShapeDrawer.h"

#include "LinearMath/btIDebugDraw.h"

#include "GLDebugDrawer.h"
#include "RagdollDemo.h"


// Enrico: Shouldn't these three variables be real constants and not defines?

#ifndef M_PI
#define M_PI       3.14159265358979323846
#endif

#ifndef M_PI_2
#define M_PI_2     1.57079632679489661923
#endif

#ifndef M_PI_4
#define M_PI_4     0.785398163397448309616
#endif


class RagDoll
{
	enum
	{
		BODYPART_PELVIS = 0,
		BODYPART_SPINE,
		BODYPART_HEAD,

		BODYPART_LEFT_UPPER_LEG,
		BODYPART_LEFT_LOWER_LEG,

		BODYPART_RIGHT_UPPER_LEG,
		BODYPART_RIGHT_LOWER_LEG,

		BODYPART_LEFT_UPPER_ARM,
		BODYPART_LEFT_LOWER_ARM,

		BODYPART_RIGHT_UPPER_ARM,
		BODYPART_RIGHT_LOWER_ARM,

		BODYPART_COUNT
	};

	enum
	{
		JOINT_PELVIS_SPINE = 0,
		JOINT_SPINE_HEAD,

		JOINT_LEFT_HIP,
		JOINT_LEFT_KNEE,

		JOINT_RIGHT_HIP,
		JOINT_RIGHT_KNEE,

		JOINT_LEFT_SHOULDER,
		JOINT_LEFT_ELBOW,

		JOINT_RIGHT_SHOULDER,
		JOINT_RIGHT_ELBOW,

		JOINT_COUNT
	};

	btDynamicsWorld* m_ownerWorld;
	btCollisionShape* m_shapes[BODYPART_COUNT];
	btRigidBody* m_bodies[BODYPART_COUNT];
	btTypedConstraint* m_joints[JOINT_COUNT];

	btRigidBody* localCreateRigidBody (btScalar mass, const btTransform& startTransform, btCollisionShape* shape)
	{
		bool isDynamic = (mass != 0.f);

		btVector3 localInertia(0,0,0);
		if (isDynamic)
			shape->calculateLocalInertia(mass,localInertia);

		btDefaultMotionState* myMotionState = new btDefaultMotionState(startTransform);
		
		btRigidBody::btRigidBodyConstructionInfo rbInfo(mass,myMotionState,shape,localInertia);
		btRigidBody* body = new btRigidBody(rbInfo);

		m_ownerWorld->addRigidBody(body);

		return body;
	}

public:
	RagDoll (btDynamicsWorld* ownerWorld, const btVector3& positionOffset)
		: m_ownerWorld (ownerWorld)
	{
		// Setup the geometry
		m_shapes[BODYPART_PELVIS] = new btCapsuleShape(btScalar(0.15), btScalar(0.20));
		m_shapes[BODYPART_SPINE] = new btCapsuleShape(btScalar(0.15), btScalar(0.28));
		m_shapes[BODYPART_HEAD] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));
		m_shapes[BODYPART_LEFT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_LEFT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_RIGHT_UPPER_LEG] = new btCapsuleShape(btScalar(0.07), btScalar(0.45));
		m_shapes[BODYPART_RIGHT_LOWER_LEG] = new btCapsuleShape(btScalar(0.05), btScalar(0.37));
		m_shapes[BODYPART_LEFT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_LEFT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));
		m_shapes[BODYPART_RIGHT_UPPER_ARM] = new btCapsuleShape(btScalar(0.05), btScalar(0.33));
		m_shapes[BODYPART_RIGHT_LOWER_ARM] = new btCapsuleShape(btScalar(0.04), btScalar(0.25));

		// Setup all the rigid bodies
		btTransform offset; offset.setIdentity();
		offset.setOrigin(positionOffset);

		btTransform transform;
		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.), btScalar(0.)));
		m_bodies[BODYPART_PELVIS] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_PELVIS]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.2), btScalar(0.)));
		m_bodies[BODYPART_SPINE] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_SPINE]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.), btScalar(1.6), btScalar(0.)));
		m_bodies[BODYPART_HEAD] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_HEAD]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_LEFT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_LEFT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.65), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_UPPER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_LEG]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.18), btScalar(0.2), btScalar(0.)));
		m_bodies[BODYPART_RIGHT_LOWER_LEG] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_LEG]);

		transform.setOrigin(btVector3(btScalar(-0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(-0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,M_PI_2);
		m_bodies[BODYPART_LEFT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_LEFT_LOWER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.35), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_UPPER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_UPPER_ARM]);

		transform.setIdentity();
		transform.setOrigin(btVector3(btScalar(0.7), btScalar(1.45), btScalar(0.)));
		transform.getBasis().setEulerZYX(0,0,-M_PI_2);
		m_bodies[BODYPART_RIGHT_LOWER_ARM] = localCreateRigidBody(btScalar(1.), offset*transform, m_shapes[BODYPART_RIGHT_LOWER_ARM]);

		// Setup some damping on the m_bodies
		for (int i = 0; i < BODYPART_COUNT; ++i)
		{
			m_bodies[i]->setDamping(0.05, 0.85);
			m_bodies[i]->setDeactivationTime(0.8);
			m_bodies[i]->setSleepingThresholds(1.6, 2.5);
		}

		//Optional Wind Dimensions
		//int i;
		//for (i=m_shapes->getNumCollisionObjects()-1; i>=0 ;i--)
		//{
		  // btCollisionObject* obj = *m_shapes->getCollisionObjectArray()[i];
		  // btRigidBody* body = btRigidBody::upcast(obj);
		   //if(!body->isStaticObject())
		   //body->applyCentralForce(btVector3(10.f,0.f,0.f)); 
		//}

		// Now setup the constraints
		btHingeConstraint* hingeC;
		btConeTwistConstraint* coneC;

		btTransform localA, localB;

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.15), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_SPINE], localA, localB);
		hingeC->setLimit(btScalar(-M_PI_4), btScalar(M_PI_2));
		m_joints[JOINT_PELVIS_SPINE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_PELVIS_SPINE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI_2); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.30), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_HEAD], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, M_PI_2);
		m_joints[JOINT_SPINE_HEAD] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_SPINE_HEAD], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,-M_PI_4*5); localA.setOrigin(btVector3(btScalar(-0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,-M_PI_4*5); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_LEFT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_LEFT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_LEG], *m_bodies[BODYPART_LEFT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		  localA.getBasis().setEulerZYX(0,0,M_PI_4); localA.setOrigin(btVector3(btScalar(0.18), btScalar(-0.10), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_4); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.225), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_PELVIS], *m_bodies[BODYPART_RIGHT_UPPER_LEG], localA, localB);
		coneC->setLimit(M_PI_4, M_PI_4, 0);
		m_joints[JOINT_RIGHT_HIP] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_HIP], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(-0.225), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(0.185), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_LEG], *m_bodies[BODYPART_RIGHT_LOWER_LEG], localA, localB);
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_KNEE] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_KNEE], true);


		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,M_PI); localA.setOrigin(btVector3(btScalar(-0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_LEFT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_joints[JOINT_LEFT_SHOULDER] = coneC;
		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_LEFT_UPPER_ARM], *m_bodies[BODYPART_LEFT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_LEFT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_LEFT_ELBOW], true);



		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,0,0); localA.setOrigin(btVector3(btScalar(0.2), btScalar(0.15), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,0,M_PI_2); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.18), btScalar(0.)));
		coneC = new btConeTwistConstraint(*m_bodies[BODYPART_SPINE], *m_bodies[BODYPART_RIGHT_UPPER_ARM], localA, localB);
		coneC->setLimit(M_PI_2, M_PI_2, 0);
		m_joints[JOINT_RIGHT_SHOULDER] = coneC;
		coneC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);

		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_SHOULDER], true);

		localA.setIdentity(); localB.setIdentity();
		localA.getBasis().setEulerZYX(0,M_PI_2,0); localA.setOrigin(btVector3(btScalar(0.), btScalar(0.18), btScalar(0.)));
		localB.getBasis().setEulerZYX(0,M_PI_2,0); localB.setOrigin(btVector3(btScalar(0.), btScalar(-0.14), btScalar(0.)));
		hingeC =  new btHingeConstraint(*m_bodies[BODYPART_RIGHT_UPPER_ARM], *m_bodies[BODYPART_RIGHT_LOWER_ARM], localA, localB);
//		hingeC->setLimit(btScalar(-M_PI_2), btScalar(0));
		hingeC->setLimit(btScalar(0), btScalar(M_PI_2));
		m_joints[JOINT_RIGHT_ELBOW] = hingeC;
		hingeC->setDbgDrawSize(CONSTRAINT_DEBUG_SIZE);
		
		m_ownerWorld->addConstraint(m_joints[JOINT_RIGHT_ELBOW], true);
	}

	virtual	~RagDoll ()
	{
		int i;

		// Remove all constraints
		for ( i = 0; i < JOINT_COUNT; ++i)
		{
			m_ownerWorld->removeConstraint(m_joints[i]);
			delete m_joints[i]; m_joints[i] = 0;
		}

		// Remove all bodies and shapes
		for ( i = 0; i < BODYPART_COUNT; ++i)
		{
			m_ownerWorld->removeRigidBody(m_bodies[i]);
			
			delete m_bodies[i]->getMotionState();

			delete m_bodies[i]; m_bodies[i] = 0;
			delete m_shapes[i]; m_shapes[i] = 0;
		}
	}
};

void RagdollDemo::CreateSphereBox(){
	//       index x y z width height length

	int cage_height = 2;
	
	int box1offset = 9;
	int box2offset = -9;
	
	//cage 1     x                          y       z       width  height           legnth
	CreateBox(0, box1offset + 0,			0,		0, 		6.5, 		cage_height,    0.25); //Back
	CreateBox(1, box1offset + -6.625,	    0,     -6.5, 		0.25,   cage_height, 	6.75); //Right
	CreateBox(2, box1offset + 0,			0,     -13, 	6.5, 		cage_height, 	0.25); //front
	CreateBox(3, box1offset + 6.625,		0,     -6.5, 		0.25, 	cage_height, 	6.75); //LEFT

	
					// x                Y    Z     Width  height        length
	CreateBox(4, box2offset + 0,        0,   0,    6.5,     cage_height,	 0.25); //Back
	CreateBox(5, box2offset + -6.625,   0,  -6.5,    0.25,  cage_height,   6.75); //Right
	CreateBox(6, box2offset + 0,        0,  -13,   6.5,     cage_height,   0.25); //front
	CreateBox(7, box2offset + 6.625,    0,  -6.5,    0.25,  cage_height,   6.75); //LEFT
	
	//Create box under a ball. 
	CreateBox(8, 13.5,					0,	-2,		1,		.001,			1); //bottom selector box

}

void RagdollDemo::CreateSpheres(){
	//spheres[0] = new btCapsuleShape(btScalar(0.10), btScalar(0.05));

	//             index    x    y     z    radius
	//Left most column
	CreateCylinder(0, 		13.5,	 0,	   -2, 	  1);
	CreateCylinder(1, 		13.5,	 0,	   -5, 	  1);
	CreateCylinder(2, 		13.5,	 0,	   -8, 	  1);
	CreateCylinder(3, 		13.5,	 0,	   -11,   1);

	//second to left
	CreateCylinder(4, 		10.5,	 0,	   -2, 	  1);
	CreateCylinder(5, 		10.5,	 0,	   -5, 	  1);
	CreateCylinder(6, 		10.5,	 0,	   -8, 	  1);
	CreateCylinder(7, 		10.5,	 0,	   -11,   1);

	//third from left
	CreateCylinder(8, 		7.5,	 0,	   -2, 	  1);
	CreateCylinder(9, 		7.5,	 0,	   -5, 	  1);
	CreateCylinder(10, 		7.5,	 0,	   -8, 	  1);
	CreateCylinder(11, 		7.5,	 0,	   -11,   1);

	//right most column
	CreateCylinder(12, 		4.5,	 0,	   -2, 	  1);
	CreateCylinder(13, 		4.5,	 0,	   -5, 	  1);
	CreateCylinder(14, 		4.5,	 0,	   -8, 	  1);
	CreateCylinder(15, 		4.5,	 0,	   -11,   1);


	//create windball
	windball_shape= new btCapsuleShape(btScalar(1),btScalar(0));
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(0),btScalar(0),btScalar(-15)));

	btTransform transform;
	transform.setOrigin(btVector3(btScalar(0),btScalar(1),btScalar(0)));
	transform.getBasis().setEulerZYX(1,0,0); 

	windball_body= localCreateRigidBody(btScalar(1.0),offset*transform,windball_shape);


}

//This is actually create Sphere. will change
void RagdollDemo::CreateCylinder(int index,double x, double y, double z,double radius){
	
	//	geom[index] = new btCylinderShape(btVector3(btScalar(radius),btScalar(length),btScalar(0)));

	spheres_shape[index] = new btCapsuleShape(btScalar(radius),btScalar(0));
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z)));

	btTransform transform;
	transform.setOrigin(btVector3(btScalar(0),btScalar(1),btScalar(0)));
	transform.getBasis().setEulerZYX(1,0,0); 

	spheres_body[index] = localCreateRigidBody(btScalar(1.0),offset*transform,spheres_shape[index]);
	//spheres_bodybody[index]->setUserPointer(&IDs[index]);
}


//Note this is the real create cylinder
btRigidBody* RagdollDemo::CreateCylinder2(double x, double y, double z,double radius, double length, double eulerX, double eulerY, double eulerZ){
	
	//geom[index] = new btCylinderShape(btVector3(btScalar(radius),btScalar(length),btScalar(0)));

	//geom[index] 
	btCollisionShape* cylinder_shape = new btCapsuleShape(btScalar(radius),btScalar(length));
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z)));

	btTransform transform;
	transform.setOrigin(btVector3(btScalar(0),btScalar(1),btScalar(0)));
	transform.getBasis().setEulerZYX(eulerX,eulerY,eulerZ); 

	//body[index] = 
	btRigidBody* cylinder_body =  localCreateRigidBody(btScalar(1.0),offset*transform,cylinder_shape);
	//body[index]->setUserPointer(&IDs[index]);

	cylinder_body->setMassProps(100, btVector3(0,0,0));

	return cylinder_body;
}

void RagdollDemo::CreateBox(int index, double x, double y, double z, double width, double height, double length){
	geom[index] = new btBoxShape(btVector3(btScalar(width),btScalar(height),btScalar(length))); 
	btTransform offset; 
	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(x),btScalar(y),btScalar(z))); 
	body[index] = localCreateRigidBody(btScalar(1.0),offset,geom[index]); 

	//body[index]->setUserPointer(&IDs[index]);

	//if(index>9){
		body[index]->setMassProps(100, btVector3(0,0,0));
//	}
}

void RagdollDemo::CreateHinge(btRigidBody* bodyA, btRigidBody* bodyB, const btVector3& axisInA, const btVector3& axisInB,
		const btVector3& pivotInA, const btVector3& pivotInB){
	
	btHingeConstraint* joint = new btHingeConstraint(*bodyA, *bodyB, pivotInA, pivotInB, axisInA, axisInB);
	//btHingeConstraint* tempHinge = new btHingeConstraint(*body[bodyAIndex], *body[bodyBIndex], pivotInA, pivotInB, axisInA, axisInB);
	m_dynamicsWorld->addConstraint(joint, true);
}


void RagdollDemo::initPhysics()
{
	// Setup the basic world
	pause = false;
	timeStep=0;
	setTexturing(true);
	setShadows(true);

	setCameraDistance(btScalar(25.));

	m_collisionConfiguration = new btDefaultCollisionConfiguration();

	m_dispatcher = new btCollisionDispatcher(m_collisionConfiguration);

	btVector3 worldAabbMin(-10000,-10000,-10000);
	btVector3 worldAabbMax(10000,10000,10000);
	m_broadphase = new btAxisSweep3 (worldAabbMin, worldAabbMax);

	m_solver = new btSequentialImpulseConstraintSolver;

	m_dynamicsWorld = new btDiscreteDynamicsWorld(m_dispatcher,m_broadphase,m_solver,m_collisionConfiguration);
	//m_dynamicsWorld->getDispatchInfo().m_useConvexConservativeDistanceUtil = true;
	//m_dynamicsWorld->getDispatchInfo().m_convexConservativeDistanceThreshold = 0.01f;

	m_dynamicsWorld->setGravity(btVector3(0,-10,0));

	// Setup a big ground box
	{
		btCollisionShape* groundShape = new btBoxShape(btVector3(btScalar(200.),btScalar(10.),btScalar(200.)));
		m_collisionShapes.push_back(groundShape);
		btTransform groundTransform;
		groundTransform.setIdentity();
		groundTransform.setOrigin(btVector3(0,-10,0));

#define CREATE_GROUND_COLLISION_OBJECT 1
#ifdef CREATE_GROUND_COLLISION_OBJECT
		btCollisionObject* fixedGround = new btCollisionObject();
		fixedGround->setCollisionShape(groundShape);
		fixedGround->setWorldTransform(groundTransform);
		m_dynamicsWorld->addCollisionObject(fixedGround);
#else
		localCreateRigidBody(btScalar(0.),groundTransform,groundShape);
#endif //CREATE_GROUND_COLLISION_OBJECT

	}

	// Spawn one ragdoll
	btVector3 startOffset(1,0.5,0);
	//spawnRagdoll(startOffset);
	startOffset.setValue(-1,0.5,0);
	//spawnRagdoll(startOffset);

	//Create the box for the spheres
	CreateSphereBox();
	CreateSpheres();

	//CreateCylinder2(double x, double y, double z,double radius, double length, double eulerX, double eulerY, double eulerZ)
	//Create Flag pole
	int flagx=-4;
	int flagy=13;
	int flagz=6;
	int flagwidth=4;
	int flagheight=2;
	double flaglength=0.15;

	flag_pole = CreateCylinder2(0, 0, flagz, 0.25, 14, 0, 0, 0);

	//Create flag (put this into function later)


	flag_shape = new btBoxShape(btVector3(btScalar(flagwidth),btScalar(flagheight),btScalar(flaglength))); 
	btTransform offset; 

	offset.setIdentity(); 
	offset.setOrigin(btVector3(btScalar(flagx),btScalar(flagy),btScalar(flagz))); 
	flag_body = localCreateRigidBody(btScalar(1.0),offset,flag_shape); 

	//body[index]->setUserPointer(&IDs[index]);

	//if(index>9){
	
		flag_body->setActivationState(DISABLE_DEACTIVATION);
	//CreateHinge(btRigidBody* bodyA, btRigidBody* bodyB, const btVector3& axisInA, const btVector3& axisInB,
		//const btVector3& pivotInA, const btVector3& pivotInB);


		//btVector3 axisInA = btTransform local1 = body->getCenterOfMassTransform().inverse();
		//local1*p;

		//btTransform local1 = body->getCenterOfMassTransform().inverse();
	//return local1*p;

	btHingeConstraint* flagjoint = new  btHingeConstraint(*flag_pole, *flag_body ,btVector3(0, 6, 0), btVector3(4, 1, 0), btVector3(0,1,0),  btVector3(0,1,0));
	
	//btHingeConstraint* flagjoint = new  btHingeConstraint(*flag_body, btVector3(4, 1, 0),  btVector3(0,-1,0));
	m_dynamicsWorld->addConstraint(flagjoint, true);
	

	flagjoint->setLimit(M_PI/15, M_PI-M_PI/15);

	//reset some ball stuff
	ct=0;


	clientResetScene();


	for(int i=0; i<16; i++){
		//ResetGravity(i);
		isFloating[i]=false;
	}
		
}

void RagdollDemo::spawnRagdoll(const btVector3& startOffset)
{
	RagDoll* ragDoll = new RagDoll (m_dynamicsWorld, startOffset);
	m_ragdolls.push_back(ragDoll);
}	

void RagdollDemo::clientMoveAndDisplay()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	//simple dynamics world doesn't handle fixed-time-stepping
	float ms = getDeltaTimeMicroseconds();

	float minFPS = 1000000.f/60.f;
	if (ms > minFPS)
		ms = minFPS;

	if (m_dynamicsWorld)
	{
		if(!pause){
				
				float windSpeed = 10;
				int downwardGravity=-30; //whent he ball gets over the right box
				//int windballSpeed = 0;
				maxTimeStep=1200;

				timeStep++;
				timeStep=timeStep % maxTimeStep;
				
				if(timeStep<maxTimeStep/2){

					windDirection=-1; //-1 for right
					

				}else{
					windDirection=1;
				}



				 printf("%d\n", timeStep);
							int i;

				//control the windball movement
			//	if(windDirection==-1){
					windball_body->applyCentralForce(btVector3(windDirection*1.5, 0.f, 0.f));
			//	}else{
				//	windball_body->applyCentralForce(btVector3(3, 0.f, 0.f));
			//	}

				//Apply force to flag
				flag_body->activate(true);
				flag_body->applyCentralForce(btVector3(windDirection*windSpeed,0.f,0)); 

				for (i=0;i<16 ;i++)
				{
				   //btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
				  btRigidBody* body = spheres_body[i];// btRigidBody::upcast(obj);
				  
				  const btVector3 pos = body->getCenterOfMassPosition();
				  double posy = pos.getY();
				  double posx = pos.getX();

				  if(posy>=6 && posx >2){

					  if(!body->isStaticObject()){
					   body->applyCentralForce(btVector3(windDirection*windSpeed,0.f,0.f)); 
					  }
				  }//end if posy>6

				 
				  if(posx<0){
					body->setGravity(btVector3(btScalar(0),btScalar(downwardGravity),btScalar(0)));
					//body->applyCentralForce(btVector3(windDirection*2,0.f,0.f)); 
				  }
					  
				}//end for
				//	m_dynamicsWorld->stepSimulation(m_frameTimer.getDeltaTime());
				m_dynamicsWorld->stepSimulation(ms / 1000000.f);


		}
		
		//optional but useful: debug drawing
		m_dynamicsWorld->debugDrawWorld();


	}

	renderme(); 

	glFlush();

	glutSwapBuffers();
}

void RagdollDemo::displayCallback()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT); 

	renderme();

	//optional but useful: debug drawing
	if (m_dynamicsWorld)
		m_dynamicsWorld->debugDrawWorld();

	glFlush();
	glutSwapBuffers();

}

void RagdollDemo::keyboardCallback(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 'e':
		{
		btVector3 startOffset(0,2,0);
		spawnRagdoll(startOffset);
		break;
		}
	case 'p':
		{
			if(!pause){
				pause = true;
			}else{
				pause = false;
			}
			break;
		}
	case 'f':
		{
			if(isFloating[ct]){
				dropBall(ct);
			}else{
				levitateBall(ct);
			}
			break;
		}
	case 'n':
		{
			ct = (ct + 1)%16;
			//spheres_body[0]-
			break;
		}
	case 'b':
		{
			ct = ct - 1;
			break;
		}
	default:
		DemoApplication::keyboardCallback(key, x, y);
	}

	
}

btVector3 RagdollDemo::PointWorldToLocal(btRigidBody* body, btVector3& p)
{
	btTransform local1 = body->getCenterOfMassTransform().inverse();
	return local1*p;
}

btVector3 RagdollDemo::AxisWorldToLocal(btRigidBody* body, btVector3& a) {
	btTransform local1 = body->getCenterOfMassTransform().inverse();
	btVector3 zero(0,0,0);
	local1.setOrigin(zero);
	return local1*a;
}

void RagdollDemo::ResetGravity(int i){
	spheres_body[i]->activate(true);
	spheres_body[i]->setGravity(btVector3(btScalar(0),btScalar(0),btScalar(0)));
	isFloating[i]=false;
} 


void RagdollDemo::dropBall(int i){
	//spheres_body[i]->activate(false);
	spheres_body[i]->setGravity(btVector3(btScalar(0),btScalar(-2),btScalar(0)));
	isFloating[i]=false;
}

void RagdollDemo::levitateBall(int i)
{
	spheres_body[i]->activate(true);
	spheres_body[i]->setGravity(btVector3(btScalar(0),btScalar(1),btScalar(0)));

	isFloating[i]=true;
}

void	RagdollDemo::exitPhysics()
{

	int i;

	for (i=0;i<m_ragdolls.size();i++)
	{
		RagDoll* doll = m_ragdolls[i];
		delete doll;
	}

	//cleanup in the reverse order of creation/initialization

	//remove the rigidbodies from the dynamics world and delete them
	
	for (i=m_dynamicsWorld->getNumCollisionObjects()-1; i>=0 ;i--)
	{
		btCollisionObject* obj = m_dynamicsWorld->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		if (body && body->getMotionState())
		{
			delete body->getMotionState();
		}
		m_dynamicsWorld->removeCollisionObject( obj );
		delete obj;
	}

	//delete collision shapes
	for (int j=0;j<m_collisionShapes.size();j++)
	{
		btCollisionShape* shape = m_collisionShapes[j];
		delete shape;
	}

	//delete dynamics world
	delete m_dynamicsWorld;

	//delete solver
	delete m_solver;

	//delete broadphase
	delete m_broadphase;

	//delete dispatcher
	delete m_dispatcher;

	delete m_collisionConfiguration;

	ct=0; 
	for(int i=0; i<16; i++){
		//ResetGravity(i);
		isFloating[i]=false;
	}

	
}





