/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2015 Google Inc. http://bulletphysics.org

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#include "BasicExample.h"

#include "btBulletDynamicsCommon.h"

#include "LinearMath/btVector3.h"
#include "LinearMath/btAlignedObjectArray.h"

#include "../CommonInterfaces/CommonRigidBodyBase.h"
#include <iostream>


struct BasicExample : public CommonRigidBodyBase
{
	BasicExample(struct GUIHelperInterface* helper)
		: CommonRigidBodyBase(helper)
	{
	}
	virtual ~BasicExample() {}
	virtual void initPhysics();
	virtual void renderScene();
	void resetCamera()
	{
		float dist = 4;
		float pitch = -35;
		float yaw = 52;
		float targetPos[3] = {0, 0, 0};
		m_guiHelper->resetCamera(dist, yaw, pitch, targetPos[0], targetPos[1], targetPos[2]);
	}
};



static bool CustomMaterialCombinerCallback(btManifoldPoint& cp,	const btCollisionObjectWrapper* colObj0,int partId0,int index0,const btCollisionObjectWrapper* colObj1,int partId1,int index1)
{

	//std::cout << "in callback" << std::endl;
	// No longer enabled 
	// https://github.com/bulletphysics/bullet3/blob/daadfacfff365852ffc96f373c834216a25b11e5/src/BulletCollision/NarrowPhaseCollision/btManifoldPoint.h#L123
	//cp.m_lateralFrictionInitialized = true;
	
	cp.m_contactPointFlags |= btContactPointFlags::BT_CONTACT_FLAG_LATERAL_FRICTION_INITIALIZED;
	///choose a target velocity in the friction dir1 direction, for a conveyor belt effect
	cp.m_lateralFrictionDir1.setValue(1.0,0.0,0.0);
	cp.m_lateralFrictionDir1.normalize();

	///optionally downscale the friction direction 2 for lower (anisotropic) friction (rather than a unit vector)
	btScalar downscale = 1.f;
	cp.m_lateralFrictionDir2 = downscale*cp.m_lateralFrictionDir1.cross(cp.m_normalWorldOnB);
	
	cp.m_contactMotion1 = 1.f;
	//cp.m_contactCFM2 = 0.1;
	//cp.m_combinedFriction = 10;
	//cp.m_combinedRestitution = calculateCombinedRestitution(restitution0,restitution1);
	return true;
}






void BasicExample::initPhysics()
{
	gContactAddedCallback = CustomMaterialCombinerCallback;

	m_guiHelper->setUpAxis(2);

	createEmptyDynamicsWorld();
	//
	m_dynamicsWorld->setGravity(btVector3(btScalar(0.), btScalar(0.), btScalar(-10)));
	m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_DISABLE_VELOCITY_DEPENDENT_FRICTION_DIRECTION+SOLVER_USE_2_FRICTION_DIRECTIONS;
	m_dynamicsWorld->getSolverInfo().m_solverMode |= SOLVER_ENABLE_FRICTION_DIRECTION_CACHING;



	//m_dynamicsWorld->setGravity(btVector3(0,0,0));
	m_guiHelper->createPhysicsDebugDrawer(m_dynamicsWorld);

	if (m_dynamicsWorld->getDebugDrawer())
		m_dynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe + btIDebugDraw::DBG_DrawContactPoints);

	///create a few basic rigid bodies
	btBoxShape* groundShape = createBoxShape(btVector3(btScalar(1.), btScalar(0.5), btScalar(0.5)));

	m_collisionShapes.push_back(groundShape);

	btTransform groundTransform;
	groundTransform.setIdentity();
	groundTransform.setOrigin(btVector3(btScalar(0), btScalar(0), btScalar(0.5)));

	{
		btScalar mass(0.);
		auto body = createRigidBody(mass, groundTransform, groundShape, btVector4(0, 0, 0, 1));
		
		//enable custom material callback
		body->setCollisionFlags(body->getCollisionFlags()  | btCollisionObject::CF_CUSTOM_MATERIAL_CALLBACK);
	}

	{
		btBoxShape* boxShape = createBoxShape(btVector3(btScalar(0.1), btScalar(0.1), btScalar(0.1)));
		m_collisionShapes.push_back(boxShape);

		
		btTransform boxTransform;
		boxTransform.setIdentity();
		boxTransform.setOrigin(btVector3(btScalar(0), btScalar(0), btScalar(2.0)));
		
		auto body = createRigidBody(0.05, boxTransform, boxShape, btVector4(0, 0, 0, 1));
	}

	m_guiHelper->autogenerateGraphicsObjects(m_dynamicsWorld);
}

void BasicExample::renderScene()
{
	CommonRigidBodyBase::renderScene();
}

CommonExampleInterface* BasicExampleCreateFunc(CommonExampleOptions& options)
{
	return new BasicExample(options.m_guiHelper);
}

B3_STANDALONE_EXAMPLE(BasicExampleCreateFunc)
