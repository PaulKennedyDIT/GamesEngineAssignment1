#include "Assignment.h"
#include "PhysicsController.h"
#include "Sphere.h"
#include "PhysicsCamera.h"
#include "Box.h"
#include "Cylinder.h"
#include "Steerable3DController.h"
#include "Ground.h"
#include "Content.h"
#include <btBulletDynamicsCommon.h>
#include <gtc/quaternion.hpp>
#include <gtx/quaternion.hpp>
#include <gtx/euler_angles.hpp>
#include <gtx/norm.hpp>
#include "VectorDrawer.h"
#include "Utils.h"

using namespace BGE;

Assignment::Assignment(void)
{
	physicsFactory = NULL;
	dynamicsWorld = NULL;
	broadphase = NULL;
	dispatcher = NULL;
	solver = NULL;
	fullscreen = false;

}

Assignment::~Assignment(void)
{
}

bool Assignment::Initialise() 
{	
	riftEnabled = false;
	// Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
	soundSystem->PlaySoundW("sound",glm::vec3(0,0,0));

    // The world.
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	broadphase = new btAxisSweep3(worldMin,worldMax);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-9,0));

	
	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();
	physicsFactory->CreateFromModel("buddha",glm::vec3(-10,0,-40),glm::quat(),glm::vec3(10,10,10));
	physicsFactory->CreateFromModel("buddha",glm::vec3(0,0,-40),glm::quat(),glm::vec3(10,10,10));
	physicsFactory->CreateFromModel("buddha",glm::vec3(10,0,-40),glm::quat(),glm::vec3(10,10,10));
	if (!Game::Initialise()) {
		return false;
	}

	shared_ptr<PhysicsController> box1 = physicsFactory->CreateContainerWall(110,200,15, glm::vec3(0, 10, -50), glm::quat());
	shared_ptr<PhysicsController> box2 = physicsFactory->CreateContainerWall(110,200,15, glm::vec3(0, 10, 50), glm::quat());
	shared_ptr<PhysicsController> box3 = physicsFactory->CreateContainerWall(15,200,110, glm::vec3(50, 10, 0), glm::quat());
	shared_ptr<PhysicsController> box4 = physicsFactory->CreateContainerWall(15,200,110, glm::vec3(-50, 10, 0), glm::quat());
	camera->GetController()->position = glm::vec3(0,6, 0);
	
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	dynamicsWorld->stepSimulation(timeDelta,10);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}
