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
#include "FountainEffect.h"
#include "SnowEffect.h"

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
	//soundSystem->PlaySoundW("apollo13",glm::vec3(0,0,0));

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
	
	truster = physicsFactory->CreateFromModel("USSEnterprise",glm::vec3(-10,0,-40),glm::quat(),glm::vec3(10,10,10));

	shared_ptr<FountainEffect> jet1 = make_shared<FountainEffect>(500);
	jet1->position = truster->position;
	Ljets.push_back(jet1);
	truster->Attach(jet1);

	shared_ptr<FountainEffect> jet2 = make_shared<FountainEffect>(500);
	jet2->position = truster->position;
	Rjets.push_back(jet2);
	truster->Attach(jet2);

	mass = 1.0f;
	dforce = 10;

	body = physicsFactory->CreateRagDoll(0.5f,0.5f,2,glm::vec3(0,0,0));
	body->velocity = glm::vec3(0,0,0);

	shared_ptr<SnowEffect> snow = make_shared<SnowEffect>();
	Attach(snow);

	if (!Game::Initialise()) {
		return false;
	}

	camera->GetController()->position = glm::vec3(0,6, 0);
	
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	for (int i = 0 ; i < Ljets.size() ; i ++)
	{
		Ljets[i]->position.x = truster->position.x + 15;
		Ljets[i]->position.y = truster->position.y + 30;
		Ljets[i]->position.z = truster->position.z - 40;

		Rjets[i]->position.x = truster->position.x - 15;
		Rjets[i]->position.y = truster->position.y + 30;
		Rjets[i]->position.z = truster->position.z - 40;

		if(truster->position.y < 40)
		{
			truster->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::basisUp * dforce));
			truster->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-look * dforce));
		}

		if (keyState[SDL_SCANCODE_RIGHT])
		{
			truster->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::right * (dforce )));
		}

		if (keyState[SDL_SCANCODE_LEFT])
		{
			truster->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-GameComponent::right * (dforce )));
		}
	}
	
	dynamicsWorld->stepSimulation(timeDelta,20);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

