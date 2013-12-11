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
	
	shared_ptr<GameComponent> Enterprise = make_shared<GameComponent>();
	Enterprise->Attach(Content::LoadModel("USSEnterprise", glm::rotate(glm::mat4(1), 180.0f, glm::vec3(0,1,0))));
	Enterprise->position = glm::vec3(10, 5, 10);
	Enterprise->scale = glm::vec3(5,5,5);
	Enterprise->diffuse = glm::vec3(1,0,0);
	Attach(Enterprise);
	
	shared_ptr<FountainEffect> jet = make_shared<FountainEffect>(500);
	jet->position.x = jet->position.y = 0;
	jet->position.y = 0;
	jets.push_back(jet);
	Attach(jet);

	mass = 1.0f;
	dforce = 4000.0f;

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
	for (int i = 0 ; i < jets.size() ; i ++)
	{
		glm::vec3 toBody = body->position - jets[i]->position;
		if (glm::length(toBody) < 5)
		{
			body->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::basisUp * dforce));
		}
	}

	dynamicsWorld->stepSimulation(timeDelta,200);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

