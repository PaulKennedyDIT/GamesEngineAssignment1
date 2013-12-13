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
	cameraLock = true;
	fireRate= 5.0f;
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
	physicsFactory->CreateFromModel("buddha",glm::vec3(-10,0,4000),glm::quat(),5,glm::vec3(100,100,100));
	
	USSEnterprise = physicsFactory->CreateFromModel("USSEnterprise",glm::vec3(0,50,0),glm::quat(),2,glm::vec3(1,1,1));

	mass = 1.0f;
	dforce = 100;
	t =0.0f;

	body = physicsFactory->CreateRagDoll(1,1,4,glm::vec3(0,0,0));
	body->velocity = glm::vec3(0,0,0);

	if (!Game::Initialise()) {
		return false;
	}
	camera->GetController()->position = USSEnterprise->position;
	camera->look = -USSEnterprise->PhysicsController::look;
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	float newtons = 10.0f;
	float epsilon = glm::epsilon<float>();
	float theta;
	glm::vec3 axis;
	glm::vec3 toShip2;

	if (USSEnterprise->position.y < 40)
	{
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->PhysicsController::up * dforce));
	}

	if (USSEnterprise->position.y > 1000)
	{
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::up * dforce));
	}

	if (keyState[SDL_SCANCODE_LEFT])
	{
		USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,0.3,0));
		camera->GetController()->position.z = camera->GetController()->position.z + 1;
	}
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		 USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,-0.3,0));
	}

	if(keyState[SDL_SCANCODE_DOWN])
    {
       USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0.1,0,0));
    }
	if(keyState[SDL_SCANCODE_UP])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(-0.1,0,0));
    }

	if(keyState[SDL_SCANCODE_S])
    {
		USSEnterprise->PhysicsController::rigidBody->clearForces();
    }

	camera->GetController()->look = -USSEnterprise->PhysicsController::look;
	camera->GetController()->right = USSEnterprise->PhysicsController::right;
	camera->GetController()->up = USSEnterprise->PhysicsController::up;
	
	float moveSpeed = speed;
	float timeToPass = 1.0f / fireRate;
	if ((keyState[SDL_SCANCODE_SPACE]) && (elapsed > timeToPass))
	{
		glm::vec3 pos = glm::vec3(USSEnterprise->PhysicsController::position.x,USSEnterprise->PhysicsController::position.y - 3, USSEnterprise->PhysicsController::position.z + 10)  + (USSEnterprise->PhysicsController::look * 5.0f);
		glm::quat q(RandomFloat(), RandomFloat(), RandomFloat(), RandomFloat());
		glm::normalize(q);
		shared_ptr<PhysicsController> physicsComponent = physicsFactory->CreateBox(1,1,1, pos, q);
		
		float force = dforce * 2;
		physicsComponent->rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::look) * force);
		elapsed = 0.0f;
	}
		else
	{
		elapsed += timeDelta;
	}
	
	camera->GetController()->position = USSEnterprise->PhysicsController::position;
	camera->orientation = USSEnterprise->PhysicsController::orientation * camera->GetController()->orientation;
	camera->RecalculateVectors();
	camera->view = glm::lookAt(camera->position, camera->position + camera->look, camera->up);
	camera->GetController()->position.z = camera->GetController()->position.z - 100; 
	camera->GetController()->position.y = camera->GetController()->position.y + 5; 

	USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::look * dforce));

	USSEnterprise->PhysicsController::rigidBody->setDamping(0.20,0.70);
	
	dynamicsWorld->stepSimulation(timeDelta,300);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

