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

	
	
		for(int i = 0; i < 10; i++)
		{
			for(int j = 0; j < 5; j++)
			{
				shared_ptr<FountainEffect> fountain = make_shared<FountainEffect>(500);
				fountain->diffuse = glm::vec3(1,0,0);
				fountain->position.x = 100 - (5 * i);
				fountain->position.z = 485 + (500 * j);
				fountain->position.y =100;
				Djets.push_back(fountain);
				Attach(fountain);

				
			}
		}

	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);

	physicsFactory->CreateGroundPhysics();
	physicsFactory->CreateCameraPhysics();

	shared_ptr<PhysicsController> wallL = physicsFactory->CreateContainerWall(100, 300, 5000, glm::vec3(100,0,2500), glm::quat());
	shared_ptr<PhysicsController> wallR = physicsFactory->CreateContainerWall(100, 300, 5000, glm::vec3(-100,0,2500), glm::quat());
	shared_ptr<PhysicsController> roof  = physicsFactory->CreateContainerWall(50, 20, 5000, glm::vec3(0,150,2500), glm::quat());

	for(int i = 0; i < 5;i++)
	{
		if( i % 3 == 0)
		{
			shared_ptr<PhysicsController> obstacle  = physicsFactory->CreateContainerWall(100, 20, 20, glm::vec3(0,40,((i * 500) + 485)), glm::quat());
		}
		else if( i % 3 ==1)
		{
			shared_ptr<PhysicsController> obstacle  = physicsFactory->CreateContainerWall(100, 20, 20, glm::vec3(0,20,(i * 500) + 485), glm::quat());
		}
		else
		{
			shared_ptr<PhysicsController> obstacle  = physicsFactory->CreateContainerWall(100, 20, 20, glm::vec3(0,60,(i * 500) + 485), glm::quat());
		}
	}

	USSEnterprise = physicsFactory->CreateFromModel("USSEnterprise",glm::vec3(0,50,0),glm::quat(),2,glm::vec3(5,2.5,5));

	mass = 1.0f;
	dforce = 1000;
	t =0.0f;

	body = physicsFactory->CreateRagDoll(1,1,4,glm::vec3(0,0,460));
	body->position = glm::vec3(USSEnterprise->PhysicsController::position.x,USSEnterprise->PhysicsController::position.y - 3,USSEnterprise->PhysicsController::position.z);
	if (!Game::Initialise()) {
		return false;
	}
	camera->GetController()->position = USSEnterprise->PhysicsController::position;
	camera->GetController()->look = -USSEnterprise->look;
	USSEnterprise->Attach(camera);
     
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	float newtons = 10.0f;
	float epsilon = glm::epsilon<float>();
	float theta;
	glm::vec3 axis;
	glm::vec3 toShip2;

	camera->look = -USSEnterprise->look;
	camera->GetController()->up = USSEnterprise->PhysicsController::up;
	camera->GetController()->right = USSEnterprise->PhysicsController::right;

	if (USSEnterprise->position.y < 5)
	{
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->PhysicsController::up * dforce));
	}

	if (USSEnterprise->position.y > 1000)
	{
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::up * dforce));
	}
	
	if (keyState[SDL_SCANCODE_LEFT])
	{
		USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,1,0));
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->PhysicsController::right * (dforce * 2 )));
	}
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		 USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,-1,0));
		 USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::right * (dforce * 2 )));
	}

	if(keyState[SDL_SCANCODE_DOWN])
    {
       USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(1,0,0));
    }
	if(keyState[SDL_SCANCODE_UP])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(-1,0,0));
    }

	if(keyState[SDL_SCANCODE_E])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,0,1));
    }

	if(keyState[SDL_SCANCODE_Q])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,0,-1));
    }

	if(keyState[SDL_SCANCODE_S])
    {
		dforce = 3000;
    }
	else
	{
		dforce = 1000;
	}

	for(int i = 0; i < Djets.size();i++)
	{
		float epsilon = glm::epsilon<float>();
		glm::vec3 toBody = USSEnterprise->position - Djets[i]->position;
        if (glm::length(toBody) < 20)
        {
                USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::basisUp) * -dforce);
        }

		if (i % 2 == 0)
		{
			Djets[i]->position.y = FOUNTAIN_HEIGHT + (glm::sin(fountainTheta) * FOUNTAIN_HEIGHT);
		}
		else
		{
			Djets[i]->position.y = FOUNTAIN_HEIGHT - (glm::sin(fountainTheta) * FOUNTAIN_HEIGHT);
		}
	}
	fountainTheta += timeDelta;
	if (fountainTheta >= glm::pi<float>() * 2.0f)
	{
		fountainTheta = 0.0f;
	}
	float moveSpeed = speed;
	float timeToPass = 1.0f / fireRate;

	
	if ((keyState[SDL_SCANCODE_SPACE]) && (elapsed > timeToPass))
	{
		glm::vec3 pos = glm::vec3(USSEnterprise->PhysicsController::position.x,USSEnterprise->PhysicsController::position.y - 5, USSEnterprise->PhysicsController::position.z + 10)  + (USSEnterprise->PhysicsController::look * 5.0f);
		glm::quat q(RandomFloat(), RandomFloat(), RandomFloat(), RandomFloat());
		glm::normalize(q);
		shared_ptr<PhysicsController> physicsComponent = physicsFactory->CreateBox(1,1,1, pos, q);
		
		float force = 5000.0f;
		physicsComponent->rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->look * -force));
		elapsed = 0.0f;
	}
		else
	{
		elapsed += timeDelta;
	}

	
	camera->GetController()->position = USSEnterprise->PhysicsController::position;
	camera->orientation = USSEnterprise->PhysicsController::orientation * camera->GetController()->orientation;
	camera->RecalculateVectors();
	camera->view = glm::lookAt(camera->position, camera->position + -camera->look, camera->up);

	camera->GetController()->position.z = camera->GetController()->position.z - 40; 
    camera->GetController()->position.y = camera->GetController()->position.y + 10;

	USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::look * dforce));

	USSEnterprise->PhysicsController::rigidBody->setDamping(0.99,0.99);

	dynamicsWorld->stepSimulation(timeDelta,300);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

