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

	shared_ptr<FountainEffect> centFountain = make_shared<FountainEffect>(500);
	centFountain->position.x = centFountain->position.y = 0;
	centFountain->position.y = 0;
	centFountain->diffuse = glm::vec3(1,1,0);
	Attach(centFountain);

	mass = 1.0f;
	dforce = 4000.0f;

	body = CreateRagDoll();
	body->velocity = glm::vec3(0,0,0);

	if (!Game::Initialise()) {
		return false;
	}

	camera->GetController()->position = glm::vec3(0,6, 0);
	
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	float epsilon = glm::epsilon<float>();
	glm::vec3 toBody = body->position - glm::vec3(epsilon,epsilon,epsilon);
	if (glm::length(toBody) < 5)
	{
		body->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::basisUp) * dforce);
	}

	dynamicsWorld->stepSimulation(timeDelta,10);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

shared_ptr<PhysicsController> BGE::Assignment::CreateRagDoll()
{
	arml = physicsFactory->CreateBox(0.5,0.5,1.5, glm::vec3(2,0,0), glm::quat());
	armr = physicsFactory->CreateBox(0.5,0.5,1.5, glm::vec3(-2,0,0), glm::quat());
	shared_ptr<PhysicsController> body = physicsFactory->CreateBox(1,1,4, glm::vec3(0,0,0), glm::quat()); 

	shared_ptr<PhysicsController> legl = physicsFactory->CreateBox(0.5,0.5,2, glm::vec3(3,0,5), glm::quat());
	shared_ptr<PhysicsController> legr = physicsFactory->CreateBox(0.5,0.5,2, glm::vec3(-3,0,5), glm::quat());
	
	// A hinge
	btHingeConstraint * hinge = new btHingeConstraint(*legr->rigidBody, *body->rigidBody, btVector3(0.7,0,1.8),btVector3(0,0,2), btVector3(-0.3,0,0), btVector3(-0.3,1,0), true);
	dynamicsWorld->addConstraint(hinge);

	// A hinge
	btHingeConstraint * hinge2 = new btHingeConstraint(*legl->rigidBody, *body->rigidBody, btVector3(-0.7,0,1.8),btVector3(0,0,2), btVector3(0.3,0,0), btVector3(0.3,1,0), true);
	dynamicsWorld->addConstraint(hinge2);

	btHingeConstraint * hinge3 = new btHingeConstraint(*armr->rigidBody, *body->rigidBody, btVector3(0.7,0,1.8),btVector3(0,0,-2), btVector3(-0.3,0,0), btVector3(-0.3,1,0), true);
	dynamicsWorld->addConstraint(hinge3);

	btHingeConstraint * hinge4 = new btHingeConstraint(*arml->rigidBody, *body->rigidBody, btVector3(-0.7,0,1.8),btVector3(0,0,-2), btVector3(0.3,0,0), btVector3(0.3,1,0), true);
	dynamicsWorld->addConstraint(hinge4);

	return body;
}

