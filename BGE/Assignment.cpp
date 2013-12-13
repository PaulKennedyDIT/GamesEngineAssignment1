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
	//Fire Rate for Ships Bullets
	fireRate= 5.0f;
	//Oculus Rift Enabler.
	riftEnabled = false;

	// Set up the collision configuration and dispatcher
    collisionConfiguration = new btDefaultCollisionConfiguration();
    dispatcher = new btCollisionDispatcher(collisionConfiguration);
	soundSystem->PlaySoundW("sound2",glm::vec3(0,0,0));

    // Sets up the world.Adds the solver
	//Instansiates a dynamic world for the Bullet physixs engine. 
	//Sets the value for gravity. gravity
	btVector3 worldMin(-1000,-1000,-1000);
	btVector3 worldMax(1000,1000,1000);
	broadphase = new btAxisSweep3(worldMin,worldMax);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
    dynamicsWorld->setGravity(btVector3(0,-9,0));

	//Beginning of Game logic.

	// Instansiates a set of fountain effect particles.
	// Every 500 units on the Z axis a line of particles are drawn.
	// Each Fountain is pushed back onto a list of Fountains for tracking and updating.
	// Fountain Attached to scene.
		for(int i = 0; i < 10; i++)
		{
			for(int j = 0; j < 10; j++)
			{
				shared_ptr<FountainEffect> fountain = make_shared<FountainEffect>(500);
				fountain->diffuse = glm::vec3(1,0,0);
				fountain->position.x = 100 - (15 * i);
				fountain->position.z = 485 + (500 * j);
				fountain->position.y =100;
				fountain->diffuse= glm::vec3(0.0f,0.0f,1.0f);
				Djets.push_back(fountain);
				Attach(fountain);
			}
		}
	
	// Physics Factory is defined. 
	//The Dynamic World setup for Bullet Physics is passed into the Physics Factory.
	physicsFactory = make_shared<PhysicsFactory>(dynamicsWorld);

	// Ground is initialised. 
	// Includes Ground Physics for Bullet as well as texture loading for the rendering.
	physicsFactory->CreateGroundPhysics();

	// Camera is instansiated as a kinematic object. 
	// Camera can manipulate a scene. 
	// Collisions do not effect the camera
	physicsFactory->CreateCameraPhysics();

	// Both Walls and Roof is instansiated.
	// These are declared as kimeatic objects.
	// Bullet objects collide and interact with the walls. 
	// The Camera does not.
	shared_ptr<PhysicsController> wallL = physicsFactory->CreateContainerWall(100, 300, 5000, glm::vec3(100,0,2500), glm::quat());
	shared_ptr<PhysicsController> wallR = physicsFactory->CreateContainerWall(100, 300, 5000, glm::vec3(-100,0,2500), glm::quat());
	shared_ptr<PhysicsController> roof  = physicsFactory->CreateContainerWall(50, 20, 5000, glm::vec3(0,150,2500), glm::quat());

	// Instansiates obstacles along the path of the obstacle course.
	// Set of 3 positions for the obstacles are calculated by getting the modulus of the value i with 3.
	for(int i = 0; i < 10;i++)
	{
		if( i % 3 == 0)
		{
			shared_ptr<PhysicsController> obstacle  = physicsFactory->CreateContainerWall(100, 20, 20, glm::vec3(0,40,((i * 500) + 485)), glm::quat());
			shared_ptr<PhysicsController> obstacleUP  = physicsFactory->CreateContainerWall(20, 250, 20, glm::vec3(((i) + 20),40,((i * 500) + 485)), glm::quat());
		}
		else if( i % 3 ==1)
		{
			shared_ptr<PhysicsController> obstacle  = physicsFactory->CreateContainerWall(100, 20, 20, glm::vec3(0,20,(i * 500) + 485), glm::quat());
			shared_ptr<PhysicsController> obstacleUP  = physicsFactory->CreateContainerWall(20, 250, 20, glm::vec3(((i)),40,((i * 500) + 485)), glm::quat());
		}
		else
		{
			shared_ptr<PhysicsController> obstacle  = physicsFactory->CreateContainerWall(100, 20, 20, glm::vec3(0,60,(i * 500) + 485), glm::quat());
			shared_ptr<PhysicsController> obstacleUP  = physicsFactory->CreateContainerWall(20, 250, 20, glm::vec3(((i) - 20),40,((i * 500) + 485)), glm::quat());
		}
	}

	// Defines one last obstacle at the end of the course making for a tight squeeze for the Enterprise.
	shared_ptr<PhysicsController> obstacleUP  = physicsFactory->CreateContainerWall(20, 250, 20, glm::vec3(-20,40,5000), glm::quat());

	// Enterprise model is loaded using a .objm and mtl file.
	// Object is loaded in the Physics factory.
	// Rigid body for model is declared using construction info which allows the ship to interact using Bullet physics
	USSEnterprise = physicsFactory->CreateFromModel("USSEnterprise",glm::vec3(0,50,0),glm::quat(),2,glm::vec3(5,2.5,5));

	// Snow Effect added to the scene.
	shared_ptr<SnowEffect> snow = make_shared<SnowEffect>();
	Attach(snow);

	// Mass and Force
	mass = 1.0f;
	dforce = 1000;
	t =0.0f;

	// Creates a Ragdoll. Sadly the space ship is too fast to see him :( ... but he is there
	body = physicsFactory->CreateRagDoll(1,1,4,glm::vec3(50,0,465));
	
	if (!Game::Initialise()) {
		return false;
	}
	//Attachs the Camera to the Enterprise model declared.
	USSEnterprise->Attach(camera);
  
	return true;
}

void BGE::Assignment::Update(float timeDelta)
{
	// Setup for calculating the Angle of Rotation for orientating the Camera around the ship.
	float theta;
	glm::vec3 axis;
	glm::vec3 toShip2;

	// Applies upward force to the ship in the case that the position on the y axis is less than 5.
	// Helps avoid the ship crashing.
	if (USSEnterprise->position.y < 5)
	{
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->PhysicsController::up * dforce));
	}

	// Applies downward force to the ship in the case that the position on the y axis is above than 1000.
	//Precaustionary to keep the ship close to the ground
	if (USSEnterprise->position.y > 1000)
	{
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::up * dforce));
	}
	
	// If the Left arrow key is pressed a positive Angular velocity force and a Central scalar force are applied to the ship. 
	// Used for turning the ship left.
	if (keyState[SDL_SCANCODE_LEFT])
	{
		USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,1,0));
		USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->PhysicsController::right * (dforce * 2 )));
	}

	// If the Right arrow key is pressed a negative Angular velocity force and a Central scalar force are applied to the ship. 
	// Used for turning the ship right.
	if (keyState[SDL_SCANCODE_RIGHT])
	{
		 USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,-1,0));
		 USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::right * (dforce * 2 )));
	}

	// If the down key is pressed a downward angular velocity is applied to the ship. Allows for the ship to pitch.
	if(keyState[SDL_SCANCODE_DOWN])
    {
       USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(1,0,0));
    }

		// If the up key is pressed an upward angular velocity is applied to the ship. Allows for the ship to pitch.
	if(keyState[SDL_SCANCODE_UP])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(-1,0,0));
    }

	// If the E key is pressed an angular force is applied to the Z axis. 
	// Achieves a rolling motion.
	if(keyState[SDL_SCANCODE_E])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,0,1));
    }

		// If the Q key is pressed an angular force is applied to the Z axis. 
	// Achieves a rolling motion.
	if(keyState[SDL_SCANCODE_Q])
    {
      USSEnterprise->PhysicsController::rigidBody->setAngularVelocity(btVector3(0,0,-1));
    }

	// If the S key is pressed, the ship accelerates with a force 3 times the normal force.
	if(keyState[SDL_SCANCODE_S])
    {
		dforce = 3000;
    }
	else
	{
		dforce = 1000;
	}

	// Iterates through all of the fountain particle effects
	for(int i = 0; i < Djets.size();i++)
	{
		// Calculates the distance between the position of each jet and the Enterprise.
		glm::vec3 toBody = USSEnterprise->position - Djets[i]->position;

		// If the Distance between the particle effect and the ship is 30, a central force is applied based on the BasisUp of the ship.
		// Causes the ship to rise suddenly
	    if (glm::length(toBody) < 30 && i % 2 == 0)
        {
                USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::basisUp) * (-dforce * 2));
        }
		// For every second fountain effect, apply an upward force.
		else if(glm::length(toBody) < 30 && i % 2 == 1)
		{
			 USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(GameComponent::basisUp) * (dforce * 2));
		}

		// Moves the Fountains upwards and downwards.
		// Every second fountain is in sync.
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

	// Sets the speed and fire rate for the Enterprise.
	float moveSpeed = speed;
	float timeToPass = 1.0f / fireRate;

	// If the space bar key is pressed and the time elapsed is far enough between, fire a in a forwards direction.
	if ((keyState[SDL_SCANCODE_SPACE]) && (elapsed > timeToPass))
	{
		// Get a position for the cube based on the position of the Enterprise.
		glm::vec3 pos = glm::vec3(USSEnterprise->PhysicsController::position.x,USSEnterprise->PhysicsController::position.y - 5, USSEnterprise->PhysicsController::position.z + 10)  + (USSEnterprise->PhysicsController::look * 5.0f);
		// Quaternion defined.
		glm::quat q(RandomFloat(), RandomFloat(), RandomFloat(), RandomFloat());
		// Normalised quaternion
		glm::normalize(q);
		// Generate a 1x1x1 box based on the quaternion
		shared_ptr<PhysicsController> physicsComponent = physicsFactory->CreateBox(1,1,1, pos, q);
		// Force that a cube is fired at.
		float force = 10000.0f;
		// Central force is applied to the cube.
		physicsComponent->rigidBody->applyCentralForce(GLToBtVector(USSEnterprise->look * -force));
		elapsed = 0.0f;
	}
		else
	{
		elapsed += timeDelta;
	}

	// Orientates the Position of the Camera relative to the Enterprise.
	camera->GetController()->position = USSEnterprise->PhysicsController::position;
	camera->orientation = USSEnterprise->PhysicsController::orientation * camera->GetController()->orientation;
	camera->RecalculateVectors();
	camera->view = glm::lookAt(camera->GetController()->position, camera->GetController()->position + camera->GetController()->look,camera->GetController()->up);

	// Offsets the position of the camera so it is slightly behind and above the Enterprise.
	camera->GetController()->position.z = camera->GetController()->position.z - 40; 
    camera->GetController()->position.y = camera->GetController()->position.y + 10;

	// Constant force is applied to the Enterprise in order to move it forward constantly
	USSEnterprise->PhysicsController::rigidBody->applyCentralForce(GLToBtVector(-USSEnterprise->PhysicsController::look * dforce));

	// Dampening is applied to reduce the extent of the Angular and Linear velocties applied after each keyboard interaction.
	USSEnterprise->PhysicsController::rigidBody->setDamping(0.99,0.99);

	// Simulation is stepped in order to calculate a set amount of bullet calculations. 
	//If the specified number of calculations have not been performed before each tick of the timedelta, in order to avoid slowing down the game, the calculations cut off.
	dynamicsWorld->stepSimulation(timeDelta,300);
	Game::Update(timeDelta);
}

void BGE::Assignment::Cleanup()
{
	Game::Cleanup();
}

