#pragma once
#include "Game.h"
#include "PhysicsController.h"
#include "PhysicsFactory.h"
#include <btBulletDynamicsCommon.h>
#include "FountainEffect.h"

namespace BGE
{
	class Assignment :
		public Game
	{
	private:
		btBroadphaseInterface* broadphase;
 
		// Set up the collision configuration and dispatcher
		btDefaultCollisionConfiguration * collisionConfiguration;
		btCollisionDispatcher * dispatcher;
 
		// The actual physics solver
		btSequentialImpulseConstraintSolver * solver;

	public:
		Assignment(void);
		~Assignment(void);
		bool Initialise();
		void Update(float timeDelta);
		void Cleanup();
		void CreateWall(); 
		

		shared_ptr<PhysicsController> arml;
		shared_ptr<PhysicsController> armr;
		shared_ptr<PhysicsController> body;
		shared_ptr<PhysicsController> legr;
		shared_ptr<PhysicsController> legl;
		shared_ptr<PhysicsController> USSEnterprise;
		vector<shared_ptr<FountainEffect>> Ljets;
		vector<shared_ptr<FountainEffect>> Rjets;
		shared_ptr<GameComponent> buddha;

		bool cameraLock;
		float dforce;
		float mass;
		float fireRate;
		bool slerping;
		glm::quat fromQuaternion;
		glm::quat toQuaternion;
		float t;
		
		// The world.
		std::shared_ptr<PhysicsFactory> physicsFactory;
		btDiscreteDynamicsWorld * dynamicsWorld;
	};
}