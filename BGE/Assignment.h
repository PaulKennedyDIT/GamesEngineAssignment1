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
		shared_ptr<PhysicsController> truster;
		vector<shared_ptr<FountainEffect>> Ljets;
		vector<shared_ptr<FountainEffect>> Rjets;

		float dforce;
		float mass;
		
		// The world.
		std::shared_ptr<PhysicsFactory> physicsFactory;
		btDiscreteDynamicsWorld * dynamicsWorld;
		shared_ptr<GameComponent> buddha;
	};
}