#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

//        Comment-in this part when you implement the constructor
//        for (auto &i : pinned_nodes) {
//            masses[i]->pinned = true;
//        }
        if(num_nodes < 2){ 
          std::cout << "num_nodes < 2 in Constructor of Rope" << std::endl;
          return;
        }
        Vector2D length = (end - start) / (num_nodes - 1);
        for(int i = 0; i < num_nodes; i++) {
          masses.push_back(new Mass(start + i * length,  node_mass, false));
          if(i > 0) {
            springs.push_back(new Spring(masses[i - 1], masses[i], k));
          }
        }
                      
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
    }

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D a2b = s->m2->position - s->m1->position;
            double length = a2b.norm();
            Vector2D force_b2a = -s->k * a2b / length * (length - s->rest_length);
            s->m2->forces += force_b2a;
            s->m1->forces += (-force_b2a);
        }

        for (auto &m : masses)
        {
            float kd = 0.1f;
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * gravity;
                // TODO (Part 2): Add global damping
                m->forces -= kd * m->velocity;
                Vector2D a = m->forces / m->mass;

                // explicit Euler
/*
                m->position += m->velocity * delta_t;
                m->velocity += a * delta_t;
*/

                // semi-implicit 
                m->velocity += a * delta_t;
                m->position += m->velocity * delta_t;
                
                
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
            Vector2D a2b = s->m2->position - s->m1->position;
            double length = a2b.norm();
            Vector2D force_b2a = -s->k * a2b / length * (length - s->rest_length);
            s->m2->forces += force_b2a;
            s->m1->forces += (-force_b2a);
        }

        double damping_factor = 0.00005;
        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                m->forces += m->mass * gravity;
                // TODO (Part 2): Add global damping
                //m->forces -= kd * m->velocity;
                Vector2D a = m->forces / m->mass;
                
                // TODO (Part 4): Add global Verlet damping
                m->position = m->position + (1 - damping_factor) * (m->position - m->last_position) + a * delta_t * delta_t;
                m->last_position = temp_position;
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }
}
