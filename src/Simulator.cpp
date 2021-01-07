#include "hello.hpp"
#include <algorithm>
#include <iostream>

Simulator::Simulator() noexcept : m_joints(), m_bodies() {}

void Simulator::simulate(sc timeStep, sc numSubsteps, vec3 *gravity) {
  sc dt = timeStep / numSubsteps;
  sc x = gravity->x;
  sc y = gravity->y;
  sc z = gravity->z;
  sc sum = x + y + z;

  for (int i = 0; i < numSubsteps; i++) {
    // m_bodies[0]->DBG();
    for (int j = 0; j < m_bodies.size(); j++) {
      m_bodies[j]->integrate(dt, *gravity);
    }
    for (int j = 0; j < m_joints.size(); j++)
      m_joints[j]->solvePos(dt);
    for (int j = 0; j < m_bodies.size(); j++)
      m_bodies[j]->update(dt);
    for (int j = 0; j < m_joints.size(); j++)
      m_joints[j]->solveVel(dt);
  }
}

Body *Simulator::addBody(const Body &body) {
  m_bodies.push_back(new Body(body));
  return m_bodies[m_bodies.size() - 1];
}

Joint *Simulator::addJoint(const Joint &joint) {
  m_joints.push_back(new Joint(joint));
  return m_joints[m_joints.size() - 1];
}
void Simulator::removeBody(Body *body) {
  m_bodies.erase(std::remove(m_bodies.begin(), m_bodies.end(), body),
                 m_bodies.end());
}
void Simulator::removeJoint(Joint *joint) {
  m_joints.erase(std::remove(m_joints.begin(), m_joints.end(), joint),
                 m_joints.end());
}
void Simulator::popJoint() {
  delete m_joints[m_joints.size() - 1];
  m_joints.pop_back();
}

Simulator::~Simulator() noexcept { Delete(); }
void Simulator::Delete() noexcept {
  for (size_t i = 0; i < m_bodies.size(); i++) {
    delete m_bodies[i];
  }
  for (size_t i = 0; i < m_joints.size(); i++) {
    delete m_joints[i];
  }
}

vec3 *Simulator::getG() { return new vec3(0.0, -10.0, 0.0); }