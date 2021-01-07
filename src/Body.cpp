#include "hello.hpp"

sc maxRotationPerSubstep = 0.5;

void print(const vec3& vec) {
  sc x = vec.x;
  sc y = vec.y;
  sc z = vec.z;
  std::cout << x << ' ' << y << ' ' << z << ' ';
  return;
}

void print(const Pose& p ) {
  sc py = p.p.y;
  sc px = p.p.x;
  sc pz = p.p.z;
  sc qx = p.q.x;
  sc qy = p.q.y;
  sc qz = p.q.z;
  sc qq = p.q.w;
  std::cout << px << " ";
  std::cout << py << " ";
  std::cout << pz << " | ";
  std::cout << qx << " ";
  std::cout << qy << " ";
  std::cout << qz << " ";
  std::cout << qq << " ";
}

Body::Body(const Pose &pose) noexcept
    : pose{pose}, prevPose{pose}, origPose{pose}, vel{0.0}, omega{0.0},
      invMass(1.0), invInertia{0.0} { }

void Body::setBox(const vec3 &size, sc density) {
  float mass = size.x * size.y * size.z * density;
  invMass = 1.0 / mass;
  mass /= 12.0;
  invInertia = vec3(1.0 / (size.y * size.y + size.z * size.z) / mass,
                    1.0 / (size.z * size.z + size.x * size.x) / mass,
                    1.0 / (size.x * size.x + size.y * size.y) / mass);
}

void Body::applyRotation(const vec3 &rot, sc scale) {

  // safety clamping. This happens very rarely if the solver
  // wants to turn the body by more than 30 degrees in the
  // orders of milliseconds

  sc maxPhi = 0.5;
  sc phi = rot.length();
  if (phi * scale > maxRotationPerSubstep)
    scale = maxRotationPerSubstep / phi;

  quat dq{0.0, rot.x * scale, rot.y * scale, rot.z * scale};
  dq *= pose.q;
  pose.q = quat{pose.q.w + sc(0.5) * dq.w, pose.q.x + sc(0.5) * dq.x,
                pose.q.y + sc(0.5) * dq.y, pose.q.z + sc(0.5) * dq.z};
  pose.q = glm::normalize(pose.q);
}

void Body::integrate(sc dt, const vec3 &gravity) {
  prevPose = pose;
  vel += gravity * dt;
  pose.p += vel * dt;

  applyRotation(omega, dt);
}

void Body::update(sc dt) {
  vel = (pose.p - prevPose.p) / dt;
  quat dq = pose.q * glm::conjugate(prevPose.q);
  omega =  vec3(dq.x, dq.y, dq.z) * (sc(2.0) / dt);
  if (dq.w < 0.0)
    omega *= -1;
}

vec3 Body::getVelocityAt(const vec3 &pos) const {
  vec3 vel = -cross(pos - pose.p, omega);
  return vel;
}

sc Body::getInverseMass(const vec3 &normal, const vec3 &pos) const {
  vec3 n = cross(pos - pose.p, normal);
  pose.invRotate(n);
  sc w = dot(n * n, invInertia);
  w += invMass;
  return w;
}
sc Body::getInverseMass(const vec3 &normal) const {
  vec3 n = normal;
  pose.invRotate(n);
  sc w = dot(n * n, invInertia);
  return w;
}

void Body::applyCorrection(const vec3 &corr, const vec3 &pos,
                           bool velocityLevel) {
  vec3 dq{0.0};
  if (velocityLevel)
    vel += corr * invMass;
  else
    pose.p += corr * invMass;
  dq = cross(pos - pose.p, corr);
  pose.invRotate(dq);
  dq *= invInertia;
  pose.rotate(dq);
  if (velocityLevel)
    omega += dq;
  else
    applyRotation(dq);
}
void Body::applyCorrection(const vec3 &corr, bool velocityLevel) {
  vec3 dq = corr;
  pose.invRotate(dq);
  dq *= invInertia;
  pose.rotate(dq);
  if (velocityLevel)
    omega += dq;
  else
    applyRotation(dq);
}
void Body::DBG() {
  std::cout << "pose: "; print(pose); std::cout << std::endl;
  std::cout << "prevPose: "; print(prevPose); std::cout << std::endl;
  std::cout << "origPose: "; print(origPose); std::cout << std::endl;
  std::cout << "vel: "; print(vel); std::cout << std::endl;
  std::cout << "omega: "; print(omega); std::cout << std::endl;
  std::cout << "invMass: " << invMass << std::endl;
  std::cout << "invInertia: "; print(invInertia); std::cout << std::endl;
}