#include "hello.hpp"

Pose::Pose() noexcept : p{0.0, 0.0, 0.0}, q(1.0, 0.0, 0.0, 0.0) {}
void Pose::copy(const Pose &pose) { *this = pose; }
Pose Pose::clone() const { return Pose(*this); }
void Pose::rotate(vec3 &v) const { v = glm::rotate(q, v); }
void Pose::invRotate(vec3 &v) const { v = glm::rotate(conjugate(q), v); }
void Pose::transform(vec3 &v) const { v = glm::rotate(q, v) + p; }
void Pose::invTransform(vec3 &v) const {
  v -= p;
  invRotate(v);
}
void Pose::transformPose(Pose &pose) const {
  pose.q = q * pose.q;
  rotate(pose.p);
  pose.p += p;
}
