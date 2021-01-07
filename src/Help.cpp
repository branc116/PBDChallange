#include "hello.hpp"
#include <algorithm>
Help::Help() noexcept {}

vec3 Help::getQuatAxis0(const quat &q) {
  sc x2 = q.x * 2.0;
  sc w2 = q.w * 2.0;
  return vec3((q.w * w2) - 1.0 + q.x * x2, (q.z * w2) + q.y * x2,
              (-q.y * w2) + q.z * x2);
}
vec3 Help::getQuatAxis1(const quat &q) {
  sc y2 = q.y * 2.0;
  sc w2 = q.w * 2.0;
  return vec3((-q.z * w2) + q.x * y2, (q.w * w2) - 1.0 + q.y * y2,
              (q.x * w2) + q.z * y2);
}
vec3 Help::getQuatAxis2(const quat &q) {
  sc z2 = q.z * 2.0;
  sc w2 = q.w * 2.0;
  return vec3((q.y * w2) + q.x * z2, (-q.x * w2) + q.y * z2,
              (q.w * w2) - 1.0 + q.z * z2);
}

void Help::applyBodyPairCorrection(Body *body0, Body *body1, const vec3 &corr,
                                   sc compliance, sc dt, const vec3 &pos0,
                                   const vec3 &pos1, bool velocityLevel) const {
  auto C = glm::l2Norm(corr);
  if (C == 0.0)
    return;

  vec3 normal = glm::normalize(corr);

  auto w0 = body0 ? body0->getInverseMass(normal, pos0) : 0.0;
  auto w1 = body1 ? body1->getInverseMass(normal, pos1) : 0.0;
  sc w = w0 + w1;
  if (w == 0.0)
    return;

  sc lambda = -C / (w + compliance / dt / dt);
  normal *= -lambda;
  if (body0)
    body0->applyCorrection(normal, pos0, velocityLevel);
  if (body1) {
    normal *= -1.0;
    body1->applyCorrection(normal, pos1, velocityLevel);
  }
}
void Help::applyBodyPairCorrection(Body *body0, Body *body1, const vec3 &corr,
                                   sc compliance, sc dt,
                                   bool velocityLevel) const {
  auto C = glm::l2Norm(corr);
  if (C == 0.0)
    return;

  vec3 normal = glm::normalize(corr);

  auto w0 = body0 ? body0->getInverseMass(normal) : 0.0;
  auto w1 = body1 ? body1->getInverseMass(normal) : 0.0;
  sc w = w0 + w1;
  if (w == 0.0)
    return;

  sc lambda = -C / (w + compliance / dt / dt);
  normal *= -lambda;
  if (body0)
    body0->applyCorrection(normal, velocityLevel);
  if (body1) {
    normal *= -1.0;
    body1->applyCorrection(normal, velocityLevel);
  }
}

quat Help::axisAngle(const vec3 &axis, sc angle) const {
  sc halfAngle = angle / 2, s = sin(halfAngle);
  return quat(cos(halfAngle), axis * s);
}

void Help::limitAngle(Body* body0, Body* body1, vec3 n, const vec3 &a,
                      const vec3 &b, sc minAngle, sc maxAngle, sc compliance,
                      sc dt, sc maxCorr) const {
  vec3 c = cross(a, b);

  sc phi = asin(dot(c, n));
  if (dot(a, b) < 0.0)
    phi = PI - phi;

  if (phi > PI)
    phi -= 2.0 * PI;
  if (phi < -PI)
    phi += 2.0 * PI;

  if (phi < minAngle || phi > maxAngle) {
    phi = std::min(std::max(minAngle, phi), maxAngle);
    quat q = axisAngle(n, phi);

    vec3 omega = glm::cross(glm::rotate(q, a), b);

    phi = glm::l2Norm(omega);
    if (phi > maxCorr)
      omega *= maxCorr / phi;

    applyBodyPairCorrection(body0, body1, omega, compliance, dt);
  }
}
