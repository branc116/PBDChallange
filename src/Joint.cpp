#include "hello.hpp"
#include <algorithm>

Joint::Joint(int type, Body *body0, Body *body1, const Pose &localPose0,
             const Pose &localPose1) noexcept
    : body0(body0), body1(body1), localPose0(localPose0),
      localPose1(localPose1), globalPose0(localPose0), globalPose1(localPose1),
      type(JointType(type)), compliance(0.0), rotDamping(0.0), posDamping(0.0),
      hasSwingLimits(false), minSwingAngle(-2.0 * PI), maxSwingAngle(2.0 * PI),
      swingLimitsCompliance(0.0), hasTwistLimits(false),
      minTwistAngle(-2.0 * PI), maxTwistAngle(2.0 * PI),
      twistLimitCompliance(0.0), h() {}

void Joint::updateGlobalPoses() {
  globalPose0.copy(localPose0);
  if (body0)
    body0->pose.transformPose(globalPose0);
  globalPose1.copy(localPose1);
  if (body1)
    body1->pose.transformPose(globalPose1);
}

void Joint::solvePos(sc dt) {

  updateGlobalPoses();

  // orientation

  if (type == JointType::FIXED) {
    quat q = conjugate(globalPose0.q) * globalPose0.q;
    vec3 omega(2.0 * q.x, 2.0 * q.y, 2.0 * q.z);
    if (q.w < 0.0) // how can omeaga have w???
      q *= -1.0;
    h.applyBodyPairCorrection(body0, body1, omega, compliance, dt);
  }

  if (type == JointType::HINGE) {
    vec3 a0 = h.getQuatAxis0(globalPose0.q);
    vec3 b0 = h.getQuatAxis1(globalPose0.q);
    vec3 c0 = h.getQuatAxis2(globalPose0.q);
    vec3 a1 = h.getQuatAxis0(globalPose1.q);
    a0 = cross(a0, a1);
    h.applyBodyPairCorrection(body0, body1, a0, 0.0, dt);

    // limits
    if (hasSwingLimits) {
      updateGlobalPoses();
      vec3 n = h.getQuatAxis0(globalPose0.q);
      vec3 b0 = h.getQuatAxis1(globalPose0.q);
      vec3 b1 = h.getQuatAxis1(globalPose1.q);
      h.limitAngle(body0, body1, n, b0, b1, minSwingAngle, maxSwingAngle,
                   swingLimitsCompliance, dt);
    }
  }

  if (type == JointType::SPHERICAL) {

    // swing limits
    if (hasSwingLimits) {
      updateGlobalPoses();
      vec3 a0 = h.getQuatAxis0(globalPose0.q);
      vec3 a1 = h.getQuatAxis0(globalPose1.q);
      vec3 n = normalize(cross(a0, a1));
      h.limitAngle(body0, body1, n, a0, a1, minSwingAngle, maxSwingAngle,
                   swingLimitsCompliance, dt);
    }
    // twist limits
    if (hasTwistLimits) {
      updateGlobalPoses();
      vec3 n0 = h.getQuatAxis0(globalPose0.q);
      vec3 n1 = h.getQuatAxis0(globalPose1.q);
      vec3 n = normalize(n0 + n1);
      vec3 a0 = normalize(h.getQuatAxis1(globalPose0.q) - n * dot(n, a0));
      vec3 a1 = normalize(h.getQuatAxis1(globalPose1.q) - n * dot(n, a1));

      // handling gimbal lock problem
      sc maxCorr = dot(n0, n1) > -0.5 ? 2.0 * PI : 1.0 * dt;

      h.limitAngle(body0, body1, n, a0, a1, minTwistAngle, maxTwistAngle,
                   twistLimitCompliance, dt, maxCorr);
    }
  }

  // position

  // simple attachment

  updateGlobalPoses();
  vec3 corr = globalPose1.p - globalPose0.p;
  h.applyBodyPairCorrection(body0, body1, corr, compliance, dt, globalPose0.p,
                            globalPose1.p);
}

void Joint::solveVel(sc dt) {

  // Gauss-Seidel lets us make damping unconditionally stable in a
  // very simple way. We clamp the correction for each constraint
  // to the magnitude of the currect velocity making sure that
  // we never subtract more than there actually is.

  if (rotDamping > 0.0) {
    vec3 omega{0.0};
    if (body0)
      omega -= body0->omega;
    if (body1)
      omega += body1->omega;
    omega *= std::min(sc(1.00), rotDamping * dt);
    h.applyBodyPairCorrection(body0, body1, omega, 0.0, dt, true);
  }
  if (posDamping > 0.0) {
    updateGlobalPoses();
    vec3 vel{0.0};
    if (body0)
      vel -= body0->getVelocityAt(globalPose0.p);
    if (body1)
      vel += body1->getVelocityAt(globalPose1.p);
    vel *= std::min(sc(1.0), posDamping * dt);
    h.applyBodyPairCorrection(body0, body1, vel, 0.0, dt, globalPose0.p,
                              globalPose1.p, true);
  }
}