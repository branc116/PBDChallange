#include "glm/glm.hpp"
#include "glm/gtx/quaternion.hpp"
#include <vector>
#include <iostream>

using namespace glm;
using sc = vec3::value_type;
#define PI 3.1415




class Body;
class Help {
  public:
  Help() noexcept;
  vec3 getQuatAxis0(const quat& q);
  vec3 getQuatAxis1(const quat& q);
  vec3 getQuatAxis2(const quat& q);
  
void applyBodyPairCorrection(Body* body0, Body* body1, const vec3 &corr,
                             sc compliance, sc dt, const vec3 &pos0,
                             const vec3 &pos1, bool velocityLevel = false) const;
void applyBodyPairCorrection(Body* body0, Body* body1, const vec3 &corr,
                             sc compliance, sc dt, bool velocityLevel = false) const;
// ------------------------------------------------------------------------------------------------
quat axisAngle(const vec3 &axis, sc angle) const;
void limitAngle(Body* body0, Body* body1, vec3 n, const vec3 &a, const vec3 &b,
                sc minAngle, sc maxAngle, sc compliance, sc dt,
                sc maxCorr = PI) const;

};

class Pose {
public:
  Pose() noexcept;
  void copy(const Pose &pose);
  Pose clone() const;
  void rotate(vec3 &v) const;
  void invRotate(vec3 &v) const;
  void transform(vec3 &v) const;
  void invTransform(vec3 &v) const;
  void transformPose(Pose &pose) const;
  glm::vec3 p;
  quat q;
};
class Body {
public:
  Body(const Pose &pose) noexcept;
  void setBox(const vec3& size, sc density = 1.0);
  void applyRotation(const vec3& rot, sc scale = 1.0);
  void integrate(sc dt, const vec3& gravity);
  void update(sc dt);
  vec3 getVelocityAt(const vec3& pos) const;
  sc getInverseMass(const vec3& normal, const vec3& pos) const;
  sc getInverseMass(const vec3& normal) const;
  void applyCorrection(const vec3& corr, const vec3& pos, bool velocityLevel = false);
  void applyCorrection(const vec3& corr, bool velocityLevel = false);
  void DBG();
  Pose pose;
  Pose prevPose;
  Pose origPose;
  vec3 vel;
  vec3 omega;
  sc invMass;
  vec3 invInertia;

};
enum JointType { SPHERICAL, HINGE, FIXED };

class Joint {
  public:
  Body *body0;
  Body *body1;
  Pose localPose0;
  Pose localPose1;
  Pose globalPose0;
  Pose globalPose1;
  JointType type;
  sc compliance;
  sc rotDamping;
  sc posDamping;

  bool hasSwingLimits;
  sc minSwingAngle;
  sc maxSwingAngle;
  sc swingLimitsCompliance;

  bool hasTwistLimits;
  sc minTwistAngle;
  sc maxTwistAngle;
  sc twistLimitCompliance;

  Help h;
  Joint(int type, Body *body0, Body *body1, const Pose &localPose0,
        const Pose &localPose1) noexcept;

  void updateGlobalPoses();

  void solvePos(sc dt);

  void solveVel(sc dt);
};

class Simulator {
  public:
  Simulator() noexcept;
  ~Simulator() noexcept;
  void Delete() noexcept;
  void simulate(sc timeStep, sc numSubsteps, vec3* gravity);
  Body* addBody(const Body& body);
  Joint* addJoint(const Joint& joint);
  void removeBody(Body* body);
  void removeJoint(Joint* joint);
  void popJoint();
  vec3* getG();
  private:
  std::vector<Joint*> m_joints;
  std::vector<Body*> m_bodies;
};
