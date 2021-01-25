#version 430	
layout(local_size_x = 1024) in;
struct Cest {
	vec4 poz;
	vec4 vel;
	float sta;
	float status; //0 - notSetUp; 1 - living; 2 - dead
	float st2;
	float st3;
	vec4 col;

};
layout(binding=0) buffer Cestica {
	Cest cests[];
} cest;
uniform float iDeltaTime;
uniform float iTime;
uniform float deadAfter;
uniform int grid_x;
uniform int grid_z;
uint getI2() {
   uint work_group = gl_WorkGroupID.x * gl_NumWorkGroups.y * gl_NumWorkGroups.z + gl_WorkGroupID.y * gl_NumWorkGroups.z + gl_WorkGroupID.z;
   return work_group * gl_WorkGroupSize.x * gl_WorkGroupSize.y * gl_WorkGroupSize.z + gl_LocalInvocationIndex;
}
void init(uint i) {
	
	cest.cests[i].poz = vec4(i % grid_x - grid_x/2.0, i/(grid_x*grid_z), mod(i/grid_x, grid_z) - grid_z/2.0, 0.0) * 6.0;
	cest.cests[i].vel = vec4(0.0);
	cest.cests[i].sta = 0.0;
	cest.cests[i].status = 1.0;
	cest.cests[i].col = vec4(sin(i*0.1), cos(i * 0.2), sin(i * 0.05)*cos(i*0.01), 1.0) + 0.7;
}
vec3 sfG() {
	return vec3(0.0, -100.0, 0.0);
}
vec3 sfReppel(vec3 p, vec3 pos, float magnitude) {
	vec3 dif = p - pos;
	return magnitude * (p - pos) / pow(length(p - pos), 2.0);
}
vec3 sfReppelPulse(vec3 p, vec3 pos, float magnitude, float t, float frequency) {
	vec3 dif = p - pos;
	return sin(t*frequency) *  magnitude * (p - pos) / pow(length(p - pos), 2.0);
}
vec3 sfWindUp(vec3 p, float ylim) {
	return p.y < ylim ? -12.0 * sfG() : vec3(0.0);
}
vec3 sfWindRight(vec3 p, float xlim) {
	return p.x < xlim ? vec3(12.0, 0.0, 0.0) : vec3(0.0);
}
vec3 sfWindLeft(vec3 p, float xlim) {
	return p.x > xlim ? vec3(-12.0, 0.0, 0.0) : vec3(0.0);
}
vec3 sfWindIn(vec3 p, float xlim) {
	return p.z > xlim ? vec3(0.0, 0.0, -12.0) : vec3(0.0);
}
vec3 sfWindOut(vec3 p, float xlim) {
	return p.z < xlim ? vec3(0.0, 0.0, 12.0) : vec3(0.0);
}
vec3 sfXZLim(vec3 p, float rlim, float mag) {
	vec2 a = -1 * p.xz;
	return length(a) > rlim ? vec3(a.x, 0.0, a.y) * mag : vec3(0.0);
}
vec3 sfDrag(vec3 vel) {
	return -vel*0.5;
}
vec3 sfTurbulancy(float t, float magnitude) {
	return vec3(sin(t), cos(t), sin(t) * cos(t)) * magnitude;
}
vec4 sff(vec3 poz, vec3 vel, float t) {
	
	vec3 res = vec3(0.0);
	res += sfG();
	res += sfReppel(poz, vec3(0.0, 0.0, 0.0), -500.0);
//	res += sfReppelPulse(poz, vec3(0.0, 0.0, 0.0), 500.0, t, .5);
	res += sfReppelPulse(poz, vec3(10.0, 10.0, 0.0), 224.0, t, 150.0);
	res += sfDrag(vel*2.0);
	res += sfWindUp(poz, -5.0);
//	res += sfWindRight(poz, -20.0);
//	res += sfWindLeft(poz, 20.0);
//	res += sfWindIn(poz, -20.0);
//	res += sfWindOut(poz, 20.0);
	res += sfXZLim(poz, 50.0, .3 );
//	res += sfTurbulancy(t + length(vel), .01);
	return vec4(res, 0.0);
}
void update(uint i, float dt, inout Cest c)  {
	c.vel += sff(c.poz.xyz, c.vel.xyz, iTime) * dt;
	c.poz += c.vel * dt;
}
void main() {
	uint i = getI2();

	if (cest.cests[i].status == 0.0)
	    init(i);
	if (cest.cests[i].status == 1.0) {
		cest.cests[i].sta += iDeltaTime;
		update(i, iDeltaTime, cest.cests[i]);
	}
	if (cest.cests[i].sta > deadAfter) {
		cest.cests[i].status = 2.0;
		cest.cests[i].sta += iDeltaTime;

	}
	if (cest.cests[i].sta > deadAfter * 1.01) {
		cest.cests[i].status = 0.0;
	}
}

