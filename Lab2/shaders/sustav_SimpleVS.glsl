#version 430

uniform mat4 matGeo;	
uniform mat4 matVP;
uniform vec3 CamPos;
uniform vec3 CamDir;

layout (location = 0) in vec3 pos;
layout (location = 1) in vec4 poz_off;
layout (location = 2) in vec4 vel_off;
layout (location = 3) in vec4 sta;
layout (location = 4) in vec4 col;
out vec4 vel;
out vec4 cest_col;	

void main() {
   gl_Position = matVP *  matGeo * vec4(pos + poz_off.xyz, 1.0);
   vel = vel_off;
   cest_col = col;
}
