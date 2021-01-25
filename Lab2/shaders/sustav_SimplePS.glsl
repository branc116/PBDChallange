#version 430

uniform vec2 uResolution;
uniform float uTime;
out vec4 outColor;
in vec4 vel;
in vec4 cest_col;
void main()
{
    vec2 uv = gl_FragCoord.xy;
    outColor.rgb = cest_col.rgb * 0.5;
    outColor.a = 1.0;
}