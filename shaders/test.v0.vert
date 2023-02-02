#version 330 core
layout (location = 0) in vec3 vPosition;
layout (location = 1) in vec3 vNormal;
layout (location = 2) in vec2 vTexCoord;

uniform mat4 model;
uniform mat4 view;
uniform mat4 proj;
uniform mat4 modelView;
uniform mat4 modelViewProj;
uniform mat4 normalMatrix;

out vec2 TexCoord;
out vec3 position;

void main()
{
    TexCoord =  vTexCoord;
    position = (modelViewProj * vec4(vPosition, 1.0)).xyz;
    position = vPosition;
    gl_Position = modelViewProj * vec4(vPosition, 1.0);
}
