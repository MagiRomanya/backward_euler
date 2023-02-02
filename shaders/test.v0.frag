#version 330 core

in vec2 TexCoord;
in vec3 position;

uniform sampler2D gandalf;

out vec4 FragColor;

void main()
{
    // FragColor = vec4(1.0f, 0.5f, 0.2f, 1.0f);
    // FragColor = vec4(TexCoord, 0.0f, 1.0f);
    FragColor = vec4(position, 1.0f);
    FragColor = texture(gandalf, TexCoord);
}
