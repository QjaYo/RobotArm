#version 330

in  vec4 vPosition;
in  vec4 vColor;
out vec4 color;

uniform mat4 uMat;
uniform vec4 uColor;

void main()
{
	gl_Position  = uMat*vPosition;

	if(uColor.r<0)
		color = vColor;
	else
		color = uColor;
}
