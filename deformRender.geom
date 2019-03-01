#version 330 core
layout (triangles) in;
layout (triangle_strip, max_vertices = 3) out;

in VS_OUT {
    vec3 fragPos;
} gs_in[];

out vec3 FragPos;
out vec3 Normal;

vec3 GetNormal()
{
   vec3 a = vec3(gl_in[0].gl_Position) - vec3(gl_in[1].gl_Position);
   vec3 b = vec3(gl_in[2].gl_Position) - vec3(gl_in[1].gl_Position);
   return normalize(cross(a, b));
}  

void main() {
	Normal = GetNormal();

	FragPos = gs_in[0].fragPos;
	gl_Position = gl_in[0].gl_Position;
	EmitVertex();
	FragPos = gs_in[1].fragPos;
	gl_Position = gl_in[1].gl_Position;
	EmitVertex();
	FragPos = gs_in[2].fragPos;
	gl_Position = gl_in[2].gl_Position;
	EmitVertex();
	EndPrimitive();
}