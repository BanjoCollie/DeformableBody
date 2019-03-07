#version 330 core
out vec4 FragColor;

in vec3 FragCoord;
in vec3 Normal;
in vec2 TexCoord;

struct Light {
    // vec3 position;
    vec3 direction;
  
    vec3 ambient;
    vec3 diffuse;
    vec3 specular;
};

struct Material {
    sampler2D diffuse;
    vec3 specular;    
    float shininess;
}; 

uniform Light light;
uniform Material material;
uniform vec3 viewPos;

void main()
{
	// ambient
    vec3 ambient = light.ambient * texture(material.diffuse, TexCoord).rgb;
  	
    // diffuse
    vec3 norm;
	if (!gl_FrontFacing) {
		norm = -normalize(Normal);
	}
	else {
		norm = normalize(Normal);
	}
	// vec3 lightDir = normalize(light.position - FragPos);
    vec3 lightDir = normalize(-light.direction);  
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = light.diffuse * diff * texture(material.diffuse, TexCoord).rgb;  
    
    // specular
    vec3 viewDir = normalize(viewPos - FragCoord);
    vec3 reflectDir = reflect(-lightDir, norm);  
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), material.shininess);
    vec3 specular = light.specular * spec * material.specular;  
        
    vec3 result = ambient + diffuse;
    //vec3 result = diffuse;
	FragColor = vec4(result, 1.0);
}