// Includes and defs --------------------------

// openGL functionality
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// shader helper
#include "shader.h"
// math
#include <algorithm>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
// image loading
#define STB_IMAGE_IMPLEMENTATION
#include <stb/stb_image.h>
// Mesh stuff
#include <iostream>
#include <fstream>
#include <string>

#include <utility>
#include <vector>

// Functions ---------------------------------

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

// Global variables ---------------------------
bool spaceheld = false;

// window
const int SCR_WIDTH = 1280;
const int SCR_HEIGHT = 720;

// camera
glm::vec3 cameraPos = glm::vec3(0.0f, 1.0f, 2.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
float yaw = -90.0f, pitch = 0.0f;
bool firstMouse = true;
float lastX = 400, lastY = 300;

// time
float deltaTime = 0.0f;	// Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

// deformable mesh
int numPoints;
glm::vec3 * pointCoords = NULL;
glm::vec3 * lastPointCoords = NULL;
int numEdges;
unsigned int * edgeIndices = NULL; // Every 2 ints are the indices of and edge
int numTetra;
unsigned int * tetraIndices = NULL; // Every 4 ints are the indices of a tetrahedron
int numFaces;
unsigned int * faceIndices = NULL; // Every 3 ints are the indices of a face
float * masses;
glm::vec3 * velocities;
glm::vec3 * forces;
float * startLength;
float * startVolume;
	// Parameters
float timestep = 0.001;
float mass = 50.0f;
		   //50.0f
float lowMassLimit = 0.50f;
float kv = 1000000.0f; // Volume perserving constant
		 //1000000.0f
float dampv = 5.0f; // Dampening for volume perserving force
			//5.0f
float kd = 10000.0f; // Distance perserving constant
		 //10000.0f
float dampd = 1.0f; // Dampening for distance perserving force
			//1.0f
float startHeight = 0.75f; 
float deformLimit = 1.0f; //1.0f
				  //1.0f
float cFric = 0.5f;

// Rigid bodies
bool rigid = false;
glm::vec3 COM;
glm::mat4 momInertia;
glm::vec3 rigidForce;
glm::vec3 rigidVel;
glm::vec3 torque;
glm::vec3 angularVel;


//Bounding volume heirarchy
struct node {
	// Bounds that this node covers
	unsigned int xMax, xMin, yMax, yMin, zMax, zMin;
	glm::vec3 maxes;
	glm::vec3 mins;
	// Sub-nodes, should have 2 unless leaf
	node* subNode1 = NULL;
	node* subNode2 = NULL;
	// If leaf need to store which edge you contain
	unsigned int containedFace;
};
node BVHroot;
typedef std::pair <float, unsigned int> posPair;
bool pairCompare(const posPair& l, const posPair& r)
{
	return l.first < r.first;
}
void createNode(node* n, posPair* xPos, posPair* yPos, posPair* zPos);
void deleteNode(node* n);
void buildBVH();
float maxEdgeLength;
std::vector<unsigned int> searchBVH(glm::vec3 pos, float rad, node* n);


int main()
{
	// Before loop starts ---------------------
	// glfw init
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// glfw window creation
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "Deformable Body", NULL, NULL);
	glfwMakeContextCurrent(window);

	// register callbacks
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	//Register mouse movement callback
	glfwSetCursorPosCallback(window, mouse_callback);

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Initialize glad
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

	// Enable openGL settings
	//glEnable(GL_CULL_FACE);
	//glFrontFace(GL_CW);
	glEnable(GL_DEPTH_TEST);

	// Setup ----------------------------------
	
	// deformable mesh

	// load points
	glm::mat4 transform = glm::mat4(1.0f);
	transform = glm::translate(transform, glm::vec3(0.0f, startHeight + 0.5f, 0.0f));
	transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

	glm::mat4 transform2 = glm::mat4(1.0f);
	transform2 = glm::translate(transform2, glm::vec3(1.0f, startHeight + 2.5f, -2.0f));
	transform2 = glm::rotate(transform2, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	//transform2 = glm::rotate(transform2, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

	int pointOff;

	std::ifstream pointFile;
	pointFile.open("mesh/spot.1.node");
	if (pointFile.is_open())
	{
		int index;
		float x, y, z;
		// get total number of points
		pointFile >> numPoints;
		pointOff = numPoints;
		numPoints *= 2;
		pointCoords = new glm::vec3[numPoints];
		lastPointCoords = new glm::vec3[numPoints];
		// Read a few times to get through first line
		int dimension, numAttrib, boundary;
		pointFile >> dimension >> numAttrib >> boundary;
		// now read actual data
		while (pointFile >> index >> x >> y >> z)
		{
			glm::vec3 startPos = glm::vec3(x, y, z);
			glm::vec3 transformed = transform * glm::vec4(startPos, 1.0f);
			glm::vec3 newPos = glm::vec3(transformed[0], transformed[1], transformed[2]);
			pointCoords[index] = newPos;
			lastPointCoords[index] = newPos;

			glm::vec3 startPos2 = glm::vec3(x, y, z);
			glm::vec3 transformed2 = transform2 * glm::vec4(startPos2, 1.0f);
			glm::vec3 newPos2 = glm::vec3(transformed2[0], transformed2[1], transformed2[2]);
			pointCoords[pointOff + index] = newPos2;
			lastPointCoords[pointOff + index] = newPos2;
		}
		pointFile.close();
	}
	// load edges
	std::ifstream edgeFile;
	edgeFile.open("mesh/spot.1.edge");
	if (edgeFile.is_open())
	{
		unsigned int edgeIndex, p1, p2, boundary;
		edgeFile >> numEdges;
		int edgeOff = numEdges;
		numEdges *= 2;
		edgeIndices = new unsigned int[2 * numEdges];
		edgeFile >> boundary;
		while (edgeFile >> edgeIndex >> p1 >> p2 >> boundary)
		{
			edgeIndices[edgeIndex * 2] = p1;
			edgeIndices[edgeIndex * 2 + 1] = p2;

			edgeIndices[edgeOff * 2 + edgeIndex * 2] = pointOff + p1;
			edgeIndices[edgeOff * 2 + edgeIndex * 2 + 1] = pointOff + p2;
		}
		edgeFile.close();
	}
	// load tetrahedron
	std::ifstream tetraFile;
	tetraFile.open("mesh/spot.1.ele");
	if (tetraFile.is_open())
	{
		tetraFile >> numTetra;
		int tetraOff = numTetra;
		numTetra *= 2;
		tetraIndices = new unsigned int[4 * numTetra];
		unsigned int numAttrib, boundary;
		tetraFile >> numAttrib >> boundary;
		unsigned int tetraIndex, p1, p2, p3, p4;
		while (tetraFile >> tetraIndex >> p1 >> p2 >> p3 >> p4)
		{
			tetraIndices[tetraIndex * 4] = p1;
			tetraIndices[tetraIndex * 4 + 1] = p2;
			tetraIndices[tetraIndex * 4 + 2] = p3;
			tetraIndices[tetraIndex * 4 + 3] = p4;

			tetraIndices[tetraOff*4 + tetraIndex * 4] = pointOff + p1;
			tetraIndices[tetraOff * 4 + tetraIndex * 4 + 1] = pointOff + p2;
			tetraIndices[tetraOff * 4 + tetraIndex * 4 + 2] = pointOff + p3;
			tetraIndices[tetraOff * 4 + tetraIndex * 4 + 3] = pointOff + p4;
		}
		tetraFile.close();
	}
	// load faces
	std::ifstream faceFile;
	faceFile.open("mesh/spot.1.face");
	if (faceFile.is_open())
	{
		unsigned int faceIndex, p1, p2, p3, boundary;
		faceFile >> numFaces;
		int faceOff = numFaces;
		numFaces *= 2;
		faceIndices = new unsigned int[3 * numFaces];
		faceFile >> boundary;
		while (faceFile >> faceIndex >> p1 >> p2 >> p3 >> boundary)
		{
			faceIndices[faceIndex * 3] = p1;
			faceIndices[faceIndex * 3 + 1] = p2;
			faceIndices[faceIndex * 3 + 2] = p3;

			faceIndices[faceOff*3 + (faceIndex * 3)] = pointOff + p1;
			faceIndices[faceOff*3 + (faceIndex * 3 + 1)] = pointOff + p2;
			faceIndices[faceOff*3 + (faceIndex * 3 + 2)] = pointOff + p3;
		}
		faceFile.close();
	}

	// Setup data
	masses = new float[numPoints];
	velocities = new glm::vec3[numPoints];
	forces = new glm::vec3[numPoints];
	startLength = new float[numEdges];
	startVolume = new float[numTetra];

	// Calculate masses
		// First have each point store the number of tetrahedra that contain it and sum those tetrahedron's volumes
		// initialize masses and tetraUses
	unsigned int * tetraUses = new unsigned int[numPoints];
	float totalMass = 0.0f;
	for (int i = 0; i < numPoints; i++)
	{
		masses[i] = 0.0f;
		tetraUses[i] = 0;
	}
		// sum masses and tetra uses
	for (int i = 0; i < numTetra * 4; i += 4)
	{
		glm::vec3 v0 = pointCoords[tetraIndices[i]] - pointCoords[tetraIndices[i + 1]];
		glm::vec3 v1 = pointCoords[tetraIndices[i]] - pointCoords[tetraIndices[i + 2]];
		glm::vec3 v2 = pointCoords[tetraIndices[i]] - pointCoords[tetraIndices[i + 3]];
		float area = abs(glm::dot(v0, glm::cross(v1, v2)) / 2.0f);

		tetraUses[tetraIndices[i]] += 1;
		masses[tetraIndices[i]] += area;
		totalMass += area;

		// Also get default volume
		startVolume[i/4] = area;
	}
		// average masses
	for (int i = 0; i < numPoints; i++)
	{
		float m = (masses[i] / (float)tetraUses[i]);
		if (tetraUses[i] == 0)
		{
			//std::cout << masses[i] << std::endl;
			m = 0.0f;
		}
		m = m / totalMass;
		m *= mass;
		if (m < lowMassLimit)
		{
			m = 1.0f;
		}
		masses[i] = m;
	}
	delete tetraUses;

	// get COM
	for (int i = 0; i < numPoints; i++)
	{
		COM += pointCoords[i];
	}
	COM = COM * (1.0f / numPoints);
	//std::cout << COM[0] << ", " << COM[1] << ", " << COM[2] << std::endl;


	// get default length of edges
	maxEdgeLength = -INFINITY;
	for (int i = 0; i < numEdges; i++)
	{
		glm::vec3 offset = pointCoords[edgeIndices[i*2]] - pointCoords[edgeIndices[i*2 + 1]];
		startLength[i] = glm::length(offset);

		if (startLength[i] > maxEdgeLength)
			maxEdgeLength = startLength[i];
	}


	// Now put info into a form openGL can use
	unsigned int deformVAO, deformPointBuffer, deformElementBuffer;
	glGenVertexArrays(1, &deformVAO);
	glBindVertexArray(deformVAO);
	glGenBuffers(1, &deformPointBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, deformPointBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * numPoints, pointCoords, GL_STREAM_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);
	glGenBuffers(1, &deformElementBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, deformElementBuffer);
	//glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * numEdges * 2, edgeIndices, GL_STATIC_DRAW);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * numFaces * 3, faceIndices, GL_STATIC_DRAW);

	// set up shader for deformable body
	Shader deformShader("deformRender.vert", "deformRender.frag", "deformRender.geom");
	
	

	// Floor
	float floorVertices[] = {
		//x			y		z			nX		nY		nZ		t		s
		-20.0f,		0.0f,	-20.0f,		0.0f,	1.0f,	0.0f,	0.0f, 0.0f,
		-20.0f,		0.0f,	20.0f,		0.0f,	1.0f,	0.0f,	0.0f, 8.0f,
		 20.0f,		0.0f,	-20.0f,		0.0f,	1.0f,	0.0f,	8.0f, 0.0f,
		 20.0f,		0.0f,	20.0f,		0.0f,	1.0f,	0.0f,	8.0f, 8.0f
	};
	int floorIndices[] = {
		0, 1, 2,
		1, 3, 2
	};
	// Buffer stuff for floor
	unsigned int floorVAO, floorVBO, floorEBO;
	glGenVertexArrays(1, &floorVAO);
	glBindVertexArray(floorVAO);
	glGenBuffers(1, &floorVBO);
	glBindBuffer(GL_ARRAY_BUFFER, floorVBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(floorVertices), floorVertices, GL_STATIC_DRAW);
	glGenBuffers(1, &floorEBO);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, floorEBO);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(floorIndices), floorIndices, GL_STATIC_DRAW);
	// Tell OpenGL how to use vertex data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0); //Uses whatever VBO is bound to GL_ARRAY_BUFFER
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(0);
	glEnableVertexAttribArray(1);
	glEnableVertexAttribArray(2);

	// Floor texture
	// Set up textures
	unsigned int gridTexture;
	glGenTextures(1, &gridTexture);
	glActiveTexture(GL_TEXTURE0);
	glBindTexture(GL_TEXTURE_2D, gridTexture);
	// load and generate the texture
	int width, height, nrChannels;
	stbi_set_flip_vertically_on_load(true);
	unsigned char *data = stbi_load("grid.png", &width, &height, &nrChannels, 0);
	if (data)
	{
		glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, width, height, 0, GL_RGB, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);
	}
	else
	{
		std::cout << "Failed to load texture" << std::endl;
	}
	stbi_image_free(data);

	//Shader
	Shader texturedShader("textured.vert", "textured.frag");


	// Build BHV
	//buildBVH();


	// render loop ----------------------------
	while (!glfwWindowShouldClose(window))
	{
		// Set deltaT
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;
		//std::cout << (1.0f/deltaTime) << std::endl;
		//if (deltaTime > timestep)
		{
			//deltaTime = timestep;
		}
		deltaTime = timestep;

		// input
		processInput(window);

		// processing
		// internal forces
		if (!rigid)
		{
			// Compute volume preserving force
			for (int i = 0; i < numTetra; i += 1)
			{
				glm::vec3 e1 = pointCoords[tetraIndices[i * 4 + 1]] - pointCoords[tetraIndices[i * 4]];
				glm::vec3 e2 = pointCoords[tetraIndices[i * 4 + 2]] - pointCoords[tetraIndices[i * 4]];
				glm::vec3 e3 = pointCoords[tetraIndices[i * 4 + 3]] - pointCoords[tetraIndices[i * 4]];
				float cv = ((glm::dot(e1, glm::cross(e2, e3)) / 6.0) - startVolume[i]);

				//* // With dampening
				glm::vec3 dir, relVel;
				dir = glm::normalize(glm::cross(e2 - e1, e3 - e1));
				relVel = (3.0f * velocities[tetraIndices[i * 4]] - velocities[tetraIndices[i * 4 + 1]] - velocities[tetraIndices[i * 4 + 2]] - velocities[tetraIndices[i * 4 + 3]]);
				forces[tetraIndices[i * 4]] += (kv * cv - dampv * glm::dot(relVel, dir)) * dir;

				dir = glm::normalize(glm::cross(e3, e2));
				relVel = (3.0f * velocities[tetraIndices[i * 4 + 1]] - velocities[tetraIndices[i * 4]] - velocities[tetraIndices[i * 4 + 2]] - velocities[tetraIndices[i * 4 + 3]]);
				forces[tetraIndices[i * 4 + 1]] += (kv * cv - dampv * glm::dot(relVel, dir)) * dir;

				dir = glm::normalize(glm::cross(e1, e3));
				relVel = (3.0f * velocities[tetraIndices[i * 4 + 2]] - velocities[tetraIndices[i * 4 + 1]] - velocities[tetraIndices[i * 4]] - velocities[tetraIndices[i * 4 + 3]]);
				forces[tetraIndices[i * 4 + 2]] += (kv * cv - dampv * glm::dot(relVel, dir)) * dir;

				dir = glm::normalize(glm::cross(e2, e1));
				relVel = (3.0f * velocities[tetraIndices[i * 4 + 3]] - velocities[tetraIndices[i * 4 + 1]] - velocities[tetraIndices[i * 4 + 2]] - velocities[tetraIndices[i * 4]]);
				forces[tetraIndices[i * 4 + 3]] += (kv * cv - dampv * glm::dot(relVel, dir)) * dir;
				//*/
				/* // Without dampening
				forces[tetraIndices[i*4]] += kv * cv * glm::cross(e2-e1, e3-e1);
				forces[tetraIndices[i*4 + 1]] += kv * cv * glm::cross(e3, e2);
				forces[tetraIndices[i*4 + 2]] += kv * cv * glm::cross(e1, e3);
				forces[tetraIndices[i*4 + 3]] += kv * cv * glm::cross(e2, e1);
				//*/
			}
			// Compute distance preserving force
			for (int i = 0; i < numEdges; i += 1)
			{
				glm::vec3 offset = pointCoords[edgeIndices[i * 2]] - pointCoords[edgeIndices[i * 2 + 1]];
				glm::vec3 dir = glm::normalize(offset);
				glm::vec3 velOff = velocities[edgeIndices[i * 2]] - velocities[edgeIndices[i * 2 + 1]];
				float len = glm::length(offset);
				if (len == len)
				{
					float force = kd * (len - startLength[i]);
					float damp = dampd * glm::dot(dir, velOff);

					forces[edgeIndices[i * 2]] += -(force + damp) * dir;
					forces[edgeIndices[i * 2 + 1]] += (force + damp) * dir;
				}

			}
		}

		// External forces

		/*
		for (int i = 0; i < numFaces; i++)
		{
			// Drag
			// f = -1/2p*length(v)*DragCoef*area*normal
			glm::vec3 airVel = glm::vec3(10000.0f, 0.0f, 10000.0f);
			// v is velocity of face - velocity of the air
			glm::vec3 v = (velocities[faceIndices[i*3]] + velocities[faceIndices[i * 3 + 1]] 
						+ velocities[faceIndices[i * 3 + 2]]) / 3.0f + airVel;
			// use cross product and normalize to get n
			glm::vec3 cross = glm::cross((pointCoords[faceIndices[i * 3]] - pointCoords[faceIndices[i * 3 + 1]]), 
							(pointCoords[faceIndices[i * 3]] - pointCoords[faceIndices[i * 3 + 2]])); //Pull this out to reuse
			glm::vec3 n = glm::normalize(cross);
			// area of face is half of the area of parallelogram, dot this with velocity to get area exposed to flow
			float a = glm::dot((0.5f * cross), glm::normalize(v));
			// put all together to get drag
			glm::vec3 dragForce = -0.5f * 1.0f * glm::length(v) * 1.0f * a * n;
			// Give each point on face 1/3 of force
			forces[faceIndices[i * 3]] += dragForce / 3.0f;
			forces[faceIndices[i * 3 + 1]] += dragForce / 3.0f;
			forces[faceIndices[i * 3 + 2]] += dragForce / 3.0f;
		}
		*/


		for (int i = 0; i < numPoints; i++)
		{
			// Friction
			if (pointCoords[i][1] <= 0.001)
			{
				float normalForce = abs(forces[i][1]);
				glm::vec3 vDir = glm::normalize(glm::vec3(velocities[i][0], 0.0f, velocities[i][2]));
				//glm::vec3 vDir = glm::normalize(velocities[i]);

				//Apply normal
				forces[i] += glm::vec3(0.0f, normalForce, 0.0f);
				//Apply friction
				forces[i] += cFric * normalForce * -vDir;

			}
		}

			// integrate forces
		for (int i = 0; i < numPoints; i++)
		{
			if (!rigid)
			{
				// Semi-implicit Eular
				//*
				pointCoords[i] = pointCoords[i] + velocities[i] * deltaTime;
				velocities[i] += glm::vec3(0.0f, -9.8f, 0.0f) * deltaTime;
				velocities[i] += forces[i] / masses[i] * deltaTime;
				//*/
				// Verlet
				/*
				if (spaceheld)
				{
					pointCoords[i] -= (pointCoords[i] - lastPointCoords[i]) * 1.0f;
					//forces[i] *= 0.09;//glm::vec3(0.0f);
				}


				glm::vec3 currentPos = pointCoords[i];
				forces[i] += glm::vec3(0.0f, -9.8f, 0.0f) * masses[i];
				pointCoords[i] = 2.0f * pointCoords[i] - lastPointCoords[i] + deltaTime * deltaTime * forces[i] / masses[i];
				velocities[i] = (pointCoords[i] - lastPointCoords[i]) / (2.0f * deltaTime);
				//velocities[i] = (pointCoords[i] - currentPos) / (deltaTime);
				lastPointCoords[i] = currentPos;
				//*/
			}
			else
			{
				// rigid
				rigidForce += forces[i];
				torque += glm::cross(forces[i], pointCoords[i] - COM);
			}
			// clear all forces
			forces[i] = glm::vec3(0.0f);
		}

		// rebuild BVH
		//buildBVH();

		for (int i = 0; i < numEdges; i += 1)
		{
			// deformation limits
			//*
			glm::vec3 offset = pointCoords[edgeIndices[i * 2]] - pointCoords[edgeIndices[i * 2 + 1]];
			glm::vec3 dir = glm::normalize(offset);
			float len = glm::length(offset);
			if (len==len)
			{
				if (len > deformLimit * startLength[i])
				{
					float massRatio = masses[edgeIndices[i * 2]] / (masses[edgeIndices[i * 2]] + masses[edgeIndices[i * 2 + 1]]);
					pointCoords[edgeIndices[i * 2]] -= massRatio * (len - deformLimit * startLength[i]) * dir;
					pointCoords[edgeIndices[i * 2 + 1]] += (1-massRatio) * (len - deformLimit * startLength[i]) * dir;
				}
			}
			//*/

			// Collisions with faces
			/*
			//for (int j = 0; j < numFaces; j++)
			std::vector<unsigned int> e;
			e = searchBVH(pointCoords[edgeIndices[i * 2]], len * 1.0f, &BVHroot);
			if (e.size() != 0)
			{
				//std::cout << "size: " << e.size() << std::endl;
			}
			for (int j = 0; j < e.size(); j++)
			{
				//std::cout << "got here" << std::endl;
				// based on http://geomalgorithms.com/a06-_intersect-2.html
				glm::vec3 p0 = pointCoords[edgeIndices[i * 2]];
				glm::vec3 p1 = pointCoords[edgeIndices[i * 2 + 1]];
				glm::vec3 v0 = pointCoords[faceIndices[e[j]]];
				glm::vec3 v1 = pointCoords[faceIndices[e[j]]];
				glm::vec3 v2 = pointCoords[faceIndices[e[j]]];

				glm::vec3 u = v1 - v0; // One side of triangle
				glm::vec3 v = v2 - v0; // 2nd side
				glm::vec3 n = glm::cross(u, v); // Normal of triangle

				// Get point on line that lies in the plane of the face
				// Point on edge defined by p(r) = p0 + r*(p1 - p0)
				float r = glm::dot(n, (v0 - p0)) / glm::dot(n, (p1 - p0));
				glm::vec3 intersect = p0 + r * (p1 - p0);

				// Now see if that point in the face
				// face is defined by p(s,t) were p is in the triangle if s>=0, t>=0 and s+t<=1
				glm::vec3 w = intersect - v0;
				float denom = (glm::dot(u, v)*glm::dot(u, v) - glm::dot(u, u)*glm::dot(v, v));
				float t = -1.0f;
				float s = -1.0f;
				if (denom != 0)
				{
					s = (glm::dot(u, v) * glm::dot(w, v) - glm::dot(v, v), glm::dot(w, u)) / denom;
					t = (glm::dot(u, v) * glm::dot(w, u) - glm::dot(u, u), glm::dot(w, v)) / denom;
				}

				if ((t >= 0) && (s >= 0) && (t + s <= 1))
				{
					// Edge does intersect face
					std::cout << "intersect" << std::endl;
					float d1 = glm::dot(p0 - v0, n);
					float d2 = glm::dot(p0 - v0, n);
					glm::vec3 displacement;
					if (d1 < d2)
					{
						displacement = d1 * n;
					}
					else
					{
						displacement = d2 * n;
					}

					// Move edge outside of face
					pointCoords[edgeIndices[i * 2]] += displacement;
					pointCoords[edgeIndices[i * 2 + 1]] += displacement;

					//forces[edgeIndices[i * 2]] += displacement * 1.0f;
					//forces[edgeIndices[i * 2 + 1]] += displacement * 1.0f;
				}

			}
			//*/
		}
		
		for (int i = 0; i < numPoints; i++)
		{
			// Check for collisions with ground
			if (pointCoords[i][1] <= 0.0f)
			{
				pointCoords[i][1] = 0.0f;
				//velocities[i] = (pointCoords[i] - lastPointCoords[i]) / (2.0f * deltaTime);
				//velocities[i] = glm::vec3(0.0f);
				//velocities[i] *= 0.02f;
			}
		}

		// Put data into buffers for openGL
		glBindVertexArray(deformVAO);
		glBindBuffer(GL_ARRAY_BUFFER, deformPointBuffer);
		glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * numPoints, pointCoords, GL_STREAM_DRAW);


		// rendering commands here
		glClearColor(0.592f, 0.808f, 0.922f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 model = glm::mat4(1.0f);

		deformShader.use();
		glm::vec3 lightPos(0.0f, -2.0f, 2.0f);
		deformShader.setVec3("light.position", lightPos);
		deformShader.setVec3("viewPos", cameraPos);

		// light properties
		glm::vec3 lightColor(1.0, 1.0, 1.0);
		glm::vec3 diffuseColor = lightColor * glm::vec3(0.5f); // decrease the influence
		glm::vec3 ambientColor = diffuseColor * glm::vec3(0.2f); // low influence
		deformShader.setVec3("light.ambient", ambientColor);
		deformShader.setVec3("light.diffuse", diffuseColor);
		deformShader.setVec3("light.specular", 1.0f, 1.0f, 1.0f);

		// material properties
		deformShader.setVec3("material.ambient", 1.0f, 0.5f, 0.31f);
		deformShader.setVec3("material.diffuse", 1.0f, 0.5f, 0.31f);
		deformShader.setVec3("material.specular", 0.5f, 0.5f, 0.5f); // specular lighting doesn't have full effect on this object's material
		deformShader.setFloat("material.shininess", 36.0f);

		deformShader.setMat4("view", view);
		deformShader.setMat4("projection", projection);
		//model = glm::rotate(model, glm::radians(-90.0f), glm::vec3(0.0f, 1.0f, 0.0f));
		deformShader.setMat4("model", model);

		glBindVertexArray(deformVAO);
		glDrawElements(GL_LINES, numEdges * 2, GL_UNSIGNED_INT, 0);
		glDrawElements(GL_TRIANGLES, numFaces * 3, GL_UNSIGNED_INT, 0);

		glActiveTexture(GL_TEXTURE0);
		texturedShader.use();
		texturedShader.use();
		texturedShader.setMat4("view", view);
		texturedShader.setMat4("projection", projection);
		texturedShader.setMat4("model", model);

		texturedShader.setVec3("viewPos", cameraPos);
		texturedShader.setVec3("light.direction", glm::vec3(0.0f, -1.0f, 1.0f));
		texturedShader.setVec3("light.ambient", glm::vec3(0.3f, 0.3f, 0.3f));
		texturedShader.setVec3("light.diffuse", glm::vec3(0.9f, 0.9f, 0.9f));
		texturedShader.setVec3("light.specular", glm::vec3(1.0f, 1.0f, 1.0f));

		texturedShader.setInt("material.diffuse", 0);
		texturedShader.setVec3("material.specular", glm::vec3(0.5f, 0.5f, 0.5f));
		texturedShader.setFloat("material.shininess", 0.1f);
		glBindVertexArray(floorVAO);
		glDrawElements(GL_TRIANGLES, 6, GL_UNSIGNED_INT, 0);

		// check and call events and swap the buffers
		glfwPollEvents();
		glfwSwapBuffers(window);
	}

	glfwTerminate();

	return 0;
}

// This function is called whenever window is resized
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	glViewport(0, 0, width, height);
}

// Process all ketboard input here
void processInput(GLFWwindow *window)
{
	if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
		glfwSetWindowShouldClose(window, true);

	float cameraSpeed = 5.0f * deltaTime;
	if (glfwGetKey(window, GLFW_KEY_W) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_S) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraFront;
	if (glfwGetKey(window, GLFW_KEY_A) == GLFW_PRESS)
		cameraPos -= glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_D) == GLFW_PRESS)
		cameraPos += glm::normalize(glm::cross(cameraFront, cameraUp)) * cameraSpeed;
	if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		cameraPos += cameraSpeed * cameraUp;
	if (glfwGetKey(window, GLFW_KEY_LEFT_CONTROL) == GLFW_PRESS)
		cameraPos -= cameraSpeed * cameraUp;

	spaceheld = (glfwGetKey(window, GLFW_KEY_SPACE) == GLFW_PRESS);
}

void mouse_callback(GLFWwindow* window, double xpos, double ypos)
{
	if (firstMouse)
	{
		lastX = xpos;
		lastY = ypos;
		firstMouse = false;
	}

	float xoffset = xpos - lastX;
	float yoffset = lastY - ypos;
	lastX = xpos;
	lastY = ypos;

	float sensitivity = 0.2;
	xoffset *= sensitivity;
	yoffset *= sensitivity;

	yaw += xoffset;
	pitch += yoffset;

	if (pitch > 89.0f)
		pitch = 89.0f;
	if (pitch < -89.0f)
		pitch = -89.0f;

	glm::vec3 front;
	front.x = cos(glm::radians(yaw)) * cos(glm::radians(pitch));
	front.y = sin(glm::radians(pitch));
	front.z = sin(glm::radians(yaw)) * cos(glm::radians(pitch));
	cameraFront = glm::normalize(front);
}

void createNode(node* n, posPair* xPos, posPair* yPos, posPair* zPos)
{
	int numX = n->xMax - n->xMin;
	int numY = n->yMax - n->yMin;
	int numZ = n->zMax - n->zMin;
//	std::cout << numX << ',' << numY << ',' << numZ << std::endl;
		
	n->maxes = glm::vec3(xPos[n->xMax].first, yPos[n->yMax].first, zPos[n->zMax].first);
	n->mins = glm::vec3(xPos[n->xMin].first, yPos[n->yMin].first, zPos[n->zMin].first);
	//std::cout << n->maxes[0] << ',' << n->mins[0] << std::endl;
		
	if (numX == 0 && numY == 0 && numZ == 0)
	{
		n->containedFace = xPos[n->xMax].second;
	}
	else
	{
		
		int midX = n->xMin + (numX / 2);
		int midY = n->yMin + (numY / 2);
		int midZ = n->zMin + (numZ / 2);

		node* n1 = new node;
		n->subNode1 = n1;
		n1->xMin = n->xMin;
		n1->xMax = midX;
		n1->yMin = n->yMin;
		n1->yMax = midY;
		n1->zMin = n->zMin;
		n1->zMax = midZ;
		//std::cout << "child go" << std::endl;
		createNode(n1, xPos, yPos, zPos);
		
		node* n2 = new node;
		n->subNode2 = n2;
		n2->xMax = n->xMax;
		n2->xMin = midX+1;
		n2->yMax = n->yMax;
		n2->yMin = midY+1;
		n2->zMax = n->zMax;
		n2->zMin = midZ+1;
		createNode(n2, xPos, yPos, zPos);
	}
}

void deleteNode(node* n)
{
	if (n != NULL)
	{
		if (n->subNode1 != NULL)
			deleteNode(n->subNode1);
		if (n->subNode2 != NULL)
			deleteNode(n->subNode2);

		delete n;
	}
}

void buildBVH()
{
	// remove old stuff
	deleteNode(BVHroot.subNode1);
	deleteNode(BVHroot.subNode2);

	// We want to sort faces based on their first vertex, this was arbitrarily chosen
	float mxX = -INFINITY, mxY = -INFINITY, mxZ = -INFINITY,
		mnX = INFINITY, mnY = INFINITY, mnZ = INFINITY;
	posPair *xPos = new posPair[numFaces];
	posPair *yPos = new posPair[numFaces];
	posPair *zPos = new posPair[numFaces];
	for (int i = 0; i < numFaces; i++)
	{
		glm::vec3 pos = pointCoords[faceIndices[i]];
		xPos[i] = posPair(pos[0], i);
		yPos[i] = posPair(pos[1], i);
		zPos[i] = posPair(pos[2], i);
	}
	std::sort(&xPos[0], &xPos[numFaces], pairCompare);
	std::sort(&yPos[0], &yPos[numFaces], pairCompare);
	std::sort(&zPos[0], &zPos[numFaces], pairCompare);
	BVHroot.xMax = numFaces;
	BVHroot.yMax = numFaces;
	BVHroot.zMax = numFaces;
	BVHroot.xMin = 0;
	BVHroot.yMin = 0;
	BVHroot.zMin = 0;

	createNode(&BVHroot, xPos, yPos, zPos);
	delete [] xPos;
	delete [] yPos;
	delete [] zPos;
}

std::vector<unsigned int> searchBVH(glm::vec3 pos, float rad, node* n)
{
	// Is the sphere totally outside your area
	//std::cout << "check" << std::endl;
	bool outX = (pos[0] - rad > n->maxes[0]) || (pos[0] + rad < n->mins[0]);
	bool outY = (pos[1] - rad > n->maxes[1]) || (pos[1] + rad < n->mins[1]);
	bool outZ = (pos[2] - rad > n->maxes[2]) || (pos[2] + rad < n->mins[2]);

	if (n == &BVHroot)
	{
		//std::cout << pos[0] << ',' << rad << ';' << n->mins[0] << ',' << n->maxes[0] << std::endl;
	}

	if (!(outX || outY || outZ))
	{
		//std::cout << "in" << std::endl;
		// if you have children recurse to them
		if (n->subNode1 != NULL)
		{
			// Get vector from both children of faces they contain
			std::vector<unsigned int> facesA = searchBVH(pos, rad, n->subNode1);
			std::vector<unsigned int> facesB = searchBVH(pos, rad, n->subNode2);
			// Combine vectors
			facesA.insert(facesA.end(), facesB.begin(), facesB.end());
			//std::cout << "faces found: "<< facesA.size() << std::endl;
			return facesA;
		}
		else
		{
			// no children so you should be holding a face
			//std::cout << "face in" << std::endl;
			std::vector<unsigned int> v;
			v.push_back(n->containedFace);
			//std::cout << "within range, dist: " << glm::length(pointCoords[faceIndices[n->containedFace]] - pos) << std::endl;
			return v;
		}
	}
	else
	{
		// You aren't close enough to the target to be worth checking
		return std::vector<unsigned int>();
	}
}