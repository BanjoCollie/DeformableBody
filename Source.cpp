// Includes and defs --------------------------

// openGL functionality
#include <glad/glad.h>
#include <GLFW/glfw3.h>
// shader helper
#include "shader.h"
// math
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

// Functions ---------------------------------

void framebuffer_size_callback(GLFWwindow* window, int width, int height);
void processInput(GLFWwindow *window);
void mouse_callback(GLFWwindow* window, double xpos, double ypos);

// Global variables ---------------------------

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
float inverseMassLimit = 1.0f;
float kv = 1000000.0f; // Volume perserving constant
float dampv = 5.0f; // Dampening for volume perserving force
float kd = 10000.0f; // Distance perserving constant
float dampd = 1.0f; // Dampening for distance perserving force
float startHeight = 0.75f;


int main()
{
	// Before loop starts ---------------------
	// glfw init
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);

	// glfw window creation
	GLFWwindow* window = glfwCreateWindow(SCR_WIDTH, SCR_HEIGHT, "5611 HW1", NULL, NULL);
	glfwMakeContextCurrent(window);

	// register callbacks
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
	//Register mouse movement callback
	glfwSetCursorPosCallback(window, mouse_callback);

	glfwSetInputMode(window, GLFW_CURSOR, GLFW_CURSOR_DISABLED);

	// Initialize glad
	gladLoadGLLoader((GLADloadproc)glfwGetProcAddress);

	// Enable openGL settings
	glEnable(GL_CULL_FACE);
	glFrontFace(GL_CW);
	glEnable(GL_DEPTH_TEST);

	// Setup ----------------------------------
	
	// deformable mesh

	// load points
	glm::mat4 transform = glm::mat4(1.0f);
	transform = glm::translate(transform, glm::vec3(0.0f, startHeight + 0.5f, 0.0f));
	transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(0.0f, 0.0f, 1.0f));
	transform = glm::rotate(transform, glm::radians(90.0f), glm::vec3(0.0f, 1.0f, 0.0f));

	std::ifstream pointFile;
	pointFile.open("mesh/spot.1.node");
	if (pointFile.is_open())
	{
		int index;
		float x, y, z;
		// get total number of points
		pointFile >> numPoints;
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
		edgeIndices = new unsigned int[2 * numEdges];
		edgeFile >> boundary;
		while (edgeFile >> edgeIndex >> p1 >> p2 >> boundary)
		{
			edgeIndices[edgeIndex * 2] = p1;
			edgeIndices[edgeIndex * 2 + 1] = p2;
		}
		edgeFile.close();
	}
	// load tetrahedron
	std::ifstream tetraFile;
	tetraFile.open("mesh/spot.1.ele");
	if (tetraFile.is_open())
	{
		tetraFile >> numTetra;
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
		faceIndices = new unsigned int[3 * numFaces];
		faceFile >> boundary;
		while (faceFile >> faceIndex >> p1 >> p2 >> p3 >> boundary)
		{
			faceIndices[faceIndex * 3] = p1;
			faceIndices[faceIndex * 3 + 1] = p2;
			faceIndices[faceIndex * 3 + 2] = p3;
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
		m = 1.0f / m;
		if (m > inverseMassLimit)
		{
			m = 1.0f;
		}
		masses[i] = m;
	}
	delete tetraUses;
	// get default length of edges
	for (int i = 0; i < numEdges; i++)
	{
		glm::vec3 offset = pointCoords[edgeIndices[i*2]] - pointCoords[edgeIndices[i*2 + 1]];
		startLength[i] = glm::length(offset);
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
			// clear all forces
		for (int i = 0; i < numPoints; i++)
		{
			forces[i] = glm::vec3(0.0f);
		}
			// Compute volume preserving force
		for (int i = 0; i < numTetra; i += 1)
		{
			glm::vec3 e1 = pointCoords[tetraIndices[i*4+1]] - pointCoords[tetraIndices[i*4]];
			glm::vec3 e2 = pointCoords[tetraIndices[i*4+2]] - pointCoords[tetraIndices[i*4]];
			glm::vec3 e3 = pointCoords[tetraIndices[i*4+3]] - pointCoords[tetraIndices[i*4]];
			float cv = (abs(glm::dot(e1, glm::cross(e2, e3)) / 6.0) - startVolume[i]);

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
			glm::vec3 offset = pointCoords[edgeIndices[i*2]] - pointCoords[edgeIndices[i*2 + 1]];
			glm::vec3 dir = glm::normalize(offset);
			glm::vec3 velOff = velocities[edgeIndices[i * 2]] - velocities[edgeIndices[i * 2 + 1]];
			float len = glm::length(offset);
			if (len == len)
			{
				float force = kd * (len - startLength[i]);
				float damp = dampd * glm::dot(dir, velOff);

				forces[edgeIndices[i * 2]] += -(force+damp) * dir;
				forces[edgeIndices[i * 2 + 1]] += (force + damp) * dir;
			}

		}
			// integrate forces
		for (int i = 0; i < numPoints; i++)
		{
			// Friction
			if (pointCoords[i][1] <= 0.01)
			{
				forces[i] += -velocities[i] / (deltaTime);
			}

			// Semi-implicit Eular
			/*
			velocities[i] += forces[i] * masses[i] * deltaTime;
			velocities[i] += glm::vec3(0.0f, -9.8f, 0.0f) * deltaTime;
			pointCoords[i] = pointCoords[i] + velocities[i] * deltaTime;
			//*/
			// Verlet
			//*
			glm::vec3 currentPos = pointCoords[i];
			forces[i] += glm::vec3(0.0f, -9.8f, 0.0f) / masses[i];
			pointCoords[i] = 2.0f * pointCoords[i] - lastPointCoords[i] + deltaTime * deltaTime * forces[i] * masses[i];
			velocities[i] = (pointCoords[i] - lastPointCoords[i]) / (2.0f * deltaTime);
			lastPointCoords[i] = currentPos;
			//*/
			// Check for collisions with ground
			if (pointCoords[i][1] <= 0.0f)
			{
				pointCoords[i][1] = 0.0f;
				velocities[i] = (pointCoords[i] - lastPointCoords[i]) / (2.0f * deltaTime);
			}
		}

		for (int i = 0; i < numEdges; i += 1)
		{
			// deformation limits
			//*
			glm::vec3 offset = pointCoords[edgeIndices[i * 2]] - pointCoords[edgeIndices[i * 2 + 1]];
			glm::vec3 dir = glm::normalize(offset);
			float len = glm::length(offset);
			if (len==len)
			{
				float alpha = 1.0f;
				if (len > alpha * startLength[i])
				{
					float massRatio = masses[edgeIndices[i * 2]] / (masses[edgeIndices[i * 2]] + masses[edgeIndices[i * 2 + 1]]);
					pointCoords[edgeIndices[i * 2]] -= massRatio * (len - alpha * startLength[i]) * dir;
					pointCoords[edgeIndices[i * 2 + 1]] += (1-massRatio) * (len - alpha * startLength[i]) * dir;
				}
			}
			//*/

			// Collisions with faces
			/*
			for (int j = 0; j < numFaces; j++)
			{
				// based on http://geomalgorithms.com/a06-_intersect-2.html
				glm::vec3 p0 = pointCoords[edgeIndices[i * 2]];
				glm::vec3 p1 = pointCoords[edgeIndices[i * 2 + 1]];
				glm::vec3 v0 = pointCoords[faceIndices[j * 3]];
				glm::vec3 v1 = pointCoords[faceIndices[j * 3 + 1]];
				glm::vec3 v2 = pointCoords[faceIndices[j * 3 + 2]];

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
				}

			}
			//*/
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
		//glDrawElements(GL_LINES, numEdges * 2, GL_UNSIGNED_INT, 0);
		glDrawElements(GL_TRIANGLES, numFaces * 3, GL_UNSIGNED_INT, 0);

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