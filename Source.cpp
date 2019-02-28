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
glm::vec3 cameraPos = glm::vec3(0.0f, 3.0f, 3.0f);
glm::vec3 cameraFront = glm::vec3(0.0f, 0.0f, -1.0f);
glm::vec3 cameraUp = glm::vec3(0.0f, 1.0f, 0.0f);
float yaw = 0.0f, pitch = 0.0f;
bool firstMouse = true;
float lastX = 400, lastY = 300;

// time
float deltaTime = 0.0f;	// Time between current frame and last frame
float lastFrame = 0.0f; // Time of last frame

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
	glEnable(GL_DEPTH_TEST);

	// Setup ----------------------------------
	
	// deformable mesh
	// load points
	std::ifstream pointFile;
	pointFile.open("mesh/spot.1.node");
	int numPoints;
	glm::vec3 * pointCoords = NULL;
	if (pointFile.is_open())
	{
		int index;
		float x, y, z;
		// get total number of points
		pointFile >> numPoints;
		pointCoords = new glm::vec3[numPoints];
		// Read a few times to get through first line
		int dimension, numAttrib, boundary;
		pointFile >> dimension >> numAttrib >> boundary;
		// now read actual data
		while (pointFile >> index >> x >> y >> z)
		{
			pointCoords[index] = glm::vec3(x, y, z);
		}
		pointFile.close();
	}
	// load edges
	std::ifstream edgeFile;
	edgeFile.open("mesh/spot.1.edge");
	int numEdges;
	unsigned int * edges = NULL; // Every 2 ints are the indices of and edge
	if (edgeFile.is_open())
	{
		unsigned int edgeIndex, p1, p2, boundary;
		edgeFile >> numEdges;
		edges = new unsigned int[2 * numEdges];
		edgeFile >> boundary;
		while (edgeFile >> edgeIndex >> p1 >> p2 >> boundary)
		{
			edges[edgeIndex * 2] = p1;
			edges[edgeIndex * 2 + 1] = p2;
		}
		edgeFile.close();
	}

	// Now put info into a form openGL can use
	unsigned int deformVAO, deformPointBuffer, deformElementBuffer;
	glGenVertexArrays(1, &deformVAO);
	glBindVertexArray(deformVAO);
	glGenBuffers(1, &deformPointBuffer);
	glBindBuffer(GL_ARRAY_BUFFER, deformPointBuffer);
	glBufferData(GL_ARRAY_BUFFER, sizeof(glm::vec3) * numPoints, pointCoords, GL_STATIC_DRAW);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(glm::vec3), (void*)0);
	glEnableVertexAttribArray(0);
	glGenBuffers(1, &deformElementBuffer);
	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, deformElementBuffer);
	glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned int) * numEdges * 2, edges, GL_STATIC_DRAW);

	// set up shader for deformable body
	Shader deformShader("deformRender.vert", "deformRender.frag");
	
	
	// render loop ----------------------------
	while (!glfwWindowShouldClose(window))
	{
		// Set deltaT
		float currentFrame = glfwGetTime();
		deltaTime = currentFrame - lastFrame;
		lastFrame = currentFrame;

		// input
		processInput(window);

		// processing


		// rendering commands here
		glClearColor(0.592f, 0.808f, 0.922f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		glm::mat4 view = glm::lookAt(cameraPos, cameraPos + cameraFront, cameraUp);
		glm::mat4 projection = glm::mat4(1.0f);
		projection = glm::perspective(glm::radians(45.0f), (float)SCR_WIDTH / (float)SCR_HEIGHT, 0.1f, 100.0f);
		glm::mat4 model = glm::mat4(1.0f);

		deformShader.use();
		deformShader.setMat4("view", view);
		deformShader.setMat4("projection", projection);
		deformShader.setMat4("model", model);

		glBindVertexArray(deformVAO);
		glDrawElements(GL_LINES, numEdges * 2, GL_UNSIGNED_INT, 0);

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

	float cameraSpeed = 2.5f * deltaTime;
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