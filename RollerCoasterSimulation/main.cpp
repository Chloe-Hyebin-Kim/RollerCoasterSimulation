#include "stdafx.h"

#include "RollerCoasterSimulation.h"
#include "CatmullRomSpline.h"

RollerCoasterSimulation g_Simulation;

void Display()
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glMatrixMode(GL_MODELVIEW);//포즈 설정
    glLoadIdentity();

    // 일단 track 전체가 보이도록 고정 카메라
    // gluLookAt(30.0, 25.0, 45.0, 10.0, 6.0, -5.0, 0.0, 1.0, 0.0);
    g_Simulation.ApplyFirstPersonCamera();
    g_Simulation.Render();

    glutSwapBuffers();
}

void Reshape(int width, int height)
{
    if (height == 0)
    {
        height = 1;
    }

    float aspect = static_cast<float>(width) / static_cast<float>(height);

    glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);//카메라 시야 설정
    glLoadIdentity();

    gluPerspective(60.0,aspect,0.1,1000.0);

	glMatrixMode(GL_MODELVIEW); //카메라포즈 설정
}

void Timer(int value)
{
    g_Simulation.Update(0.016f);

    glutPostRedisplay();
    glutTimerFunc(16, Timer, 0);
}

void InitOpenGL()
{
    glEnable(GL_DEPTH_TEST);

    glClearColor(0.05f,0.05f,0.08f,1.0f);
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);

    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(640, 360);
    glutCreateWindow("Roller Coaster Simulation");

    InitOpenGL();

    g_Simulation.Initialize();

    glutDisplayFunc(Display);
    glutReshapeFunc(Reshape);
    glutTimerFunc(16, Timer, 0);

    glutMainLoop();

    return 0;
}
