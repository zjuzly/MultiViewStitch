#include <opencv2\opencv.hpp>
#include <Windows.h>
#include <GL/x64/gl.h>
#include <GL/x64/glext.h>
#include <GL/x64/glut.h>

#include "Model2Depth.h"
#include "../Depth2Model/Depth2Model.h"
#include "../PlyObj/PlyObj.h"
#include "../Parameter/ParamParser.h"

std::vector<Eigen::Vector3f>	Model2Depth::points		= std::vector<Eigen::Vector3f>();
std::vector<Eigen::Vector3f>	Model2Depth::normals	= std::vector<Eigen::Vector3f>();
std::vector<int>				Model2Depth::facets		= std::vector<int>();
std::vector<Camera>				Model2Depth::cameras	= std::vector<Camera>();
std::string						Model2Depth::path		= "";
int								Model2Depth::frmNo		= 0;
float							Model2Depth::znear		= 0.01f;
float							Model2Depth::zfar		= 2000.0f;
int								Model2Depth::w			= 0;
int								Model2Depth::h			= 0;
Depth2Model*					Model2Depth::p_d2m		= new Depth2Model(ParamParser::m_fMinDsp, ParamParser::m_fMaxDsp, ParamParser::smooth_thres);
bool							Model2Depth::isExit		= false;

void Model2Depth::InitGL(){
	//static GLfloat pos[4] = {5.0, 5.0, -50.0, 1.0 };
	static GLfloat pos[4] = { 1, 5, -10, 1 };
	GLfloat mat_amb_diff[] = { 0.8f, 0.8f, 0.8f, 1.0f };
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE, mat_amb_diff);

	glLightModeli(GL_LIGHT_MODEL_TWO_SIDE, 1);
	glLightfv(GL_LIGHT0, GL_POSITION, pos);
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_LEQUAL);
	glEnable(GL_NORMALIZE);
	glEnable(GL_ALPHA_TEST);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glShadeModel(GL_FLAT);
	glClearColor(0.f, 0.f, 0.f, 0.f);
}

void Model2Depth::Reshape(int w, int h){
	glutReshapeWindow(Model2Depth::w, Model2Depth::h);
	glViewport(0, 0, Model2Depth::w, Model2Depth::h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60, (float)Model2Depth::w / (float)Model2Depth::h, znear, zfar);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void Model2Depth::Draw(){
	glMatrixMode(GL_MODELVIEW);

	static GLfloat colr[4] = { 1.f, .9f, .75f, 1.0f };
	static GLfloat colrb[4] = { 1.f, .9f, .75f, 1.0f };
	glMaterialfv(GL_FRONT, GL_AMBIENT_AND_DIFFUSE, colr);
	glMaterialfv(GL_BACK, GL_AMBIENT_AND_DIFFUSE, colrb);

	glBegin(GL_TRIANGLES);
	for (int i = 0; i < (int)facets.size(); i += 3){
		int a = facets[i];
		int b = facets[i + 1];
		int c = facets[i + 2];
		glNormal3f(normals[a][0], normals[a][1], normals[a][2]);
		glVertex3f(points[a][0], points[a][1], points[a][2]);
		glNormal3f(normals[b][0], normals[b][1], normals[b][2]);
		glVertex3f(points[b][0], points[b][1], points[b][2]);
		glNormal3f(normals[c][0], normals[c][1], normals[c][2]);
		glVertex3f(points[c][0], points[c][1], points[c][2]);
	}
	glEnd();
}

void Model2Depth::RenderSence(){
	glClearDepth(1.0f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	
	bool b = UpdateOnce();
	Draw();
	if(b){ RenderDepth(); }

	glutSwapBuffers();
	glutPostRedisplay();
}

bool Model2Depth::UpdateOnce(){
	if (frmNo < cameras.size()){
		Eigen::Matrix4f RT = cameras[frmNo].GetObjAbsTransformGL();
		glMatrixMode(GL_MODELVIEW);
		glLoadMatrixf(RT.data());

		float left, right, bottom, top;
		cameras[frmNo].GetFrustumGL(znear, left, right, bottom, top);
		Eigen::Matrix4f projMatrix = cameras[frmNo].GetProjectGL(left, right, bottom, top, znear, zfar);
		projMatrix.transposeInPlace();

		glMatrixMode(GL_PROJECTION);
		glLoadMatrixf(projMatrix.data());

		frmNo++;
		return true;
	}
	isExit = true;
	return false;
}

void Model2Depth::RenderDepth(){
	GLint viewport[4];
    GLdouble projection[16];
    //GLfloat winX, winY;
	glGetDoublev(GL_PROJECTION_MATRIX, projection);
	glGetIntegerv(GL_VIEWPORT, viewport);

	//GLfloat *depth = new GLfloat[w * h];
	std::vector<float> depth(w * h);
	glReadPixels(0, 0, w, h, GL_DEPTH_COMPONENT, GL_FLOAT, &depth[0]);

	std::vector<double> raw(h * w, 0.0);
	double znear, zfar;
	GetClippingPlane(projection, znear, zfar);
	for(int i = 0; i < w; ++i){
		for(int j = 0; j < h; ++j){
			float z_b = depth[(h - j - 1) * w + i];
			if(z_b >= 1 || z_b <= 0)	continue;
			float z_n = 2 * z_b - 1.0f;
			float z_e = (float)(2.0 * znear * zfar / (zfar + znear - z_n * (zfar - znear)));
			//raw[j][i] = z_e;
			if(z_e > 1e-6) raw[j * w + i] = 1.0 / z_e;
			else raw[j * w + i] = 0.0;
		}
	}
	
	int iRet = CreateDir(path + "DATA/Render/");
	char fn[128];
	sprintf_s(fn, "%sDATA/Render/_depth%d.jpg", path.c_str(), frmNo - 1);
	RenderDepthMap(fn, raw, w, h);

	sprintf_s(fn, "%sDATA/Render/_depth%d.raw", path.c_str(), frmNo - 1);
	std::cout << fn << std::endl;
	SaveDepth(fn, raw);
#if 1
	sprintf_s(fn, "%sDATA/Render/pc%d.obj", path.c_str(), frmNo - 1);
	p_d2m->SaveModel(raw, cameras[frmNo - 1], fn, true);
#endif
}

void Model2Depth::IdleFunc(){
	if (isExit){
		throw "MY_EXIT_MESSAGE"; 
	}
}

void Model2Depth::Run(int argv, char *argc[]){
	try{
		glutInit(&argv, argc);
		glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH);
	
		glutInitWindowPosition(500, 0);
		std::cout << "run: " << w << " " << h << std::endl;
		glutInitWindowSize(w, h);

		int winid = glutCreateWindow("MultiViewStitch");

		frmNo = 0;

		InitGL();
		glutReshapeFunc(Reshape);
		glutDisplayFunc(RenderSence);
		glutIdleFunc(IdleFunc);
		glutMainLoop();
	}catch(const char* msg){
		std::cout << "Render Terminated" << std::endl;
	}
}

void Model2Depth::GetClippingPlane(double proj[], double &znear, double &zfar){
	double m22 = proj[10];
	double m32 = proj[14];
	znear = m32 / (m22 - 1.0f);
	zfar = m32 / (m22 + 1.0f);
}