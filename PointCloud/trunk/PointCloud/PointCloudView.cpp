// PointCloudView.cpp : CPointCloudView 类的实现
//

#include "stdafx.h"
#include "PointCloud.h"

#include "PointCloudDoc.h"
#include "PointCloudView.h"
#include "Mesh.h"
#include "PointMesh.h"
#include <math.h>

#include "BallMesh/BallMesh.h"
#define PI 3.1415926535897932384626433832795

#ifdef _DEBUG
#define new DEBUG_NEW
#endif


IMPLEMENT_DYNCREATE(CPointCloudView, CView)

BEGIN_MESSAGE_MAP(CPointCloudView, CView)
	ON_WM_CREATE()
	ON_WM_DESTROY()
	ON_WM_SIZE()
	ON_WM_LBUTTONUP()
	ON_WM_LBUTTONDOWN()
	ON_WM_MOUSEMOVE()
	ON_WM_RBUTTONUP()
	ON_WM_RBUTTONDOWN()
	ON_WM_KEYDOWN()
END_MESSAGE_MAP()

CPointCloudView::CPointCloudView()
{
	eye[0] = 26;
	eye[1] = -48;
	eye[2] = 40;

	center[0] = 0.0;
	center[1] = 0.0;
	center[2] = 0.0;

	view_angle = 22.5;

	isLPressed = FALSE;
	isRPressed = FALSE;

	Rx = 0;
	Ry = 6;
	Rz = -69;

	vx[0] = 0;
	vx[1] = 0;
	vx[2] = 1;

	vy[0] = -(float)(eye[1]/sqrt(eye[0]*eye[0]+eye[1]*eye[1]));
	vy[1] = (float)(eye[0]/sqrt(eye[0]*eye[0]+eye[1]*eye[1]));
	vy[2] = 0;

	vz[0] = -vy[1];
	vz[1] = vy[0];
	vz[2] = 0;

	shift_x = 0;
	shift_y = 0;
	shift_z = 0;

	bCtrlDown = false;
	bLeftDown = false;
}

CPointCloudView::~CPointCloudView()
{
}

BOOL CPointCloudView::PreCreateWindow(CREATESTRUCT& cs)
{
	return CView::PreCreateWindow(cs);
}

void CPointCloudView::OnDraw(CDC* pDC)
{
	CPointCloudDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	wglMakeCurrent(pDC->m_hDC, m_hRC);

	//glClearColor(0.644, 0.828, 0.1875, 0.9);
	glClearColor(0.4f, 0.48f, 0.175f, 0.9f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

// 	TRACE("%f, %f\n", Ry, Rz);

	glPushMatrix();
	glTranslatef(shift_x,shift_y,shift_z);

	glRotatef(Rx,vz[0],vz[1],vz[2]);
	glRotatef(Ry,vy[0],vy[1],vy[2]);
	glRotatef(Rz,vx[0],vx[1],vx[2]);

	drawAxis();
 	pDoc->drawData();

	glPopMatrix();

	glFlush();
	SwapBuffers(wglGetCurrentDC());
	wglMakeCurrent(pDC->m_hDC, NULL);
}

// CPointCloudView 诊断

#ifdef _DEBUG
void CPointCloudView::AssertValid() const
{
	CView::AssertValid();
}

void CPointCloudView::Dump(CDumpContext& dc) const
{
	CView::Dump(dc);
}

CPointCloudDoc* CPointCloudView::GetDocument() const // 非调试版本是内联的
{
	ASSERT(m_pDocument->IsKindOf(RUNTIME_CLASS(CPointCloudDoc)));
	return (CPointCloudDoc*)m_pDocument;
}
#endif //_DEBUG

void CPointCloudView::initGL(void)
{
	GLfloat light_position1[4] = {-520, -160, -500, 0};
	GLfloat light_position2[4] = {-260, -480, -500, 0};
	GLfloat light_position3[4] = { 160, -520, -500, 0};

	GLfloat direction1[3] = {52,16,50};
	GLfloat direction2[3] = {26,48,50};
	GLfloat direction3[3] = {-16,52,50};

	GLfloat light_position4[4] = {520, 160, 500, 0};
	GLfloat light_position5[4] = {260, 480, 500, 0};
	GLfloat light_position6[4] = {-160, 520, 500, 0};

	GLfloat direction4[3] = {-52,-16,-50};
	GLfloat direction5[3] = {-26,-48,-50};
	GLfloat direction6[3] = {16,-52,-50};

	GLfloat color1[4] = {1,0,0,1};
	GLfloat color2[4] = {0,1,0,1};
	GLfloat color3[4] = {0,0,1,1};

	GLfloat color4[4] = {1,0,0,1};
	GLfloat color5[4] = {0,1,0,1};
	GLfloat color6[4] = {0,0,1,1};

	GLfloat ambient[4] = {0.3f,0.3f,0.3f,0.5f};

	GLfloat material_color[4] = {1,1,1,0.3f};
	GLfloat material_specular[4] = {0.5,0.5,0.5,0.5};
	GLfloat material_ambient[4] = {0.0,0.0,0.0,0.0};

	glClearColor(1,1,1,0);

	glLightfv(GL_LIGHT0, GL_POSITION, light_position1);
	glLightfv(GL_LIGHT0, GL_SPOT_DIRECTION, direction1);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, color1);
	glLightfv(GL_LIGHT0, GL_SPECULAR, color1);

	glLightfv(GL_LIGHT1, GL_POSITION, light_position2);
	glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, direction2);
	glLightfv(GL_LIGHT1, GL_DIFFUSE, color2);
	glLightfv(GL_LIGHT1, GL_SPECULAR, color2);

	glLightfv(GL_LIGHT2, GL_POSITION, light_position3);
	glLightfv(GL_LIGHT2, GL_SPOT_DIRECTION, direction3);
	glLightfv(GL_LIGHT2, GL_DIFFUSE, color3);
	glLightfv(GL_LIGHT2, GL_SPECULAR, color3);

	glLightfv(GL_LIGHT3, GL_POSITION, light_position4);
	glLightfv(GL_LIGHT3, GL_SPOT_DIRECTION, direction4);
	glLightfv(GL_LIGHT3, GL_DIFFUSE, color4);
	glLightfv(GL_LIGHT3, GL_SPECULAR, color4);

	glLightfv(GL_LIGHT4, GL_POSITION, light_position5);
	glLightfv(GL_LIGHT4, GL_SPOT_DIRECTION, direction5);
	glLightfv(GL_LIGHT4, GL_DIFFUSE, color5);
	glLightfv(GL_LIGHT4, GL_SPECULAR, color5);

	glLightfv(GL_LIGHT5, GL_POSITION, light_position6);
	glLightfv(GL_LIGHT5, GL_SPOT_DIRECTION, direction6);
	glLightfv(GL_LIGHT5, GL_DIFFUSE, color6);
	glLightfv(GL_LIGHT5, GL_SPECULAR, color6);

	glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, material_specular);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, material_color);
	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, material_ambient);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, 128);

	glEnable(GL_LIGHTING);

// 	glEnable(GL_LIGHT0);
// 	glEnable(GL_LIGHT1);
// 	glEnable(GL_LIGHT2);

	glEnable(GL_LIGHT3);
	glEnable(GL_LIGHT4);
	glEnable(GL_LIGHT5);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	glEnable(GL_POINT_SMOOTH);
	glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

	glEnable(GL_LINE_SMOOTH);
	glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);

	glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

	glLineWidth(1.0);
	glPointSize(1.0);
}

int CPointCloudView::OnCreate(LPCREATESTRUCT lpCreateStruct)
{
	if (CView::OnCreate(lpCreateStruct) == -1)
		return -1;

	PIXELFORMATDESCRIPTOR pfd =
	{
		sizeof(PIXELFORMATDESCRIPTOR),
		1,
		PFD_DRAW_TO_WINDOW | PFD_SUPPORT_OPENGL | PFD_DOUBLEBUFFER,
		PFD_TYPE_RGBA,
		16,
		0,0,0,0,0,0,
		0,0,0,0,0,0,0,
		32,
		0,0,
		PFD_MAIN_PLANE,
		0,
		0,0,0
	};

	CClientDC  clientDC(this);

	int pixelFormat = ChoosePixelFormat(clientDC.m_hDC, &pfd);
	if(!SetPixelFormat(clientDC.m_hDC, pixelFormat, &pfd))
		return -1;

	m_hRC = wglCreateContext(clientDC.m_hDC);

	wglMakeCurrent(clientDC.m_hDC,m_hRC);
	initGL();
	wglMakeCurrent(clientDC.m_hDC,NULL);

	return 0;
}

void CPointCloudView::OnDestroy()
{
	CView::OnDestroy();

	wglDeleteContext(m_hRC);
}

void CPointCloudView::OnSize(UINT nType, int cx, int cy)
{
	CView::OnSize(nType, cx, cy);

	int i;
	GLdouble up[3];
	GLdouble vector[3];
	GLdouble norm;

	CClientDC clientDC(this);

	wglMakeCurrent(clientDC.m_hDC, m_hRC);

	glViewport(0, 0, cx, cy);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(view_angle, (GLfloat)cx/(GLfloat)cy, 10.0, 4000.0);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	for(i=0; i<3; i++)
		vector[i] = center[i] - eye[i];

	up[0] =  vector[0] * vector[2];
	up[1] =  vector[1] * vector[2];
	up[2] =  vector[0] * vector[0] + vector[1] * vector[1];

	norm = up[0] * up[0] + up[1] * up[1] + up[2] * up[2];
	norm = sqrt(norm);
	for(i=0; i<3; i++)
		up[i] = up[i] / norm;

	gluLookAt(eye[0], eye[1], eye[2], center[0], center[1], center[2],
		up[0], up[1], up[2]);

	wglMakeCurrent(clientDC.m_hDC, NULL);
}

void CPointCloudView::OnLButtonDown(UINT nFlags, CPoint point)
{
	isLPressed = TRUE;
	px = point.x;
	py = point.y;

	CView::OnLButtonDown(nFlags, point);
}

void CPointCloudView::OnLButtonUp(UINT nFlags, CPoint point)
{
	isLPressed = FALSE;
	draw();
	CView::OnLButtonUp(nFlags, point);
}

void CPointCloudView::OnMouseMove(UINT nFlags, CPoint point)
{
	if(isLPressed){
		int nx = point.x;
		int ny = point.y;

		Rz += (nx - px)/3;
		Ry += (ny - py)/3;
		px = nx;
		py = ny;
		draw();
	}
	else if(isRPressed){
		int nx = point.x;
		int ny = point.y;

		view_angle += (ny - py)/20.0f;
		px = nx;
		py = ny;

		RECT rect;
		GetClientRect(&rect);
		OnSize(SIZE_RESTORED, rect.right, rect.bottom);
		draw();
	}

	CView::OnMouseMove(nFlags, point);
}

void CPointCloudView::OnRButtonDown(UINT nFlags, CPoint point)
{
	isRPressed = TRUE;
	px = point.x;
	py = point.y;

	CView::OnRButtonDown(nFlags, point);
}

void CPointCloudView::OnRButtonUp(UINT nFlags, CPoint point)
{
	isRPressed = FALSE;
	draw();

	CView::OnRButtonUp(nFlags, point);
}

void CPointCloudView::drawAxis(void)
{
	glColor3f(1.f, 0.4f, 0.1f);
	glBegin(GL_LINES);
	{
		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(15.f, 0.f, 0.f);

		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(0.f, 15.f, 0.f);

		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(0.f, 0.f, 15.f);
	}
	glEnd();
}

void CPointCloudView::drawPoint(CPointSet* ps){
	glShadeModel(GL_SMOOTH);
	int pointN = ps->m_pointN;
	float **point = ps->m_point;
	float (*normal)[3] = ps->m_normal;
 	if(normal == NULL){
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
		{
			glColor3f(0.0f,0.0f,0.7f);
			for(int i=0; i<pointN; i++)
				glVertex3f(point[i][0], point[i][1], point[i][2]);
		}
		glEnd();
		glEnable(GL_LIGHTING);
	}
	else{
		glBegin(GL_POINTS);
		for(int i=0; i<pointN; i++){
			glNormal3f(-normal[i][0], -normal[i][1], -normal[i][2]);
			glVertex3f(point[i][0], point[i][1], point[i][2]);
		}
		glEnd();
	}
}

void CPointCloudView::drawMesh( CMesh* mesh )
{
	glShadeModel(GL_SMOOTH);
	//glEnable(GL_LIGHTING);
	glDisable(GL_LIGHTING);
	int nFace = mesh->m_faceVects.size();
	float** ps = mesh->m_ps->m_point;
	int pointN = mesh->m_ps->m_pointN;

	glBegin(GL_POINTS);
	{
		glColor3f(0.0f,0.0f,0.7f);
		for(int i=0; i<pointN; i++)
			glVertex3f(ps[i][0], ps[i][1], ps[i][2]);
	}
	glEnd();
	glEnable(GL_LIGHTING);

	float norm[3];
// 	for (int i = 0; i < nFace-1 ; i++)
	for (int i = 0 ; i < nFace-1 ; i++)
	{
		CTriangle* face = mesh->m_faceVects.at(i);
		face->getNorm(norm);
		glBegin(GL_POLYGON);
		{
			glNormal3f(norm[0], norm[1], norm[2]);
			glVertex3fv(ps[face->getA()]);
			glVertex3fv(ps[face->getB()]);
			glVertex3fv(ps[face->getC()]);
		}
		glEnd();
	}
	CTriangle* face = mesh->m_faceVects.at(nFace-1);
	glDisable(GL_LIGHTING);
	glBegin(GL_TRIANGLES);
	{
		//	glNormal3f(norm[0], norm[1], norm[2]);
		glColor3f(1.0, 0.0, 0.0);	glVertex3fv(ps[face->getA()]);
		glColor3f(0.0, 1.0, 0.0);	glVertex3fv(ps[face->getB()]);
		glColor3f(0.0, 0.0, 1.0);	glVertex3fv(ps[face->getC()]);
	}
	glEnd();
}

void CPointCloudView::drawPointMesh(CPointMesh* pointMesh){
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING);
	float** ps = pointMesh->m_ps->m_point;
	int pointN = pointMesh->m_ps->m_pointN;
	glBegin(GL_POINTS);
	{
		glColor3f(0.0f, 0.0f, 0.5f);
		for(int i = 0; i < pointN; i++)
			glVertex3f(ps[i][0], ps[i][1], ps[i][2]);
	}
	glEnd();
// 	glEnable(GL_LIGHTING);
	size_t size = pointMesh->m_faces.size();
	float norm[3];
	for (size_t t = 0; t < size; t++) {
		CTriangle& face = pointMesh->m_faces.at(t);
		face.getNorm(norm);
		glBegin(GL_POLYGON);
		{
// 			glNormal3f(norm[0], norm[1], norm[2]);
			glColor3f(0.6f, 0.0f, 0.4f);  glVertex3fv(ps[face.getA()]);
			glColor3f(0.4f, 0.6f, 0.0f);  glVertex3fv(ps[face.getB()]);
			glColor3f(0.0f, 0.4f, 0.6f);  glVertex3fv(ps[face.getC()]);
		}
		glEnd();
	}
}

void CPointCloudView::drawPointMeshWithoutPoint(CPointMesh* pointMesh){
	glShadeModel(GL_SMOOTH);
	glDisable(GL_LIGHTING);
	float** ps = pointMesh->m_ps->m_point;
 	glEnable(GL_LIGHTING);
	size_t size = pointMesh->m_faces.size();
	float norm[3];
	for (size_t t = 0; t < size; t++) {
		CTriangle& face = pointMesh->m_faces.at(t);
		face.getNorm(norm);
		glBegin(GL_POLYGON);
		{
			glNormal3f(norm[0], norm[1], norm[2]);
			glColor3f(0.6f, 0.0f, 0.4f);  glVertex3fv(ps[face.getA()]);
			glColor3f(0.4f, 0.6f, 0.0f);  glVertex3fv(ps[face.getB()]);
			glColor3f(0.0f, 0.4f, 0.6f);  glVertex3fv(ps[face.getC()]);
		}
		glEnd();
	}
}

void CPointCloudView::OnKeyDown(UINT nChar, UINT nRepCnt, UINT nFlags)
{
	if (VK_LEFT == nChar){
		shift_x -= 1;
	}else if (VK_RIGHT == nChar){
		shift_x += 1;
	}else if (VK_UP == nChar){
		shift_y += 1;
	}else if (VK_DOWN == nChar){
		shift_y -= 1;
	}else if (VK_ADD == nChar){
		shift_z += 1;
	}else if (VK_SUBTRACT == nChar){
		shift_z -= 1;
	}
	draw();

	CView::OnKeyDown(nChar, nRepCnt, nFlags);
}

//==========FOR BALLMESH============//
void CPointCloudView::drawBalls( CPointSet* balls )
{
	glShadeModel(GL_SMOOTH);
	int ballN = balls->m_pointN;
	float **center = balls->m_point;
	float *radius = balls->m_weight;
	if (radius == NULL)	return ;
	int k = 10;
	if(isLPressed || isRPressed)
		k = 5;
	for(int i=0; i<ballN; i++)
		sphere(radius[i], center[i], k);
}

void CPointCloudView::drawBallMesh( BallMesh* ballMesh )
{
	glShadeModel(GL_SMOOTH);
	int faceN = ballMesh->faceN;
	int (*face)[3] = ballMesh->face;
	float (*normal_f)[3] = ballMesh->normal_f;
	float (*vertex)[3] = ballMesh->vertex;
	for(int i=0; i<faceN; i++){
		int *f = face[i];
		glBegin(GL_POLYGON);
		glNormal3fv(normal_f[i]);
		glVertex3fv(vertex[f[0]]);
		glVertex3fv(vertex[f[1]]);
		glVertex3fv(vertex[f[2]]);
		glEnd();
	}
}

void CPointCloudView::drawBallWire( BallMesh* ballMesh )
{
	glShadeModel(GL_SMOOTH);
	int faceN = ballMesh->faceN;
	int (*face)[3] = ballMesh->face;
	float (*normal_v)[3] = ballMesh->normal_v;
	float (*vertex)[3] = ballMesh->vertex;

	glDisable(GL_LIGHTING);
	glColor3f(1,1,1);
	int i,j;
	float d = (float)(0.05*view_angle/22.5);
	for(i=0; i<faceN; i++){
		int *f = face[i];
		float p[3];
		glBegin(GL_POLYGON);
		{
			for(j=0; j<3; j++){
				p[0] = vertex[f[j]][0] - d*normal_v[f[j]][0];
				p[1] = vertex[f[j]][1] - d*normal_v[f[j]][1];
				p[2] = vertex[f[j]][2] - d*normal_v[f[j]][2];
				glVertex3fv(p);
			}
		}
		glEnd();
	}

	glColor3f(0,0,0);
	for(i=0; i<faceN; i++){
		if(face[i][0] < 0)
			continue;
		int *f = face[i];
		for(j=0; j<3; j++){
			int i1 = f[j];
			int i2 = f[(j+1)%3];
			if(i1 > i2){
				glBegin(GL_LINES);
				glVertex3fv(vertex[i1]);
				glVertex3fv(vertex[i2]);
				glEnd();
			}
		}
	}
	glEnable(GL_LIGHTING);
}

void CPointCloudView::sphere( float r, float p[], int n )
{
	float (**vertex)[3] = (float(**)[3])new float[n];
	float (**normal)[3] = (float(**)[3])new float[n];
	int i = 0;
	for( i=0; i<n; i++){
		vertex[i] = (float (*)[3])new float[n][3];
		normal[i] = (float (*)[3])new float[n][3];
	}

	for(i=0; i<n; i++){
		float u = (float)(PI*i/(n-1) - 0.5f*PI);
		for(int j=0; j<n; j++){
			float v = (float)(2*PI*j/(n-1));
			normal[i][j][0] = (float)(cos(v)*cos(u));
			normal[i][j][1] = (float)(sin(v)*cos(u));
			normal[i][j][2] = (float)sin(u);

			vertex[i][j][0] = (float)(r*normal[i][j][0] + p[0]);
			vertex[i][j][1] = (float)(r*normal[i][j][1] + p[1]);
			vertex[i][j][2] = (float)(r*normal[i][j][2] + p[2]);
		}
	}

	for(i=0; i<n-1; i++){
		for(int j=0; j<n-1; j++){
			glBegin(GL_POLYGON);

			glNormal3fv(normal[i][j]);
			glVertex3fv(vertex[i][j]);

			glNormal3fv(normal[i][j+1]);
			glVertex3fv(vertex[i][j+1]);

			glNormal3fv(normal[i+1][j+1]);
			glVertex3fv(vertex[i+1][j+1]);

			glNormal3fv(normal[i+1][j]);
			glVertex3fv(vertex[i+1][j]);

			glEnd();
		}
	}

	for(i=0; i<n; i++){
		delete[] vertex[i];
		delete[] normal[i];
	}
	delete[] vertex;
	delete[] normal;
}