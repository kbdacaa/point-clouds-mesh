// PointCloudView.cpp : CPointCloudView 类的实现
//

#include "stdafx.h"
#include "PointCloud.h"

#include "PointCloudDoc.h"
#include "PointCloudView.h"
#include <math.h>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CPointCloudView

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
END_MESSAGE_MAP()

// CPointCloudView 构造/析构

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
}

CPointCloudView::~CPointCloudView()
{
}

BOOL CPointCloudView::PreCreateWindow(CREATESTRUCT& cs)
{
	// TODO: 在此处通过修改
	//  CREATESTRUCT cs 来修改窗口类或样式

	return CView::PreCreateWindow(cs);
}

// CPointCloudView 绘制

void CPointCloudView::OnDraw(CDC* pDC)
{
	CPointCloudDoc* pDoc = GetDocument();
	ASSERT_VALID(pDoc);
	if (!pDoc)
		return;

	wglMakeCurrent(pDC->m_hDC, m_hRC);

	glClearColor(0.644, 0.828, 0.1875, 0.9);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

// 	TRACE("%f, %f\n", Ry, Rz);

	glPushMatrix();
	glTranslatef(shift_x,shift_y,shift_z);

	glRotatef(Rx,vz[0],vz[1],vz[2]);
	glRotatef(Ry,vy[0],vy[1],vy[2]);
	glRotatef(Rz,vx[0],vx[1],vx[2]);

// 	drawData();
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

// CPointCloudView 消息处理程序

void CPointCloudView::initGL(void)
{
	GLfloat light_position1[4] = {-52, -16, -50, 0};
	GLfloat light_position2[4] = {-26, -48, -50, 0};
	GLfloat light_position3[4] = { 16, -52, -50, 0};

	GLfloat direction1[3] = {52,16,50};
	GLfloat direction2[3] = {26,48,50};
	GLfloat direction3[3] = {-16,52,50};

	GLfloat light_position4[4] = {52, 16, 50, 0};
	GLfloat light_position5[4] = {26, 48, 50, 0};
	GLfloat light_position6[4] = {-16, 52, 50, 0};

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
	glPointSize(3.0);
}

void CPointCloudView::draw(){
	CClientDC* pDC = new CClientDC(this);
	OnDraw(pDC);
	delete pDC;
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
	CClientDC* dc = new CClientDC(this);
	OnDraw(dc);
	delete dc;

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
		CClientDC* dc = new CClientDC(this);
		OnDraw(dc);
		delete dc;
	}
	else if(isRPressed){
		int nx = point.x;
		int ny = point.y;

		view_angle += (ny - py)/20.0f;
		px = nx;
		py = ny;
		CClientDC* dc = new CClientDC(this);

		RECT* rect = new RECT;
		GetClientRect(rect);
		OnSize(SIZE_RESTORED, rect->right, rect->bottom);
		delete rect;
		OnDraw(dc);
		delete dc;
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
	CClientDC* dc = new CClientDC(this);
	OnDraw(dc);
	delete dc;

	CView::OnRButtonUp(nFlags, point);
}

void CPointCloudView::drawData(void)
{
	glColor3f(0.f, 0.f, 0.f);
	glBegin(GL_POINTS);
	glVertex3f(2.0f, 3.0f, 4.0f);
	glEnd();
}

void CPointCloudView::drawPoint(PointSet* ps){
	glShadeModel(GL_SMOOTH);
	int pointN = ps->m_pointN;
	float **point = ps->m_point;
	float (*normal)[3] = ps->m_normal;
// 	if(normal == NULL){
		glDisable(GL_LIGHTING);
		glBegin(GL_POINTS);
		glColor3f(0,0,1);
		for(int i=0; i<pointN; i++)
			glVertex3f(point[i][0], point[i][1], point[i][2]);
		glEnd();
		glEnable(GL_LIGHTING);
// 	}
// 	else{
// 		glBegin(GL_POINTS);
// 		for(int i=0; i<pointN; i++){
// 			glNormal3f(-normal[i][0], -normal[i][1], -normal[i][2]);
// 			glVertex3f(point[i][0], point[i][1], point[i][2]);
// 		}
// 		glEnd();
// 	}
}
void CPointCloudView::drawAxis(void)
{
	glColor3f(1.f, 0.f, 0.f);
	glBegin(GL_LINES);
		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(10.f, 0.f, 0.f);

		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(0.f, 10.f, 0.f);

		glVertex3f(0.f, 0.f, 0.f);
		glVertex3f(0.f, 0.f, 10.f);
	glEnd();
}

void CPointCloudView::drawMesh( CMesh* mesh )
{
	glShadeModel(GL_SMOOTH);
	int nFace = mesh->m_faceVects.size();
	float** ps = mesh->m_ps->m_point;

	for (int i = 0; i < nFace ; i++)
	{
		CTriangle* face = mesh->m_faceVects.at(i);
		vect3f norm = mesh->m_faceNorms.at(i);
		glBegin(GL_POLYGON);
		float t = float(i%10)/5.0f;
			glNormal3f(norm[0], norm[1]+t, norm[2]+t);
			glVertex3fv(ps[face->getA()]);
			glVertex3fv(ps[face->getB()]);
			glVertex3fv(ps[face->getC()]);
		glEnd();
	}
}