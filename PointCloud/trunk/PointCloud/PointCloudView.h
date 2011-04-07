// PointCloudView.h : CPointCloudView ��Ľӿ�
//

#ifndef CPointCloudView_H_
#define CPointCloudView_H_
#pragma once

#include <gl/GL.h>
#include <gl/glu.h>
#include <gl/glaux.h>
#include "Mesh.h"
#pragma comment(lib, "glu32.lib")
#pragma comment(lib, "glaux.lib")

class CPointCloudView : public CView
{
protected: // �������л�����
	CPointCloudView();
	DECLARE_DYNCREATE(CPointCloudView)

// ����
public:
	CPointCloudDoc* GetDocument() const;

	HGLRC m_hRC;

	GLdouble eye[3], center[3];
	BOOL isLPressed, isRPressed;
	float view_angle;

	GLfloat Rx,Ry,Rz;
	int px, py;
	float vx[3], vy[3], vz[3];
	float shift_x, shift_y, shift_z;

// ����
public:

// ��д
public:
	virtual void OnDraw(CDC* pDC);  // ��д�Ի��Ƹ���ͼ
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// ʵ��
public:
	virtual ~CPointCloudView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
protected:
	DECLARE_MESSAGE_MAP()
public:
	void initGL(void);
	void drawData(void);
	void draw();

	void drawPoint(PointSet* ps);
	void drawMesh(CMesh* mesh);
public:
	afx_msg int OnCreate(LPCREATESTRUCT lpCreateStruct);
	afx_msg void OnDestroy();
	afx_msg void OnSize(UINT nType, int cx, int cy);
	afx_msg void OnLButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnLButtonDown(UINT nFlags, CPoint point);
	afx_msg void OnMouseMove(UINT nFlags, CPoint point);
	afx_msg void OnRButtonUp(UINT nFlags, CPoint point);
	afx_msg void OnRButtonDown(UINT nFlags, CPoint point);
public:
	void drawAxis(void);
};

#ifndef _DEBUG  // PointCloudView.cpp �еĵ��԰汾
inline CPointCloudDoc* CPointCloudView::GetDocument() const
   { return reinterpret_cast<CPointCloudDoc*>(m_pDocument); }
#endif

#endif