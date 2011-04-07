// PointCloudView.h : CPointCloudView 类的接口
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
protected: // 仅从序列化创建
	CPointCloudView();
	DECLARE_DYNCREATE(CPointCloudView)

// 属性
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

// 操作
public:

// 重写
public:
	virtual void OnDraw(CDC* pDC);  // 重写以绘制该视图
	virtual BOOL PreCreateWindow(CREATESTRUCT& cs);
protected:

// 实现
public:
	virtual ~CPointCloudView();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
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

#ifndef _DEBUG  // PointCloudView.cpp 中的调试版本
inline CPointCloudDoc* CPointCloudView::GetDocument() const
   { return reinterpret_cast<CPointCloudDoc*>(m_pDocument); }
#endif

#endif