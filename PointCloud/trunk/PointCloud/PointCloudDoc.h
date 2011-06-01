// PointCloudDoc.h : CPointCloudDoc 类的接口
//

#ifndef CPointCloudDoc_H_
#define CPointCloudDoc_H_
#include "ballmesh/BallDialog.h"
#pragma once

enum MODE{
	NONE,
	POINTMODE,
	MESHMODE,
	POINTMESHMODE,
	BALLMODE,
	BALLMESHMODE,
	BALLWIREMODE
};
class CPointSet;
class CMesh;
class CPointMesh;
class BallMeshGenerator;
class BallMesh;
class BallGenerator;

class CPointCloudDoc : public CDocument
{
protected: // 仅从序列化创建
	CPointCloudDoc();
	DECLARE_DYNCREATE(CPointCloudDoc)

// 属性
public:
	//==========FOR SHOW============//
	bool m_bMesh;
	bool m_bWire;
	bool m_bPoint;

	CPointSet* ps;
	CMesh* m_mesh;
	CPointMesh* m_pointMesh;

	MODE mode;
	//=========FOR BALLMESH==========//
	BallMeshGenerator* m_ballMeshGen;
	BallGenerator* m_ballGen;
	CPointSet* m_balls;
	BallMesh* m_ballMesh;
	CBallDialog m_ballDlg;
// 操作
public:
	void Draw();
	void drawData();

// 重写
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// 实现
public:
	virtual ~CPointCloudDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// 生成的消息映射函数
protected:
	DECLARE_MESSAGE_MAP()
	afx_msg void OnFileOpen();
public:
	afx_msg void OnEditSimply();
	afx_msg void OnMesh();
	afx_msg void OnEditFlip();
	afx_msg void OnFtpMesh();
	afx_msg void OnPointMesh();
	afx_msg void OnBallMeshNormalAndWeight();
	afx_msg void OnBallMeshGeneratorBall();
	afx_msg void OnBallMeshGeneratorMesh();
	afx_msg void OnBallMeshCleanMesh();
	afx_msg void OnBallMeshFilpNormal();

	afx_msg void OnOctreeSimply();
	afx_msg void OnShowMesh();
	afx_msg void OnShowPoint();
	afx_msg void OnShowWire();
	afx_msg void OnPointmesh2();
};
#endif