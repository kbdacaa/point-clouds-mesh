// PointCloudDoc.h : CPointCloudDoc ��Ľӿ�
//

#ifndef CPointCloudDoc_H_
#define CPointCloudDoc_H_
#include "ballmesh/BallDialog.h"
#pragma once

struct ParamSet{
	// ���Ƽ���������
	unsigned int simplyK;
	// ��Ԫ�����е��������
	unsigned int simplyMaxPointSize;
	// Ҫ����İ˲����������С���
	unsigned int simplyMinDeep;

	// ���ཻ��������
	float ballTerr;
	// ���ཻ������뾶Ӱ�����ֵ
	float ballTq;

	// ������չ��������ʱ���������С
	unsigned int pointMeshK;
	// ������չ��������ʱ��������С�ڽ����ڱ���
	float pointMeshA;
	// ������չ��������ʱ�������ռ����(��ƽ����)
	float pointMeshB;

	void initParam(){
		simplyK = 10;
		simplyMaxPointSize = 15;
		simplyMinDeep = 4;
		ballTerr = 4.0e-6F;
		ballTq = 2.0F;
		pointMeshK = 12;
		pointMeshA = 1.0F;
		pointMeshB = 1.0F;
	}
};

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
protected: // �������л�����
	CPointCloudDoc();
	DECLARE_DYNCREATE(CPointCloudDoc)

// ����
public:
	BOOL m_bCmpNormal;
	BOOL m_bAjstNormal;
	bool m_bUsedGlobalSetting;
	ParamSet m_paramSet;

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
// ����
public:
	void Draw();
	void drawData();

// ��д
public:
	virtual BOOL OnNewDocument();
	virtual void Serialize(CArchive& ar);

// ʵ��
public:
	virtual ~CPointCloudDoc();
#ifdef _DEBUG
	virtual void AssertValid() const;
	virtual void Dump(CDumpContext& dc) const;
#endif

protected:

// ���ɵ���Ϣӳ�亯��
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
	afx_msg void OnComputeNormal();
	afx_msg void OnAdjustNormal();
	afx_msg void OnUpdateComputeNormal(CCmdUI *pCmdUI);
	afx_msg void OnUpdateAdjustNormal(CCmdUI *pCmdUI);
	afx_msg void OnParamSetting();
};
#endif