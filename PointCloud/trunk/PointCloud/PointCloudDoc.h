// PointCloudDoc.h : CPointCloudDoc ��Ľӿ�
//

#ifndef CPointCloudDoc_H_
#define CPointCloudDoc_H_

#pragma once

enum MODE{
	NONE,
	POINTMODE,
	MESHMODE
};
class PointSet;
class CIPDMesh;
class CPointCloudDoc : public CDocument
{
protected: // �������л�����
	CPointCloudDoc();
	DECLARE_DYNCREATE(CPointCloudDoc)

// ����
public:
	PointSet* ps;
		CIPDMesh* mesh;

	MODE mode;

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
};
#endif