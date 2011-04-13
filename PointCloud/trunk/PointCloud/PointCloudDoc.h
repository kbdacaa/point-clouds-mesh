// PointCloudDoc.h : CPointCloudDoc 类的接口
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
protected: // 仅从序列化创建
	CPointCloudDoc();
	DECLARE_DYNCREATE(CPointCloudDoc)

// 属性
public:
	PointSet* ps;
		CIPDMesh* mesh;

	MODE mode;

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
};
#endif