// PointCloudDoc.cpp : CPointCloudDoc 类的实现
//

#include "stdafx.h"
#include "PointCloud.h"

#include "PointCloudDoc.h"
#include "PointCloudView.h"
#include "SimplyParamDlg.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

// CPointCloudDoc

IMPLEMENT_DYNCREATE(CPointCloudDoc, CDocument)

BEGIN_MESSAGE_MAP(CPointCloudDoc, CDocument)
	ON_COMMAND(ID_FILE_OPEN, &CPointCloudDoc::OnFileOpen)
	ON_COMMAND(ID_EDIT_SIMPLY, &CPointCloudDoc::OnEditSimply)
	ON_COMMAND(ID_MESH, &CPointCloudDoc::OnMesh)
END_MESSAGE_MAP()

// CPointCloudDoc 构造/析构

CPointCloudDoc::CPointCloudDoc()
{
	ps = NULL;
	mesh = NULL;
	mode = NONE;
}

CPointCloudDoc::~CPointCloudDoc()
{
	if (ps != NULL)
		delete ps;
	if (mesh != NULL)
		delete mesh;
	ps = NULL;
}

BOOL CPointCloudDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	// TODO: 在此添加重新初始化代码
	// (SDI 文档将重用该文档)

	return TRUE;
}

// CPointCloudDoc 序列化

void CPointCloudDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
		// TODO: 在此添加存储代码
	}
	else
	{
		// TODO: 在此添加加载代码
	}
}

// CPointCloudDoc 诊断

#ifdef _DEBUG
void CPointCloudDoc::AssertValid() const
{
	CDocument::AssertValid();
}

void CPointCloudDoc::Dump(CDumpContext& dc) const
{
	CDocument::Dump(dc);
}
#endif //_DEBUG

// CPointCloudDoc 命令

void CPointCloudDoc::OnFileOpen()
{
	CFileDialog* dlg = new CFileDialog(TRUE, NULL, NULL,
		OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT, LPCTSTR("All files(*.*)|*.*|pts(*.pts)|*.pts||"), NULL);
	if (dlg->DoModal() == IDOK){
		if (ps != NULL){
			delete ps;
			ps = NULL;
		}
		CString filePath = dlg->GetPathName();
		char* inFile = filePath.GetBuffer(1000);
		if (strstr(inFile, ".pts") != NULL)
		{
			SetTitle(dlg->GetFileTitle());
			ps = new PointSet;
			ps->readPts(inFile);
			mode = POINTMODE;
		}
		if(ps != NULL)
			ps->rescale(30.0f);

		Draw();
	}
	delete dlg;
}

void CPointCloudDoc::Draw(){
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);
	pView->draw();
}

void CPointCloudDoc::drawData(){
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);

	switch (mode)
	{
	case POINTMODE:
		pView->drawPoint(ps);
		break;
	case MESHMODE:
		pView->drawMesh(mesh);
		break;
	}
}

void CPointCloudDoc::OnEditSimply()
{
	// TODO: 在此添加命令处理程序代码
	CSimplyParamDlg simplyParamDlg;
	if (simplyParamDlg.DoModal() == IDOK){
		ps->computeNormal(simplyParamDlg.m_K);
	}
}

void CPointCloudDoc::OnMesh()
{
	// TODO: 在此添加命令处理程序代码
	if (ps != NULL){
		mesh = new CIPDMesh(ps);
		mesh->start();
		mode = MESHMODE;
	}
}