// PointCloudDoc.cpp : CPointCloudDoc 类的实现
//

#include "stdafx.h"
#include "PointCloud.h"
#include "Mesh.h"
#include "FtpMesh.h"
#include "PointMesh.h"
#include "PointCloudDoc.h"
#include "PointCloudView.h"
#include "SimplyParamDlg.h"
#include "Dialog/SetK.h"
#include "Dialog/ParamSettingDlg.h"
#include "Common/FileReadPlug.h"

#include "BallMesh/Ball.h"
#include "BallMesh/BallGenerator.h"
#include "BallMesh/BallMesh.h"
#include "BallMesh/BallMeshGenerator.h"

#include "Simply/SimplyPlugin.h"
#include "Simply/AdjustMesh.h"

#ifdef _DEBUG
#define new DEBUG_NEW
#endif
extern ExtRegister theExtRegPlugin;
// CPointCloudDoc

IMPLEMENT_DYNCREATE(CPointCloudDoc, CDocument)

BEGIN_MESSAGE_MAP(CPointCloudDoc, CDocument)
	ON_COMMAND(ID_FILE_OPEN, &CPointCloudDoc::OnFileOpen)
	ON_COMMAND(ID_EDIT_SIMPLY, &CPointCloudDoc::OnEditSimply)
	ON_COMMAND(ID_MESH, &CPointCloudDoc::OnMesh)
	ON_COMMAND(ID_EDIT_FLIP, &CPointCloudDoc::OnEditFlip)
	ON_COMMAND(ID_FTPMESH, &CPointCloudDoc::OnFtpMesh)
	ON_COMMAND(ID_POINTMESH, &CPointCloudDoc::OnPointMesh)
	ON_COMMAND(ID_BALLMESH_NORMALANDWEIGHT, &CPointCloudDoc::OnBallMeshNormalAndWeight)
	ON_COMMAND(ID_BALLMESH_GENERATORBALL, &CPointCloudDoc::OnBallMeshGeneratorBall)
	ON_COMMAND(ID_BALLMESH_GENERATORMESH, &CPointCloudDoc::OnBallMeshGeneratorMesh)
	ON_COMMAND(ID_BALLMESH_CLEANMESH, &CPointCloudDoc::OnBallMeshCleanMesh)
	ON_COMMAND(ID_BALLMESH_FILPNORMAL, &CPointCloudDoc::OnBallMeshFilpNormal)
	ON_COMMAND(ID_OCTSIM, &CPointCloudDoc::OnOctreeSimply)
	ON_COMMAND(ID_SHOWMESH, &CPointCloudDoc::OnShowMesh)
	ON_COMMAND(ID_SHOWPOINT, &CPointCloudDoc::OnShowPoint)
	ON_COMMAND(ID_SHOWWIRE, &CPointCloudDoc::OnShowWire)
	ON_COMMAND(ID_POINTMESH2, &CPointCloudDoc::OnPointmesh2)
	ON_COMMAND(ID_CMPNORMAL, &CPointCloudDoc::OnComputeNormal)
	ON_COMMAND(ID_ADJUSTNORMAL, &CPointCloudDoc::OnAdjustNormal)
	ON_UPDATE_COMMAND_UI(ID_CMPNORMAL, &CPointCloudDoc::OnUpdateComputeNormal)
	ON_UPDATE_COMMAND_UI(ID_ADJUSTNORMAL, &CPointCloudDoc::OnUpdateAdjustNormal)
	ON_COMMAND(ID_PARAMSETTING, &CPointCloudDoc::OnParamSetting)
END_MESSAGE_MAP()

// CPointCloudDoc 构造/析构

CPointCloudDoc::CPointCloudDoc()
{
	ps = NULL;
	m_mesh = NULL;
	m_pointMesh = NULL;

	m_ballMeshGen = NULL;
	m_ballGen = NULL;
	m_balls = NULL;
	m_ballMesh = NULL;

	mode = NONE;

	m_bPoint = false;
	m_bWire = false;
	m_bMesh = true;

	m_bCmpNormal = FALSE;
	m_bAjstNormal = FALSE;
	m_bUsedGlobalSetting = false;
	m_paramSet.initParam();
}

CPointCloudDoc::~CPointCloudDoc()
{
	if (m_pointMesh != NULL)
		delete m_pointMesh;
	m_pointMesh = NULL;
	if (m_mesh != NULL)
		delete m_mesh;
	m_mesh = NULL;
	if (ps != NULL)
		delete ps;
	ps = NULL;

	if(m_ballGen != NULL){
		delete m_ballGen;
		m_ballGen = NULL;
	}
	if(m_balls != NULL){
		delete m_balls;
		m_balls = NULL;
	}
	if(m_ballMesh != NULL){
		delete m_ballMesh;
		m_ballMesh = NULL;
	}
	if(m_ballMeshGen != NULL){
		delete m_ballMeshGen;
		m_ballMeshGen = NULL;
	}
}

BOOL CPointCloudDoc::OnNewDocument()
{
	if (!CDocument::OnNewDocument())
		return FALSE;

	return TRUE;
}

void CPointCloudDoc::Serialize(CArchive& ar)
{
	if (ar.IsStoring())
	{
	}
	else
	{
	}
}

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

void CPointCloudDoc::Draw(){
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);
	pView->draw();
}

// 主要的绘制函数
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
		pView->drawMesh(m_mesh);
		break;
	case POINTMESHMODE:
		if (m_bPoint)
			pView->drawPointMesh(m_pointMesh);
		else
			pView->drawPointMeshWithoutPoint(m_pointMesh);
		break;

	case BALLMODE:
		pView->drawBalls(m_balls);
		break;
	case BALLMESHMODE:
		if (m_bMesh)		pView->drawBallMesh(m_ballMesh);
		else if (m_bWire)pView->drawBallWire(m_ballMesh);
		break;
	}
}

void CPointCloudDoc::OnFileOpen()
{
	String fileExt(_T("All files(*.*)|*.*|"));
	fileExt += theExtRegPlugin.extString()+_T("|");

	CFileDialog* dlg = new CFileDialog(TRUE, NULL, NULL,
		OFN_HIDEREADONLY|OFN_OVERWRITEPROMPT, fileExt.c_str(), NULL);//LPCTSTR("All files(*.*)|*.*|pts(*.pts)|*.pts||")
	if (dlg->DoModal() == IDOK){
		if (ps != NULL){
			delete ps;
			ps = NULL;
		}
		if (m_mesh != NULL){
			delete m_mesh;
			m_mesh = NULL;
		}
		CString filePath = dlg->GetPathName();
		CString ext = dlg->GetFileExt();

		if (ext.CompareNoCase(_T("pts")) == 0){
			char* inFile = filePath.GetBuffer(1000);
			SetTitle(dlg->GetFileTitle());
			ps = new CPointSet;
			ps->readPts(inFile);
			mode = POINTMODE;
		} else {
			String sExt(ext.GetBuffer(10));
			sExt.insert(0, _T("."));
			String sFileName(filePath.GetBuffer(256));
			HINSTANCE hDll = NULL;
			ReadFileFunc readFunc = theExtRegPlugin.findExtProFunc(sExt, hDll);
			if (readFunc != NULL){
				vector<Point3> points;
				size_t pointSize = readFunc(sFileName, points);
				FreeLibrary(hDll);
				ps = new CPointSet(pointSize);
				for(size_t t = 0; t < pointSize; t++){
					ps->setPoint(t, points[t].p[0], points[t].p[1],points[t].p[2]);
				}
				mode = POINTMODE;
			}
		}

		if(ps != NULL)
			ps->rescale(30.0f);

		Draw();
		m_bCmpNormal = FALSE;
		m_bAjstNormal = FALSE;
	}
	delete dlg;
}

//===============================//
void CPointCloudDoc::OnComputeNormal()
{
	if (ps != NULL && ps->m_normal == NULL){
		if (!m_bUsedGlobalSetting){
			CSimplyParamDlg dlg;
			if (dlg.DoModal() == IDOK){
				m_paramSet.simplyK = dlg.m_K;
				m_paramSet.simplyMaxPointSize = dlg.m_M;
				m_paramSet.simplyMinDeep = dlg.m_Deep;
			}else{
				return ;
			}
		}
		ps->computeNormal(m_paramSet.simplyK);// 参数需要设置
		m_bCmpNormal = TRUE;
	}
}

void CPointCloudDoc::OnAdjustNormal()
{
	if (ps != NULL ){
		if (ps->m_normal == NULL){
			OnComputeNormal();
		}
		ps->adjustNormal(m_paramSet.simplyK);// 参数需要设置
		m_bAjstNormal = TRUE;
	}
}

void CPointCloudDoc::OnUpdateComputeNormal(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(!m_bCmpNormal);
}

void CPointCloudDoc::OnUpdateAdjustNormal(CCmdUI *pCmdUI)
{
	pCmdUI->Enable(!m_bAjstNormal);
}

//====== 均匀网格和曲率精简方法========//
void CPointCloudDoc::OnEditSimply()
{
	CSimplyParamDlg simplyParamDlg;
	if (simplyParamDlg.DoModal() == IDOK){
		ps->computeNormalAndSimpled(simplyParamDlg.m_K);
		m_bCmpNormal = TRUE;
		m_bAjstNormal = TRUE;
	}
	drawData();
}

// ========非均匀网格曲率精简=========//
void CPointCloudDoc::OnOctreeSimply()
{
	CSimplyPlugin simplyPlug(ps, m_paramSet.simplyK, m_paramSet.simplyMaxPointSize);
	simplyPlug.doSimply();
	drawData();
}

//=========IPD 方法生成网格==========//
void CPointCloudDoc::OnMesh()
{
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);

	if (ps != NULL){
		m_mesh = new CIPDMesh(ps, pView);
		mode = MESHMODE;
		m_mesh->start();
		drawData();
	}
}

//========三角形面法矢逆向==========//
void CPointCloudDoc::OnEditFlip()
{
	if (m_pointMesh != NULL){
 		//mesh->filpFaceNorm();
		m_pointMesh->filpNormal();
		POSITION pos = GetFirstViewPosition();
		if (!pos) return ;
		CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);
		pView->draw();
	}
}

//=======FTP 方法生成网格方法========//
void CPointCloudDoc::OnFtpMesh()
{
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);
	if (ps != NULL){
		if (m_mesh!= NULL) delete m_mesh;
		m_mesh = new CFTPMesh(ps, pView);
		mode = MESHMODE;
		m_mesh->start();
		drawData();
	}
}

//======基于顶点扩展的网格生成========//
void CPointCloudDoc::OnPointMesh(){
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);
	if (ps != NULL){
		if (m_pointMesh!= NULL) delete m_pointMesh;
		mode = POINTMESHMODE;
		m_pointMesh = new CPointMesh(ps, pView, 7);
		m_pointMesh->start();
		pView->drawPointMesh(m_pointMesh);

		AdjustMeshPlugin clearBadTrianglePlungin;
		clearBadTrianglePlungin.cleanBadTriangles(m_pointMesh);
		clearBadTrianglePlungin.adjustTriNormalByPointNormal(m_pointMesh);

	//	m_pointMesh->checkBoundaryPoint();
	}
}

void CPointCloudDoc::OnPointmesh2()
{
	POSITION pos = GetFirstViewPosition();
	if (!pos) return ;
	CPointCloudView* pView = (CPointCloudView*)GetNextView(pos);
	if (ps != NULL){
		if (m_pointMesh!= NULL) delete m_pointMesh;
		mode = POINTMESHMODE;
		CSetK dlg;
		if (dlg.DoModal() == IDOK){
			m_pointMesh = new CPointMesh(ps, pView,  dlg.m_K);
			m_pointMesh->setAB(dlg.m_A, dlg.m_B);
			//m_pointMesh->m_B = 0.1;
			m_pointMesh->startT();
			pView->drawPointMesh(m_pointMesh);

			AdjustMeshPlugin clearBadTrianglePlungin;
			clearBadTrianglePlungin.cleanBadTriangles(m_pointMesh);
			clearBadTrianglePlungin.adjustTriNormalByPointNormal(m_pointMesh);
	//		clearBadTrianglePlungin.adjustTriNormal(m_pointMesh);
			m_pointMesh->checkBoundaryPoint();
		}
	}
}

// =========自适应球相交的网格生成========= //
void CPointCloudDoc::OnBallMeshNormalAndWeight()
{
	if (ps == NULL) return;
	if (m_ballGen == NULL)
		m_ballGen = new BallGenerator(ps);
	m_ballGen->precom();
	Draw();
}

void CPointCloudDoc::OnBallMeshGeneratorBall()
{
	if (ps == NULL) return;
	if (m_ballGen == NULL || ps->m_weight== NULL || ps->m_normal == NULL){
		m_ballGen = new BallGenerator(ps);
		m_ballGen->precom();
	}
	if (m_ballDlg.DoModal() != IDOK)
		return;
	m_ballGen->generate(m_ballDlg.m_err, 0, m_ballDlg.m_qem);
	m_balls = m_ballGen->getBalls();
	mode = BALLMODE;
	Draw();
}

void CPointCloudDoc::OnBallMeshGeneratorMesh()
{
	m_ballMeshGen = new BallMeshGenerator;
	m_ballMeshGen->createTriangles(m_balls);  // 此处已经将balls删除了，但是balls的指针值没有变
	m_balls = NULL;
	CPointSet* vs = m_ballGen->getVertices();    // vs 和 balls 应该相同
	m_ballMeshGen->createMesh(vs);
	vs = NULL;
	m_ballMesh = m_ballMeshGen->mesh;
	m_ballMesh->computeNormal();

	mode = BALLMESHMODE;
	Draw();
}

void CPointCloudDoc::OnBallMeshCleanMesh()
{
	m_ballMeshGen->cleanBadTriangles();
	m_ballMesh->computeNormal();
	mode = BALLMESHMODE;
	Draw();
}

void CPointCloudDoc::OnBallMeshFilpNormal()
{
	if (ps != NULL)
		ps->flipNormal();
	if (m_ballMesh != NULL)
		m_ballMesh->flipNormal();
	Draw();
}
//===============================//
void CPointCloudDoc::OnShowPoint()
{
	m_bPoint = true;
}

void CPointCloudDoc::OnShowWire()
{
	m_bMesh = false;
	m_bWire = true;
}

void CPointCloudDoc::OnShowMesh()
{
	m_bMesh = true;
	m_bWire = false;
}

void CPointCloudDoc::OnParamSetting()
{
	CParamSettingDlg settingDlg;
	if (settingDlg.DoModal()){
		m_bUsedGlobalSetting = settingDlg.m_bUseGlobalSetting;
		m_paramSet.simplyK = settingDlg.m_simplyK;
		m_paramSet.simplyMaxPointSize = settingDlg.m_simplyMaxPointSize;
		m_paramSet.simplyMinDeep = settingDlg.m_simplyMinDeep;
		m_paramSet.ballTerr = settingDlg.m_ballTerr;
		m_paramSet.ballTq = settingDlg.m_ballTq;
		m_paramSet.pointMeshK = settingDlg.m_pointMeshK;
		m_paramSet.pointMeshA = settingDlg.m_pointMeshA;
		m_paramSet.pointMeshB = settingDlg.m_pointMeshB;
	}
}