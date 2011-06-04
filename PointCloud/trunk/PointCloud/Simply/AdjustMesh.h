#ifndef _AdjustMesh_
#define _AdjustMesh_

#include "../PointMesh.h"
#include <algorithm>
using namespace std;

class AdjustMeshPlugin{
public:
	size_t m_pointN;
	vector<vector<size_t> > m_ptLinkTris;	// 与每一个顶点相连的三角形链表

	typedef vector<vector<size_t> >::iterator vvIt;
    typedef vector<size_t>::iterator vIt;

public:
	void makePtLinkTris(CPointMesh* mesh);
	void fillSmallHole(CPointMesh* mesh);

    void cleanBadTriangles(CPointMesh* mesh);
    void cleanBadTriangle(vector<size_t>& triList, vector<CTriangle>& tris, vector<bool>& bTriDel);
    bool existInVector(size_t ptIndex, vector<size_t>& ptHoles);
    void clearDelTris(vector<CTriangle>& tris, vector<bool>& bTriDel);

	void adjustTriNormal(CPointMesh* mesh);
	void adjustPtTrisNormal(vector<size_t>& ptTrislink, vector<CTriangle>& tris, vector<bool>& bTriNormalAdjust);

	void AdjustMeshPlugin::adjustTriNormalByPointNormal(CPointMesh* mesh);
	void AdjustMeshPlugin::adjustPtTriNormalByPointNormal(float normalPt[3],
		vector<size_t>& ptTrilink, vector<CTriangle>& tris, vector<bool>& bTriNormalAdjust);
};

inline void AdjustMeshPlugin::makePtLinkTris(CPointMesh* mesh){
	vector<CTriangle>& tris = mesh->m_faces;
	vector<CTriangle>::iterator itT = tris.begin();
	m_ptLinkTris.resize(mesh->m_ps->getPointSize());
	size_t index = 0;
	for (; itT != tris.end(); itT++){
		m_ptLinkTris[itT->getA()].push_back(index);
		m_ptLinkTris[itT->getB()].push_back(index);
		m_ptLinkTris[itT->getC()].push_back(index++);
	}
}

inline void AdjustMeshPlugin::fillSmallHole(CPointMesh* mesh){
	m_ptLinkTris.clear();
	m_pointN = mesh->m_ps->getPointSize();
	makePtLinkTris(mesh);

	vector<CTriangle>& tris = mesh->m_faces;
	vvIt it = m_ptLinkTris.begin();
	for (; it != m_ptLinkTris.end(); it++)
	{
		vector<size_t>& ptTris = *it;// 与当前中心点相连的三角形集合
		vector<size_t> ptIndexs;	// 与当前中心点相连的点集

		for (vector<size_t>::iterator sit = ptTris.begin(); sit != ptTris.end(); sit++)
		{
			CTriangle& tri = tris[*sit];
			ptIndexs.push_back(tri.getA());
			ptIndexs.push_back(tri.getB());
			ptIndexs.push_back(tri.getC());
		}
		// ==在ptIndexs中查找只包含一次的顶点：孔洞点(也可能为错误网格)===//
		vector<size_t> ptHoles;		// 保存与当前中心点相连的孔洞点
		std::sort(ptIndexs.begin(), ptIndexs.end());
		size_t i = 0, j = 1;
		while (j < ptIndexs.size()){
			if ( i == ptIndexs.size()-1) ptHoles.push_back(ptIndexs[i]);
            while (j < ptIndexs.size() && ptIndexs[i] == ptIndexs[j]) j++;
            if (i == j -1) ptHoles.push_back(ptIndexs[i]);
            i = j;
		}
	}
}

inline void AdjustMeshPlugin::cleanBadTriangles(CPointMesh* mesh){
    m_ptLinkTris.clear();
    m_pointN = mesh->m_ps->getPointSize();
    makePtLinkTris(mesh);

    vector<CTriangle>& tris = mesh->m_faces;
    vector<bool> bTriDel;
    bTriDel.resize(tris.size(), false);

    vvIt it = m_ptLinkTris.begin();
    for (; it != m_ptLinkTris.end(); it++){
        vector<size_t>& ptTris = *it;// 与当前中心点相连的三角形集合
        cleanBadTriangle(ptTris, tris, bTriDel);
    }
    clearDelTris(tris, bTriDel);
}
// 如果一个三角形的其他两个顶点和中心点都相连一次时，为错误三角形《错误三角形》
inline void AdjustMeshPlugin::cleanBadTriangle(vector<size_t>& triList, vector<CTriangle>& tris, vector<bool>& bTriDel){
    vector<size_t> ptIndexs;	// 与当前中心点相连的点集

    for (vector<size_t>::iterator sit = triList.begin(); sit != triList.end(); sit++)
    {
        CTriangle& tri = tris[*sit];
        ptIndexs.push_back(tri.getA());
        ptIndexs.push_back(tri.getB());
        ptIndexs.push_back(tri.getC());
    }
    // ==在ptIndexs中查找只包含一次的顶点：孔洞点(也可能为错误网格)===//
    vector<size_t> ptHoles;		// 保存与当前中心点相连一次的孔洞点
    std::sort(ptIndexs.begin(), ptIndexs.end());
    size_t i = 0, j = 1;
    while (j < ptIndexs.size()){
        if ( i == ptIndexs.size()-1) ptHoles.push_back(ptIndexs[i]);
        while (j < ptIndexs.size() && ptIndexs[i] == ptIndexs[j]) j++;
        if (i == j -1) ptHoles.push_back(ptIndexs[i]);
        i = j;
    }

    for (vector<size_t>::iterator it = triList.begin(); it != triList.end(); it++){
        CTriangle& tri = tris[*it];
        int ptInHoleTimes = 0;
        if (existInVector(tri.getA(), ptHoles)) ptInHoleTimes++;
        if (existInVector(tri.getB(), ptHoles)) ptInHoleTimes++;
        if (existInVector(tri.getC(), ptHoles)) ptInHoleTimes++;
        if (ptInHoleTimes == 2){// 三角形三个顶点中有两个顶点均连接一次，这是错误网格
            bTriDel[*it] = true;
        }
    }
}

inline bool AdjustMeshPlugin::existInVector(size_t ptIndex, vector<size_t>& ptHoles){
    for (vector<size_t>::iterator it = ptHoles.begin(); it != ptHoles.end(); it++){
        if (*it == ptIndex) return true;
    }
    return false;
}

inline void AdjustMeshPlugin::clearDelTris(vector<CTriangle>& tris, vector<bool>& bTriDel){
    vector<CTriangle> newTris;

    for (size_t i = 0; i < bTriDel.size(); i ++){
        if ( !bTriDel[i] ) newTris.push_back( tris[i] );
    }
    tris.clear();
    tris.swap(newTris);
}

// 使用相连面的法矢直接进行调整面法矢的方向
inline void AdjustMeshPlugin::adjustTriNormal(CPointMesh* mesh){
	m_ptLinkTris.clear();
	m_pointN = mesh->m_ps->getPointSize();
	makePtLinkTris(mesh);

	vector<bool> bTriNormalAdjust;
	bTriNormalAdjust.resize(mesh->m_faces.size(), false);
	vector<CTriangle>& tris = mesh->m_faces;

	for(size_t i = 0; i < m_pointN; i++){
		vector<size_t>& ptTrilink = m_ptLinkTris[i];
		if (ptTrilink.size() > 1)
			adjustPtTrisNormal(ptTrilink, tris, bTriNormalAdjust);
	}
	bTriNormalAdjust.clear();
}

inline void AdjustMeshPlugin::adjustPtTrisNormal(vector<size_t>& ptTrislink, vector<CTriangle>& tris, vector<bool>& bTriNormalAdjust){
	vector<size_t>::iterator it = ptTrislink.begin();
	for ( ; it != ptTrislink.end(); it++){
		if (bTriNormalAdjust[*it] == true) break;
	}

	float normal[3]={0,0,0};
	if (it != ptTrislink.end()){
		tris[*it].getNorm(normal);
	}else{
		tris[ ptTrislink[0] ].getNorm(normal);
		bTriNormalAdjust[ ptTrislink[0] ] = true;
	}

	for (it = ptTrislink.begin(); it != ptTrislink.end(); it++){
		size_t triIndex = *it;
		if (!bTriNormalAdjust[triIndex]){
			float normA[3] = {0,0,0};
			tris[triIndex].getNorm(normA);
			if (eCosNoSqrt(normal, normA) < 0){
				tris[triIndex].filpNorm();
			}
			bTriNormalAdjust[triIndex] = true;
		}
	}
}

// 使用顶点法矢来调整相连面法矢的方向
inline void AdjustMeshPlugin::adjustTriNormalByPointNormal(CPointMesh* mesh){
	m_ptLinkTris.clear();
	m_pointN = mesh->m_ps->getPointSize();
	makePtLinkTris(mesh);

	vector<bool> bTriNormalAdjust;
	vector<CTriangle>& tris = mesh->m_faces;
	bTriNormalAdjust.resize(tris.size(), false);

	for (size_t i = 0; i < m_pointN; i++){
		vector<size_t>& ptTrilink = m_ptLinkTris[i];
		if (ptTrilink.size() > 0) continue;
		float* normalPt = mesh->m_ps->m_normal[i];
		adjustPtTriNormalByPointNormal(normalPt, ptTrilink, tris, bTriNormalAdjust);
	}
	bTriNormalAdjust.clear();
}

inline void AdjustMeshPlugin::adjustPtTriNormalByPointNormal(float normalPt[3],
	vector<size_t>& ptTrilink, vector<CTriangle>& tris, vector<bool>& bTriNormalAdjust){
		for (int i = 0; i < ptTrilink.size(); i++){
			size_t triIndex = ptTrilink[i];
			if (bTriNormalAdjust[ triIndex ] == true) continue;
			float normal[3];
			tris[triIndex].getNorm(normal);
			if (eCosNoSqrt(normalPt, normal) < 0)
				tris[triIndex].filpNorm();
			bTriNormalAdjust[triIndex] = true;
		}
}

#endif  //_AdjustMesh_