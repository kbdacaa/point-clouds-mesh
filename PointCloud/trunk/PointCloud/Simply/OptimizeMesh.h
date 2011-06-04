#ifndef _OPTIMIZEMESH_
#define _OPTIMIZEMESH_
#include "../PointMesh.h"
#include "../BallMesh/BallMesh.h"
#include <set>
#include <algorithm>
using namespace std;

struct PtValue{
	int idx;
	float value;
};

inline bool biger(const PtValue& a, const PtValue& b){
	return a.value > b.value;
}

class COptimizeMeshPlugin{
public:
	int * m_degTris;// 顶点连接三角形个数
	int ** m_linkTris;// 顶点连接三角形链表
	bool * m_delTri;	// 三角形是否为删除
public:

	void fillSmallHoles(TriList2*& tri_list, vector<size_t>& vertexs, vector<size_t>& faces, vector<CTriangle>& tris, int N);
	void traceOpt(TriList2*& tri_list, int i, int k, int ** opt, int* v);

	void optimizeUsePoint(BallMesh* mesh);
	float computeVertexValue(int idx, int* tris, int triN, BallMesh* mesh);
	bool deleteVertex(int idx, BallMesh* mesh);
	void fillVertexHole(TriList2*& tir_list, int* stack, int N, BallMesh* mesh);
	float ComputeAngel(int v[3], float (*vertex)[3]);
}

inline COptimizeMeshPlugin::optimizeUsePoint(BallMesh* mesh){
	int i, j, k;
	int (*face)[3] = mesh->face;

	m_degTris = new int[mesh->vertexN];
	for (i = 0; i < mesh->vertexN; i++) m_degTris[i] = 0;
	for (i = 0; i < mesh->faceN; i++){
		int *f = face[i];
		m_degTris[ f[0] ]++;
		m_degTris[ f[1] ]++;
		m_degTris[ f[2] ]++;
	}

	m_linkTris = new int *[mesh->vertexN];
	for (i = 0; i < mesh->vertexN; i++){
		m_linkTris[i] = new int [ m_degTris[i] ];
		m_degTris[i] = 0;
	}

	for (i = 0; i < mesh->faceN; i++){
		int *f = face[i];
		m_linkTris[ f[0] ][ m_degTris[ f[0] ]++ ] = i;
		m_linkTris[ f[1] ][ m_degTris[ f[1] ]++ ] = i;
		m_linkTris[ f[2] ][ m_degTris[ f[2] ]++ ] = i;
	}

	PtValue* ptValues = new PtValue[mesh->vertexN];
	for (i = 0; i < mesh->vertexN; i++){
		ptValues[i].idx = i;
		ptValues[i].value = computeVertexValue(i, m_linkTris[i], m_degTris[i], mesh);
	}

	std::sort(&ptValues[0], &ptValues[mesh->vertexN], biger);

	delete[] m_degTris; m_degTris = NULL;
	for (i = 0; i < mesh->vertexN; i++)
		delete[] m_linkTris[i];
	delete m_linkTris; m_linkTris = NULL;
}

inline float COptimizeMeshPlugin::computeVertexValue(
	int idx, int* tris, int triN, BallMesh* mesh){
		float* ptNor = mesh->normal_v[idx];
		float valueS = 0.0F;
		for (int i = 0; i < triN; i++){
			int triIdx = tris[i];
			float* triNor = mesh->normal_f[triIdx];
			valueS += abs( eCos(ptNor, triNor) );
		}
		return valueS / triN;// 值越大说明越平滑[0-1]
}

inline bool COptimizeMeshPlugin::deleteVertex(int idx, BallMesh* mesh){
	int triN = m_degTris[idx];
	int * tris = m_linkTris[idx];
	int (*face)[3] = mesh->face;

	bool * visit = new bool[triN];
	for (int i = 0; i < triN; i++) visit[i] = false;
	int *stack = new int [triN+1];
	int top = 0;
	int v = face[ tris[0] ][0],
		v1 = face[ tris[0] ][1],
		v2 = face[ tris[0] ][2];
	if (v == idx){
		stack[top++] = v1;
		stack[top] = v2;
	}else if (v1 == idx){
		stack[top++] = v;
		stack[top] = v2;
	}else{
		stack[top++] = v;
		stack[top] = v1;
	}
	visit[0] = true;

	bool bFilling = true;
	while (top < triN+1){
		v = stack[top];
		if (v == stack[0]){
			bFilling = true;
			break;
		}

		int j = 0;
		for (; j < triN; j++){
			if (visit[j]) continue;

			int * f = face[ tris[j] ];
			if (f[0] == v){
				if (f[1] == idx) stack[++top] = f[2];
				else stack[++top] = f[1];
				visit[j] = true;
				break;
			}else if (f[1] == v){
				if (f[0] == idx) stack[++top] = f[2];
				else stack[++top] = f[0];
				visit[j] = true;
				break;
			}else if (f[2] == v){
				if (f[1] == idx) stack[++top] = f[0];
				else stack[++top] = f[1];
				visit[j] = true;
				break;
			}
		}
		if (j == triN){
			bFilling = false;
			break;
		}
	}
	delete[] visit;
	TriList2* tri_list = new TriList2(-1, -1, -1);
	if (bFilling){
//		for (int k =0; k < triN; k++) m_delTri[ tris[k] ] = true;
		fillVertexHole(tri_list, stack, top, mesh);
		// 此处需要更新其他顶点的连接
		delete[] stack;
		return true;
	}
	delete[] stack;
	return false;
}

inline void COptimizeMeshPlugin::fillVertexHole(
	TriList2*& tir_list, int* stack, int N, BallMesh* mesh){
		if (N == 3){
			tir_list->add( stack[0], stack[1], stack[2]);
			return ;
		}
		float* values = new float [N];
		float (*vertex)[3] = mesh->vertex;
		int v[3] = { stack[N-1], stack[0], stack[1] };
		values[0] = ComputeAngel(v, vertex);
		for (int i = 1; i < N-1; i++){
			int* v3 = stack[i-1];
			values[i] = ComputeAngel(v3, vertex);
		}
		v[0] = stack[N-2];
		v[1] = stack[N-1];
		v[2] = stack[0];
		values[N-1] = ComputeAngel(v, vertex);

		float maxValue = values[0];
		int best = 0;
		for (int j = 1; j < N; j++){
			if (maxValue < values[j]){
				maxValue = values[j];
				best = j;
			}
		}

		if (best == 0){
			tir_list->add(stack[N-1], stack[0], stack[1]);
		}else if (best == N-1){
			tir_list->add(stack[N-2], stack[N-1], stack[0]);
		}else{
			tir_list->add(stack[best-1], stack[best], stack[best+1]);
		}
		int * stact = new int [N-1];
		int t = 0;
		for (int i = 0; i < N; i++){
			if (i == best) continue;
			stact[t++] = stack[i];
		}
		fillVertexHole(tir_list, stact, N-1, mesh);
		delete[] stact;
		delete[] values;
}

inline float COptimizeMeshPlugin::ComputeAngel(int v[3], float (*vertex)[3]){
	float * pt0 = vertex[ v[0] ],
		* pt1 = vertex[ v[1] ],
		* pt2 = vertex[ v[2] ];
	float e10[3] = Edge(pt1, pt0),
		e12 = Edge(pt1, pt2);
	return eCos(e12, e10);
}
#endif	//  _OPTIMIZEMESH_