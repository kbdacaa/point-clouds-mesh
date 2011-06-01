#ifndef MESHGENERATOR_H
#define MESHGENERATOR_H

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include "../PointSet.h"
#include "KdTree.h"
#include "Ball.h"
#include "BallMesh.h"

#include <stdlib.h>
#include <math.h>

class BallMeshGenerator {
public:
	BallMesh *mesh;
	int(*tris)[3];
	int triN; //+三角形的个数+

	void createTriangles(CPointSet *bs) {
		cout << "Finding triangles" << endl;

		int i, j, k;

		//Conectivity using spheres
		cout << "Creating ball intersection graph" << endl;
		KdTree *tree = new KdTree(bs);

		int baN = bs->m_pointN;

		int **node = new int *[baN]; // 统计每个球与相邻球相交的序号
		int *nodeN = new int[baN]; // 每个球与相邻球相交的个数

		for (i = 0; i < baN; i++)
			nodeN[i] = 0;  // 相交个数 初始为0

		float**point = bs->m_point;
		float *radius = bs->m_weight;

		for (i = 0; i < baN; i++) {
			float *p = point[i]; //球心p
			float r = radius[i]; //球p的半径

			int listN,  *list;
			tree->collectPointIndexInSphere(list, listN, p, 2.0f *r); //+得到点p的临近点集+

			for (j = 0; j < listN; j++) {
				int i1 = list[j];
				if (i == i1)
					continue;
				float *p1 = point[i1]; //球心p1
				float r1 = radius[i1]; //球p1的半径

				float vx = p[0] - p1[0];
				float vy = p[1] - p1[1];
				float vz = p[2] - p1[2];
				double d = sqrt(vx *vx + vy * vy + vz * vz); //+两圆心p、p1间距+

				if (r + r1 > d && fabs(r - r1) < d) { // 两球相交
					int *tmp = node[i]; // 上一个统计相交信息
					int nN = nodeN[i];
					int *n = node[i] = new int[nN + 1]; // 增加一个
					for (k = 0; k < nN; k++)
						n[k] = tmp[k];
					if (nN != 0)
						delete []tmp;
					n[k] = i1;
					nodeN[i]++;

					if (2 *r1 < d) {
						tmp = node[i1];
						nN = nodeN[i1];
						n = node[i1] = new int[nN + 1];
						for (k = 0; k < nN; k++)
							n[k] = tmp[k];
						if (nN != 0)
							delete []tmp;
						n[k] = i;
						nodeN[i1]++;
					}
				}
			}
		}
		delete tree;

		int triN_max = 3 * baN;
		triN = 0;
		tris = new int[triN_max][3];

		//create candidates of triangles
		cout << "Investigating intersections" << endl;
		for (i = 0; i < baN; i++) {
			float *c1 = point[i];
			float r1 = radius[i];

			int *ni = node[i];
			int niN = nodeN[i];
			for (int n1 = 0; n1 < niN; n1++) {
				int j = ni[n1];
				if (j < i)	// 避免重复计算
					continue;
				float *c2 = point[j];
				float r2 = radius[j];


				for (int n2 = 0; n2 < niN; n2++) {
					int k = ni[n2];
					if (k <= j)	// 避免重复计算
						continue;

					float *c3 = point[k];
					float r3 = radius[k];

					float p1[3], p2[3];	// 三球相交的两点
					if (!intersectionOf3Balls(p1, p2, c1, c2, c3, r1, r2, r3))
						continue;

					//check wheather the intersection is most outside of balls
					bool flag1 = true;	// 是否在其他球外部
					bool flag2 = true;
					for (int n3 = 0; n3 < niN; n3++) {
						int index = ni[n3];
						if (index == j || index == k)  continue;

						float *c4 = point[index];
						float r4 = radius[index];
						r4 *= r4;

						if ((p1[0] - c4[0])*(p1[0] - c4[0]) + (p1[1] - c4[1])*(p1[1] - c4[1]) + (p1[2] - c4[2])*(p1[2] - c4[2]) < r4)
							flag1 = false;
						if ((p2[0] - c4[0])*(p2[0] - c4[0]) + (p2[1] - c4[1])*(p2[1] - c4[1]) + (p2[2] - c4[2])*(p2[2] - c4[2]) < r4)
							flag2 = false;

						if (!flag1 && !flag2)
							break;
					}

					if (!flag1 && !flag2)
						continue;
					// 如果已经达到当前三角面上限，申请更多的内存，并复制以前的面
					if (triN == triN_max) {
						triN_max *= 2;
						int(*tris1)[3] = tris;
						tris = new int[triN_max][3];
						for (int o = 0; o < triN; o++) {
							tris[o][0] = tris1[o][0];
							tris[o][1] = tris1[o][1];
							tris[o][2] = tris1[o][2];
						}
						delete []tris1;
						//mesh->face = tris;
					}

					tris[triN][0] = i;
					tris[triN][1] = j;
					tris[triN][2] = k;
					triN++;
				}
			}
		}
		for (i = 0; i < baN; i++)
			// 清理资源
			if (nodeN[i] != 0)
				delete node[i];
		delete bs;
		bs = NULL;
		//ps = bs;
		delete []node;
		delete []nodeN;

		cout << triN << " triangles are found" << endl;
	}

	void createMesh(CPointSet *vs) {
		//vs = ps;

		int N = vs->m_pointN;
		mesh = new BallMesh(N, triN);

		float**point = vs->m_point;
		int i;
		for (i = 0; i < N; i++) {
			float *p = point[i];
			mesh->setVertex(i, p[0], p[1], p[2]);
		}
		delete vs;

		for (i = 0; i < triN; i++) {
			int *t = tris[i];
			mesh->setFace(i, t[0], t[1], t[2]);
		}
		delete []tris;
		tris = mesh->face;
	}

	void cleanBadTrianglesE() {
		int **edge_to_tri;
		int *edge_to_triN;
		int(*tri_to_edge)[3];

		int i, j, k;

		//make edge data
		cout << "Making the edge data" << endl;
		int N = mesh->vertexN;
		int *ver_to_triN = new int[N];
		for (i = 0; i < N; i++)
			ver_to_triN[i] = 0;

		for (i = 0; i < triN; i++) {
			int *t = tris[i];
			ver_to_triN[t[0]]++;
			ver_to_triN[t[1]]++;
			ver_to_triN[t[2]]++;
		}

		int **ver_to_tri = new int *[N];
		for (i = 0; i < N; i++) {
			if (ver_to_triN[i] != 0) {
				ver_to_tri[i] = new int[ver_to_triN[i]];
				ver_to_triN[i] = 0;
			}
		}

		for (i = 0; i < triN; i++) {
			int *t = tris[i];
			for (j = 0; j < 3; j++) {
				k = t[j];
				ver_to_tri[k][ver_to_triN[k]++] = i;
			}
		}

		//count #edge
		int edgeN = 0;
		for (i = 0; i < N; i++) {
			int vtN = ver_to_triN[i];
			int *vt = ver_to_tri[i];
			int *v = new int[2 *vtN];
			int vN = 0;
			for (j = 0; j < vtN; j++) {
				int *t = tris[vt[j]];
				int v1, v2;
				if (t[0] == i) {
					v1 = t[1];
					v2 = t[2];
				} else if (t[1] == i) {
					v1 = t[2];
					v2 = t[0];
				} else {
					v1 = t[0];
					v2 = t[1];
				}

				if (v1 > i) {
					bool flag = true;
					for (k = 0; k < vN; k++) {
						if (v1 == v[k]) {
							flag = false;
							break;
						}
					}
					if (flag)
						v[vN++] = v1;
				}

				if (v2 > i) {
					bool flag = true;
					for (k = 0; k < vN; k++) {
						if (v2 == v[k]) {
							flag = false;
							break;
						}
					}
					if (flag)
						v[vN++] = v2;
				}
			}
			if (vtN != 0)
				delete []v;
			edgeN += vN;
		}

		edge_to_triN = new int[edgeN];
		edge_to_tri = new int *[edgeN];
		tri_to_edge = new int[triN][3];
		for (i = 0; i < edgeN; i++)
			edge_to_triN[i] = 0;

		edgeN = 0;
		for (i = 0; i < N; i++) {
			int vtN = ver_to_triN[i];
			int *vt = ver_to_tri[i];
			int *v = new int[2 *vtN];
			int vN = 0;
			for (j = 0; j < vtN; j++) {
				int *t = tris[vt[j]];
				int v1, v2, iv;
				if (t[0] == i) {
					iv = 0;
					v1 = t[1];
					v2 = t[2];
				} else if (t[1] == i) {
					iv = 1;
					v1 = t[2];
					v2 = t[0];
				} else {
					iv = 2;
					v1 = t[0];
					v2 = t[1];
				}

				if (v1 > i) {
					for (k = 0; k < vN; k++)
						if (v1 == v[k])
							break;
					int ie = edgeN - vN + k;
					if (k == vN) {
						edgeN++;
						v[vN++] = v1;
					}
					tri_to_edge[vt[j]][iv] = ie;

					int *et = edge_to_tri[ie];
					int etN = edge_to_triN[ie];
					int *tmp = new int[etN + 1];
					for (k = 0; k < etN; k++)
						tmp[k] = et[k];
					tmp[k] = vt[j];
					if (etN != 0)
						delete []et;
					edge_to_tri[ie] = tmp;
					edge_to_triN[ie]++;
				}

				if (v2 > i) {
					for (k = 0; k < vN; k++)
						if (v2 == v[k])
							break;
					int ie = edgeN - vN + k;
					if (k == vN) {
						edgeN++;
						v[vN++] = v2;
					}
					tri_to_edge[vt[j]][(iv + 2) % 3] = ie;

					int *et = edge_to_tri[ie];
					int etN = edge_to_triN[ie];
					int *tmp = new int[etN + 1];
					for (k = 0; k < etN; k++)
						tmp[k] = et[k];
					tmp[k] = vt[j];
					if (etN != 0)
						delete []et;
					edge_to_tri[ie] = tmp;
					edge_to_triN[ie]++;
				}
			}
			if (vtN != 0)
				delete []v;
		}

		for (i = 0; i < N; i++)
			if (ver_to_triN[i] != 0)
				delete []ver_to_tri[i];
		delete []ver_to_tri;
		delete []ver_to_triN;

		//find max conponent
		cout << "Finding the seed triangle" << endl;
		bool *visitT = new bool[triN];
		for (i = 0; i < triN; i++)
			visitT[i] = 0;

		int *stack = new int[triN];
		int top;
		int seed = 0;
		int max_tri = 1;
		for (i = 0; i < triN; i++) {
			if (visitT[i])
				continue;
			stack[0] = i;
			top = 0;
			int count = 0;
			while (top >= 0) {
				count++;
				int current = stack[top--];
				int *e = tri_to_edge[current];
				for (j = 0; j < 3; j++) {
					int ej = e[j];
					int *et = edge_to_tri[ej];
					int etN = edge_to_triN[ej];
					if (etN != 2)
						continue;
					int t_ad;
					if (et[0] == current)
						t_ad = et[1];
					else
						t_ad = et[0];
					if (visitT[t_ad])
						continue;
					visitT[t_ad] = true;
					stack[++top] = t_ad;
				}
			}
			if (count > max_tri) {
				seed = i;
				max_tri = count;
			}
		}

		//Propergate orientation
		cout << "Propergating an orientation" << endl;
		for (i = 0; i < triN; i++)
			visitT[i] = false;

		bool *visitV = new bool[N];
		for (i = 0; i < N; i++)
			visitV[i] = false;

		float *cost = new float[triN];

		int *heap = new int[triN + 1];
		int *index = new int[triN];
		int last_heap = 0;

		for (i = 0; i < triN; i++) {
			cost[i] = 1000000000000.0f;
			index[i] =  - 1;
		}
		//insert
		heap[++last_heap] = seed;
		index[seed] = last_heap;
		upheap(cost, last_heap, last_heap, heap, index);
		while (last_heap > 0) {
			//remove top
			int start = heap[1];
			index[start] =  - 1;
			heap[1] = heap[last_heap--];
			index[heap[1]] = 1;
			downheap(cost, last_heap, 1, heap, index);

			if (visitT[start])
				continue;
			/*
			int current = start;
			int* e = tri_to_edge[current];
			int ad[3] = {-1, -1, -1};
			bool flag = true;
			for(j=0; j<3; j++){
			int ej = e[j];
			int* et = edge_to_tri[ej];
			int etN = edge_to_triN[ej];
			if(etN < 2)
			continue;
			for(k=0; k<etN; k++){
			int etk = et[k];
			if(etk != current && visitT[etk]){
			if(ad[j] < 0)
			ad[j] = etk;
			else{
			flag = false;
			break;
			}
			}
			}
			if(!flag)
			break;
			}
			if(!flag)
			continue;

			int *t = tris[current];

			if(visitV[t[0]] && visitV[t[1]] && visitV[t[2]]){
			int eN = 0;
			if(ad[0] >= 0) eN++;
			if(ad[1] >= 0) eN++;
			if(ad[2] >= 0) eN++;
			if(eN == 1)
			continue;
			else
			cout << eN << endl;
			}

			visitT[current] = true;
			visitV[t[0]] = true;
			visitV[t[1]] = true;
			visitV[t[2]] = true;

			for(j=0; j<3; j++){
			int ej = e[j];
			int* et = edge_to_tri[ej];
			int etN = edge_to_triN[ej];
			if(etN == 1)
			continue;
			int t_ad = -1;
			for(k=0; k<etN; k++){
			int etk = et[k];
			if(etk != current && visitT[etk]){
			t_ad = etk;
			break;
			}
			}

			if(t_ad < 0){
			float kmin = 10000000000.0f;
			for(k=0; k<etN; k++){
			int etk = et[k];
			if(etk == current)
			continue;
			float kk = curvature(current, j, etk, tri_to_edge[etk]);
			if(kk < kmin){
			t_ad = etk;
			kmin = kk;
			}
			}
			if(index[t_ad] < 0){
			//insert
			cost[t_ad] = kmin;
			heap[++last_heap] = t_ad;
			index[t_ad] = last_heap;
			upheap(cost, last_heap, last_heap, heap, index);
			}
			else if(kmin < cost[t_ad]){
			//update
			cost[t_ad] = kmin;
			if(index[t_ad] != 1 && cost[t_ad] < cost[heap[index[t_ad]/2]])
			upheap(cost, last_heap, index[t_ad], heap, index);
			else
			downheap(cost, last_heap, index[t_ad], heap, index);
			}
			}
			else{
			//remove other triangles
			for(k=0; k<etN; k++){
			int etk = et[k];
			if(etk == current || visitT[etk] || index[etk] < 0)
			continue;

			cost[etk] = -1;
			upheap(cost, last_heap, index[etk], heap, index);
			index[etk] = -1;
			heap[1] = heap[last_heap--];
			index[heap[1]] = 1;
			downheap(cost, last_heap, 1, heap, index);
			}
			}
			}*/

			visitT[start] = true;
			stack[0] = start;
			top = 0;
			while (top >= 0) {
				int current = stack[top--];
				int *e = tri_to_edge[current];

				bool flag = true;
				for (j = 0; j < 3; j++) {
					int ej = e[j];
					int *et = edge_to_tri[ej];
					int etN = edge_to_triN[ej];
					if (etN < 3)
						continue;
					int count = 0;
					for (k = 0; k < etN; k++) {
						int etk = et[k];
						if (etk != current && visitT[etk])
							count++;
					}
					if (count > 1) {
						visitT[current] = false;
						flag = false;
						break;
					}
				}
				if (!flag)
					continue;

				for (j = 0; j < 3; j++) {
					int ej = e[j];
					int *et = edge_to_tri[ej];
					int etN = edge_to_triN[ej];
					if (etN == 1)
						continue;
					else if (etN != 2) {
						int t_ad =  - 1;
						for (k = 0; k < etN; k++) {
							int etk = et[k];
							if (etk != current && visitT[etk]) {
								t_ad = etk;
								break;
							}
						}
						if (t_ad < 0) {
							float kmin = 10000000000.0f;
							for (k = 0; k < etN; k++) {
								int etk = et[k];
								if (etk == current)
									continue;
								float kk = curvature(current, j, etk, tri_to_edge[etk]);
								if (kk < kmin) {
									t_ad = etk;
									kmin = kk;
								}
							}
							if (index[t_ad] < 0) {
								//insert
								cost[t_ad] = kmin;
								heap[++last_heap] = t_ad;
								index[t_ad] = last_heap;
								upheap(cost, last_heap, last_heap, heap, index);
							} else if (kmin < cost[t_ad]) {
								//update
								cost[t_ad] = kmin;
								if (index[t_ad] != 1 && cost[t_ad] < cost[heap[index[t_ad] / 2]])
									upheap(cost, last_heap, index[t_ad], heap, index);
								else
									downheap(cost, last_heap, index[t_ad], heap, index);
							}
						} else {
							//remove other triangles
							for (k = 0; k < etN; k++) {
								int etk = et[k];
								if (etk == current || visitT[etk] || index[etk] < 0)
									continue;

								cost[etk] =  - 1;
								upheap(cost, last_heap, index[etk], heap, index);
								index[etk] =  - 1;
								heap[1] = heap[last_heap--];
								index[heap[1]] = 1;
								downheap(cost, last_heap, 1, heap, index);
							}
						}
					} else {
						int t_ad;
						if (et[0] == current)
							t_ad = et[1];
						else
							t_ad = et[0];
						if (visitT[t_ad])
							continue;
						visitT[t_ad] = true;
						stack[++top] = t_ad;

						int *t1 = tris[current];
						int v1 = t1[j];
						int v2 = t1[(j + 1) % 3];
						int *t2 = tris[t_ad];

						bool flip = false;
						if (t2[0] == v1)
							flip = (t2[2] != v2);
						else if (t2[1] == v1)
							flip = (t2[0] != v2);
						else
							flip = (t2[1] != v2);

						if (flip) {
							int tmp = t2[0];
							t2[0] = t2[1];
							t2[1] = tmp;

							int *te2 = tri_to_edge[t_ad];
							tmp = te2[1];
							te2[1] = te2[2];
							te2[2] = tmp;
						}
					}
				}
			}
		}

		delete []heap;
		delete []index;
		delete []cost;

		delete []visitV;

		for (i = 0; i < edgeN; i++)
			delete []edge_to_tri[i];
		delete []edge_to_tri;
		delete []tri_to_edge;
		delete []edge_to_triN;

		int valid_triN = 0;
		for (i = 0; i < triN; i++) {
			if (!visitT[i])
				continue;

			int *f = tris[i];

			tris[valid_triN][0] = f[0];
			tris[valid_triN][1] = f[1];
			tris[valid_triN][2] = f[2];

			valid_triN++;
		}
		triN = mesh->faceN = valid_triN;
		delete []visitT;

		cout << valid_triN << "triangles are generated" << endl;

		int *table = new int[N];
		for (i = 0; i < N; i++)
			table[i] =  - 1;
		int id = 0;
		for (i = 0; i < triN; i++) {
			int *f = tris[i];
			for (j = 0; j < 3; j++)
				if (table[f[j]] < 0)
					table[f[j]] = id++;
		}
		float(*vertex_tmp)[3] = new float[id][3];
		for (i = 0; i < N; i++) {
			int j = table[i];
			if (j < 0)
				continue;
			vertex_tmp[j][0] = mesh->vertex[i][0];
			vertex_tmp[j][1] = mesh->vertex[i][1];
			vertex_tmp[j][2] = mesh->vertex[i][2];
		}
		delete []mesh->vertex;
		mesh->vertex = vertex_tmp;
		mesh->vertexN = N = id;

		for (i = 0; i < triN; i++) {
			int *f = tris[i];
			f[0] = table[f[0]];
			f[1] = table[f[1]];
			f[2] = table[f[2]];
		}
		delete []table;

		//mesh->fillSmallHoles(6);
		//mesh->fillSmallHoles(6);
	}

	void cleanBadTriangles() {
		cout << "Removing bad tringles" << endl;

		int i, j;

		int N = mesh->vertexN;	// 顶点数

		bool *decideT = new bool[triN];
		bool *decideV = new bool[N];
		int **link = new int *[N];	// 顶点 i 相邻的三角形链表
		int *linkN = new int[N];	// 顶点 i 相邻的三角形个数

		for (i = 0; i < N; i++) {
			link[i] = NULL;
			linkN[i] = 0;
			decideV[i] = false;
		}

		// 创建每一个顶点相连的三角形列表
		for (i = 0; i < triN; i++) {
			decideT[i] = false;
			for (j = 0; j < 3; j++) {
				int index = tris[i][j];

				int lN = linkN[index];
				int *l_tmp = new int[lN + 1];
				int *l = link[index];
				for (int k = 0; k < lN; k++)
					l_tmp[k] = l[k];
				l_tmp[lN] = i;
				if (lN != 0)
					delete []l;
				link[index] = l_tmp;
				linkN[index]++;
			}
		}

		int *visitID = new int[N];
		for (i = 0; i < N; i++)
			visitID[i] =  - 1;

		//Find seed 1-ring
		cout << "Finding a seed 1-ring disk" << endl;
		float min_error = 10000000000.0f;
		int seed =  - 1;// 取所有顶点中k(D)最小的顶点
		for (i = 0; i < N; i++) {
			if (linkN[i] == 0)
				continue;
			if (sort1rings(decideT, link[i], linkN[i], i, visitID)) {
				float e = measure1ringError(link[i], linkN[i], i);
				if (e < min_error && e > 0) {
					seed = i;
					min_error = e;    // 取所有顶点中k(D)最小的顶点
				}
			}
		}

		if (seed < 0) {
			cout << "Seed 1 -ring can not be found." << endl;
			cout << "Maybe, the input points contains too large noise." << endl;
			cout << "Now exiting, sorry." << endl;
			return ;
		}

		delete []visitID;
		visitID = new int[triN];
		for (i = 0; i < triN; i++)
			visitID[i] =  - 1;

		bool *validT = new bool[triN];
		for (i = 0; i < triN; i++)
			validT[i] = true;

		for (i = 0; i < triN; i++)
			decideT[i] = false;
		sort1rings(decideT, link[seed], linkN[seed], seed, visitID);

		//compute global orientation using priority Q
		cout << "Propergating 1-ring disks" << endl;
		float *cost = new float[N];

		int *heap = new int[N + 1];
		int *index = new int[N];
		int last_heap = 0;

		for (i = 0; i < N; i++) {
			cost[i] = 1000000000000.0f;
			index[i] =  - 1;
		}

		int *best, bestN;
		getBest1ring(best, bestN, cost[seed], decideT, validT, link[seed], linkN[seed], seed, visitID);
		delete best;
		//insert
		heap[++last_heap] = seed;
		index[seed] = last_heap;
		upheap(cost, last_heap, last_heap, heap, index);
		int counter = 0;
		while (last_heap > 0) {
			//if(counter++ == 10000)
			//  break;
			//remove top
			int current = heap[1];
			index[current] =  - 1;
			heap[1] = heap[last_heap--];
			index[heap[1]] = 1;
			downheap(cost, last_heap, 1, heap, index);


			int *lc = link[current];
			int lcN = linkN[current];
			if (!getBest1ring(best, bestN, cost[current], decideT, validT, lc, lcN, current, visitID)) {
				//delete lc;
				//link[current] = NULL;
				//linkN[current] = 0;
				continue;
			}

			//decide vertex and triangles
			for (j = 0; j < lcN; j++)
				decideT[lc[j]] = false;

			//decide vertex and triangles
			decideV[current] = true;
			for (j = 0; j < bestN; j++)
				decideT[best[j]] = true;

			//remove un-used triangles
			for (j = 0; j < lcN; j++)
				if (!decideT[lc[j]])
					validT[lc[j]] = false;

			//replace trinagle list
			delete link[current];
			link[current] = best;
			linkN[current] = bestN;

			//update or insert vertices
			for (j = 0; j < bestN; j++) {
				int *ti = tris[best[j]];
				int v;
				if (ti[0] == current)
					v = ti[1];
				else if (ti[1] == current)
					v = ti[2];
				else
					v = ti[0];

				if (decideV[v])
					continue;

				if (index[v] < 0) {//insert new vertex                          
					int *bv;
					int bvN;
					if (getBest1ring(bv, bvN, cost[v], decideT, validT, link[v], linkN[v], v, visitID)) {
						heap[++last_heap] = v;
						index[v] = last_heap;
						upheap(cost, last_heap, last_heap, heap, index);
						delete bv;
					}
				}
			}
		}

		delete []heap;
		delete []index;
		delete []cost;          
		delete []visitID;          
		delete []validT;

		for (i = 0; i < N; i++)
			if (linkN[i] != 0)
				delete []link[i];
		delete []link;
		delete []linkN;

		cout << "Removing non-manifold parts" << endl;
		while (!removeNonManifoldV2(decideT))
			;

		delete []decideV;
		while (true) {
			int valid_triN = 0;
			for (i = 0; i < triN; i++) {
				if (!decideT[i])
					continue;

				int *f = tris[i];

				tris[valid_triN][0] = f[0];
				tris[valid_triN][1] = f[1];
				tris[valid_triN][2] = f[2];

				valid_triN++;
			}
			triN = mesh->faceN = valid_triN;
			delete []decideT;

			cout << valid_triN << "triangles are generated" << endl;

			int *table = new int[N];
			for (i = 0; i < N; i++)
				table[i] =  - 1;
			int id = 0;
			for (i = 0; i < triN; i++) {
				int *f = tris[i];
				for (j = 0; j < 3; j++)
					if (table[f[j]] < 0)
						table[f[j]] = id++;
			}
			float(*vertex_tmp)[3] = new float[id][3];
			for (i = 0; i < N; i++) {
				int j = table[i];
				if (j < 0)
					continue;
				vertex_tmp[j][0] = mesh->vertex[i][0];
				vertex_tmp[j][1] = mesh->vertex[i][1];
				vertex_tmp[j][2] = mesh->vertex[i][2];
			}
			delete []mesh->vertex;
			mesh->vertex = vertex_tmp;
			mesh->vertexN = N = id;

			for (i = 0; i < triN; i++) {
				int *f = tris[i];
				f[0] = table[f[0]];
				f[1] = table[f[1]];
				f[2] = table[f[2]];
			}
			delete []table;

			//cout << "Filling small holes" << endl;
			mesh->fillSmallHolesMinA(15);

			tris = mesh->face;
			triN = mesh->faceN;

			decideT = new bool[triN];
			for (i = 0; i < triN; i++)
				decideT[i] = true;

			bool again = false;
			while (!removeNonManifoldV2(decideT))
				again = true;
			//again = false;
			if (!again) {
				delete []decideT;
				break;
			}
		}
	}
	//+计算三个球的两个交点p1,p2(返回是否相交)+
	bool intersectionOf3Balls(float p1[3], float p2[3], float c1[3], float c2[3], float c3[3], float r1, float r2, float r3) {
		float vx1 = c2[0] - c1[0];
		float vy1 = c2[1] - c1[1];
		float vz1 = c2[2] - c1[2];
		float d2 = vx1 * vx1 + vy1 * vy1 + vz1 * vz1;
		float d = (float)sqrt(d2);
		vx1 /= d;
		vy1 /= d;
		vz1 /= d;
		if (r1 + r2 - d <= 0 || d <= fabs(r1 - r2))
			return false;

		float w1 = d2 + r2 * r2 - r1 * r1;
		float w2 = d2 + r1 * r1 - r2 * r2;
		float w = 2 * d2;
		w1 /= w;
		w2 /= w;

		float cx1 = w1 * c1[0] + w2 * c2[0];
		float cy1 = w1 * c1[1] + w2 * c2[1];
		float cz1 = w1 * c1[2] + w2 * c2[2];

		float d1 = d * w2;
		float R1 = (float)sqrt(r1 *r1 - d1 * d1);

		float dot = vx1 *(c3[0] - cx1) + vy1 *(c3[1] - cy1) + vz1 *(c3[2] - cz1);
		if (fabs(dot) >= r3)
			return false;

		float cx2 = c3[0] - dot * vx1;
		float cy2 = c3[1] - dot * vy1;
		float cz2 = c3[2] - dot * vz1;
		float R2 = (float)sqrt(r3 *r3 - dot * dot);

		float vx = cx2 - cx1;
		float vy = cy2 - cy1;
		float vz = cz2 - cz1;
		d2 = vx * vx + vy * vy + vz * vz;
		d = (float)sqrt(d2);

		if (R1 + R2 - d <= 0 || d <= fabs(R1 - R2))
			return false;

		w1 = d2 + R2 * R2 - R1 * R1;
		w2 = d2 + R1 * R1 - R2 * R2;
		w = 2 * d2;
		w1 /= w;
		w2 /= w;

		float cx = w1 * cx1 + w2 * cx2;
		float cy = w1 * cy1 + w2 * cy2;
		float cz = w1 * cz1 + w2 * cz2;

		float nx = vy * vz1 - vz * vy1;
		float ny = vz * vx1 - vx * vz1;
		float nz = vx * vy1 - vy * vx1;
		float len = (float)sqrt(nx *nx + ny * ny + nz * nz);
		if (len == 0)
			return false;
		nx /= len;
		ny /= len;
		nz /= len;

		d1 = d * w2;
		float h = (float)sqrt(R1 *R1 - d1 * d1);

		p1[0] = cx + h * nx;
		p1[1] = cy + h * ny;
		p1[2] = cz + h * nz;

		p2[0] = cx - h * nx;
		p2[1] = cy - h * ny;
		p2[2] = cz - h * nz;

		return true;
	}

	bool removeNonManifoldV(bool *decideT) {
		int i, j, k;

		bool isOK = true;

		int N = mesh->vertexN;
		int *ver_to_triN = new int[N];
		int **ver_to_tri = new int *[N];

		while (true) {
			for (i = 0; i < N; i++)
				ver_to_triN[i] = 0;

			for (i = 0; i < triN; i++) {
				if (!decideT[i])
					continue;
				int *t = tris[i];
				ver_to_triN[t[0]]++;
				ver_to_triN[t[1]]++;
				ver_to_triN[t[2]]++;
			}

			bool isChanged = false;
			for (i = 0; i < triN; i++) {
				if (!decideT[i])
					continue;
				int *t = tris[i];
				for (j = 0; j < 3; j++) {
					if (ver_to_triN[t[j]] == 1) {
						decideT[i] = false;
						isChanged = true;
					}
				}
			}

			if (!isChanged)
				break;
			else
				isOK = false;
		}

		for (i = 0; i < N; i++) {
			if (ver_to_triN[i] != 0) {
				ver_to_tri[i] = new int[ver_to_triN[i]];
				ver_to_triN[i] = 0;
			}
		}

		for (i = 0; i < triN; i++) {
			if (!decideT[i])
				continue;
			int *t = tris[i];
			for (j = 0; j < 3; j++) {
				k = t[j];
				ver_to_tri[k][ver_to_triN[k]++] = i;
			}
		}

		for (i = 0; i < N; i++) {
			int vtN = ver_to_triN[i];
			if (vtN < 2)
				continue;
			int *vt = ver_to_tri[i];
			int start = 0;
			for (j = 0; j < vtN; j++) {
				int *t = tris[vt[j]];
				int v;
				if (t[0] == i)
					v = t[1];
				else if (t[1] == i)
					v = t[2];
				else
					v = t[0];
				for (k = 0; k < vtN; k++) {
					if (k == j)
						continue;
					int *t1 = tris[vt[k]];
					if (t1[0] == v || t1[1] == v || t1[2] == v)
						break;
				}
				if (k == vtN) {
					start = j;
					break;
				}
			}

			int *t = tris[vt[start]];
			int startV;
			if (t[0] == i)
				startV = t[1];
			else if (t[1] == i)
				startV = t[2];
			else
				startV = t[0];
			bool remove = false;

			for (j = 0; j < vtN - 1; j++) {
				t = tris[vt[(j + start) % vtN]];
				int v;
				if (t[0] == i)
					v = t[2];
				else if (t[1] == i)
					v = t[0];
				else
					v = t[1];

				if (v == startV) {
					remove = true;
					break;
				}

				int index =  - 1;
				for (k = 0; k < vtN; k++) {
					if (k == j)
						continue;
					int *t1 = tris[vt[(k + start) % vtN]];
					int v1;
					if (t1[0] == i)
						v1 = t1[1];
					else if (t1[1] == i)
						v1 = t1[2];
					else
						v1 = t1[0];
					if (v1 == v) {
						if (index < 0)
							index = k;
						else {
							remove = true;
							break;
						}
					}
				}
				if (index < 0 || remove) {
					remove = true;
					break;
				}
				int tmp = vt[(j + 1+start) % vtN];
				vt[(j + 1+start) % vtN] = vt[(index + start) % vtN];
				vt[(index + start) % vtN] = tmp;
			}

			if (remove) {
				for (j = 0; j < vtN; j++)
					decideT[vt[j]] = false;
				isOK = false;
			}

			/*
			int* v = new int[vtN*2];
			int vN = 0;

			for(j=0; j<vtN; j++){
			int* t = tris[vt[j]];
			int v1, v2;
			if(t[0] == i){
			v1 = t[1];
			v2 = t[2];
			}
			else if(t[1] == i){
			v1 = t[2];
			v2 = t[0];
			}
			else{
			v1 = t[0];
			v2 = t[1];
			}

			for(k=0; k<vN; k++)
			if(v[k] == v1)
			break;
			if(k == vN)
			v[vN++] = v1;

			for(k=0; k<vN; k++)
			if(v[k] == v2)
			break;
			if(k == vN)
			v[vN++] = v2;
			}



			if(start < 0){
			if(vN != vtN){
			for(j=0; j<vtN; j++)
			decideT[vt[j]] = false;
			isOK = false;
			}
			}
			else{
			if(vN != vtN+1){
			for(j=0; j<vtN; j++)
			decideT[vt[j]] = false;
			isOK = false;
			}
			}
			delete[] v;*/
		}
		/*
		for(i=0; i<N; i++){
		int vtN = ver_to_triN[i];
		if(vtN < 2)
		continue;
		int* vt = ver_to_tri[i];
		int* v = new int[vtN*2];
		int vN = 0;

		for(j=0; j<vtN; j++){
		int* t = tris[vt[j]];
		int v1, v2;
		if(t[0] == i){
		v1 = t[1];
		v2 = t[2];
		}
		else if(t[1] == i){
		v1 = t[2];
		v2 = t[0];
		}
		else{
		v1 = t[0];
		v2 = t[1];
		}

		for(k=0; k<vN; k++)
		if(v[k] == v1)
		break;
		if(k == vN)
		v[vN++] = v1;

		for(k=0; k<vN; k++)
		if(v[k] == v2)
		break;
		if(k == vN)
		v[vN++] = v2;
		}

		if(vN > vtN+1){
		for(j=0; j<vtN; j++)
		decideT[vt[j]] = false;
		isOK = false;
		}
		delete[] v;
		}
		*/
		for (i = 0; i < N; i++)
			if (ver_to_triN[i] != 0)
				delete []ver_to_tri[i];
		delete []ver_to_tri;
		delete []ver_to_triN;

		return isOK;
	}

	void removeNonManifoldE(bool *decideT) {
		int **edge_to_tri;
		int *edge_to_triN;
		int(*tri_to_edge)[3];

		int i, j, k;

		//make edge data
		cout << "Making the edge data" << endl;
		int N = mesh->vertexN;
		int *ver_to_triN = new int[N];
		for (i = 0; i < N; i++)
			ver_to_triN[i] = 0;

		for (i = 0; i < triN; i++) {
			if (!decideT[i])
				continue;
			int *t = tris[i];
			ver_to_triN[t[0]]++;
			ver_to_triN[t[1]]++;
			ver_to_triN[t[2]]++;
		}

		int **ver_to_tri = new int *[N];
		for (i = 0; i < N; i++) {
			if (ver_to_triN[i] != 0) {
				ver_to_tri[i] = new int[ver_to_triN[i]];
				ver_to_triN[i] = 0;
			}
		}

		for (i = 0; i < triN; i++) {
			if (!decideT[i])
				continue;
			int *t = tris[i];
			for (j = 0; j < 3; j++) {
				k = t[j];
				ver_to_tri[k][ver_to_triN[k]++] = i;
			}
		}

		//count #edge
		int edgeN = 0;
		for (i = 0; i < N; i++) {
			int vtN = ver_to_triN[i];
			int *vt = ver_to_tri[i];
			int *v = new int[2 *vtN];
			int vN = 0;
			for (j = 0; j < vtN; j++) {
				int *t = tris[vt[j]];
				int v1, v2;
				if (t[0] == i) {
					v1 = t[1];
					v2 = t[2];
				} else if (t[1] == i) {
					v1 = t[2];
					v2 = t[0];
				} else {
					v1 = t[0];
					v2 = t[1];
				}

				if (v1 > i) {
					bool flag = true;
					for (k = 0; k < vN; k++) {
						if (v1 == v[k]) {
							flag = false;
							break;
						}
					}
					if (flag)
						v[vN++] = v1;
				}

				if (v2 > i) {
					bool flag = true;
					for (k = 0; k < vN; k++) {
						if (v2 == v[k]) {
							flag = false;
							break;
						}
					}
					if (flag)
						v[vN++] = v2;
				}
			}
			if (vtN != 0)
				delete []v;
			edgeN += vN;
		}

		edge_to_triN = new int[edgeN];
		edge_to_tri = new int *[edgeN];
		tri_to_edge = new int[triN][3];
		for (i = 0; i < edgeN; i++)
			edge_to_triN[i] = 0;

		edgeN = 0;
		for (i = 0; i < N; i++) {
			int vtN = ver_to_triN[i];
			int *vt = ver_to_tri[i];
			int *v = new int[2 *vtN];
			int vN = 0;
			for (j = 0; j < vtN; j++) {
				int *t = tris[vt[j]];
				int v1, v2, iv;
				if (t[0] == i) {
					iv = 0;
					v1 = t[1];
					v2 = t[2];
				} else if (t[1] == i) {
					iv = 1;
					v1 = t[2];
					v2 = t[0];
				} else {
					iv = 2;
					v1 = t[0];
					v2 = t[1];
				}

				if (v1 > i) {
					for (k = 0; k < vN; k++)
						if (v1 == v[k])
							break;
					int ie = edgeN - vN + k;
					if (k == vN) {
						edgeN++;
						v[vN++] = v1;
					}
					tri_to_edge[vt[j]][iv] = ie;

					int *et = edge_to_tri[ie];
					int etN = edge_to_triN[ie];
					int *tmp = new int[etN + 1];
					for (k = 0; k < etN; k++)
						tmp[k] = et[k];
					tmp[k] = vt[j];
					if (etN != 0)
						delete []et;
					edge_to_tri[ie] = tmp;
					edge_to_triN[ie]++;
				}

				if (v2 > i) {
					for (k = 0; k < vN; k++)
						if (v2 == v[k])
							break;
					int ie = edgeN - vN + k;
					if (k == vN) {
						edgeN++;
						v[vN++] = v2;
					}
					tri_to_edge[vt[j]][(iv + 2) % 3] = ie;

					int *et = edge_to_tri[ie];
					int etN = edge_to_triN[ie];
					int *tmp = new int[etN + 1];
					for (k = 0; k < etN; k++)
						tmp[k] = et[k];
					tmp[k] = vt[j];
					if (etN != 0)
						delete []et;
					edge_to_tri[ie] = tmp;
					edge_to_triN[ie]++;
				}
			}
			if (vtN != 0)
				delete []v;
		}

		for (i = 0; i < N; i++)
			if (ver_to_triN[i] != 0)
				delete []ver_to_tri[i];
		delete []ver_to_tri;
		delete []ver_to_triN;

		for (i = 0; i < edgeN; i++) {
			int eN = edge_to_triN[i];
			int *et = edge_to_tri[i];
			if (eN > 2) {
				for (j = 0; j < eN; j++) {
					decideT[et[j]] = false;
				}
			} else if (eN == 2) {
				int i1 = et[0];
				int *te1 = tri_to_edge[i1];
				int e1;
				if (te1[0] == i)
					e1 = 0;
				else if (te1[1] == i)
					e1 = 1;
				else
					e1 = 2;
				int v11 = tris[i1][e1];
				int v12 = tris[i1][(e1+1) % 3];

				int i2 = et[1];
				int *te2 = tri_to_edge[i2];
				int e2;
				if (te2[0] == i)
					e2 = 0;
				else if (te2[1] == i)
					e2 = 1;
				else
					e2 = 2;
				int v21 = tris[i2][e2];
				int v22 = tris[i2][(e2+1) % 3];

				if (v11 == v21) {
					decideT[i1] = false;
					decideT[i2] = false;
				}
			}
		}

		/*
		for(i=0; i<triN; i++){
		if(!decideT[i])
		continue;

		int* te = tri_to_edge[i];
		int bN = 0;
		for(j=0; j<3; j++)
		if(edge_to_triN[te[j]] == 1)
		bN++;
		//cout << bN << endl;
		if(bN > 1){
		decideT[i] = false;
		cout << "2 BOUNDAY" << endl;
		}
		}*/

		for (i = 0; i < edgeN; i++)
			delete []edge_to_tri[i];
		delete []edge_to_tri;
		delete []tri_to_edge;
		delete []edge_to_triN;
	}

	bool getBest1ring(int * &blist, int &blistN, float &error, bool *decideT, bool *validT, int *list, int N, int current, int *visitID) {
		int(*tris)[3] = mesh->face;

		blist = NULL;
		blistN = 0;

		int *stack = new int[N + 1];
		//TriList** stack = new TriList*[N+1];
		int top =  - 1;          
		int start =  - 1;

		int i;
		for (i = 0; i < N; i++) {
			if (decideT[list[i]] && validT[list[i]]) {
				start = i;
				break;
			}
		}
		if (start < 0) {
			delete []stack;
			return false;
		}

		int startV;
		int *ti = tris[list[start]];
		if (ti[0] == current)
			startV = ti[2];
		else if (ti[1] == current)
			startV = ti[0];
		else
			startV = ti[1];

		stack[++top] = start;
		stack[++top] =  - 1;
		while (top > 0) {
			int v;
			int t_id = list[stack[top - 1]];
			int *ti = tris[t_id];
			if (ti[0] == current)
				v = ti[1];
			else if (ti[1] == current)
				v = ti[2];
			else
				v = ti[0];

			if (v == startV) {
				int *list_tmp = new int[top];
				for (i = 0; i < top; i++)
					list_tmp[i] = list[stack[i]];

				bool flag = true;
				for (i = 0; i < N; i++) {
					if (!decideT[list[i]])
						continue;
					bool flag1 = false;
					for (int j = 0; j < top; j++) {
						if (list_tmp[j] == list[i]) {
							flag1 = true;
							break;
						}
					}
					if (!flag1) {
						flag = false;
						break;
					}
				}

				if (flag) {
					float error_tmp = measure1ringError(list_tmp, top, current);
					if (blistN == 0 || error_tmp < error) {
						error = error_tmp;
						delete blist;
						blist = list_tmp;
						blistN = top;
					} else
						delete list_tmp;
				}else delete list_tmp;
				top--;
				continue;
			}

			visitID[t_id] = current;

			bool flag = false;
			if (stack[top] < 0)
				i = 0;
			else
				i = stack[top] + 1;

			top--;
			for (; i < N; i++) {
				if (i == stack[top] || visitID[list[i]] == current || !validT[list[i]])
					continue;

				ti = tris[list[i]];
				int ver_id;
				if (ti[0] == v)
					ver_id = 0;
				else if (ti[1] == v)
					ver_id = 1;
				else if (ti[2] == v)
					ver_id = 2;
				else
					continue;

				if (decideT[list[i]]) {
					int i1 = (ver_id + 1) % 3;
					if (ti[i1] != current)
						continue;
				}

				flag = true;
				stack[++top] = i;
				int i1 = (ver_id + 1) % 3;
				if (ti[i1] != current) {
					int i2 = (ver_id + 2) % 3;
					int tmp = ti[i1];
					ti[i1] = ti[i2];
					ti[i2] = tmp;
				}
				break;
			}
			if (flag)
				stack[++top] =  - 1;
			else
				visitID[t_id] =  - 1;
		}
		delete []stack;

		if (blistN < 3)
			return false;

		for (i = 0; i < blistN; i++) {
			int j = blist[i];

			if (i + 1 == blistN)
				continue;

			ti = tris[j];
			int v;
			if (ti[0] == current)
				v = ti[1];
			else if (ti[1] == current)
				v = ti[2];
			else
				v = ti[0];

			ti = tris[blist[i + 1]];
			int ver_id;
			if (ti[0] == v)
				ver_id = 0;
			else if (ti[1] == v)
				ver_id = 1;
			else
				ver_id = 2;

			int i1 = (ver_id + 1) % 3;
			if (ti[i1] != current) {
				int i2 = (ver_id + 2) % 3;
				int tmp = ti[i1];
				ti[i1] = ti[i2];
				ti[i2] = tmp;
			}
		}

		return true;
	}

	// @计算k(D)@ list 为相邻三角形编号，N为三角形个数，current为中心点编号
	float measure1ringError(int *list, int N, int current) {
		float(*verts)[3] = mesh->vertex;
		int(*tris)[3] = mesh->face;

		if (N < 3)
			return 100000.0f;
		double(*normal)[3] = new double[N][3];    // 三角形面法矢
		double *area = new double[N]; // 三角形面积（边长）
		int i = 0;
		for (i = 0; i < N; i++) {
			int *ti = tris[list[i]];
			float *p0,  *p1,  *p2;    // p0为中心点
			if (ti[0] == current) {
				p0 = verts[ti[0]];
				p1 = verts[ti[1]];
				p2 = verts[ti[2]];
			} else if (ti[1] == current) {
				p0 = verts[ti[1]];
				p1 = verts[ti[2]];
				p2 = verts[ti[0]];
			} else {
				p0 = verts[ti[2]];
				p1 = verts[ti[0]];
				p2 = verts[ti[1]];
			}

			float v1[] =  { p1[0] - p0[0], p1[1] - p0[1], p1[2] - p0[2] };    // 边 OP1
			float v2[] =  { p2[0] - p0[0], p2[1] - p0[1], p2[2] - p0[2] };    // 边 OP2

			double *n = normal[i];
			n[0] = v1[1] *v2[2] - v1[2] *v2[1];
			n[1] = v1[2] *v2[0] - v1[0] *v2[2];
			n[2] = v1[0] *v2[1] - v1[1] *v2[0];
			double len = sqrt(n[0] *n[0] + n[1] *n[1] + n[2] *n[2]);
			if ((float)len != 0) {
				n[0] /= len;
				n[1] /= len;
				n[2] /= len;
			}
			area[i] = sqrt(v1[0] *v1[0] + v1[1] *v1[1] + v1[2] *v1[2]);
		}
		double error = 0;
		for (i = 0; i < N; i++) {
			double *n1 = normal[i];
			double *n2 = normal[(i + 1) % N];
			double dot = n1[0] *n2[0] + n1[1] *n2[1] + n1[2] *n2[2];
			if (dot <  - 1.0)
				dot =  - 1.0;
			else if (dot > 1.0)
				dot = 1.0;

			float a = (float)acos(dot);
			if (a > error)
				error = a;    // 取角度最大值
			//               continue;
			//               error += (area[i] /* + area[(i+1)%N]*/) *acos(dot);// 没有执行
		}
		delete []normal;
		delete []area;
		return (float)error;
	}

	bool getHalfDisk(int * &half, int &halfN, bool *validT, int *list, int N, int current, int *visitID) {
		int i, j;
		int start =  - 1;
		for (i = 0; i < N; i++) {
			if (!validT[list[i]])
				continue;
			int v;
			int *t = tris[list[i]];
			if (t[0] == current)
				v = t[1];
			else if (t[1] == current)
				v = t[2];
			else
				v = t[0];
			bool flag = false;
			for (j = 0; j < N; j++) {
				if (i == j || !validT[list[j]])
					continue;
				int *tj = tris[list[j]];
				if (tj[0] == v || tj[1] == v || tj[2] == v) {
					flag = true;
					break;
				}
			}
			if (!flag) {
				start = list[i];
				break;
			}
		}

		if (start < 0)
			return false;

		half = new int[N];
		half[0] = start;
		halfN = 1;
		while (true) {
			int i = half[halfN - 1];
			int *t = tris[i];
			int v;
			if (t[0] == current)
				v = t[2];
			else if (t[1] == current)
				v = t[0];
			else
				v = t[1];
			bool flag = false;
			for (j = 0; j < N; j++) {
				if (i == list[j] || !validT[list[j]])
					continue;
				int *tj = tris[list[j]];
				if (tj[0] == v || tj[1] == v || tj[2] == v) {
					half[halfN++] = list[j];
					flag = true;
					break;
				}
			}
			if (!flag || halfN == N)
				break;
		}
		return true;
	}
	// 将点 current 周围的三角形按照顺时针或逆时针排列
	bool sort1rings(bool *decideT, int *list, int N, int current, int *visitID) {
		if (N < 3)
			return false;

		int i;
		int(*tris)[3] = mesh->face;

		int start = list[0];
		for (i = 1; i < N; i++)
			if (decideT[list[i]]) {
				start = list[i];
				break;
			}

			int end = start;
			do {
				int *ti = tris[end];
				int v;
				if (ti[0] == current)
					v = ti[1];
				else if (ti[1] == current)
					v = ti[2];
				else
					v = ti[0];

				if (visitID[v] == current)
					return false;
				visitID[v] = current;

				bool flag = false;
				int next;
				for (i = 0; i < N; i++) {
					int s = list[i];
					if (s == end)
						continue;
					ti = tris[s];
					int ver_id;
					if (ti[0] == v)
						ver_id = 0;
					else if (ti[1] == v)
						ver_id = 1;
					else if (ti[2] == v)
						ver_id = 2;
					else
						continue;
					if (flag)
						return false;
					flag = true;
					next = s;
					int i1 = (ver_id + 1) % 3;
					if (ti[i1] != current) {
						int i2 = (ver_id + 2) % 3;
						int tmp = ti[i1];
						ti[i1] = ti[i2];
						ti[i2] = tmp;
					}
				}
				if (!flag)
					return false;
				end = next;
			} while (start != end);

			for (i = 0; i < N; i++)
				decideT[list[i]] = true;

			return true;
	}
	// 计算 二面角 K(D)
	float curvature(int t1i, int e1, int t2i, int *te) {
		int *t1 = tris[t1i];
		int v1 = t1[e1];
		int v2 = t1[(e1+1) % 3];
		int *t2 = tris[t2i];

		bool flip = false;
		if (t2[0] == v1)
			flip = (t2[2] != v2);
		else if (t2[1] == v1)
			flip = (t2[0] != v2);
		else
			flip = (t2[1] != v2);

		if (flip) {
			int tmp = t2[0];
			t2[0] = t2[1];
			t2[1] = tmp;

			tmp = te[1];
			te[1] = te[2];
			te[2] = tmp;
		}

		float n1[3], n2[3];
		mesh->faceNormal(n1, t1i);
		mesh->faceNormal(n2, t2i);
		double dot = n1[0] *n2[0] + n1[1] *n2[1] + n1[2] *n2[2];
		if (dot > 1.0)
			dot = 1.0;
		else if (dot <  - 1.0)
			dot =  - 1.0;

		float *p0 = mesh->vertex[v1];
		float *p1 = mesh->vertex[v2];
		float vx = p0[0] - p1[0];
		float vy = p0[1] - p1[1];
		float vz = p0[2] - p1[2];

		return (float)(acos(dot) *sqrt(vx *vx + vy * vy + vz * vz));
	}

	inline void upheap(float *a, int N, int k, int *p, int *q) {
		int v;
		v = p[k];
		while (k > 1 && a[p[k / 2]] >= a[v]) {
			p[k] = p[k / 2];
			q[p[k / 2]] = k;
			k = k / 2;
		}
		p[k] = v;
		q[v] = k;
	}

	inline void downheap(float *a, int N, int k, int *p, int *q) {
		int j, v;
		v = p[k];
		while (k <= N / 2) {
			j = k + k;
			if (j < N && a[p[j]] > a[p[j + 1]])
				j++;
			if (a[v] <= a[p[j]])
				break;
			p[k] = p[j];
			q[p[j]] = k;
			k = j;
		}
		p[k] = v;
		q[v] = k;
	}

	bool removeNonManifoldV2(bool *decideT) {
		int i, j, k;

		bool isOK = true;

		int N = mesh->vertexN;
		int *ver_to_triN = new int[N];
		int **ver_to_tri = new int *[N];

		while (true) {
			for (i = 0; i < N; i++)
				ver_to_triN[i] = 0;

			for (i = 0; i < triN; i++) {
				if (!decideT[i])
					continue;
				int *t = tris[i];
				ver_to_triN[t[0]]++;
				ver_to_triN[t[1]]++;
				ver_to_triN[t[2]]++;
			}

			bool isChanged = false;
			for (i = 0; i < triN; i++) {
				if (!decideT[i])
					continue;
				int *t = tris[i];
				for (j = 0; j < 3; j++) {
					if (ver_to_triN[t[j]] == 1) {
						decideT[i] = false;
						isChanged = true;
					}
				}
			}

			if (!isChanged)
				break;
			else
				isOK = false;
		}

		for (i = 0; i < N; i++) {
			if (ver_to_triN[i] != 0) {
				ver_to_tri[i] = new int[ver_to_triN[i]];
				ver_to_triN[i] = 0;
			}
		}

		for (i = 0; i < triN; i++) {
			if (!decideT[i])
				continue;
			int *t = tris[i];
			for (j = 0; j < 3; j++) {
				k = t[j];
				ver_to_tri[k][ver_to_triN[k]++] = i;
			}
		}

		for (i = 0; i < N; i++) {
			int vtN = ver_to_triN[i];
			if (vtN < 2)
				continue;
			int *vt = ver_to_tri[i];
			int start = 0;
			for (j = 0; j < vtN; j++) {
				int *t = tris[vt[j]];
				int v;
				if (t[0] == i)
					v = t[1];
				else if (t[1] == i)
					v = t[2];
				else
					v = t[0];
				for (k = 0; k < vtN; k++) {
					if (k == j)
						continue;
					int *t1 = tris[vt[k]];
					if (t1[0] == v || t1[1] == v || t1[2] == v)
						break;
				}
				if (k == vtN) {
					start = j;
					break;
				}
			}

			int *t = tris[vt[start]];
			int startV;
			if (t[0] == i)
				startV = t[1];
			else if (t[1] == i)
				startV = t[2];
			else
				startV = t[0];
			int remove =  - 1;

			for (j = 0; j < vtN - 1; j++) {
				t = tris[vt[(j + start) % vtN]];
				int v;
				if (t[0] == i)
					v = t[2];
				else if (t[1] == i)
					v = t[0];
				else
					v = t[1];

				if (v == startV) {
					remove = j + 1;
					break;
				}

				int index =  - 1;
				for (k = 0; k < vtN; k++) {
					if (k == j)
						continue;
					int *t1 = tris[vt[(k + start) % vtN]];
					int v1;
					if (t1[0] == i)
						v1 = t1[1];
					else if (t1[1] == i)
						v1 = t1[2];
					else
						v1 = t1[0];
					if (v1 == v) {
						if (index < 0)
							index = k;
						else {
							remove = j;
							break;
						}
					}
				}
				if (index < 0)
					remove = j + 1;

				if (remove >= 0)
					break;

				int tmp = vt[(j + 1+start) % vtN];
				vt[(j + 1+start) % vtN] = vt[(index + start) % vtN];
				vt[(index + start) % vtN] = tmp;
			}

			if (remove >= 0) {
				for (j = remove; j < vtN; j++)
					decideT[vt[(j + start) % vtN]] = false;
				isOK = false;
			}
		}

		for (i = 0; i < N; i++)
			if (ver_to_triN[i] != 0)
				delete []ver_to_tri[i];
		delete []ver_to_tri;
		delete []ver_to_triN;

		return isOK;
	}

};




#endif
