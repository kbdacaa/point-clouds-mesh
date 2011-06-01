#ifndef MESH_H
#define MESH_H

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include <stdio.h>

#define PI 3.1415926535897932384626433832795

class TriList{
public:
	int index;    //+三角面片的索引+
	TriList* next;

	TriList(){
		index = -1;
		next = NULL;
	}

	TriList(int i){
		index = i;
		next = NULL;
	}

	~TriList(){
		if(next != NULL)
			delete next;
	}

	TriList* add(int i){
		if(index < 0){
			index = i;
			return this;
		}
		else{
			TriList* head = new TriList;
			head->index = i;
			head->next = this;
			return head;
		}
	}
};

class TriList2{
public:
	int i0, i1, i2;   //+三角面片的三个顶点序号+
	TriList2* next;

	TriList2(int i0, int i1, int i2){
		this->i0 = i0;
		this->i1 = i1;
		this->i2 = i2;
		next = NULL;
	}

	~TriList2(){
		if(next != NULL)
			delete next;
	}

	TriList2* add(int i0, int i1, int i2){
		TriList2* head = new TriList2(i0, i1, i2);
		head->next = this;
		return head;
	}
};

class BallMesh{
public:
	int vertexN;  //+顶点个数+
	int faceN;        //+三角面个数+

	float (*vertex)[3];   //+顶点数组(存储点坐标)+
	int (*face)[3];           //+面数组(存储3个顶点序号)+

	float (*normal_f)[3]; //+面的法矢+
	float (*normal_v)[3]; //+顶点法矢+

	BallMesh(int vN, int fN){
		setSize(vN, fN);

		normal_f = NULL;
		normal_v = NULL;
	}

	~BallMesh(){
		delete[] vertex;
		delete[] face;

		if(normal_f != NULL)
			delete[] normal_f;
		if(normal_v != NULL)
			delete[] normal_v;
	}
	//+设置点和面的大小+
	void setSize(int vN, int fN){
		vertexN = vN;
		faceN = fN;
		vertex = new float[vN][3];
		face = new int[fN][3];
	}
	//+设置顶点值+
	void setVertex(int i, float x, float y, float z){
		vertex[i][0] = x;
		vertex[i][1] = y;
		vertex[i][2] = z;
	}
	//+设置一个面的三个顶点索引+
	void setFace(int i, int i0, int i1, int i2){
		face[i][0] = i0;
		face[i][1] = i1;
		face[i][2] = i2;
	}
	//+法矢反向+
	void flipNormal(){
		int i;

		for(i=0; i<vertexN; i++){
			float* n = normal_v[i];
			n[0] = -n[0];
			n[1] = -n[1];
			n[2] = -n[2];
		}

		for(i=0; i<faceN; i++){
			float* n = normal_f[i];
			n[0] = -n[0];
			n[1] = -n[1];
			n[2] = -n[2];
		}
	}
	//+计算顶点法矢和面法矢+
	void computeNormal(){
		if(normal_f != NULL)
			delete[] normal_f;
		if(normal_v != NULL)
			delete[] normal_v;

		normal_f = new float[faceN][3];
		normal_v = new float[vertexN][3];

		int i, j;
		for(i=0; i<vertexN; i++)
			normal_v[i][0] = normal_v[i][1] = normal_v[i][2] = 0;

		for(i=0; i<faceN; i++){
			int* f = face[i];
			//+取面所在的顶点+
			float* p0 = vertex[f[0]];
			float* p1 = vertex[f[1]];
			float* p2 = vertex[f[2]];

			float v1x = p1[0] - p0[0];
			float v1y = p1[1] - p0[1];
			float v1z = p1[2] - p0[2];

			float v2x = p2[0] - p0[0];
			float v2y = p2[1] - p0[1];
			float v2z = p2[2] - p0[2];

			float *n = normal_f[i]; //+n=p1p0*p2p0+
			n[0] = v1y*v2z - v1z*v2y;//+面的法矢计算方法+
			n[1] = v1z*v2x - v1x*v2z;
			n[2] = v1x*v2y - v1y*v2x;
			//+将顶点法矢取为相邻面的法矢总和+
			for(j=0; j<3; j++){
				float *nv;
				nv = normal_v[f[j]];
				nv[0] += n[0];
				nv[1] += n[1];
				nv[2] += n[2];
			}
		}
		//+取顶点的单位法矢+
		for(i=0; i<vertexN; i++){
			float* n = normal_v[i];
			float l = 1.0f/(float)sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
			n[0] *= l;
			n[1] *= l;
			n[2] *= l;
		}
		//+取面的单位法矢+
		for(i=0; i<faceN; i++){
			float* n = normal_f[i];
			float l = 1.0f/(float)sqrt(n[0]*n[0] + n[1]*n[1] + n[2]*n[2]);
			n[0] *= l;
			n[1] *= l;
			n[2] *= l;
		}
	}
	//+由面所在的三个顶点计算面的法矢+
	void faceNormal(float n[3], int i){
		int* f = face[i];

		float* p0 = vertex[f[0]];
		float* p1 = vertex[f[1]];
		float* p2 = vertex[f[2]];

		float v1x = p1[0] - p0[0];
		float v1y = p1[1] - p0[1];
		float v1z = p1[2] - p0[2];

		float v2x = p2[0] - p0[0];
		float v2y = p2[1] - p0[1];
		float v2z = p2[2] - p0[2];

		double nx = v1y*v2z - v1z*v2y;
		double ny = v1z*v2x - v1x*v2z;
		double nz = v1x*v2y - v1y*v2x;

		double len = sqrt(nx*nx + ny*ny + nz*nz);
		if((float)len != 0){
			len = 1.0/len;
			n[0] = (float)(nx/len);
			n[1] = (float)(ny/len);
			n[2] = (float)(nz/len);
		}
		else
			n[0] = n[1] = n[2] = 0;
	}

	void fillSmallHoles2(int T){
		int i, j, k;
		int *degree = new int[vertexN];
		for(i=0; i<vertexN; i++)
			degree[i] = 0;
		//+计算每一个顶点的相邻面的个数+
		for(i=0; i<faceN; i++){
			int* f = face[i];
			degree[f[0]]++;
			degree[f[1]]++;
			degree[f[2]]++;
		}

		int** link = new int*[vertexN];
		for(i=0; i<vertexN; i++){
			link[i] = new int[degree[i]];
			degree[i] = 0;
		}

		for(i=0; i<faceN; i++){
			int* f = face[i];
			for(j=0; j<3; j++){
				k = f[j];//顶点序号
				link[k][degree[k]++] = i;
			}
		}

		int** linkT = new int*[vertexN];
		for(i=0; i<vertexN; i++){
			int degT = degree[i];
			int degE = 0;
			int* lT = link[i];
			int* lE = new int[2*degT];
			int* lF = new int[2*degT];
			for(j=0; j<degT; j++){
				int *f = face[lT[j]];
				int v;
				if(f[0]==i)
					v = f[1];
				else if(f[1]==i)
					v = f[2];
				else
					v = f[0];
				for(k=0; k<degT; k++){
					if(j==k)
						continue;
					int * f1 = face[lT[k]];
					if(f1[0]==v || f1[1]==v || f1[2]==v)
						break;
				}
				if(k==degT){
					lE[degE] = v;
					lF[degE] = lT[j];
					degE++;
				}
			}

			for(j=0; j<degT; j++){
				int *f = face[lT[j]];
				int v;
				if(f[0]==i)
					v = f[2];
				else if(f[1]==i)
					v = f[0];
				else
					v = f[1];
				for(k=0; k<degT; k++){
					if(j==k)
						continue;
					int * f1 = face[lT[k]];
					if(f1[0]==v || f1[1]==v || f1[2]==v)
						break;
				}
				if(k==degT){
					lE[degE] = v;
					lF[degE] = lT[j];
					degE++;
				}
			}
			degree[i] = degE;
			delete[] lT;
			if(degE == 0){
				delete[] lE;
				delete[] lF;
			}
			else{
				link[i] = lE;
				linkT[i] = lF;
			}
		}

		bool* visit = new bool[vertexN];
		for(i=0; i<vertexN; i++)
			visit[i] = false;
		int top;
		int *stack = new int[T+1];
		int (*stackF)[3] = new int[T+1][3];
		TriList2* tri_list = NULL;
		for(i=0; i<vertexN; i++){
			if(visit[i] == true)
				continue;

			top = 0;
			stack[0] = i;
			visit[i] = true;

			bool isFilled = false;
			while(top < T){
				int index = stack[top];

				if(degree[index] < 2){
					if(degree[index] != 0){
						delete[] link[index];
						degree[index] = 0;
					}
					break;
				}

				int next = link[index][0];
				int f = linkT[index][0];
				stackF[top][0] = face[f][0];
				stackF[top][1] = face[f][1];
				stackF[top][2] = face[f][2];

				delete[] link[index];
				delete[] linkT[index];
				degree[index] = 0;

				if(next == i){
					isFilled = true;
					break;
				}

				if(visit[next])
					break;

				stack[++top] = next;
				visit[next] = true;
			}

			if(!isFilled)
				continue;
			int size = top+1;
			while(size-2 > 0){
				float min = 10;
				int index = 0;
				for(j=0; j<size; j++){
					float* p0 = vertex[stack[j]];
					float* p1 = vertex[stack[(j+1)%size]];
					float* p2 = vertex[stack[(j-1+size)%size]];
					float v1[3] = {p1[0]-p0[0], p1[1]-p0[1], p1[2]-p0[2]};
					float v2[3] = {p2[0]-p0[0], p2[1]-p0[1], p2[2]-p0[2]};
					float l1 = v1[0]*v1[0] + v1[1]*v1[1] + v1[2]*v1[2];
					float l2 = v2[0]*v2[0] + v2[1]*v2[1] + v2[2]*v2[2];
					float l = (float)(1.0/sqrt(l1*l2));

					float co = (v1[0]*v2[0] + v1[1]*v2[1] + v1[2]*v2[2])*l;
					if(co < -1)
						co = -1;
					else if(co > 1)
						co = 1;
					float angle = (float)acos(co);

					float cx = v1[1]*v2[2] - v1[2]*v2[1];
					float cy = v1[2]*v2[0] - v1[0]*v2[2];
					float cz = v1[0]*v2[1] - v1[1]*v2[0];

					float* q0 = vertex[stackF[j][0]];
					float* q1 = vertex[stackF[j][1]];
					float* q2 = vertex[stackF[j][2]];
					float w1[3] = {q1[0]-q0[0], q1[1]-q0[1], q1[2]-q0[2]};
					float w2[3] = {q2[0]-q0[0], q2[1]-q0[1], q2[2]-q0[2]};
					float dx = w1[1]*v2[2] - w1[2]*v2[1];
					float dy = w1[2]*v2[0] - w1[0]*v2[2];
					float dz = w1[0]*v2[1] - w1[1]*v2[0];

					if(cx*dx + cy*dy + cz*dz > 0)
						angle = (float)(2*PI - angle);

					if(angle < min){
						min = angle;
						index = j;
					}
				}
				int f0 = stack[index];
				int f1 = stack[(index-1+size)%size];
				int f2 = stack[(index+1)%size];
				if(tri_list == NULL)
					tri_list = new TriList2(f0, f1, f2);
				else
					tri_list = tri_list->add(f0, f1, f2);

				int count = 0;
				for(j=0; j<size; j++){
					if(j == index)
						continue;

					stack[count] = stack[j];
					if(j == (index-1+size)%size){
						stackF[count][0] = f0;
						stackF[count][1] = f1;
						stackF[count][2] = f2;
					}
					else{
						stackF[count][0] = stackF[j][0];
						stackF[count][1] = stackF[j][1];
						stackF[count][2] = stackF[j][2];
					}

					count++;
				}

				size--;
			}

			/*
			for(j=0; j<top-1; j++){
			if(tri_list == NULL)
			tri_list = new TriList2(i, stack[j+2], stack[j+1]);
			else
			tri_list = tri_list->add(i, stack[j+2], stack[j+1]);
      }*/
      /*
      degree[i1] = 0;
      degree[i2] = 0;
      delete[] link[i];
      if(degree[i1] != 0)
	  delete[] link[i1];
      if(degree[i2] != 0)
			delete[] link[i2];*/
    }
    delete[] stack;
    delete[] stackF;
    delete[] visit;
    delete[] link;
    delete[] linkT;
    delete[] degree;

    int N = 0;
    TriList2* l;
    for(l=tri_list; l!=NULL; l=l->next)
		N++;

    int (*face_tmp)[3] = new int[faceN + N][3];
    for(i=0; i<faceN; i++){
		face_tmp[i][0] = face[i][0];
		face_tmp[i][1] = face[i][1];
		face_tmp[i][2] = face[i][2];
    }
    delete[] face;
    face = face_tmp;

    for(l=tri_list; l!=NULL; l=l->next){
		face[faceN][0] = l->i0;
		face[faceN][1] = l->i1;
		face[faceN][2] = l->i2;
		faceN++;
    }
  }
  //+计算网格中的边数+
  int countEdge(){
	  int i, j, k;
	  int *degree = new int[vertexN];//+每个顶点的度数（相邻三角形的个数）+
	  for(i=0; i<vertexN; i++)
		  degree[i] = 0;

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  degree[f[0]]++;
		  degree[f[1]]++;
		  degree[f[2]]++;
	  }

	  int** link = new int*[vertexN];//+每个顶点连接的三角面链表+
	  for(i=0; i<vertexN; i++){
		  link[i] = new int[degree[i]];//+顶点i相邻的三角面链表+
		  degree[i] = 0;
	  }

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  for(j=0; j<3; j++){
			  k = f[j];   //+面的顶点k+
			  link[k][degree[k]++] = i;//+面的序号+
		  }
	  }

	  int edgeN = 0;
	  for(i=0; i<vertexN; i++){
		  int* vt = link[i];
		  int vtN = degree[i];

		  int* v = new int[vtN*2];
		  int vN = 0;

		  for(j=0; j<vtN; j++){
			  int* t = face[vt[j]];
			  int v1, v2;//+i、v1、v2为面的三个顶点序号+
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
		  edgeN += vN;
		  delete[] v;
		  if(vtN != 0)
			  delete[] link[i];
	  }
	  delete[] link;
	  delete[] degree;

	  return edgeN/2;
  }

  int countHoles(){
	  int i, j, k;
	  int *degree = new int[vertexN];
	  for(i=0; i<vertexN; i++)
		  degree[i] = 0;

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  degree[f[0]]++;
		  degree[f[1]]++;
		  degree[f[2]]++;
	  }

	  int** link = new int*[vertexN];
	  for(i=0; i<vertexN; i++){
		  link[i] = new int[degree[i]];
		  degree[i] = 0;
	  }

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  for(j=0; j<3; j++){
			  k = f[j];
			  link[k][degree[k]++] = i;
		  }
	  }

	  for(i=0; i<vertexN; i++){
		  int degT = degree[i];
		  int degE = 0;
		  int* lT = link[i];
		  int* lE = new int[2*degT];
		  for(j=0; j<degT; j++){
			  int *f = face[lT[j]];
			  int v;
			  if(f[0]==i)
				  v = f[1];
			  else if(f[1]==i)
				  v = f[2];
			  else
				  v = f[0];
			  for(k=0; k<degT; k++){
				  if(j==k)
					  continue;
				  int * f1 = face[lT[k]];
				  if(f1[0]==v || f1[1]==v || f1[2]==v)
					  break;
			  }
			  if(k==degT)
				  lE[degE++] = v;
		  }

		  for(j=0; j<degT; j++){
			  int *f = face[lT[j]];
			  int v;
			  if(f[0]==i)
				  v = f[2];
			  else if(f[1]==i)
				  v = f[0];
			  else
				  v = f[1];
			  for(k=0; k<degT; k++){
				  if(j==k)
					  continue;
				  int * f1 = face[lT[k]];
				  if(f1[0]==v || f1[1]==v || f1[2]==v)
					  break;
			  }
			  if(k==degT)
				  lE[degE++] = v;
		  }
		  degree[i] = degE;
		  delete[] lT;
		  if(degE == 0)
			  delete[] lE;
		  else
			  link[i] = lE;
	  }

	  int holeN = 0;
	  bool* visit = new bool[vertexN];
	  for(i=0; i<vertexN; i++)
		  visit[i] = false;
	  int top;
	  int *stack = new int[vertexN];
	  for(i=0; i<vertexN; i++){
		  if(visit[i] == true)
			  continue;

		  visit[i] = true;

		  if(degree[i] == 0)
			  continue;

		  holeN++;

		  top = 0;
		  stack[0] = i;

		  bool isFilled = false;
		  while(true){
			  int index = stack[top];

			  if(degree[index] < 2){
				  if(degree[index] != 0){
					  delete[] link[index];
					  degree[index] = 0;
				  }
				  break;
			  }

			  int next = link[index][0];

			  delete[] link[index];
			  degree[index] = 0;

			  if(next == i){
				  isFilled = true;
				  break;
			  }

			  if(visit[next])
				  break;

			  stack[++top] = next;
			  visit[next] = true;
		  }
	  }

	  delete[] stack;
	  delete[] visit;
	  delete[] link;
	  delete[] degree;

	  return holeN;
  }

  void fillSmallHoles(int T){
	  int i, j, k;
	  int *degree = new int[vertexN];
	  for(i=0; i<vertexN; i++)
		  degree[i] = 0;

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  degree[f[0]]++;
		  degree[f[1]]++;
		  degree[f[2]]++;
	  }

	  int** link = new int*[vertexN];
	  for(i=0; i<vertexN; i++){
		  link[i] = new int[degree[i]];
		  degree[i] = 0;
	  }

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  for(j=0; j<3; j++){
			  k = f[j];
			  link[k][degree[k]++] = i;
		  }
	  }

	  for(i=0; i<vertexN; i++){
		  int degT = degree[i];
		  int degE = 0;
		  int* lT = link[i];
		  int* lE = new int[2*degT];
		  for(j=0; j<degT; j++){
			  int *f = face[lT[j]];
			  int v;
			  if(f[0]==i)
				  v = f[1];
			  else if(f[1]==i)
				  v = f[2];
			  else
				  v = f[0];
			  for(k=0; k<degT; k++){
				  if(j==k)
					  continue;
				  int * f1 = face[lT[k]];
				  if(f1[0]==v || f1[1]==v || f1[2]==v)
					  break;
			  }
			  if(k==degT)
				  lE[degE++] = v;
		  }

		  for(j=0; j<degT; j++){
			  int *f = face[lT[j]];
			  int v;
			  if(f[0]==i)
				  v = f[2];
			  else if(f[1]==i)
				  v = f[0];
			  else
				  v = f[1];
			  for(k=0; k<degT; k++){
				  if(j==k)
					  continue;
				  int * f1 = face[lT[k]];
				  if(f1[0]==v || f1[1]==v || f1[2]==v)
					  break;
			  }
			  if(k==degT)
				  lE[degE++] = v;
		  }
		  degree[i] = degE;
		  delete[] lT;
		  if(degE == 0)
			  delete[] lE;
		  else
			  link[i] = lE;
	  }

	  //int* visit = new int[vertexN];
	  //for(i=0; i<vertexN; i++)
      //visit[i] = -1;
	  //int top = -1;
	  //int *stack = new int[T];
	  TriList2* tri_list = NULL;
	  for(i=0; i<vertexN; i++){
		  int deg = degree[i];
		  if(deg != 2)
			  continue;

		  int i1 = link[i][0];
		  int i2 = link[i][1];
		  if(tri_list == NULL)
			  tri_list = new TriList2(i, i2, i1);
		  else
			  tri_list = tri_list->add(i, i2, i1);
		  degree[i1] = 0;
		  degree[i2] = 0;
		  delete[] link[i];
		  if(degree[i1] != 0)
			  delete[] link[i1];
		  if(degree[i2] != 0)
			  delete[] link[i2];

			  /*
			  visit[i] = i;
			  stack[++top] = i;
			  while(top < T){
			  int current = stack[top--];
			  int* l = link[current];
			  int deg1 = degree[current];
      }*/
	  }
	  delete[] link;
	  delete[] degree;

	  int N = 0;
	  TriList2* l;
	  for(l=tri_list; l!=NULL; l=l->next)
		  N++;

	  int (*face_tmp)[3] = new int[faceN + N][3];
	  for(i=0; i<faceN; i++){
		  face_tmp[i][0] = face[i][0];
		  face_tmp[i][1] = face[i][1];
		  face_tmp[i][2] = face[i][2];
	  }
	  delete[] face;
	  face = face_tmp;

	  for(l=tri_list; l!=NULL; l=l->next){
		  face[faceN][0] = l->i0;
		  face[faceN][1] = l->i1;
		  face[faceN][2] = l->i2;
		  faceN++;
	  }
	  delete tri_list;
  }

  void fillSmallHolesMinA(int T){
	  int i, j, k;
	  int *degree = new int[vertexN];
	  for(i=0; i<vertexN; i++)
		  degree[i] = 0;

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  degree[f[0]]++;
		  degree[f[1]]++;
		  degree[f[2]]++;
	  }

	  int** link = new int*[vertexN];
	  for(i=0; i<vertexN; i++){
		  link[i] = new int[degree[i]];
		  degree[i] = 0;
	  }

	  for(i=0; i<faceN; i++){
		  int* f = face[i];
		  for(j=0; j<3; j++){
			  k = f[j];
			  link[k][degree[k]++] = i;
		  }
	  }

	  int** linkT = new int*[vertexN];
	  for(i=0; i<vertexN; i++){
		  int degT = degree[i];
		  int degE = 0;
		  int* lT = link[i];
		  int* lE = new int[2*degT];
		  int* lF = new int[2*degT];
		  for(j=0; j<degT; j++){
			  int *f = face[lT[j]];
			  int v;
			  if(f[0]==i)
				  v = f[1];
			  else if(f[1]==i)
				  v = f[2];
			  else
				  v = f[0];
			  for(k=0; k<degT; k++){
				  if(j==k)
					  continue;
				  int * f1 = face[lT[k]];
				  if(f1[0]==v || f1[1]==v || f1[2]==v)
					  break;
			  }
			  if(k==degT){
				  lE[degE] = v;
				  lF[degE] = lT[j];
				  degE++;
			  }
		  }

		  for(j=0; j<degT; j++){
			  int *f = face[lT[j]];
			  int v;
			  if(f[0]==i)
				  v = f[2];
			  else if(f[1]==i)
				  v = f[0];
			  else
				  v = f[1];
			  for(k=0; k<degT; k++){
				  if(j==k)
					  continue;
				  int * f1 = face[lT[k]];
				  if(f1[0]==v || f1[1]==v || f1[2]==v)
					  break;
			  }
			  if(k==degT){
				  lE[degE] = v;
				  lF[degE] = lT[j];
				  degE++;
			  }
		  }
		  degree[i] = degE;
		  delete[] lT;
		  if(degE == 0){
			  delete[] lE;
			  delete[] lF;
		  }
		  else{
			  link[i] = lE;
			  linkT[i] = lF;
		  }
	  }

	  bool* visit = new bool[vertexN];
	  for(i=0; i<vertexN; i++)
		  visit[i] = false;
	  int top;
	  int *stack = new int[T+1];
	  int *stackF = new int[T+1];
	  TriList2* tri_list = new TriList2(-1, -1, -1);
	  for(i=0; i<vertexN; i++){
		  if(visit[i] == true)
			  continue;

		  top = 0;
		  stack[0] = i;
		  visit[i] = true;

		  bool isFilled = false;
		  while(top < T){
			  int index = stack[top];

			  if(degree[index] < 2){
				  if(degree[index] != 0){
					  delete[] link[index];
					  degree[index] = 0;
				  }
				  break;
			  }

			  int next = link[index][0];
			  int f = linkT[index][0];
			  stackF[top] = f;

			  delete[] link[index];
			  delete[] linkT[index];
			  degree[index] = 0;

			  if(next == i){
				  isFilled = true;
				  break;
			  }

			  if(visit[next])
				  break;

			  stack[++top] = next;
			  visit[next] = true;
		  }

		  if(!isFilled)
			  continue;
		  int size = top+1;

		  fillPolygon(tri_list, stack, stackF, size);
	  }
	  delete[] stack;
	  delete[] stackF;
	  delete[] visit;
	  delete[] link;
	  delete[] linkT;
	  delete[] degree;

	  int N = 0;
	  TriList2* l;
	  for(l=tri_list; l!=NULL; l=l->next)
		  N++;
	  N--;
	  //printf("%d faces\n", N);
	  int (*face_tmp)[3] = new int[faceN + N][3];
	  for(i=0; i<faceN; i++){
		  face_tmp[i][0] = face[i][0];
		  face_tmp[i][1] = face[i][1];
		  face_tmp[i][2] = face[i][2];
	  }
	  delete[] face;
	  face = face_tmp;

	  for(l=tri_list; l->next!=NULL; l=l->next){
		  face[faceN][0] = l->i0;
		  face[faceN][1] = l->i1;
		  face[faceN][2] = l->i2;
		  faceN++;
		  //printf("(%d,%d,%d)\n", l->i0, l->i1, l->i2);
	  }
	  // 需要清理tri_list
	  delete tri_list;
  }

  inline float areaF(int i, int j, int k){
	  float* p0 = vertex[i];
	  float* p1 = vertex[j];
	  float* p2 = vertex[k];

	  float v1x = p1[0] - p0[0];
	  float v1y = p1[1] - p0[1];
	  float v1z = p1[2] - p0[2];

	  float v2x = p2[0] - p0[0];
	  float v2y = p2[1] - p0[1];
	  float v2z = p2[2] - p0[2];

	  float nx = v1y*v2z - v1z*v2y;
	  float ny = v1z*v2x - v1x*v2z;
	  float nz = v1x*v2y - v1y*v2x;

	  return (float)sqrt(nx*nx + ny*ny + nz*nz);
  }
  //+使用点i、j、k求单位面法矢n[3]+
  inline void normal(float n[3], int i, int j, int k){
	  float* p0 = vertex[i];
	  float* p1 = vertex[j];
	  float* p2 = vertex[k];

	  float v1x = p1[0] - p0[0];
	  float v1y = p1[1] - p0[1];
	  float v1z = p1[2] - p0[2];

	  float v2x = p2[0] - p0[0];
	  float v2y = p2[1] - p0[1];
	  float v2z = p2[2] - p0[2];

	  float nx = v1y*v2z - v1z*v2y;   //向量叉乘公式
	  float ny = v1z*v2x - v1x*v2z;
	  float nz = v1x*v2y - v1y*v2x;

	  float len = (float)sqrt(nx*nx + ny*ny + nz*nz);
	  if(len != 0){
		  nx /= len;
		  ny /= len;
		  nz /= len;
	  }

	  n[0] = nx;
	  n[1] = ny;
	  n[2] = nz;
  }

  //1 - Cos(dihedral angle) 计算面(i1,j1,k1)和面(i2,j2,k2)的法矢的夹角的余弦
  inline float dihedral(int i1, int j1, int k1,
	  int i2, int j2, int k2){
	  float n1[3], n2[3];
	  normal(n1, i1, j1, k1);
	  normal(n2, i2, j2, k2);
	  float dot = n1[0]*n2[0] + n1[1]*n2[1] + n1[2]*n2[2];//+两个面法矢点乘+
	  return 1.0f - dot;
  }

  void fillPolygon(TriList2 *&tri_list, int* v, int* f, int N){
	  float** area = new float*[N];
	  float** angle = new float*[N];
	  int** opt = new int*[N];
	  int i, j, k, m;
	  for(i=0; i<N; i++){
		  area[i] = new float[N];
		  angle[i] = new float[N];
		  opt[i] = new int[N];
	  }

	  for(i=0; i<N-1; i++){
		  area[i][i+1] = 0;
		  angle[i][i+1] = 0;
	  }

	  for(i=0; i<N-2; i++){
		  area[i][i+2] = areaF(v[i], v[i+1], v[i+2]);
		  float a1 = dihedral(v[i], v[i+1], v[i+2],
			  face[f[i]][0], face[f[i]][2], face[f[i]][1]);
		  float a2 = dihedral(v[i], v[i+1], v[i+2],
			  face[f[i+1]][0], face[f[i+1]][2], face[f[i+1]][1]);
		  if(a1 > a2)
			  angle[i][i+2] = a1;
		  else
			  angle[i][i+2] = a2;
		  opt[i][i+2] = i+1;
	  }

	  for(j=3; j<N; j++){
		  for(i=0; i<N-j; i++){
			  k = i + j;
			  area[i][k] = 1000000.0f;
			  angle[i][k] = 1000000.0f;
			  for(m=i+1; m<k; m++){
				  float a = area[i][m] + area[m][k] + areaF(v[i], v[m], v[k]);
				  float a1, a2;
				  if(i+1 == m)
					  a1 = dihedral(v[i], v[m], v[k],
					  face[f[i]][0], face[f[i]][2], face[f[i]][1]);
				  else
					  a1 = dihedral(v[i], v[m], v[k], v[i], v[opt[i][m]], v[m]);
				  if(m+1 == k)
					  a2 = dihedral(v[i], v[m], v[k],
					  face[f[m]][0], face[f[m]][2], face[f[m]][1]);
				  else
					  a2 = dihedral(v[i], v[m], v[k], v[m], v[opt[m][k]], v[k]);
				  float maxA = a1;
				  if(maxA < a2) maxA = a2;
				  if(maxA < angle[i][m]) maxA = angle[i][m];
				  if(maxA < angle[m][k]) maxA = angle[m][k];
				  if(maxA < angle[i][k]){
					  if(maxA < angle[i][k]){
						  area[i][k] = a;
						  angle[i][k] = maxA;
						  opt[i][k] = m;
					  }
				  }
				  else if(maxA == angle[i][k]){
					  if(a < area[i][k]){
						  area[i][k] = a;
						  angle[i][k] = maxA;
						  opt[i][k] = m;
					  }
				  }
			  }
		  }
	  }

	  traceOpt(tri_list, 0, N-1, opt, v);

	  for(i=0; i<N; i++){
		  delete[] area[i];
		  delete[] opt[i];
		  delete[] angle[i];
	  }
	  delete[] opt;
	  delete[] area;
	  delete[] angle;
  }

  void traceOpt(TriList2 *&tri_list, int i, int k, int** opt, int *v){
	  if(i+2 == k)
		  tri_list = tri_list->add(v[i], v[k], v[i+1]);
	  else{
		  int o = opt[i][k];
		  if(o != i+1)
			  traceOpt(tri_list, i, o, opt, v);

		  tri_list = tri_list->add(v[i], v[k], v[o]);

		  if(o != k-1)
			  traceOpt(tri_list, o, k, opt, v);
	  }
  }
};

#undef PI

#endif