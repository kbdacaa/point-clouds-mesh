#include "stdafx.h"
#include "CellData.h"

/*
	�������Ƶ���άդ����
	���������lΪդ��߳��ĵ�����cellsBoxΪ���Ƶİ�Χ�У�dataPtsΪ�������ݣ�nPtsΪ���Ƹ���
	���������cellsΪ������դ���д洢�ĵ����
*/
void readPtsToCell(Array3D<CellData>& cells, float l, BBox& cellsBox, float** dataPts, int nPts){

	for (int i = 0; i < nPts ; i++){
		float* pt = dataPts[i];
		int x = (int)((pt[0]-cellsBox.min[0]) * l);
		int y = (int)((pt[1]-cellsBox.min[1]) * l);
		int z = (int)((pt[2]-cellsBox.min[2]) * l);

		if (x == cells.size[0])
			x--;
		if (y == cells.size[1])
			y--;
		if (z == cells.size[2])
			z--;
		
		cells(x, y, z).pts.push_back(i);
	}
}
/*
	��������֮��ľ����ƽ��
*/
template <class T>
inline T Ratio2(T* p, T* c){
	return (p[0]-c[0])*(p[0]-c[0]) + (p[1]-c[1])*(p[1]-c[1]) + (p[2]-c[2])*(p[2]-c[2]);
}

/*
	������άդ�񷨽��о����������
	���������tΪ�������>1��dataPts�������ݣ�nPts���Ƹ���
	���������beSimpled �Ƿ񱻾����
	����ֵ��������ĸ���
	beSimpled ʹ��֮ǰ����Ϊtrue
	��άդ�������ݵ��ƾ����е�Ӧ��
*/
int simplyByCell(bool* beSimpled, float t, float** dataPts, int nPts){
	assert(t > 0);
	assert(dataPts!= NULL);
	assert(nPts > 0);
	for (int i = 0; i < nPts ; i++){
		beSimpled[i] = true;
	}
	int leftNum = 0;

	BBox cellsBox;
	cellsBox.init();
	for (int i = 0; i < nPts ; i++){
		float* pt = dataPts[i];
		for (int j = 0; j < 3 ; j++){
			if (cellsBox.min[j]>pt[j])
				cellsBox.min[j] = pt[j];
			if (cellsBox.max[j] < pt[j])
				cellsBox.max[j] = pt[j];
		}
	}
	vect3f cellSize;
	cellSize = cellsBox.size();
	// TODO:: 
	//+��μ���դ�񳤶ȵĵ�����lΪդ��߳��ĵ���+
	float	l = sqrt(nPts*(cellSize[0]+cellSize[1]+cellSize[2]) / (6*t*cellSize[0]*cellSize[1]*cellSize[2]));

	int xs = (int)(cellSize[0] * l);
	int ys = (int)(cellSize[1] * l);
	int zs = (int)(cellSize[2] * l);

	Array3D<CellData> cells;
	cells.resize(xs, ys, zs);
	readPtsToCell(cells, l, cellsBox, dataPts, nPts);
	int size = cells.size[0]*cells.size[1]*cells.size[2];

	for (int i = 0; i < size ; i++){
		CellData cell = cells.data[i];
		int cellptsize = cell.pts.size();
		if (cellptsize == 0) continue;

		float centerPt[3];	//+դ������ĵ�+
		centerPt[0] = centerPt[1] = centerPt[2] = 0;
		
		for (int j = 0; j < cellptsize ; j++){
			int idx = cell.pts[j];
			centerPt[0] += dataPts[idx][0];
			centerPt[1] += dataPts[idx][1];
			centerPt[2] += dataPts[idx][2];
		}
		centerPt[0] /= cellptsize;
		centerPt[1] /= cellptsize;
		centerPt[2] /= cellptsize;
		
		double minR2 = DBL_MAX;
		int idxMin = -1;
		for (int j = 0; j < cellptsize ; j++){
			float* pt = dataPts[cell.pts[j]];

			double R2 = Ratio2(pt, centerPt);
			if (minR2 > R2){
				minR2 = R2;
				idxMin = cell.pts[j];
			}
		}
		beSimpled[idxMin] = false;	//+����դ����������ĵ㱣��+
		leftNum++;
	}
	return leftNum;
}