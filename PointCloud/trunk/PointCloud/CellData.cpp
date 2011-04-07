#include "stdafx.h"
#include "CellData.h"

/*
	构建点云到三维栅格中
	输入参数：l为栅格边长的倒数，cellsBox为点云的包围盒，dataPts为点云数据，nPts为点云个数
	输出参数：cells为构建的栅格中存储的点序号
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
	计算两点之间的距离的平方
*/
template <class T>
inline T Ratio2(T* p, T* c){
	return (p[0]-c[0])*(p[0]-c[0]) + (p[1]-c[1])*(p[1]-c[1]) + (p[2]-c[2])*(p[2]-c[2]);
}

/*
	根据三维栅格法进行精简点云数据
	输入参数：t为精简比例>1，dataPts点云数据，nPts点云个数
	输出参数：beSimpled 是否被精简掉
	返回值：保留点的个数
	beSimpled 使用之前设置为true
	三维栅格法在数据点云精简中的应用
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
	//+如何计算栅格长度的倒数？l为栅格边长的倒数+
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

		float centerPt[3];	//+栅格的中心点+
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
		beSimpled[idxMin] = false;	//+距离栅格中心最近的点保留+
		leftNum++;
	}
	return leftNum;
}