#ifndef CellData_h__
#define CellData_h__
#include <float.h>
#include "Common/vect.h"
#include "Common/array3d.h"

struct BBox{
	vect3f min;
	vect3f max;

	void init(){
		min[0] = min[1] = min[2] = FLT_MAX;
		max[0] = max[1] = max[2] = FLT_MIN;
	}
	vect3f size(){
		vect3f boxSize;
		boxSize[0] = max[0]-min[0];
		boxSize[1] = max[1]-min[1];
		boxSize[2] = max[2]-min[2];
		return boxSize;
	}
};

struct Point{
	vect3f pos;
};

struct CellData{
	std::vector<int> pts;
	
	CellData(){
		clear();
	}
	void clear(){
		pts.clear();
	}
};

template <class T>
inline T Ratio2(T* p, T* c);
void readPtsToCell(Array3D<CellData>& cells, float l, float** dataPts, int nPts);
int simplyByCell(bool* beSimpled, float t, float** dataPts, int nPts);



#endif // CellData_h__