#ifndef SORT_H
#define SORT_H

//===== 快速排序算法=使用主键T1排序,T2跟随=======//
template <class T1, class T2, class T3>
inline int partition(T1* pSort,T2* nnidx,T3 (*pts)[3], int start, int end){
	int i = start -1;
	-- end;
	T1 fD = pSort[end];
	T1 tmp;
	T2 iD = nnidx[end];
	T2 itmp;
	T3 iPt[3] = { pts[end][0], pts[end][1], pts[end][2] };
	T3 iPtTmp[3];
	for (int j = start; j < end; j++) {
		if (pSort[j] <= fD){
			++ i;
			tmp = pSort[i];
			pSort[i] = pSort[j];
			pSort[j] = tmp;

			itmp = nnidx[i];
			nnidx[i] = nnidx[j];
			nnidx[j] = itmp;

			iPtTmp[0] = pts[i][0];
			iPtTmp[1] = pts[i][1];
			iPtTmp[2] = pts[i][2];
			pts[i][0] = pts[j][0];
			pts[i][1] = pts[j][1];
			pts[i][2] = pts[j][2];
			pts[j][0] = iPtTmp[0];
			pts[j][1] = iPtTmp[1];
			pts[j][2] = iPtTmp[2];
		}
	}
	++ i;
	pSort[end] = pSort[i];
	pSort[i] = fD;
	nnidx[end] = nnidx[i];
	nnidx[i] = iD;
	pts[end][0] = pts[i][0];
	pts[end][1] = pts[i][1];
	pts[end][2] = pts[i][2];
	pts[i][0] = iPt[0];
	pts[i][1] = iPt[1];
	pts[i][2] = iPt[2];
	return i;
}

template <class T1, class T2, class T3>
inline void quickSort( T1* arc, T2* nnidx,T3 (*pts)[3], int start , int end){
	if (end - start < 2) return ;
	int i = partition(arc, nnidx, pts, start, end);
	quickSort(arc, nnidx, pts, start, i);
	quickSort(arc, nnidx, pts, i+1, end);
}

#endif // SORT_H