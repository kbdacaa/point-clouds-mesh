#include <iostream>
#include <vector>
#include <algorithm>
using namespace std;

#define NODEMAXPTSIZE 14	// 每个方格中点的最大个数
typedef unsigned int uint;
enum NodeType{
	LeafNodeT,
	SplitNodeT,
};
class CPointSet;

struct Node{
	NodeType type;	// 类型
	Node* parent;		// 父节点
 	uint level;				// 当前节点的层次

	Node(Node* parent, NodeType type, uint level){
		this->parent = parent;
 		this->level = level;
		this->type = type;
	}
};

// 叶子结点
template <class Index = uint>
struct LeafNode : Node{
	vector<Index> ptIndex;	// 存放叶子节点中点的序号

	LeafNode(Node* parent, uint level) : Node(parent, LeafNodeT, level){}

	void setIndexs(vector<Index>& idxlist){
		ptIndex.resize(idxlist.size());
		std::copy(idxlist.begin(), idxlist.end(), ptIndex.begin());
	}
};

vector<LeafNode<int>*> gLeafNodeVector;		// 保存所有叶子节点的数组

// 中间的分裂结点
struct SplitNode : Node{
	vector<Node*> children;	// 八个儿子节点(八个儿子问题？？顺序)
	float  cellMin[3];
	float  cellMax[3];

	SplitNode(Node* parent, uint level, float cellMin[3], float cellMax[3]):Node(parent, SplitNodeT, level){
		this->cellMin[0] = cellMin[0];
		this->cellMin[1] = cellMin[1];
		this->cellMin[2] = cellMin[2];
		this->cellMax[0] = cellMax[0];
		this->cellMax[1] = cellMax[1];
		this->cellMax[2] = cellMax[2];
	}
	~SplitNode(){
		std::vector<Node*>::iterator it = children.begin();
		for (; it != children.end(); it++){
			delete (*it);
		}
	}

	void addNode(Node* node){ children.push_back(node); }

	template <class Index>
	void buildTreeNode(vector<Index>& ptVst, CPointSet* ps, uint cellMaxPointSize){
		float cellSplit[3];
		cellSplit[0] =0.5f * (cellMax[0]+cellMin[0]);
		cellSplit[1] =0.5f * (cellMax[1]+cellMin[1]);
		cellSplit[2] =0.5f * (cellMax[2]+cellMin[2]);

		vector<Index> vlist[8];
		for (size_t i = 0; i < ptVst.size(); i++){
			Index idx = ptVst[i];
			float* pt = ps->getPoint(idx);
			if (pt[0] < cellSplit[0]){// X pt < Xsplit
				if (pt[1] < cellSplit[1]){	// Y pt < Ysplit
					if (pt[2] < cellSplit[2]){		// Z pt < Zsplit
						vlist[0].push_back(idx);
					} else {	// Z pt >= Zsplit
						vlist[1].push_back(idx);
					}
				} else {	// Y pt >= Ysplit
					if (pt[2] < cellSplit[2]){		// Z pt < Zsplit
						vlist[2].push_back(idx);
					} else {	// Z pt >= Zsplit
						vlist[3].push_back(idx);
					}
				}
			} else {	// X pt >= Xsplit
				if (pt[1] < cellSplit[1]){	// Y pt < Ysplit
					if (pt[2] < cellSplit[2]){		// Z pt < Zsplit
						vlist[4].push_back(idx);
					} else {	// Z pt >= Zsplit
						vlist[5].push_back(idx);
					}
				} else {	// Y pt >= Ysplit
					if (pt[2] < cellSplit[2]){		// Z pt < Zsplit
						vlist[6].push_back(idx);
					} else {	// Z pt >= Zsplit
						vlist[7].push_back(idx);
					}
				}
			}
		}
		ptVst.clear();

		for (int k = 0; k < 8; k++){
			if (vlist[k].size() > 0){
				if (vlist[k].size() < cellMaxPointSize){
					LeafNode<Index>* leafNode = new LeafNode<Index>(this, level+1);
					leafNode->setIndexs(vlist[k]);
					//addNode(leafNode);
					gLeafNodeVector.push_back(leafNode);
					vlist[k].clear();
				}else{
					float subCellMin[3];
					float subCellMax[3];
					if (k < 4){
						subCellMin[0] = cellMin[0];
						subCellMax[0] = cellSplit[0];
					}else{
						subCellMin[0] = cellSplit[0];
						subCellMax[0] =cellMax[0];
					}

					if (k%4<2) {
						subCellMin[1] = cellMin[1];
						subCellMax[1] = cellSplit[1];
					}else{
						subCellMin[1] = cellSplit[1];
						subCellMax[1] = cellMax[1];
					}

					if (k%2 == 0){
						subCellMin[2] = cellMin[2];
						subCellMax[2] = cellSplit[2];
					}else{
						subCellMin[2] = cellSplit[2];
						subCellMax[2] = cellMax[2];
					}

					SplitNode* splitNode = new SplitNode(this, level+1, subCellMin, subCellMax);
					addNode(splitNode);
					splitNode->buildTreeNode(vlist[k], ps, cellMaxPointSize);
					vlist[k].clear();
				}
			}
		}
	}
};

// 创建整个树，返回树根
template <class Index>
inline SplitNode* createOctTree(CPointSet* ps, int maxNodePtSize = NODEMAXPTSIZE){
	float Min[3], Max[3];
	ps->getBound(Min, Max);
	vector<Index> psIndex(ps->getPointSize());
	for (int i = 0; i < ps->getPointSize(); i++)
		psIndex[i] = i;

	gLeafNodeVector.clear();
	SplitNode* root = new SplitNode(NULL, 0, Min, Max);
	root->buildTreeNode<Index>(psIndex, ps, maxNodePtSize);
	return root;
}