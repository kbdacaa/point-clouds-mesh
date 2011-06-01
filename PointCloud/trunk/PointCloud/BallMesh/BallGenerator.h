#ifndef BALLGENERATOR_H
#define BALLGENERATOR_H

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

#include "PointSet.h"
#include "KdTree.h"
#include "Ball.h"

#include "../NumericalC/SVD.h"
#include "../NumericalC/jacobi.h"

#include <stdlib.h>
#include <math.h>

//#include <iostream>
//#include <fstream>
//using namespace  std;
#define BALL_FILE_TXT "ball_tmp_file.txt"
#define VERTEX_FILE_TXT "vertex_tmp_file.txt"

#define MAX_BISECTION 20
#define CONV_BISECTION 0.0001f
#define K 10
#define MCN 15
#define Tover 1000000.0f
#define Nmin 6
#define BALL_FILE "ball_tmp_binary_file"
#define VERTEX_FILE "vertex_tmp_binary_file"
#define OUT_NUMBER 10000

class BallGenerator {
public:
    CPointSet *ps; //+��������+
    KdTree *tree; //+���Ƶ�KDTree+
    
    double totalA;
    double dia, dia2; //+���ư�Χ�жԽ��߳�dia = L /�Խ��߳�ƽ��dia2+
    
    double Terr;    // ���㾫��
    double Terr2;
    float Tqem;
    float Tave; // ƽ���뾶 ����
    
    int TTNmin;
    
    int baN; //+��ĸ���+
    
    FILE *ball_out;
    FILE *vertex_out;
    
    BallGenerator(CPointSet *ps) {
        this->ps = ps;
        cout << "Constructing a Kd-tree" << endl;
        tree = new KdTree(ps);
    }
    
    ~BallGenerator() {
        delete tree;
        tree = NULL;
    }
    
    void precom() {
        float max[3], min[3];
        ps->getBound(min, max);
        float sx = max[0] - min[0];
        float sy = max[1] - min[1];
        float sz = max[2] - min[2];
        dia2 = sx * sx + sy * sy + sz * sz;
        dia = sqrt(dia2); //+��Χ�жԽ��߳�+
        
        if (ps->m_normal != NULL)
            cout << "Computing the area at each point." << endl;
        else
            cout << "Computing the area and normal at each point." << endl;
        computeWeightAndNormal();
    }
    //+errΪ������aveΪ 0��qemΪ 2��Խ��Խϸ�壩+
    void generate(float err, float ave, float qem) {
        Terr = err * dia;
        Terr2 = totalA * Terr * Terr;
        Tave = (float)(dia *ave);
        Tqem = qem;
        TTNmin = (int)(qem *qem * Nmin);
        
        baN = 0;
        ball_out = fopen(BALL_FILE, "w+b");
        vertex_out = fopen(VERTEX_FILE, "w+b");
        generateBalls();
        fclose(ball_out);
        fclose(vertex_out);
        
        //delete tree;
    }
    //+����ʱ���ļ��ж�ȡ�����Ȩ��+
    CPointSet *getBalls() {
        float *buff = new float[4 *baN];
        ball_out = fopen(BALL_FILE, "r+b");
        fread(buff, sizeof(float), 4 *baN, ball_out);
        fclose(ball_out);
        
        CPointSet *bs = new CPointSet(baN);
        int i;
        int j = 0;
        for (i = 0; i < baN; i++) {
            float x = buff[j++];
            float y = buff[j++];
            float z = buff[j++];
            float r = buff[j++];
            
            bs->setPoint(i, x, y, z);
            bs->setWeight(i, r);
        }
        delete []buff;
        return bs;
    }
    //+����ʱ�����ļ��ж�ȡ����+
    CPointSet *getVertices() {
        float *buff = new float[3 *baN];
        vertex_out = fopen(VERTEX_FILE, "r+b");
        fread(buff, sizeof(float), 3 *baN, vertex_out);
        fclose(vertex_out);
        
        CPointSet *vs = new CPointSet(baN);
        int i;
        int j = 0;
        for (i = 0; i < baN; i++) {
            float x = buff[j++];
            float y = buff[j++];
            float z = buff[j++];
            
            vs->setPoint(i, x, y, z);
        }
        delete []buff;
        return vs;
    }
    //+������������Ȩ�غͷ�ʸ+
    void computeWeightAndNormal() {
        bool computeN = (ps->m_normal == NULL);
        
        int i, j, k;
        
        int N = ps->m_pointN;
        float**point = ps->m_point;
        float *weight = ps->m_weight;
        
        float size = tree->getLeafSize(); //+��������Ҷ�ӽڵ�ƽ������+
        double *d_array = new double[K]; //+K�����������ĵ�ľ���+
        int *p_array = new int[K + 1];
        totalA = 0;
        for (i = 0; i < N; i++) {
            //+����ÿһ������ķ�ʸ��Ȩ��+
            float *p = point[i]; //+��ǰ�����+
            float r = size;
            int listN,  *list;
            do {
                tree->collectPointIndexInSphere(list, listN, p, r);
                //+����K�ٽ���(���ܶ���K)+
                r *= 1.5f;
            } while (listN < K + 1);
            
            for (j = 0; j < K; j++)
                d_array[j] = dia2;
            //+ֱ��ƽ��+
            // ���վ�������
            for (j = 0; j < listN; j++) {
                float *q = point[list[j]];
                if (p == q)
                    continue;
                float vx = q[0] - p[0];
                float vy = q[1] - p[1];
                float vz = q[2] - p[2];
                double d = vx * vx + vy * vy + vz * vz;
                
                int index =  - 1;
                for (k = 0; k < K; k++) {
                    if (d < d_array[k]) {
                        index = k;
                        break;
                    }
                }
                if (index < 0)
                    continue;
                
                for (k = K - 1; k > index; k--) {
                    p_array[k] = p_array[k - 1];
                    d_array[k] = d_array[k - 1];
                }
                p_array[index] = list[j];
                d_array[index] = d;
            }
            double a = 0;
            for (j = 0; j < K; j++)
                a += d_array[j];
            a = a / K; //+K���ٽ����ƽ������ƽ��+
            weight[i] *= (float)a; //+�����Ȩ��+
            totalA += a;
            
            if (computeN) {
                p_array[K] = i; //+��ǰ�����+
                float n[3]; //+����ķ�ʸ+
                computeNormalWithCV(n, p_array, K + 1);
                ps->setNormal(i, n[0], n[1], n[2]);
            }
        }
        //   ps->writeNormToFile("Normal.pts");
        delete []p_array;
        delete []d_array;
    }
    //+�������ɵ���+
    void generateBalls() {
        int i, j;
        int N = ps->m_pointN;
        float**center = ps->m_point;
        KdTree *c_tree = tree;
        CPointSet *cs = ps;        
        //============��ֵƽ��============//
        if (Tave != 0) {
            float *weight = ps->m_weight;
            float(*normal)[3] = ps->m_normal;
            cout << "Averaging" << endl;
            CPointSet *sps = new CPointSet(N); //�µĻ���Ȩ�ص����ĵ㼯
            for (i = 0; i < N; i++) {
                float *p = center[i];
                
                int listN,  *list;
                c_tree->collectPointIndexInSphere(list, listN, p, Tave);
                float totalW = 0; //+K������Ȩ��֮��+
                float c[3] =  { 0, 0, 0 }; //+K�����ڵ����ĵ�+
                for (j = 0; j < listN; j++) {
                    int k = list[j]; //+����������+
                    float *q = center[k];
                    float w = weight[k];
                    c[0] += w * q[0];
                    c[1] += w * q[1];
                    c[2] += w * q[2];
                    totalW += w;
                }
                c[0] /= totalW;
                c[1] /= totalW;
                c[2] /= totalW;
                
                sps->setPoint(i, c[0], c[1], c[2]); //�µ㼯                
            }
            cs = sps;
            center = sps->m_point;
            c_tree = new KdTree(sps);
        }

        int tableN = N;
        float *overlap = new float[tableN];	// ��ʾ���Ƿ����е��������ǣ�����=Tover��
        int *active_table = new int[tableN];
        int activeN = 0;	 // ��ǰδ�����ǵĵ�ĸ���
        
        for (i = 0; i < N; i++) {//��ʼ��            
            active_table[activeN++] = i;
            overlap[i] = 0;
        }
        FILE *Ballout = fopen(BALL_FILE_TXT, "w");      // �ı�������� ����Ӧ��
        FILE *Vertexout = fopen(VERTEX_FILE_TXT, "w");// �ı�������� ʹ�õ�

        // ==========��ʼ��������Ӧ��============//
        //  N = tableN = activeN; //+=N+
        
        int *can = new int[MCN];    // �洢 ����
        long idum = 1;
        
        int t_baN = 0;  // ͳ��ÿ����Ҫд����ĸ���
        Ball **t_ba = new Ball *[OUT_NUMBER];
        
        //For counter to stdout
        int pre_activeN = activeN;
        int percent = 0;
        
        while (activeN > 0) {
            int index;	// ѡ��ĵ� C �����
            if (activeN > MCN) {    // ���ʣ��δ���ǵ�϶�
                for (j = 0; j < MCN; j++) {
                    int k = (int)(ran0(&idum)*(tableN - 1));	// ȡһ����������
                    can[j] = active_table[k];
                    if (overlap[can[j]] >= Tover)   // �Ѿ������ǣ��ӹ�
                        j--;
                }
                
                //search minimun from multiple-choice
                index = can[0];
                for (j = 1; j < MCN; j++) {
                    if (overlap[index] > overlap[can[j]])
                        index = can[j]; // ѡ��overlap��С��
                }
            } else {
                index =  - 1;
                for (j = 0; j < tableN; j++) {
                    int k = active_table[j];
                    if (overlap[k] >= Tover)
                        continue;
                    if (index < 0 || overlap[index] > overlap[k])
                        index = k;// ѡ��overlap��С��
                }
                if (index < 0)
                    break;
            }
            
            float *c = center[index];	// ѡ��� C		step 2
            overlap[index] = Tover; // ��index������
            activeN--;
            
            // ��������Ӧ��İ뾶�͸�����
            Ball *ba = new Ball(c[0], c[1], c[2]);
            computeOptimalSupportL2(ba);		// step 3

            float r = ba->r;
            int *list, listN;
            c_tree->collectPointIndexInSphere(list, listN, c, r);		// step 4
            
            //updtae overlap
            bool *onHull = new bool[listN]; //��ǵ����͹����
            computeHull(onHull, list, listN, c);		// step 4 ����͹��
            for (j = 0; j < listN; j++) {
                int k = list[j];
                if (overlap[k] >= Tover) continue;
                   
                if (!onHull[j]) {   // ��͹���ڲ��ĵ�����Ϊ������
                    overlap[k] = Tover;
                    if (overlap[k] >= Tover) {
                        activeN--;
                    }
                } else {
                    float *p = center[k];
                    overlap[k] += (float)(ba->weight(p[0], p[1], p[2]));// ������͹���ϵ�� overlap�� ��ʾ�� ѡ��� C ������
                }
            }
            delete []onHull;
            
            //===Write the ball to temp file ÿOUT_NUMBER����д��һ��===//
            baN++;
            t_ba[t_baN++] = ba;
            if (t_baN % OUT_NUMBER == 0) {  
                float *buff = new float[4 *OUT_NUMBER];
                int k = 0;
                for (j = 0; j < OUT_NUMBER; j++) {
                    Ball *b = t_ba[j];
                    buff[k++] = b->cx;
                    buff[k++] = b->cy;
                    buff[k++] = b->cz;
                    buff[k++] = b->r;
                    fprintf(Ballout, "%f,%f,%f,%f\n", b->cx, b->cy, b->cz, b->r);
                }
                fwrite(buff, sizeof(float), 4 *OUT_NUMBER, ball_out);   // д�뵽�������ļ�
                delete []buff;
                
                buff = new float[3 *OUT_NUMBER];
                k = 0;
                for (j = 0; j < OUT_NUMBER; j++) {
                    Ball *b = t_ba[j];
                    buff[k++] = b->px;
                    buff[k++] = b->py;
                    buff[k++] = b->pz;
                    fprintf(Vertexout, "%f,%f,%f\n", b->px, b->py, b->pz);
                    delete b;
                }
                fwrite(buff, sizeof(float), 3 *OUT_NUMBER, vertex_out);
                delete []buff;
                
                t_baN = 0;
            }
            // ====== ����Υ�����ǵ�����===========//
            if (activeN < tableN / 2) {//resize table                
                int count = 0;  // ͳ��δ�����ǵĵ�ĸ���
                for (j = 0; j < tableN; j++) {
                    int k = active_table[j];
                    if (overlap[k] < Tover) {
                        active_table[count++] = k;
                    }
                }
                tableN = count;
            }
            // =========���������� %����=============//
            if (100 *(N - pre_activeN) / N <= percent && 100 *(N - activeN) / N > percent)
                printf("%d%%: %d basis functions", ++percent, baN);
            pre_activeN = activeN;
          }

           // ======= ��Դ���� =========//
          delete []can;
          delete []active_table;
          delete []overlap;          
          if (Tave != 0) {
              delete c_tree;
              delete cs;
          }

          //=======д���ļ���========//
          if (t_baN != 0) {
              float *buff = new float[4 *t_baN];
              int k = 0;
              for (j = 0; j < t_baN; j++) {
                  Ball *b = t_ba[j];
                  buff[k++] = b->cx;
                  buff[k++] = b->cy;
                  buff[k++] = b->cz;
                  buff[k++] = b->r;
                  fprintf(Ballout, "%f,%f,%f,%f\n", b->cx, b->cy, b->cz, b->r);
              }
              fwrite(buff, sizeof(float), 4 *t_baN, ball_out);
              delete []buff;
              
              buff = new float[3 *t_baN];
              k = 0;
              for (j = 0; j < t_baN; j++) {
                  Ball *b = t_ba[j];
                  buff[k++] = b->px;
                  buff[k++] = b->py;
                  buff[k++] = b->pz;
                  fprintf(Vertexout, "%f,%f,%f\n", b->px, b->py, b->pz);
                  delete b;
              }
              fwrite(buff, sizeof(float), 3 *t_baN, vertex_out);
              delete []buff;
          }
          delete []t_ba;
          fclose(Ballout);
          fclose(Vertexout);
          cout << baN << " balls" << endl;
      }

      //+�޸�Ball�İ뾶r��px��py��pz+
      // ������İ뾶������  3.2 Radius of Sphere and Auxiliary Point
      void computeOptimalSupportL2(Ball *ba) {
          int i;
          float scaleL = (float)(10 *Terr); //+���Ŵ�+  ��������
          float scaleS = (float)(Terr); //+�����С+
          double errorL, errorS;
          
          ba->r = scaleS;
          errorS = localFitQ(ba);
          if (errorS > Terr2) {
              while (errorS > Terr2) {	// ����
                  scaleL = scaleS;
                  scaleS *= 0.5f;
                  errorL = errorS;
                  ba->r = scaleS;
                  errorS = localFitQ(ba);
              }
          } else {
              ba->r = scaleL;
              errorL = localFitQ(ba);
              while (errorL < Terr2) {	// ����
                  scaleS = scaleL;
                  scaleL *= 2;
                  errorS = errorL;
                  ba->r = scaleL;
                  errorL = localFitQ(ba);
              }
          }
          
          for (i = 0; i < MAX_BISECTION; i++) {
              float r = 0.5f *(scaleS + scaleL);
              ba->r = r;
              double e = localFitQ(ba);
              
              if (e < Terr2) {
                  scaleS = r;
                  errorS = e;
              } else {
                  scaleL = r;
                  errorL = e;
              }
              
              if (scaleL - scaleS < CONV_BISECTION *dia)
                  break;
          }
      }
      //+ֻ�޸���Ball��px��py��pz+
      // ����Q( c, r, x )
      double localFitQ(Ball *ba) {
          int i;
          
          float** point = ps->m_point;
          float(*normal)[3] = ps->m_normal;
          float *weight = ps->m_weight;
          
          float c[3] =  {
              ba->cx, ba->cy, ba->cz
          };
          float r = Tqem * ba->r;	// Tq Ĭ��Ϊ2
          int *list, listN;
          tree->collectPointIndexInSphere(list, listN, c, r); //������ C �뾶 r ��Χ�ڵĵ�
          
          double Q[10] =  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
          double W = 0;
          double W2 = 0; //Ȩ��ֵ֮��
          for (i = 0; i < listN; i++) {
              int j = list[i];
              float *p = point[j];
              float *n = normal[j];
              
              float vx = p[0] - c[0];
              float vy = p[1] - c[1];
              float vz = p[2] - c[2];
              double w = Ball::weight(sqrt(vx *vx + vy * vy + vz * vz), r);		// GR( || pj - c || )
              W2 += w;
              w *= weight[j];	// wj * GR( || pj - c || )
              
              double wnx = w * n[0];
              double wny = w * n[1];
              double wnz = w * n[2];
              
              Q[0] += wnx * n[0];
              Q[1] += wnx * n[1];
              Q[2] += wnx * n[2];
              Q[3] += wny * n[1];
              Q[4] += wny * n[2];
              Q[5] += wnz * n[2];
              
              double dot = n[0] *p[0] + n[1] *p[1] + n[2] *p[2];	// nj �� pj
              Q[6] += dot * wnx;
              Q[7] += dot * wny;
              Q[8] += dot * wnz;
              
              Q[9] += w * dot * dot;	// Q(c, r, x) = Ej( wj * GR( || pj - c || ) * (nj �� pj)#2 )
              
              W += w;
          }
          
          if ((float)W == 0) {	// ������������ʱ������ֻ�е� C
              ba->px = c[0];
              ba->py = c[1];
              ba->pz = c[2];
              return 0;
          }
          
          double **A = new double *[4];
          double **v = new double *[4];
          double b[4], w[4], x[4];
          for (i = 1; i < 4; i++) {
              A[i] = new double[4];
              v[i] = new double[4];
          }
          
          W = 1.0 / W;
          A[1][1] = Q[0] *W;
          A[1][2] = A[2][1] = Q[1] *W;
          A[1][3] = A[3][1] = Q[2] *W;
          A[2][2] = Q[3] *W;
          A[2][3] = A[3][2] = Q[4] *W;
          A[3][3] = Q[5] *W;
          
          b[1] = (Q[6] - Q[0] *c[0] - Q[1] *c[1] - Q[2] *c[2]) *W;
          b[2] = (Q[7] - Q[1] *c[0] - Q[3] *c[1] - Q[4] *c[2]) *W;
          b[3] = (Q[8] - Q[2] *c[0] - Q[4] *c[1] - Q[5] *c[2]) *W;
          
          if (!SVD::svdcmp(A, 3, 3, w, v)) {
              ba->px = c[0];
              ba->py = c[1];
              ba->pz = c[2];
              return 0;
          }
          
          int k;
          float wmax = (float)fabs(w[1]);
          for (k = 2; k < 4; k++)
              if (wmax < fabs(w[k]))
                  wmax = (float)fabs(w[k]);
              
              for (k = 1; k < 4; k++) {
                  if (fabs(w[k]) < 0.1 *wmax)
                      w[k] = 0.0;
              }
              SVD::svbksb(A, w, v, 3, 3, b, x);
              
              for (i = 1; i < 4; i++) {
                  delete []A[i];
                  delete []v[i];
              }
              delete []A;
              delete []v;
              
              ba->px = c[0] + (float)x[1];
              ba->py = c[1] + (float)x[2];
              ba->pz = c[2] + (float)x[3];
              
              double mx = ba->px;
              double my = ba->py;
              double mz = ba->pz;
              
              double Ax = Q[0] *mx + Q[1] *my + Q[2] *mz;
              double Ay = Q[1] *mx + Q[3] *my + Q[4] *mz;
              double Az = Q[2] *mx + Q[4] *my + Q[5] *mz;
              
              double err = (Ax - 2.0 * Q[6]) *mx + (Ay - 2.0 * Q[7]) *my + (Az -
                  2.0 *Q[8]) *mz + Q[9];
              
              double len = sqrt(x[1] *x[1] + x[2] *x[2] + x[3] *x[3]);
              if (len > ba->r) {
                  ba->px = c[0];
                  ba->py = c[1];
                  ba->pz = c[2];
              }
              
              if (listN > TTNmin)
                  return err;
              //(Ax-2.0*Q[6])*mx + (Ay-2.0*Q[7])*my + (Az-2.0*Q[8])*mz + Q[9];
              else
                  return 0;
      }
      
      void computeHull(bool *onHull, int *list, int N, float c[3]) {
          //compute base plane
          float n[3];
          computeNormalWithCV(n, list, N);
          float t1[3], t2[3];
          if (fabs(n[0]) < fabs(n[1])) {
              double l = sqrt(n[1] *n[1] + n[2] *n[2]);
              t1[0] = 0;
              t1[1] =  - (float)(n[2] / l);
              t1[2] = (float)(n[1] / l);
          } else {
              double l = sqrt(n[0] *n[0] + n[2] *n[2]);
              t1[0] = (float)(n[2] / l);
              t1[1] = 0;
              t1[2] =  - (float)(n[0] / l);
          }
          t2[0] = n[1] *t1[2] - n[2] *t1[1];
          t2[1] = n[2] *t1[0] - n[0] *t1[2];
          t2[2] = n[0] *t1[1] - n[1] *t1[0];
          
          float(*p)[2] = new float[N][2];   // ת����ƽ���ϵĵ�
          int *index = new int[N];
          int i;
          for (i = 0; i < N; i++) {
              float *p1 = ps->m_point[list[i]];
              p[i][0] = t1[0]*(p1[0] - c[0]) + t1[1]*(p1[1] - c[1]) + t1[2] * (p1[2] - c[2]);
              p[i][1] = t2[0]*(p1[0] - c[0]) + t2[1]*(p1[1] - c[1]) + t2[2] * (p1[2] - c[2]);
              index[i] = i;
              onHull[i] = false;
          }
          
          int left = 0; // �����
          int right = 0;    // ���ҵ�
          for (i = 1; i < N; i++) {
              if (p[i][0] < p[left][0])
                  left = i;
              else if (p[i][0] > p[right][0])
                  right = i;
          }
          onHull[left] = true;
          onHull[right] = true;
          
          index[0] = left;
          index[left] = 0;
          int tmp = index[N - 1];
          index[N - 1] = index[right];
          index[right] = tmp;
          
          quickHull(0, N - 1, p, onHull, index);
          
          for (i = 0; i < N; i++)
              index[i] = i;
          
          index[0] = right;
          index[right] = 0;
          tmp = index[N - 1];
          index[N - 1] = index[left];
          index[left] = tmp;
          
          quickHull(0, N - 1, p, onHull, index);
          
          delete []p;
          delete []index;
      }
      
      void quickHull(int s, int e, float(*p)[2], bool *onHull, int *index) {
          //cout << s << "," << e << endl;
          if (s + 1 >= e)
              return ;
          float *p1 = p[index[s]];
          float *p2 = p[index[e]];
          float nx = p2[1] - p1[1];
          float ny =  - (p2[0] - p1[0]);
          int i;
          float max = 0;
          int maxI = s;
          for (i = s + 1; i < e; i++) {
              float *q = p[index[i]];
              float d = nx *(q[0] - p1[0]) + ny *(q[1] - p1[1]);
              if (d > max) {
                  max = d;
                  maxI = index[i];
              }
          }
          onHull[maxI] = true;
          
          float *p3 = p[maxI];
          nx = p3[1] - p1[1];
          ny =  - (p3[0] - p1[0]);
          i = s + 1;
          int j = e-1;
          while (i <= j) {
              int k = index[i];
              float *q = p[k];
              if (k != maxI && nx *(q[0] - p1[0]) + ny *(q[1] - p1[1]) > 0)
                  i++;
              else {
                  int tmp = index[i];
                  index[i] = index[j];
                  index[j] = tmp;
                  j--;
              }
          }
          int c1 = i;
          
          nx = p2[1] - p3[1];
          ny =  - (p2[0] - p3[0]);
          i = e-1;
          j = c1;
          while (i >= j) {
              int k = index[i];
              float *q = p[k];
              if (k != maxI && nx *(q[0] - p2[0]) + ny *(q[1] - p2[1]) > 0)
                  i--;
              else {
                  int tmp = index[i];
                  index[i] = index[j];
                  index[j] = tmp;
                  j++;
              }
          }
          int c2 = i;
          
          index[c1] = maxI;
          index[c2] = maxI;
          
          quickHull(s, c1, p, onHull, index);
          quickHull(c2, e, p, onHull, index);
      }
      //+����K+1������ĵ�K+1������ķ�ʸ��n[3]��ŷ�ʸ,listΪ�����������,NΪ���������+
      void computeNormalWithCV(float n[3], int *list, int N) {
          int i;
          float** point = ps->m_point;
          //+��ǰN��������ĵ�+
          float cx = 0;
          float cy = 0;
          float cz = 0;
          
          for (i = 0; i < N; i++) {
              float *p = point[list[i]];
              cx += p[0];
              cy += p[1];
              cz += p[2];
          }
          float Ni = 1.0f / N;
          cx *= Ni;
          cy *= Ni;
          cz *= Ni;
          
          //data for Jacobi method
          double **A = new double *[4];
          double **v = new double *[4];
          double w[4];
          int nrot;
          for (i = 1; i < 4; i++) {
              A[i] = new double[4];
              A[i][1] = A[i][2] = A[i][3] = 0;
              v[i] = new double[4];
          }
          
          //CV matrix
          for (i = 0; i < N; i++) {
              float *p = point[list[i]];
              
              float vx = p[0] - cx;
              float vy = p[1] - cy;
              float vz = p[2] - cz;
              
              A[1][1] += vx * vx;
              A[1][2] += vx * vy;
              A[1][3] += vx * vz;
              
              A[2][2] += vy * vy;
              A[2][3] += vy * vz;
              
              A[3][3] += vz * vz;
          }
          A[2][1] = A[1][2];
          A[3][1] = A[1][3];
          A[3][2] = A[3][2];
          
          Jacobi::jacobi(A, 3, w, v, &nrot);
          
          int mini;
          if (fabs(w[1]) < fabs(w[2]))
              mini = 1;
          else
              mini = 2;
          if (fabs(w[mini]) > fabs(w[3]))
              mini = 3;
          
          n[0] = (float)v[1][mini];
          n[1] = (float)v[2][mini];
          n[2] = (float)v[3][mini];
          
          for (i = 1; i < 4; i++) {
              delete []A[i];
              delete []v[i];
          }
          delete []A;
          delete []v;
      }
      
      //From Numerical Recipes in C
#define IA 16807
#define IM 2147483647
#define AM (1.0/IM)
#define IQ 127773
#define IR 2836
#define MASK 123459876
      // �������������
      double ran0(long *idum) {
          long k;
          double ans;
          
          *idum ^= MASK;
          k = (*idum) / IQ;
          *idum = IA *(*idum - k * IQ) - IR * k;
          if (*idum < 0)
              *idum += IM;
          ans = AM *(*idum);
          *idum ^= MASK;
          return ans;
      }
#undef IA
#undef IM
#undef AM
#undef IQ
#undef IR
#undef MASK
      /* (C) Copr. 1986-92 Numerical Recipes Software 9z!+!1(t+%. */
  };
  
#endif
