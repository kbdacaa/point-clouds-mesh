#ifndef FILEMANAGER_H
#define FILEMANAGER_H

#include "Ball.h"
#include "BallMesh.h"
#include "PointSet.h"
#include <stdio.h>
#include <fstream.h>
#include <iostream.h>

class FileManager{
public:
  static CPointSet* readPtsBFile(char* name){
    FILE* in = fopen(name, "r+b");
    
    cout << "PtsB File read is started" << endl;
    
    int N;
    int N_tmp[1];
    fread(N_tmp, sizeof(int), 1, in);
    N = N_tmp[0];
    
    cout << N << " points" << endl;
    
    CPointSet* ps = new CPointSet(N);
    
    int i;
    float* buff = new float[3*N];
    fread(buff, sizeof(float), 3*N, in);
    int j = 0;
    for(i=0; i<N; i++){
      float x = buff[j++];
      float y = buff[j++];
      float z = buff[j++];
      ps->setPoint(i, x, y, z);
      ps->setWeight(i, 1.0f);
    }
    delete[] buff;
    cout << "File read is done." << endl;
    
    fclose(in);
    
    return ps;
  }
  
  static CPointSet* readPtsFile(char* name){
    ifstream in(name);
    
    cout << "Pts File read is started" << endl;
    
    int N;
    in >> N;
    
    cout << N << " points" << endl;
    
    CPointSet* ps = new CPointSet(N);
    
    int i;
    for(i=0; i<N; i++){
      float x, y, z;
      in >> x >> y >> z;
      ps->setPoint(i, x, y, z);
      ps->setWeight(i, 1.0f);
    }
    cout << "File read is done." << endl;
    
    in.close();
    
    return ps;
  }
  
  static BallMesh* readMeshFile(char* name){
    ifstream in(name);
    
    cout << "Mesh File read is started" << endl;
    
    int vN, fN;
    in >> vN >> fN;
    
    BallMesh* mesh = new BallMesh(vN, fN);
    
    int i;
    for(i=0; i<vN; i++){
      float x, y, z;
      in >> x >> y >> z;
      mesh->setVertex(i, x, y, z);
    }
    
    for(i=0; i<fN; i++){
      int d, f0, f1, f2;
      in >> d >> f0 >> f1 >> f2;
      mesh->setFace(i, f0, f1, f2);
    }
    
    in.close();
    
    return mesh;
  }
  
  static CPointSet* readPwnCFile(char* name){
    ifstream in(name);
    
    cout << "PwnC File read is started" << endl;
    
    int N;
    in >> N;
    
    cout << N << " points" << endl;
    
    CPointSet* ps = new CPointSet(N);
    
    int i;
    for(i=0; i<N; i++){
      float x,y,z;
      in >> x >> y >> z;
      ps->setPoint(i, x, y, z);
    }
    for(i=0; i<N; i++){
      float w;
      in >> w;
      ps->setWeight(i, w);
    }
    
    for(i=0; i<N; i++){
      float x,y,z;
      in >> x >> y >> z;
      ps->setNormal(i, x, y, z);
     }
    
    cout << "File read is done." << endl;
    
    in.close();
    
    return ps;
  }
  
  
  /*
  static writePwncFile( PointSet* ps, char* name){
    ofstream out(name);
    
    cout << "File write is started" << endl;
    
    int N = ps->pointN;
    out << N << endl;
    
    cout << N << " points" << endl;
    
    int i;
    for(i=0; i<N; i++){
      Point* p = ps->points[i];
      out << p->x << " "
          << p->y << " " 
          << p->z << endl;
    }
    for(i=0; i<N; i++){
      Point* p = ps->points[i];
      out << p->w << endl;
    }
    
    for(i=0; i<N; i++){
      Point* p = ps->points[i];
      out << p->nx << " "
          << p->ny << " "
          << p->nz << endl;
     }
    
    cout << "File write is done." << endl;
    
    out.close();
  }*/
  
  static CPointSet* readPwnFile(char* name){
    ifstream in(name);
    
    cout << "Pwn File read is started" << endl;
    
    int N;
    in >> N;
    
    cout << N << " points" << endl;
    
    CPointSet* ps = new CPointSet(N);
    
    int i;
    for(i=0; i<N; i++){
      float x,y,z;
      in >> x >> y >> z;
      ps->setPoint(i, x, y, z);
      ps->setWeight(i, 1);
    }
    
    for(i=0; i<N; i++){
      float x,y,z;
      in >> x >> y >> z;
      ps->setNormal(i, x, y, z);
    }
    
    cout << "File read is done." << endl;
    
    in.close();
    
    return ps;
  }
  
  static void writePwnFile(CPointSet* ps, char* name){
    ofstream out(name);
    
    cout << "File write is started." << endl;
    
    int N = ps->m_pointN;
    out << N << endl;
    
    int i;
    float ** point = ps->m_point;
    for(i=0; i<N; i++){
      float *p = point[i];
      out << p[0] << " " << p[1] << " " << p[2] << endl;
    }
    
    float (*normal)[3] = ps->m_normal;
    for(i=0; i<N; i++){
      float* n = normal[i];
      out << n[0] << " " << n[1] << " " << n[2] << endl;
    }
    
    out.close();
  }
  
  static void writeMeshFile(BallMesh* mesh, char* name){
    ofstream out(name);
    
    cout << "File write is started." << endl;
    
    int vN = mesh->vertexN;
    int fN = mesh->faceN;
    out << vN << endl;
    out << fN << endl;
    int i;
    for(i=0; i<vN; i++){
      float *v = mesh->vertex[i];
      out << v[0] << " "
          << v[1] << " "
          << v[2] << endl;
    }
    
    for(i=0; i<fN; i++){
      int* f = mesh->face[i];
      out << "3 " << f[0] << " "
                  << f[1] << " "
                  << f[2] << endl;
    }
    
    out.close();
    
    printf("File write is done.        \n\n");
  }
  
  static void writeBallFile(Ball** bas, int baN, char* name){
    ofstream out(name);
    
    printf("File write is started.\n");
    
    out << baN << endl;
    
    cout << baN <<  " balls" << endl;;
    
    int i;
    for(i=0; i<baN; i++){
      Ball* b = bas[i];
      out << b->cx << " " << b->cy << " " <<  b->cz
        << " " << "0" << " " << "0" << " " <<  "0"
          << " " << "0" << " " <<  "0" << " " <<  "0"
            << " " << b->px << " " << b->py << " " <<  b->pz
              << " " << "0" << " " << b->r << endl;
    }
    out.close();
    
    cout << "File write is done." << endl;
  }
};

#endif
