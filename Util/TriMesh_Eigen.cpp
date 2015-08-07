#include"TriMesh_Eigen.h"
void TriMesh_Eigen::clear()
{
    vertices.resize(3,0);
    colors.resize(3,0);
    normals.resize(3,0);
    faces.resize(3,0);
}
void TriMesh_Eigen::write(const char* filename)
{
    FILE* fout = fopen(filename, "w");
    for(int i=0; i<vertices.cols(); i++)
    {
        fprintf(fout, "v %lf %lf %lf\n", vertices(0,i), vertices(1,i), vertices(2,i));
    }
    fclose(fout);
}
