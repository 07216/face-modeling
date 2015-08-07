#ifndef SHOWPARAMETER_H
#define SHOWPARAMETER_H

enum ShowPointCloudFormat
{
    VERTICES_POINTCLOUD,
    FACES_POINTCLOUD,
    NONE_POINTCLOUD
};
enum ShowFaceMeshFormat
{
    FACES_FACEMESH,
    TEXTURE_FACEMESH,
    NONE_FACEMESH
};

typedef struct
{
   ShowPointCloudFormat pointCloudFormat;
   ShowFaceMeshFormat faceMeshFormat;
}ShowConfig;

#endif // SHOWPARAMETER_H
