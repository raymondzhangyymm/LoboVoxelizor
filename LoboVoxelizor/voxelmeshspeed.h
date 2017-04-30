#ifndef VOXELMESHSPEED_H
#define VOXELMESHSPEED_H

#include "voxelmesh.h"

class VoxelMeshSpeed : public VoxelMesh
{
public:
    VoxelMeshSpeed(TriangleMesh* pmesh, int grid_dens, int method_);

    bool updateGridDensity(int new_density, int method_);
    void computeVoxelMesh();
    bool allocateVoxelMap();

    inline void markPointSpeed(const long long vox_idx);
    inline bool equaltoMarkerSpeed(const long long vox_idx);
};

#endif // VOXELMESHSPEED_H
