#ifndef VOXELMESHSPEEDTEST_H
#define VOXELMESHSPEEDTEST_H

#include "voxelmesh.h"

class VoxelMeshSpeedTest : public VoxelMesh
{
public:
    VoxelMeshSpeedTest(TriangleMesh* pmesh, int grid_dens, int method_);
    ~VoxelMeshSpeedTest(){}

    // For speed test of surface voxelization only
    void computeVoxelMeshSpeed(int method_idx);

private:
    // For speed test of surface voxelization only
    void markFaceByScanlineSpeed(TriangleFace* facep);
    void markFaceByScanlineIntegerSpeed(TriangleFace* facep);
    void markFaceBySATMethodSpeed(TriangleFace* facep);

    void mark3DLineSpeed(const double start_point[], const double end_point[],
                         const int start_point_vox[], const int end_point_vox[]);
    int mark3DLineSpeed(const double start_point[], const double end_point[],
                        const int start_point_vox[], const int end_point_vox[], int voxsequ[][3]);
    void mark2DLineSpeed(const double start_x, const double start_y, const double end_x, const double end_y,
                         int axis_x, int axis_y, int axis_z, int slice_idx);

    //----------------------------------------------------------------------------
    // 2D line voxelization.
    //----------------------------------------------------------------------------
    // mark a 2D line using FLT computation.
    void mark2DLine(const double start_x, const double start_y, const double end_x, const double end_y,
                    int axis_x, int axis_y, int axis_z, int slice_idx, int marker);

    // mark a 2D line using INT computation.
    void mark2DLineInteger(const int start_pix_x, const int start_pix_y, const int end_pix_x, const int end_pix_y,
                           int axis_x, int axis_y, int axis_z, int slice_idx, int marker);

    //----------------------------------------------------------------------------
    // Heavily used functions for checking and marking a voxel.
    //----------------------------------------------------------------------------
    inline void markPoint(const long long vox_idx, int marker);

    //----------------------------------------------------------------------------
    // Heavily used inline functions.
    //----------------------------------------------------------------------------
    inline void computeVoxIdx(int pp_vox[], const double pp[]);
    inline int computeVoxOneIdx(double coor, int axis_idx);
    inline int computeDomiAxis(const double pp0[], const double pp1[], const double pp2[]);
    inline void sortVertByDomiAxis(double pp0[], double pp1[], double pp2[], int dd);
    inline void assignPoint(double ary[], const double pp[]);
};

#endif // VOXELMESHSPEEDTEST_H
