#ifndef VOXELMESH
#define VOXELMESH

#include "geometry.h"
#include <QElapsedTimer>
#include <stack>
#include <thread>

class VoxelMesh
{
public:
    VoxelMesh(TriangleMesh* pmesh, int grid_dens, int method_);
    ~VoxelMesh();

    TriangleMesh *triangle_mesh;
    int grid_density;
    int voxelization_method;

    // The marker map. Each voxel has 2 bits indicator.
    // i.e. 0 = unvixited, 1 = surface, 2 = interior, 3 = exterior.
    unsigned char *voxel_map;
    long long voxel_map_size;

    // The length of the edge of a voxel, the max and min coordinates of the triangle mesh
    double voxel_dim;
    double coordinate_min[3];
    double coordinate_max[3];

    bool isFillingHole;
    bool isMultiThread;

    double time_surface;  // time for voxelizing the surface
    double time_interior;  // time for voxelizing the interior

    long long num_element_surface_speedtest;  // the number of elements found by surface voxelization
    double time_surface_speedtest;  // time for voxelizing the surface

    //-----------------------------------------------------------------------------------------
    // The volumatric mesh is corresponding to the input triangle mesh.
    CubicMesh volume_mesh;

    // Clear the lists.
    void cleanAll();

    // Update the grid density.
    // Return false if no updating is needed.
    virtual bool updateGridDensity(int new_density, int method_);

    // Compute the voxel mesh.
    virtual void computeVoxelMesh();

    virtual bool allocateVoxelMap();
    void setzeroVoxelMap();

    void voxelizeTriangleMesh();
    void voxelizeSubTriangleMesh(int idx);

    // the aux nodes and triangles are generated by dividing the type 6,7 or8 triangle
    vector<TriangleFace> aux_triangle_list;

    // the sorted triangles are pre-setup and ready to process voxelization
    vector<TriangleFace*> sorted_triangle_list;

    // for multi threads
    thread th[4];
//    bool th_finished[4];
    void sleep1(int ss, int id);
    void voxelizeOneFourthSortedTrianglesSpeedSAT(int start_id);
    void voxelizeOneFourthSortedTrianglesSpeedPan11(int start_id);
    void voxelizeOneFourthSortedTrianglesSpeedFLT(int start_id);
    void voxelizeOneFourthSortedTrianglesSpeedINT(int start_id);

    // SAT method by using pre-setup of the triangle sequence
    void setupSortedTriangleListSATPan11();
    void markSortedFaceSATSpeed(TriangleFace* facep);
    void markSortedFaceSAT(TriangleFace* facep, int marker);
    void markSortedFacePan11Speed(TriangleFace* facep);
    void markSortedFacePan11(TriangleFace* facep, int marker);

    // FLT and INT method by using pre-setup of the triangle sequence
    void setupSortedTriangleListFLTINT();
    void markSortedFaceFLTSpeed(TriangleFace* facep);
    void markSortedFaceINTSpeed(TriangleFace* facep);

    // Mark a face/triangle using floating scanlines.
    void markFaceByScanline(TriangleFace* facep, int marker);
    void markFaceByScanline1D(TriangleFace* facep, int marker);
    void markFaceByScanlineFLT(TriangleFace* facep, int marker);
    void markTriangle2DYZ(const double p0[], const double p1[], const double p2[], const int xi);
    void markTriangle2DZX(const double p0[], const double p1[], const double p2[], const int yi);
    void markTriangle2DXY(const double p0[], const double p1[], const double p2[], const int zi);

    // Mark a face/triangle using integer scanlines.
    void markFaceByScanlineInteger(TriangleFace* facep, int marker);

    void markLine2D(const double start_x, const double start_y, const double end_x, const double end_y,
                    int axis_x, int axis_y, int axis_z, int slice_idx, int marker);
    void markLine2DYZ(const double p0[], const double p1[], const int xi);
    void markLine2DZX(const double p0[], const double p1[], const int yi);
    void markLine2DXY(const double p0[], const double p1[], const int zi);

    // Mark a face/triangle using SAT-based method.
    void markFaceBySATMethod(TriangleFace* facep, int marker);

    //----------------------------------------------------------------------------
    // Interior voxelization.
    //----------------------------------------------------------------------------
    void fillHole(int marker);
    void markExteriorPart(int marker);
    long long boundaryMarked();
    void bfsMark(long long idx, int marker);

    //----------------------------------------------------------------------------
    // For computing the voxel mesh after voxelization.
    //----------------------------------------------------------------------------
    void generateELEMesh(vector<Node*> *node_list_p,
                         vector<CubicElement*> *element_list_p);  //Generate element (volumetric) mesh.
    void buildOBJMeshList();  //Generate obj surface mesh.
    void computeNormalizedCoordinates();  //Compute normalized coordinates.

    //----------------------------------------------------------------------------
    // Mark a slice.
    //----------------------------------------------------------------------------
    void markSliceTriangle(const double pp0_x, const double pp0_y, const double pp1_x, const double pp1_y,
                           const double pp2_x, const double pp2_y,
                           int dim_x, int dim_y, int dim_z, int slice_idx, int marker);
    void markSliceTrapezoid(const double pp0_x, const double pp0_y, const double pp1_x, const double pp1_y,
                            const double pp2_x, const double pp2_y, const double pp3_x, const double pp3_y,
                            int dim_x, int dim_y, int dim_z, int slice_idx, int marker);

    //----------------------------------------------------------------------------
    // 3D line voxelization.
    //----------------------------------------------------------------------------
    // mark a 3D line using FLT computation, return the array of voxel indices and the size.
    int mark3DLine(const double start_point[], const double end_point[],
                   const int start_point_vox[], const int end_point_vox[],
                   int voxsequ[][3], int marker);

    // mark a 3D line using FLT computation.
    void mark3DLine(const double start_point[], const double end_point[],
                    const int start_point_vox[], const int end_point_vox[],
                    int marker);

    // mark a 3D central line using INT computation, return the array of voxel indices and the size.
    int mark3DLineInteger(const int start_point_vox[], const int end_point_vox[],
                          int voxsequ[][3], int marker);

    //----------------------------------------------------------------------------
    // 2D line voxelization.
    //----------------------------------------------------------------------------
    // mark a 2D line using FLT computation, add the array of voxel indices to pixsequ, return the total size.
    int mark2DLine(const double start_point[], const double end_point[],
                   int axis_x, int axis_y, int axis_z, int slice_idx, int marker,
                   int pixsequ[][3], int start_pix_idx);

    // compute a 2D line pixel array using INT computation, return the size.
    int mark2DLineInteger(const int start_pix_x, const int start_pix_y, const int end_pix_x, const int end_pix_y,
                          int sequ_pixel[][2]);


    //----------------------------------------------------------------------------
    // For speed test of the surface voxelization
    //----------------------------------------------------------------------------
    void markFaceByScanlineSpeed(TriangleFace* facep);
    void markFaceByScanlineIntegerSpeed(TriangleFace* facep);
    void markFaceBySATMethodSpeed(TriangleFace* facep);
    void mark3DLineSpeed(const double start_point[], const double end_point[],
                         const int start_point_vox[], const int end_point_vox[]);
    int mark3DLineSpeed(const double start_point[], const double end_point[],
                        const int start_point_vox[], const int end_point_vox[], int voxsequ[][3]);
    void mark2DLineSpeed(const double start_x, const double start_y, const double end_x, const double end_y,
                         int axis_x, int axis_y, int axis_z, int slice_idx);
    virtual inline void markPointSpeed(const long long vox_idx);
    virtual inline bool equaltoMarkerSpeed(const long long vox_idx);

    void markFaceByScanlineIntegerDetailed(TriangleFace* facep, int marker);
    //----------------------------------------------------------------------------
    // 2D line voxelization.
    //----------------------------------------------------------------------------
    // mark a 2D line using FLT computation, return the array of voxel indices for display only.
    void mark2DLine(const double start_x, const double start_y, const double end_x, const double end_y,
                    int axis_x, int axis_y, int axis_z, int slice_idx, int marker,
                    vector<int*>* p_sequ_voxel);

    // mark a 2D line using INT computation, return the array of voxel indices for display only.
    void mark2DLineInteger(const int start_pix_x, const int start_pix_y, const int end_pix_x, const int end_pix_y,
                           int axis_x, int axis_y, int axis_z, int slice_idx, int marker,
                           vector<int*>* p_sequ_voxel);

    //----------------------------------------------------------------------------
    // Mark a voxel.
    //----------------------------------------------------------------------------
    inline bool equaltoMarkerByteMap(const long long vox_idx, int marker);
    inline bool equaltoMarker(const long long vox_idx, int marker);
    inline bool equaltoMarkerOneBitMap(const long long vox_idx, int marker);
    inline void markPointByteMap(const long long vox_idx, int marker);
    inline void markPoint(const long long vox_idx, int marker);
    inline void markPointOneBitMap(const long long vox_idx, int marker);

    //----------------------------------------------------------------------------
    // Heavily used inline functions.
    //----------------------------------------------------------------------------
    inline void computeVoxIdx(int pp_vox[], const double pp[]);
    inline int computeVoxOneIdx(double coor, int axis_idx);
    inline void computeDomiAxis(int &domi_x, int &domi_y, int &domi_z,
                                const double pp0[], const double pp1[], const double pp2[]);
    inline int computeDomiAxis(const double pp0[], const double pp1[], const double pp2[]);
    inline void sortVertByDomiAxis(double pp0[], double pp1[], double pp2[], int dd);
    inline void assignPoint(double ary[], const double pp[]);

    inline void getMinMaxValueINT(int &mi, int &ma, int aa[]);
    inline void getMinMaxValueFLT(double &mi, double &ma, double aa[]);
    inline int getBoundingBoxType(int vox0[], int vox1[], int vox2[]);
    void sortVertices2D(double v0[], double v1[], double v2[], int axis_x, int axis_y);

    inline bool getRangePa11(double &r1, double &r2,
                             double n0, double u0, double n1, double u1, double n2, double u2);

    long long scanline_count;
    long long voxel_marking_count;
    int op_atom, op_cmp, op_add, op_mult;
};

#endif // VOXELMESH

