#ifndef MY_GEOMETRY_H
#define MY_GEOMETRY_H

#include <iostream>
#include <fstream>
#include <vector>
#include <map>
#include <assert.h>
#include <time.h>
#include "zVector.h"

#define PI 3.1415926536

#define RED0 232.0/255.0
#define RED1 12.0/255.0
#define RED2 0.0
#define GREEN0 50.0/255.0
#define GREEN1 245.0/255.0
#define GREEN2 150.0/255.0
#define BLUE0 0.0
#define BLUE1 162.0/255.0
#define BLUE2 232.0/255.0

#define LONGEST_EDGE 4096
#define MAXIMUM_INTEGER 0x7FFFFFFF

#define INIT_DENSITY 32
#define HIGH_GRID_DENSITY 2048
#define WORKING_DIR "E:/Qt_workplace"

using namespace std;

class Node;
class TriangleFace;
class TriangleMesh;
class QuadFace;
class QuadMesh;
class CubicElement;
class CubicMesh;

typedef vector<Node*>::iterator iter_pnode;
typedef vector<TriangleFace*>::iterator iter_ptriangleface;

typedef vector<Node*>::const_iterator const_iter_pnode;
typedef vector<TriangleFace*>::const_iterator const_iter_ptriangleface;
typedef vector<CubicElement*>::const_iterator const_iter_pcubicelement;

class Node
{
public:
    Node(){}
    ~Node(){}

    long long index;  //regular index

//    double ori_coordinate[3];
//    double norm_coordinate[3];
    zVec3 ori_coordinate;
    zVec3 norm_coordinate;

    bool isOnSurface;  //the node is on surface.
    long long idx_surface;  //valid if on surface.

    bool isOnSurfaceVoxelization;  //the node is on the surface voxelization.
    long long idx_surface_ele;  //valid if on the surface voxelization.

    //For the vertex inside a cubic element.
    long long idx_element;
    double brc_coordinate[3]; //Barycentric coordinates.
//    zVec3 brc_coordinate; //Barycentric coordinates.
};

class TriangleFace
{
public:
    TriangleFace(){}
    ~TriangleFace(){}

    // The basic information of the triangle
    Node *node0, *node1, *node2;
    long long idx0, idx1, idx2;  // the index of the nodes in the node list

    double norm[3];  // the norm of the triangle, i.e. crossproduct(n1-n0, n2-n0)
    double norm_abs[3];  // the absoluted norm of the triangle
    int norm_sign[3]; // the sign vector of the norm

    // These are the three vertices used to process the voxelization.
    // In SAT method, they are copies of the nodes' ori_coordinates.
    // In Scanline method, they are further sorted as the following:
    // In 2D cases, sorted such that for edges (v2-v1)>(v1-v0)>(v0-v2).
    // In 3D cases, sorted along dominant axis such that, e.g. v2[2]>v1[2]>v0[2] for z axis.
    double v0[3], v1[3], v2[3];
    int vox0[3], vox1[3], vox2[3]; // the voxel id corresponding to the above vertices.

    // the minumum and maximun voxel within the bounding box
    int voxel_min[3], voxel_max[3];

    // the mapped axes, axis z represents the dominant axis.
    int axis_x, axis_y, axis_z;

    // Nine triangle types in total.
    // 1D cases - 0,1,2 : extending only along x, y or z axis.
    // 2D cases - 3,4,5 : extending only within xy, yz or zx plane.
    // 3D cases - 6,7,8 : taking dominant axis x,y or z axis.
    // Note: the triangle type also depenses on the grid resolution.
    int bounding_box_type;

    // If the triangle is type 6, 7 or 8, it will be divided to two parts by a line,
    // which is parallel to yz, zx or xy plane and passing the middle vertex v1.
    // The subtype is 0 if v0 is lower than v1v2, 1 if v0 is higher than v1v2.
    int bounding_box_subtype;

    // the primary scanlines and the scanlines within silces.
    vector<double*>* sequ_primscanline;  // each element is a double[6], i.e. two points
    vector<double*>* sequ_scanline;  // each element is a double[6], i.e. two points

    // the normalized scanlines for drawing.
    vector<double*>* norm_sequ_primscanline;
    vector<double*>* norm_sequ_scanline;

    // the voxel index sequence associated with the scanlines.
    vector<vector<int*>*>* sequ_primscanvoxel;  // each element is a sequence of int[3], i.e. the voxels
    vector<vector<int*>*>* sequ_scanvoxel;
};

class TriangleMesh
{
public:
    TriangleMesh();
    ~TriangleMesh();

    // The node list is shared by all sub mesh.
    vector<Node*> node_list;

    // The mesh lists contains a couple of sub meshes.
    vector<vector<TriangleFace*>> mesh_lists;

    long long num_node;      // The total number of nodes.
    long long num_face;      // The total number of triangle faces.
    long long num_mesh;      // The total number of sub meshs.

//    double max_coordinates[3];  // the max coordinates of all sub meshs
//    double min_coordinates[3];  // the min coordinates of all sub meshs
    double max_x, max_y, max_z;  // the max coordinates of all sub meshs
    double min_x, min_y, min_z;  // the min coordinates of all sub meshs
    double max_dimension;  // the length of the longest edge of the bounding box
    int max_axis; // the axis of the longest edge.

    double norm_max_x, norm_max_y, norm_max_z;  // the max normalized coordinates
    double norm_min_x, norm_min_y, norm_min_z;  // the min normalized coordinates
    double norm_max_dimension;  // the length of the longest edge of the bounding box

    void loadOBJFile(const char* obj_filename);
    void setBoundingBox();
    void setBoundingBox(const double ma_x, const double ma_y, const double ma_z,
                        const double mi_x, const double mi_y, const double mi_z);
};

class QuadFace
{
public:
    QuadFace(){}
    ~QuadFace(){}

    Node *node0, *node1, *node2, *node3;
    long long idx0, idx1, idx2, idx3;
};

class QuadMesh
{
public:
    QuadMesh(){}
    ~QuadMesh(){}

    vector<Node*> node_list;
    //vector<QuadFace*> face_list;
    vector<vector<QuadFace*>> mesh_lists;

    long long num_node;
    long long num_mesh;
    long long num_face;

    void clean()
    {
        int size = node_list.size();
        for (int i=0; i<size; ++i)
        {
            delete node_list[i];
        }
        size = mesh_lists.size();
        for (int i=0; i<size; ++i)
        {
            vector<QuadFace*> *p_mesh = &mesh_lists[i];
            int size_1 = p_mesh->size();
            for(int j=0; j<size_1; ++j)
            {
                delete p_mesh->at(j);
            }
        }

        node_list.clear();
        mesh_lists.clear();
    }

    void updateStatistic()
    {
        num_node = node_list.size();

        num_mesh = mesh_lists.size();
        num_face = 0;
        for (int i=0; i<num_mesh; ++i)
        {
            vector<QuadFace*> *p_mesh = &mesh_lists[i];
            num_face += p_mesh->size();
        }
    }
};

//This is axis-aligned volumentric cubic element class.
class CubicElement
{
public:
    CubicElement(){}
    ~CubicElement(){}

    //Each cubic element has 8 vertecies.
    Node *node0, *node1, *node2, *node3;
    Node *node4, *node5, *node6, *node7;

    //The index of each vertex in the element list of the cubic mesh.
    long long idx0, idx1, idx2, idx3;
    long long idx4, idx5, idx6, idx7;

    //regular index in the element list.
    long long index;

    //regular index in the element list.
    long long index_surface;

    //The position in vox map.
    long long vox_idx;

    //The 6 Faced neighbor pointers.
    CubicElement* x_pos;
    CubicElement* x_neg;
    CubicElement* y_pos;
    CubicElement* y_neg;
    CubicElement* z_pos;
    CubicElement* z_neg;

    //The flag indicates if the element is on the surface.
    bool isOnSurfaceVoxelization;

    //The flag indicates if the element is a collision.
    bool isCollision;

    //The flag indicates if the element is a collision.
    unsigned char hitting_times;
};

//This is volumetric cubic(i.e. voxel) mesh class.
class CubicMesh
{
public:
    CubicMesh(){}
    ~CubicMesh(){}

    vector<Node*> node_list;
    long long num_node;

    vector<vector<CubicElement*>> element_group_lists;
    long long num_group;
    long long num_element;  // the number of total elements
    long long num_element_surface;  // the number of elements found by surface voxelization
    long long num_element_interior;  // the number of elements found by solid voxelization

    vector<Node*> surface_node_list;
    long long num_node_surface;  // the number of nodes of the surface voxelization.

    vector<vector<QuadFace*>> surface_mesh_lists;
    long long num_face;  // the number of total faces of the surface voxelization

    void clean()
    {
        // delete all nodes
        int size = node_list.size();
        for (int i=0; i<size; ++i)
        {
            delete node_list[i];
        }

        // delete all elements
        size = element_group_lists.size();
        for (int i=0; i<size; ++i)
        {
            vector<CubicElement*> *p_elegroup = &element_group_lists[i];
            int size_1 = p_elegroup->size();
            for(int j=0; j<size_1; ++j)
            {
                delete p_elegroup->at(j);
            }
        }

        // delete all surface meshes
        size = surface_mesh_lists.size();
        for (int i=0; i<size; ++i)
        {
            vector<QuadFace*> *p_mesh = &surface_mesh_lists[i];
            int size_1 = p_mesh->size();
            for(int j=0; j<size_1; ++j)
            {
                delete p_mesh->at(j);
            }
        }

        node_list.clear();
        element_group_lists.clear();

        surface_node_list.clear();
        surface_mesh_lists.clear();
    }

    void updateStatistic()
    {
        num_node = node_list.size();
        num_node_surface = surface_node_list.size();
//        num_node_surface = 0;
//        for (int i=0; i<num_node; ++i)
//        {
//            if (node_list[i]->isOnSurfaceVoxelization)
//            {
//                node_list[i]->idx_surface_ele = num_node_surface++;
//            }
//        }

        num_group = element_group_lists.size();
        num_face = 0;
        num_element = 0;
        num_element_surface = 0;
        num_element_interior = 0;
        for (int i=0; i<num_group; ++i)
        {
            vector<CubicElement*> *p_elegroup = &element_group_lists[i];
            int size = p_elegroup->size();
            num_element += size;
            for (int j=0; j<size; ++j)
            {
                if (p_elegroup->at(j)->isOnSurfaceVoxelization)
                {
                    num_element_surface++;
                }
                else
                {
                    num_element_interior++;
                }
            }

            vector<QuadFace*> *p_surmesh = &surface_mesh_lists[i];
            num_face += p_surmesh->size();
        }
    }
};

# endif

