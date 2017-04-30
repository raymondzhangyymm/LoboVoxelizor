#include "voxelmesh.h"
#include <Windows.h>

//===============================================================
// Constructor and destructor
//===============================================================
VoxelMesh::VoxelMesh(TriangleMesh* p_mesh, int grid_dens, int method_)
{
    triangle_mesh = p_mesh;
    grid_density = grid_dens;
    voxelization_method = method_;

    coordinate_min[0] = p_mesh->min_x;
    coordinate_min[1] = p_mesh->min_y;
    coordinate_min[2] = p_mesh->min_z;

    voxel_map = NULL;

    isFillingHole = true;
    isMultiThread = true;
}

VoxelMesh::~VoxelMesh()
{
    volume_mesh.clean();

    if (voxel_map)
    {
        delete voxel_map;
    }
    voxel_map = NULL;
}

//================================================================================
// Update the grid density. The voxel mesh must be re-computed.
//================================================================================
bool VoxelMesh::updateGridDensity(int new_density, int new_method)
{
    // Return if the setting did not change.
    if (new_density == grid_density &&
        new_method == voxelization_method)
    {
        return false;
    }

    grid_density = new_density;
    voxelization_method = new_method;

    computeVoxelMesh();

    return true;
}

//================================================================================
// Generate the voxel mesh.
//================================================================================
void VoxelMesh::computeVoxelMesh()
{
    // return if no triangle mesh or invalid grid density.
    if (!triangle_mesh || grid_density <= 0)
    {
        return;
    }

    // determine the length of the edge of a voxel.
    voxel_dim = triangle_mesh->max_dimension / double(grid_density);

    // combine the collection of the triangle candidates
    sorted_triangle_list.clear();
    for (int i=0; i<triangle_mesh->num_mesh; ++i)
    {
        sorted_triangle_list.insert(sorted_triangle_list.end(),
                                    triangle_mesh->mesh_lists[i].begin(),
                                    triangle_mesh->mesh_lists[i].end());
    }

    if (grid_density <= 2048)
    {
        // Use Two-Bit-Map, i.e. two bits for one voxel.
        long long sizearray = grid_density * grid_density *(long long)grid_density;
        voxel_map_size = (sizearray>>2) + 1;
        allocateVoxelMap();

        int num_submeshes = triangle_mesh->mesh_lists.size();
        volume_mesh.clean();
        volume_mesh.element_group_lists.resize(num_submeshes);

        voxelizeTriangleMesh();
        generateELEMesh(&volume_mesh.node_list, &volume_mesh.element_group_lists[0]);

        //    for (int i=0; i<num_submeshes; ++i)
        //    {
        //        voxelizeSubTriangleMesh(i);
        //        generateELEMesh(&volume_mesh.node_list, &volume_mesh.element_group_lists[i]);
        //    }

        // Build the obj meshes.
        buildOBJMeshList();

        // Compute the normalized coordinates of all the nodes.
        computeNormalizedCoordinates();

        volume_mesh.updateStatistic();
    }
}

bool VoxelMesh::allocateVoxelMap()
{
    // delete the old map if applied
    if (voxel_map)
    {
        delete voxel_map;
    }

    // try memory allocation.
    try
    {
        voxel_map = new unsigned char[voxel_map_size];
    }
    catch(...)
    {
        cout << "Grid density " << grid_density
             <<"\nAllocating the voxel map of " << voxel_map_size << " bytes failed!" << endl;
        voxel_map = NULL;
        return false;
    }
    cout << "Grid density " << grid_density
         << "\nAllocating the voxel map of " << voxel_map_size << " bytes succeeded." << endl;

    // Clear the map.
    memset(voxel_map, 0, sizeof(unsigned char) * voxel_map_size);
    return true;
}

void VoxelMesh::setzeroVoxelMap()
{
    memset(voxel_map, 0, sizeof(unsigned char) * voxel_map_size);
}

// Setup each triangle for SAT based method
// and build a list sorted by their bounding box type.
void VoxelMesh::setupSortedTriangleListSATPan11()
{
    // set up each triangle
    int size = sorted_triangle_list.size();
    for (int i=0; i<size; ++i)
    {
        TriangleFace *p_tri = sorted_triangle_list[i];

        // copy the vertices
        p_tri->v0[0] = p_tri->node0->ori_coordinate[0];
        p_tri->v0[1] = p_tri->node0->ori_coordinate[1];
        p_tri->v0[2] = p_tri->node0->ori_coordinate[2];
        p_tri->v1[0] = p_tri->node1->ori_coordinate[0];
        p_tri->v1[1] = p_tri->node1->ori_coordinate[1];
        p_tri->v1[2] = p_tri->node1->ori_coordinate[2];
        p_tri->v2[0] = p_tri->node2->ori_coordinate[0];
        p_tri->v2[1] = p_tri->node2->ori_coordinate[1];
        p_tri->v2[2] = p_tri->node2->ori_coordinate[2];

        // compute the voxel
        computeVoxIdx(p_tri->vox0, p_tri->v0);
        computeVoxIdx(p_tri->vox1, p_tri->v1);
        computeVoxIdx(p_tri->vox2, p_tri->v2);

        // get the voxel id for each vertex
        int *v0 = p_tri->vox0;
        int *v1 = p_tri->vox1;
        int *v2 = p_tri->vox2;

        // determine the bounding box type
        if (*v0 == *v1 && *v1 == *v2)
        {
            // must be type 1 or 2 or 4
            if (*(v0+1) == *(v1+1) && *(v1+1) == *(v2+1))
            {
                // must be type 2, i.e. only extending along z axis
                p_tri->bounding_box_type = 2;

                p_tri->voxel_min[0] = p_tri->voxel_max[0] = *v0;
                p_tri->voxel_min[1] = p_tri->voxel_max[1] = *(v0+1);
                int aa[3];
                aa[0] = *(v0+2);
                aa[1] = *(v1+2);
                aa[2] = *(v2+2);
                getMinMaxValueINT(p_tri->voxel_min[2], p_tri->voxel_max[2], aa);
            }
            else if (*(v0+2) == *(v1+2) && *(v1+2) == *(v2+2))
            {
                // must be type 1, i.e. only extending along y axis
                p_tri->bounding_box_type = 1;

                p_tri->voxel_min[0] = p_tri->voxel_max[0] = *v0;
                p_tri->voxel_min[2] = p_tri->voxel_max[2] = *(v0+2);
                int aa[3];
                aa[0] = *(v0+1);
                aa[1] = *(v1+1);
                aa[2] = *(v2+1);
                getMinMaxValueINT(p_tri->voxel_min[1], p_tri->voxel_max[1], aa);
            }
            else
            {
                // must be type 4, i.e. only extending along yz plane
                p_tri->bounding_box_type = 4;

                // compute the normal of the triangle
                zVec3 n0 = p_tri->node0->ori_coordinate;
                zVec3 n1 = p_tri->node1->ori_coordinate;
                zVec3 n2 = p_tri->node2->ori_coordinate;
                p_tri->norm[0] = (n1.y-n0.y)*(n2.z-n0.z) - (n1.z-n0.z)*(n2.y-n0.y);
                p_tri->norm[1] = 0.0;
                p_tri->norm[2] = 0.0;

                if (p_tri->norm[0] >= 0.0)
                {
                    p_tri->norm_sign[0] = 1;
                }
                else
                {
                    p_tri->norm_sign[0] = -1;
                }

                // compute the max and min voxel
                p_tri->voxel_min[0] = p_tri->voxel_max[0] = *v0;
                int aa[3];
                aa[0] = *(v0+1);
                aa[1] = *(v1+1);
                aa[2] = *(v2+1);
                getMinMaxValueINT(p_tri->voxel_min[1], p_tri->voxel_max[1], aa);
                aa[0] = *(v0+2);
                aa[1] = *(v1+2);
                aa[2] = *(v2+2);
                getMinMaxValueINT(p_tri->voxel_min[2], p_tri->voxel_max[2], aa);
            }
        }
        else if (*(v0+1) == *(v1+1) && *(v1+1) == *(v2+1))
        {
            // must be type 0 or 5
            if (*(v0+2) == *(v1+2) && *(v1+2) == *(v2+2))
            {
                // must be type 0, i.e. only extending along x axis
                p_tri->bounding_box_type = 0;

                p_tri->voxel_min[1] = p_tri->voxel_max[1] = *(v0+1);
                p_tri->voxel_min[2] = p_tri->voxel_max[2] = *(v0+2);
                int aa[3];
                aa[0] = *(v0);
                aa[1] = *(v1);
                aa[2] = *(v2);
                getMinMaxValueINT(p_tri->voxel_min[0], p_tri->voxel_max[0], aa);
            }
            else
            {
                // must be type 5, i.e. only extending along zx plane
                p_tri->bounding_box_type = 5;

                // compute the normal of the triangle
                zVec3 n0 = p_tri->node0->ori_coordinate;
                zVec3 n1 = p_tri->node1->ori_coordinate;
                zVec3 n2 = p_tri->node2->ori_coordinate;
                p_tri->norm[0] = 0.0;
                p_tri->norm[1] = (n1.z-n0.z)*(n2.x-n0.x) - (n1.x-n0.x)*(n2.z-n0.z);
                p_tri->norm[2] = 0.0;

                if (p_tri->norm[1] >= 0.0)
                {
                    p_tri->norm_sign[1] = 1;
                }
                else
                {
                    p_tri->norm_sign[1] = -1;
                }

                // compute the max and min voxel
                p_tri->voxel_min[1] = p_tri->voxel_max[1] = *(v0+1);
                int aa[3];
                aa[0] = *(v0);
                aa[1] = *(v1);
                aa[2] = *(v2);
                getMinMaxValueINT(p_tri->voxel_min[0], p_tri->voxel_max[0], aa);
                aa[0] = *(v0+2);
                aa[1] = *(v1+2);
                aa[2] = *(v2+2);
                getMinMaxValueINT(p_tri->voxel_min[2], p_tri->voxel_max[2], aa);
            }
        }
        else if (*(v0+2) == *(v1+2) && *(v1+2) == *(v2+2))
        {
            // must be type 3, i.e. only extending along xy plane
            p_tri->bounding_box_type = 3;

            // compute the normal of the triangle
            zVec3 n0 = p_tri->node0->ori_coordinate;
            zVec3 n1 = p_tri->node1->ori_coordinate;
            zVec3 n2 = p_tri->node2->ori_coordinate;
            p_tri->norm[0] = 0.0;
            p_tri->norm[1] = 0.0;
            p_tri->norm[2] = (n1.x-n0.x)*(n2.y-n0.y) - (n1.y-n0.y)*(n2.x-n0.x);

            if (p_tri->norm[2] >= 0.0)
            {
                p_tri->norm_sign[2] = 1;
            }
            else
            {
                p_tri->norm_sign[2] = -1;
            }

            // compute the max and min voxel
            p_tri->voxel_min[2] = p_tri->voxel_max[2] = *(v0+2);
            int aa[3];
            aa[0] = *(v0);
            aa[1] = *(v1);
            aa[2] = *(v2);
            getMinMaxValueINT(p_tri->voxel_min[0], p_tri->voxel_max[0], aa);
            aa[0] = *(v0+1);
            aa[1] = *(v1+1);
            aa[2] = *(v2+1);
            getMinMaxValueINT(p_tri->voxel_min[1], p_tri->voxel_max[1], aa);
        }
        else
        {
            // compute the normal of the triangle
            zVec3 n0 = p_tri->node0->ori_coordinate;
            zVec3 n1 = p_tri->node1->ori_coordinate;
            zVec3 n2 = p_tri->node2->ori_coordinate;
            zVec3 norm = zVec3::crossproduct(n1-n0, n2-n0);
            p_tri->norm[0] = norm.x;
            p_tri->norm[1] = norm.y;
            p_tri->norm[2] = norm.z;

            // must be type 6 or 7 or 8, determined by the dominant direction.
            if (p_tri->norm[0] > 0.0)
            {
                p_tri->norm_abs[0] = p_tri->norm[0];
                p_tri->norm_sign[0] = 1;
            }
            else if (p_tri->norm[0] < 0.0)
            {
                p_tri->norm_abs[0] = -p_tri->norm[0];
                p_tri->norm_sign[0] = -1;
            }
            else
            {
                p_tri->norm_abs[0] = 0.0;
                p_tri->norm_sign[0] = 0;
            }

            if (p_tri->norm[1] > 0.0)
            {
                p_tri->norm_abs[1] = p_tri->norm[1];
                p_tri->norm_sign[1] = 1;
            }
            else if (p_tri->norm[1] < 0.0)
            {
                p_tri->norm_abs[1] = -p_tri->norm[1];
                p_tri->norm_sign[1] = -1;
            }
            else
            {
                p_tri->norm_abs[1] = 0.0;
                p_tri->norm_sign[1] = 0;
            }

            if (p_tri->norm[2] > 0.0)
            {
                p_tri->norm_abs[2] = p_tri->norm[2];
                p_tri->norm_sign[2] = 1;
            }
            else if (p_tri->norm[2] < 0.0)
            {
                p_tri->norm_abs[2] = -p_tri->norm[2];
                p_tri->norm_sign[2] = -1;
            }
            else
            {
                p_tri->norm_abs[2] = 0.0;
                p_tri->norm_sign[2] = 0;
            }

            if (p_tri->norm_abs[0] >= p_tri->norm_abs[1])
            {
                if (p_tri->norm_abs[1] >= p_tri->norm_abs[2])
                {
                    // 0>=1>=2, must be type 6, i.e. dominant at x axis
                    p_tri->bounding_box_type = 6;

                    p_tri->axis_x = 1;
                    p_tri->axis_y = 2;
                    p_tri->axis_z = 0;
                }
                else
                {
                    if (p_tri->norm_abs[0] >= p_tri->norm_abs[2])
                    {
                        // 0>=2>1, must be type 6, i.e. dominant at x axis
                        p_tri->bounding_box_type = 6;

                        p_tri->axis_x = 1;
                        p_tri->axis_y = 2;
                        p_tri->axis_z = 0;
                    }
                    else
                    {
                        // 2>0>=1, must be type 8, i.e. dominant at z axis
                        p_tri->bounding_box_type = 8;

                        p_tri->axis_x = 0;
                        p_tri->axis_y = 1;
                        p_tri->axis_z = 2;
                    }
                }
            }
            else
            {
                if (p_tri->norm_abs[0] >= p_tri->norm_abs[2])
                {
                    // 1>0>=2, must be type 7, i.e. dominant at y axis
                    p_tri->bounding_box_type = 7;

                    p_tri->axis_x = 2;
                    p_tri->axis_y = 0;
                    p_tri->axis_z = 1;
                }
                else
                {
                    if (p_tri->norm_abs[1] >= p_tri->norm_abs[2])
                    {
                        // 1>=2>0, must be type 7, i.e. dominant at y axis
                        p_tri->bounding_box_type = 7;

                        p_tri->axis_x = 2;
                        p_tri->axis_y = 0;
                        p_tri->axis_z = 1;
                    }
                    else
                    {
                        // 2>1>0, must be type 8, i.e. dominant at z axis
                        p_tri->bounding_box_type = 8;

                        p_tri->axis_x = 0;
                        p_tri->axis_y = 1;
                        p_tri->axis_z = 2;
                    }
                }
            }

            int aa[3];
            aa[0] = *(v0);
            aa[1] = *(v1);
            aa[2] = *(v2);
            getMinMaxValueINT(p_tri->voxel_min[0], p_tri->voxel_max[0], aa);
            aa[0] = *(v0+1);
            aa[1] = *(v1+1);
            aa[2] = *(v2+1);
            getMinMaxValueINT(p_tri->voxel_min[1], p_tri->voxel_max[1], aa);
            aa[0] = *(v0+2);
            aa[1] = *(v1+2);
            aa[2] = *(v2+2);
            getMinMaxValueINT(p_tri->voxel_min[2], p_tri->voxel_max[2], aa);
        }
    }
}

void VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedSAT(int start_id)
{
    int size = sorted_triangle_list.size();
    int num_residual = size % 4;
    int size_adj;
    if (start_id < num_residual)
    {
        size_adj = (size/4) * 4 + 1;
    }
    else
    {
        size_adj = (size/4) * 4;
    }

    for (int i=start_id; i<size_adj; i+=4)
    {
        markSortedFaceSATSpeed(sorted_triangle_list[i]);
    }
}

void VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedPan11(int start_id)
{
    int size = sorted_triangle_list.size();
    int num_residual = size % 4;
    int size_adj;
    if (start_id < num_residual)
    {
        size_adj = (size/4) * 4 + 1;
    }
    else
    {
        size_adj = (size/4) * 4;
    }

    for (int i=start_id; i<size_adj; i+=4)
    {
        markSortedFacePan11Speed(sorted_triangle_list[i]);
    }
}

void VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedFLT(int start_id)
{
    int size = sorted_triangle_list.size();
    int num_residual = size % 4;
    int size_adj;
    if (start_id < num_residual)
    {
        size_adj = (size/4) * 4 + 1;
    }
    else
    {
        size_adj = (size/4) * 4;
    }

    for (int i=start_id; i<size_adj; i+=4)
    {
        markSortedFaceFLTSpeed(sorted_triangle_list[i]);
    }
}

void VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedINT(int start_id)
{
    int size = sorted_triangle_list.size();
    int num_residual = size % 4;
    int size_adj;
    if (start_id < num_residual)
    {
        size_adj = (size/4) * 4 + 1;
    }
    else
    {
        size_adj = (size/4) * 4;
    }

    for (int i=start_id; i<size_adj; i+=4)
    {
        markSortedFaceINTSpeed(sorted_triangle_list[i]);
    }
}

void VoxelMesh::markSortedFaceSATSpeed(TriangleFace* facep)
{
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    // extend in xy plane
    else if (facep->bounding_box_type == 3)
    {
        // Setup for the 2D projection test
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[2] >= 0)
        {
            n0[0] = v0.y - v1.y;
            n0[1] = v1.x - v0.x;
            n1[0] = v1.y - v2.y;
            n1[1] = v2.x - v1.x;
            n2[0] = v2.y - v0.y;
            n2[1] = v0.x - v2.x;
        }
        else
        {
            n0[0] = v1.y - v0.y;
            n0[1] = v0.x - v1.x;
            n1[0] = v2.y - v1.y;
            n1[1] = v1.x - v2.x;
            n2[0] = v0.y - v2.y;
            n2[1] = v2.x - v0.x;
        }
        double d0 = - n0[0]*v0.x - n0[1]*v0.y;
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = - n1[0]*v1.x - n1[1]*v1.y;
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = - n2[0]*v2.x - n2[1]*v2.y;
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        int zi = facep->voxel_min[2];
        for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
        {
            for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
            {
                // get the minimum corner
                double p[2];
                p[0] = double(xi) * voxel_dim + coordinate_min[0];
                p[1] = double(yi) * voxel_dim + coordinate_min[1];

                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0 &&
                    n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0 &&
                    n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
                {
                    markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
                }
            }
        }
    }

    // extend in yz plane
    else if (facep->bounding_box_type == 4)
    {
        // Setup for the 2D projection test
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[0] >= 0)
        {
            n0[0] = v0.z - v1.z;
            n0[1] = v1.y - v0.y;
            n1[0] = v1.z - v2.z;
            n1[1] = v2.y - v1.y;
            n2[0] = v2.z - v0.z;
            n2[1] = v0.y - v2.y;
        }
        else
        {
            n0[0] = v1.z - v0.z;
            n0[1] = v0.y - v1.y;
            n1[0] = v2.z - v1.z;
            n1[1] = v1.y - v2.y;
            n2[0] = v0.z - v2.z;
            n2[1] = v2.y - v0.y;
        }
        double d0 = - n0[0]*v0.y - n0[1]*v0.z;
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = - n1[0]*v1.y - n1[1]*v1.z;
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = - n2[0]*v2.y - n2[1]*v2.z;
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        int xi = facep->voxel_min[0];
        for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
        {
            for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
            {
                // get the minimum corner
                double p[2];
                p[0] = double(yi) * voxel_dim + coordinate_min[1];
                p[1] = double(zi) * voxel_dim + coordinate_min[2];

                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0 &&
                    n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0 &&
                    n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
                {
                    markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
                }
            }
        }
    }

    // extend in zx plane
    else if (facep->bounding_box_type == 5)
    {
        // Setup for the 2D projection test
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[1] >= 0)
        {
            n0[0] = v0.x - v1.x;
            n0[1] = v1.z - v0.z;
            n1[0] = v1.x - v2.x;
            n1[1] = v2.z - v1.z;
            n2[0] = v2.x - v0.x;
            n2[1] = v0.z - v2.z;
        }
        else
        {
            n0[0] = v1.x - v0.x;
            n0[1] = v0.z - v1.z;
            n1[0] = v2.x - v1.x;
            n1[1] = v1.z - v2.z;
            n2[0] = v0.x - v2.x;
            n2[1] = v2.z - v0.z;
        }
        double d0 = - n0[0]*v0.z - n0[1]*v0.x;
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = - n1[0]*v1.z - n1[1]*v1.x;
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = - n2[0]*v2.z - n2[1]*v2.x;
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        int yi = facep->voxel_min[1];
        for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
        {
            for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
            {
                // get the minimum corner
                double p[2];
                p[0] = double(zi) * voxel_dim + coordinate_min[2];
                p[1] = double(xi) * voxel_dim + coordinate_min[0];

                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0 &&
                    n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0 &&
                    n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
                {
                    markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
                }
            }
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // Setup for the 2D xy projection test
        double n0_xy[2], n1_xy[2], n2_xy[2];
        if (facep->norm_sign[2] >= 0)
        {
            n0_xy[0] = v0.y - v1.y;
            n0_xy[1] = v1.x - v0.x;
            n1_xy[0] = v1.y - v2.y;
            n1_xy[1] = v2.x - v1.x;
            n2_xy[0] = v2.y - v0.y;
            n2_xy[1] = v0.x - v2.x;
        }
        else
        {
            n0_xy[0] = v1.y - v0.y;
            n0_xy[1] = v0.x - v1.x;
            n1_xy[0] = v2.y - v1.y;
            n1_xy[1] = v1.x - v2.x;
            n2_xy[0] = v0.y - v2.y;
            n2_xy[1] = v2.x - v0.x;
        }
        double d0_xy = - n0_xy[0]*v0.x - n0_xy[1]*v0.y;
        if (n0_xy[0] > 0.0)
        {
            d0_xy += voxel_dim * n0_xy[0];
        }
        if (n0_xy[1] > 0.0)
        {
            d0_xy += voxel_dim * n0_xy[1];
        }
        double d1_xy = - n1_xy[0]*v1.x - n1_xy[1]*v1.y;
        if (n1_xy[0] > 0.0)
        {
            d1_xy += voxel_dim * n1_xy[0];
        }
        if (n1_xy[1] > 0.0)
        {
            d1_xy += voxel_dim * n1_xy[1];
        }
        double d2_xy = - n2_xy[0]*v2.x - n2_xy[1]*v2.y;
        if (n2_xy[0] > 0.0)
        {
            d2_xy += voxel_dim * n2_xy[0];
        }
        if (n2_xy[1] > 0.0)
        {
            d2_xy += voxel_dim * n2_xy[1];
        }

        // Setup for the 2D yz projection test
        double n0_yz[2], n1_yz[2], n2_yz[2];
        if (facep->norm_sign[0] >= 0)
        {
            n0_yz[0] = v0.z - v1.z;
            n0_yz[1] = v1.y - v0.y;
            n1_yz[0] = v1.z - v2.z;
            n1_yz[1] = v2.y - v1.y;
            n2_yz[0] = v2.z - v0.z;
            n2_yz[1] = v0.y - v2.y;
        }
        else
        {
            n0_yz[0] = v1.z - v0.z;
            n0_yz[1] = v0.y - v1.y;
            n1_yz[0] = v2.z - v1.z;
            n1_yz[1] = v1.y - v2.y;
            n2_yz[0] = v0.z - v2.z;
            n2_yz[1] = v2.y - v0.y;
        }
        double d0_yz = - n0_yz[0]*v0.y - n0_yz[1]*v0.z;
        if (n0_yz[0] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[0];
        }
        if (n0_yz[1] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[1];
        }
        double d1_yz = - n1_yz[0]*v1.y - n1_yz[1]*v1.z;
        if (n1_yz[0] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[0];
        }
        if (n1_yz[1] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[1];
        }
        double d2_yz = - n2_yz[0]*v2.y - n2_yz[1]*v2.z;
        if (n2_yz[0] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[0];
        }
        if (n2_yz[1] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[1];
        }

        // Setup for the 2D zx projection test
        double n0_zx[2], n1_zx[2], n2_zx[2];
        if (facep->norm_sign[1] >= 0)
        {
            n0_zx[0] = v0.x - v1.x;
            n0_zx[1] = v1.z - v0.z;
            n1_zx[0] = v1.x - v2.x;
            n1_zx[1] = v2.z - v1.z;
            n2_zx[0] = v2.x - v0.x;
            n2_zx[1] = v0.z - v2.z;
        }
        else
        {
            n0_zx[0] = v1.x - v0.x;
            n0_zx[1] = v0.z - v1.z;
            n1_zx[0] = v2.x - v1.x;
            n1_zx[1] = v1.z - v2.z;
            n2_zx[0] = v0.x - v2.x;
            n2_zx[1] = v2.z - v0.z;
        }
        double d0_zx = - n0_zx[0]*v0.z - n0_zx[1]*v0.x;
        if (n0_zx[0] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[0];
        }
        if (n0_zx[1] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[1];
        }
        double d1_zx = - n1_zx[0]*v1.z - n1_zx[1]*v1.x;
        if (n1_zx[0] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[0];
        }
        if (n1_zx[1] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[1];
        }
        double d2_zx = - n2_zx[0]*v2.z - n2_zx[1]*v2.x;
        if (n2_zx[0] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[0];
        }
        if (n2_zx[1] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[1];
        }

        // dominate along x axis
        if (facep->bounding_box_type == 6)
        {
            // compute the constant of the famulation of the triangle plane
            double nx = facep->norm[0];
            double ny = facep->norm[1];
            double nz = facep->norm[2];

            // multiplied by the sign of the dominant axis in order to
            // determine the range correctly.
            int sign_ny = facep->norm_sign[1] * facep->norm_sign[0];
            int sign_nz = facep->norm_sign[2] * facep->norm_sign[0];

            double d = nx*v1.x + ny*v1.y + nz*v1.z;

            // Iterate all voxels in the 2D yz projection bounding box
            for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
            {
                for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
                {
                    // get the minimum corner
                    double p[2];
                    p[0] = double(yi) * voxel_dim + coordinate_min[1];
                    p[1] = double(zi) * voxel_dim + coordinate_min[2];

                    if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0 &&
                        n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0 &&
                        n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                    {
                        // passed the 2D yz projection test
                        // determine the x range in the x voxel column
                        double y0,z0,y1,z1;
                        if (sign_ny > 0)
                        {
                            y0 = p[0] + voxel_dim;
                            y1 = p[0];
                        }
                        else
                        {
                            y0 = p[0];
                            y1 = p[0] + voxel_dim;
                        }
                        if (sign_nz > 0)
                        {
                            z0 = p[1] + voxel_dim;
                            z1 = p[1];
                        }
                        else
                        {
                            z0 = p[1];
                            z1 = p[1] + voxel_dim;
                        }

                        int x0_id = computeVoxOneIdx((d - ny*y0 - nz*z0) / nx, 0);
                        int x1_id = computeVoxOneIdx((d - ny*y1 - nz*z1) / nx, 0);

                        // iterate the voxels range determined by x0 and x1
                        for (int xi=x0_id; xi<=x1_id; ++xi)
                        {
                            p[0] = double(xi) * voxel_dim + coordinate_min[0];
                            p[1] = double(yi) * voxel_dim + coordinate_min[1];
                            if (n0_xy[0]*p[0] + n0_xy[1]*p[1] + d0_xy >= 0.0 &&
                                n1_xy[0]*p[0] + n1_xy[1]*p[1] + d1_xy >= 0.0 &&
                                n2_xy[0]*p[0] + n2_xy[1]*p[1] + d2_xy >= 0.0)
                            {
                                p[0] = double(zi) * voxel_dim + coordinate_min[2];
                                p[1] = double(xi) * voxel_dim + coordinate_min[0];
                                if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0 &&
                                    n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0 &&
                                    n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                                {
                                    markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
                                }
                            }
                        }
                    }
                }
            }
        }

        // dominate along y axis
        else if (facep->bounding_box_type == 7)
        {
            // compute the constant of the famulation of the triangle plane
            double nx = facep->norm[0];
            double ny = facep->norm[1];
            double nz = facep->norm[2];

            // multiplied by the sign of the dominant axis in order to
            // determine the range correctly.
            int sign_nz = facep->norm_sign[2] * facep->norm_sign[1];
            int sign_nx = facep->norm_sign[0] * facep->norm_sign[1];

            double d = nx*v2.x + ny*v2.y + nz*v2.z;

            // Iterate all voxels in the 2D zx projection bounding box
            for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
            {
                for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
                {
                    // get the minimum corner
                    double p[2];
                    p[0] = double(zi) * voxel_dim + coordinate_min[2];
                    p[1] = double(xi) * voxel_dim + coordinate_min[0];

                    if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0 &&
                        n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0 &&
                        n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                    {
                        // passed the 2D xy projection test
                        // determine the z range in the z voxel column
                        double z0,x0,z1,x1;
                        if (sign_nz > 0)
                        {
                            z0 = p[0] + voxel_dim;
                            z1 = p[0];
                        }
                        else
                        {
                            z0 = p[0];
                            z1 = p[0] + voxel_dim;
                        }
                        if (sign_nx > 0)
                        {
                            x0 = p[1] + voxel_dim;
                            x1 = p[1];
                        }
                        else
                        {
                            x0 = p[1];
                            x1 = p[1] + voxel_dim;
                        }

                        int y0_id = computeVoxOneIdx((d - nz*z0 - nx*x0) / ny, 1);
                        int y1_id = computeVoxOneIdx((d - nz*z1 - nx*x1) / ny, 1);
//                        cout << "nz signnz nx signnx y0_id y1_id = "
//                             << nz << " " << sign_nz << " " << nx << " " << sign_nx << " "
//                             << y0_id << " " << y1_id << endl;

                        // iterate the voxels range determined by y0 and y1
                        for (int yi=y0_id; yi<=y1_id; ++yi)
                        {
                            p[0] = double(xi) * voxel_dim + coordinate_min[0];
                            p[1] = double(yi) * voxel_dim + coordinate_min[1];
                            if (n0_xy[0]*p[0] + n0_xy[1]*p[1] + d0_xy >= 0.0 &&
                                n1_xy[0]*p[0] + n1_xy[1]*p[1] + d1_xy >= 0.0 &&
                                n2_xy[0]*p[0] + n2_xy[1]*p[1] + d2_xy >= 0.0)
                            {
                                p[0] = double(yi) * voxel_dim + coordinate_min[1];
                                p[1] = double(zi) * voxel_dim + coordinate_min[2];
                                if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0 &&
                                    n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0 &&
                                    n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                                {

                                    markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
                                }
                            }
                        }
                    }
                }
            }
        }

        // dominate along z axis
        else if (facep->bounding_box_type == 8)
        {
            // compute the constant of the famulation of the triangle plane
            double nx = facep->norm[0];
            double ny = facep->norm[1];
            double nz = facep->norm[2];

            // multiplied by the sign of the dominant axis in order to
            // determine the range correctly.
            int sign_nx = facep->norm_sign[0] * facep->norm_sign[2];
            int sign_ny = facep->norm_sign[1] * facep->norm_sign[2];

            double d = nx*v0.x + ny*v0.y + nz*v0.z;

            // Iterate all voxels in the 2D xy projection bounding box
            for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
            {
                for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
                {
                    // get the minimum corner
                    double p[2];
                    p[0] = double(xi) * voxel_dim + coordinate_min[0];
                    p[1] = double(yi) * voxel_dim + coordinate_min[1];

                    if (n0_xy[0]*p[0] + n0_xy[1]*p[1] + d0_xy >= 0.0 &&
                        n1_xy[0]*p[0] + n1_xy[1]*p[1] + d1_xy >= 0.0 &&
                        n2_xy[0]*p[0] + n2_xy[1]*p[1] + d2_xy >= 0.0)
                    {
                        // passed the 2D xy projection test
                        // determine the z range in the z voxel column
                        double x0,y0,x1,y1;
                        if (sign_nx > 0)
                        {
                            x0 = p[0] + voxel_dim;
                            x1 = p[0];
                        }
                        else
                        {
                            x0 = p[0];
                            x1 = p[0] + voxel_dim;
                        }
                        if (sign_ny > 0)
                        {
                            y0 = p[1] + voxel_dim;
                            y1 = p[1];
                        }
                        else
                        {
                            y0 = p[1];
                            y1 = p[1] + voxel_dim;
                        }

                        int z0_id = computeVoxOneIdx((d - nx*x0 - ny*y0) / nz, 2);
                        int z1_id = computeVoxOneIdx((d - nx*x1 - ny*y1) / nz, 2);

                        // iterate the voxels range determined by z0 and z1
                        //cout << "z0_id z1_id = " << z0_id << " " << z1_id << " -- ";
                        for (int zi=z0_id; zi<=z1_id; ++zi)
                        {
                            p[0] = double(yi) * voxel_dim + coordinate_min[1];
                            p[1] = double(zi) * voxel_dim + coordinate_min[2];
                            if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0 &&
                                n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0 &&
                                n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                            {
                                p[0] = double(zi) * voxel_dim + coordinate_min[2];
                                p[1] = double(xi) * voxel_dim + coordinate_min[0];
                                if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0 &&
                                    n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0 &&
                                    n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                                {
                                    markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
                                    //cout << zi << " ";
                                }
                            }
                        }
                        //cout << endl;
                    }
                }
            }
        }
    }
}

void VoxelMesh::markSortedFacePan11Speed(TriangleFace* facep)
{
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    // extend in xy plane
    else if (facep->bounding_box_type == 3)
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[2] >= 0)
        {
            n0[0] = v0.y - v1.y;
            n0[1] = v1.x - v0.x;
            n1[0] = v1.y - v2.y;
            n1[1] = v2.x - v1.x;
            n2[0] = v2.y - v0.y;
            n2[1] = v0.x - v2.x;
        }
        else
        {
            n0[0] = v1.y - v0.y;
            n0[1] = v0.x - v1.x;
            n1[0] = v2.y - v1.y;
            n1[1] = v1.x - v2.x;
            n2[0] = v0.y - v2.y;
            n2[1] = v2.x - v0.x;
        }

        // compute the minimum corner
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[0]) + coordinate_min[0];
        p[1] = voxel_dim * double(facep->voxel_min[1]) + coordinate_min[1];

        // compute the constant di
        double d0 = n0[0]*(p[0] - v0.x) + n0[1]*(p[1] - v0.y);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1.x) + n1[1]*(p[1] - v1.y);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2.x) + n2[1]*(p[1] - v2.y);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // compute constant ni
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // Iterate all voxels in the 2D xy projection bounding box
        int zi = facep->voxel_min[2];
        int size_v = facep->voxel_max[0] - facep->voxel_min[0];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*v) / n0[1];
            double u1 = (- d1 - n1[0]*v) / n1[1];
            double u2 = (- d2 - n2[0]*v) / n2[1];

            double r1, r2;
            getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2);

            int xi = facep->voxel_min[0] + v;

            // make sure rr1 rr2 are within the bounding box.
            int size_u = facep->voxel_max[1] - facep->voxel_min[1];
            int rr1 = ceil(r1);
            if (rr1 < 0)
                rr1 = 0;
            else if (rr1 > size_u)
                rr1 = size_u;
            int rr2 = floor(r2);
            if (rr2 < 0)
                rr2 = 0;
            else if (rr2 > size_u)
                rr2 = size_u;

            for (int u=rr1; u<=rr2; u++)
            {
                int yi = facep->voxel_min[1] + u;
                markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
            }
        }
    }

    // extend in yz plane
    else if (facep->bounding_box_type == 4)
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[0] >= 0)
        {
            n0[0] = v0.z - v1.z;
            n0[1] = v1.y - v0.y;
            n1[0] = v1.z - v2.z;
            n1[1] = v2.y - v1.y;
            n2[0] = v2.z - v0.z;
            n2[1] = v0.y - v2.y;
        }
        else
        {
            n0[0] = v1.z - v0.z;
            n0[1] = v0.y - v1.y;
            n1[0] = v2.z - v1.z;
            n1[1] = v1.y - v2.y;
            n2[0] = v0.z - v2.z;
            n2[1] = v2.y - v0.y;
        }

        // compute the minimum corner
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[1]) + coordinate_min[1];
        p[1] = voxel_dim * double(facep->voxel_min[2]) + coordinate_min[2];

        // compute the constant d_i
        double d0 = n0[0]*(p[0] - v0.y) + n0[1]*(p[1] - v0.z);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1.y) + n1[1]*(p[1] - v1.z);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2.y) + n2[1]*(p[1] - v2.z);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // compute constant ni
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // Iterate all voxels in the 2D projection bounding box
        int xi = facep->voxel_min[0];
        int size_v = facep->voxel_max[1] - facep->voxel_min[1];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*v) / n0[1];
            double u1 = (- d1 - n1[0]*v) / n1[1];
            double u2 = (- d2 - n2[0]*v) / n2[1];

            double r1, r2;
            getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2);

            int yi = facep->voxel_min[1] + v;

            // make sure the range is within the bounding box.
            int size_u = facep->voxel_max[2] - facep->voxel_min[2];
            int rr1 = ceil(r1);
            if (rr1 < 0)
                rr1 = 0;
            else if (rr1 > size_u)
                rr1 = size_u;
            int rr2 = floor(r2);
            if (rr2 < 0)
                rr2 = 0;
            else if (rr2 > size_u)
                rr2 = size_u;

            for (int u=rr1; u<=rr2; u++)
            {
                int zi = facep->voxel_min[2] + u;
                markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
            }
        }
    }

    // extend in zx plane
    else if (facep->bounding_box_type == 5)
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[1] >= 0)
        {
            n0[0] = v0.x - v1.x;
            n0[1] = v1.z - v0.z;
            n1[0] = v1.x - v2.x;
            n1[1] = v2.z - v1.z;
            n2[0] = v2.x - v0.x;
            n2[1] = v0.z - v2.z;
        }
        else
        {
            n0[0] = v1.x - v0.x;
            n0[1] = v0.z - v1.z;
            n1[0] = v2.x - v1.x;
            n1[1] = v1.z - v2.z;
            n2[0] = v0.x - v2.x;
            n2[1] = v2.z - v0.z;
        }

        // compute the minimum corner
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[2]) + coordinate_min[2];
        p[1] = voxel_dim * double(facep->voxel_min[0]) + coordinate_min[0];

        // compute the constant di
        double d0 = n0[0]*(p[0] - v0.z) + n0[1]*(p[1] - v0.x);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1.z) + n1[1]*(p[1] - v1.x);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2.z) + n2[1]*(p[1] - v2.x);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        // compute constant ni
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // Iterate all voxels in the 2D zx projection bounding box
        int yi = facep->voxel_min[1];
        int size_v = facep->voxel_max[2] - facep->voxel_min[2];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*v) / n0[1];
            double u1 = (- d1 - n1[0]*v) / n1[1];
            double u2 = (- d2 - n2[0]*v) / n2[1];

            double r1, r2;
            getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2);

            int zi = facep->voxel_min[2] + v;

            // make sure the range is within the bounding box.
            int size_u = facep->voxel_max[0] - facep->voxel_min[0];
            int rr1 = ceil(r1);
            if (rr1 < 0)
                rr1 = 0;
            else if (rr1 > size_u)
                rr1 = size_u;
            int rr2 = floor(r2);
            if (rr2 < 0)
                rr2 = 0;
            else if (rr2 > size_u)
                rr2 = size_u;

            for (int u=rr1; u<=rr2; u++)
            {
                int xi = facep->voxel_min[0] + u;
                markPointSpeed(xi + (yi + zi*grid_density) * (long long)grid_density);
            }
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[axis_z] >= 0)
        {
            n0[0] = v0[axis_y] - v1[axis_y];
            n0[1] = v1[axis_x] - v0[axis_x];
            n1[0] = v1[axis_y] - v2[axis_y];
            n1[1] = v2[axis_x] - v1[axis_x];
            n2[0] = v2[axis_y] - v0[axis_y];
            n2[1] = v0[axis_x] - v2[axis_x];
        }
        else
        {
            n0[0] = v1[axis_y] - v0[axis_y];
            n0[1] = v0[axis_x] - v1[axis_x];
            n1[0] = v2[axis_y] - v1[axis_y];
            n1[1] = v1[axis_x] - v2[axis_x];
            n2[0] = v0[axis_y] - v2[axis_y];
            n2[1] = v2[axis_x] - v0[axis_x];
        }

        // compute the minimum corner in the xy plane
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[axis_x]) + coordinate_min[axis_x];
        p[1] = voxel_dim * double(facep->voxel_min[axis_y]) + coordinate_min[axis_y];

        // compute the constant di
        double d0 = n0[0]*(p[0] - v0[axis_x]) + n0[1]*(p[1] - v0[axis_y]);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1[axis_x]) + n1[1]*(p[1] - v1[axis_y]);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2[axis_x]) + n2[1]*(p[1] - v2[axis_y]);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Setup for the 2D yz projection test
        double n0_yz[2], n1_yz[2], n2_yz[2];
        if (facep->norm_sign[axis_x] >= 0)
        {
            n0_yz[0] = v0[axis_z] - v1[axis_z];
            n0_yz[1] = v1[axis_y] - v0[axis_y];
            n1_yz[0] = v1[axis_z] - v2[axis_z];
            n1_yz[1] = v2[axis_y] - v1[axis_y];
            n2_yz[0] = v2[axis_z] - v0[axis_z];
            n2_yz[1] = v0[axis_y] - v2[axis_y];
        }
        else
        {
            n0_yz[0] = v1[axis_z] - v0[axis_z];
            n0_yz[1] = v0[axis_y] - v1[axis_y];
            n1_yz[0] = v2[axis_z] - v1[axis_z];
            n1_yz[1] = v1[axis_y] - v2[axis_y];
            n2_yz[0] = v0[axis_z] - v2[axis_z];
            n2_yz[1] = v2[axis_y] - v0[axis_y];
        }
        double d0_yz = - n0_yz[0]*v0[axis_y] - n0_yz[1]*v0[axis_z];
        if (n0_yz[0] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[0];
        }
        if (n0_yz[1] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[1];
        }
        double d1_yz = - n1_yz[0]*v1[axis_y] - n1_yz[1]*v1[axis_z];
        if (n1_yz[0] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[0];
        }
        if (n1_yz[1] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[1];
        }
        double d2_yz = - n2_yz[0]*v2[axis_y] - n2_yz[1]*v2[axis_z];
        if (n2_yz[0] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[0];
        }
        if (n2_yz[1] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[1];
        }

        // Setup for the 2D zx projection test
        double n0_zx[2], n1_zx[2], n2_zx[2];
        if (facep->norm_sign[axis_y] >= 0)
        {
            n0_zx[0] = v0[axis_x] - v1[axis_x];
            n0_zx[1] = v1[axis_z] - v0[axis_z];
            n1_zx[0] = v1[axis_x] - v2[axis_x];
            n1_zx[1] = v2[axis_z] - v1[axis_z];
            n2_zx[0] = v2[axis_x] - v0[axis_x];
            n2_zx[1] = v0[axis_z] - v2[axis_z];
        }
        else
        {
            n0_zx[0] = v1[axis_x] - v0[axis_x];
            n0_zx[1] = v0[axis_z] - v1[axis_z];
            n1_zx[0] = v2[axis_x] - v1[axis_x];
            n1_zx[1] = v1[axis_z] - v2[axis_z];
            n2_zx[0] = v0[axis_x] - v2[axis_x];
            n2_zx[1] = v2[axis_z] - v0[axis_z];
        }
        double d0_zx = - n0_zx[0]*v0[axis_z] - n0_zx[1]*v0[axis_x];
        if (n0_zx[0] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[0];
        }
        if (n0_zx[1] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[1];
        }
        double d1_zx = - n1_zx[0]*v1[axis_z] - n1_zx[1]*v1[axis_x];
        if (n1_zx[0] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[0];
        }
        if (n1_zx[1] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[1];
        }
        double d2_zx = - n2_zx[0]*v2[axis_z] - n2_zx[1]*v2[axis_x];
        if (n2_zx[0] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[0];
        }
        if (n2_zx[1] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[1];
        }

        // compute constant ni in the xy plane
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // compute the constant of the famulation of the triangle plane
        double nx = facep->norm[axis_x];
        double ny = facep->norm[axis_y];
        double nz = facep->norm[axis_z];

        // multiplied by the sign of the dominant axis in order to
        // determine the range correctly.
        int sign_nx = facep->norm_sign[axis_x] * facep->norm_sign[axis_z];
        int sign_ny = facep->norm_sign[axis_y] * facep->norm_sign[axis_z];

        double d = nx*v0[axis_x] + ny*v0[axis_y] + nz*v0[axis_z];

        int moving_vox[3];
        int size_v = facep->voxel_max[axis_x] - facep->voxel_min[axis_x];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*double(v)) / n0[1];
            double u1 = (- d1 - n1[0]*double(v)) / n1[1];
            double u2 = (- d2 - n2[0]*double(v)) / n2[1];

            double r1, r2;
            getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2);

            // determine the axis x component
            moving_vox[axis_x] = facep->voxel_min[axis_x] + v;

            // make sure the range is valid.
            int size_u = facep->voxel_max[axis_y] - facep->voxel_min[axis_y];
            int rr1 = ceil(r1);
            if (rr1 < 0)
                rr1 = 0;
            else if (rr1 > size_u)
                rr1 = size_u;
            int rr2 = floor(r2);
            if (rr2 < 0)
                rr2 = 0;
            else if (rr2 > size_u)
                rr2 = size_u;

            for (int u=rr1; u<=rr2; u++)
            {
                // determine the y component
                moving_vox[axis_y] = facep->voxel_min[axis_y] + u;

                // passed the 2D xy projection test so far
                // determine the z range in the z voxel column

                // compute the min and max corner in the voxel column
                p[0] = voxel_dim * double(moving_vox[axis_x]) + coordinate_min[axis_x];
                p[1] = voxel_dim * double(moving_vox[axis_y]) + coordinate_min[axis_y];
                double x0,y0,x1,y1;
                if (sign_nx > 0)
                {
                    x0 = p[0] + voxel_dim;
                    x1 = p[0];
                }
                else
                {
                    x0 = p[0];
                    x1 = p[0] + voxel_dim;
                }
                if (sign_ny > 0)
                {
                    y0 = p[1] + voxel_dim;
                    y1 = p[1];
                }
                else
                {
                    y0 = p[1];
                    y1 = p[1] + voxel_dim;
                }

                int z0_id = computeVoxOneIdx((d - nx*x0 - ny*y0) / nz, axis_z);
                int z1_id = computeVoxOneIdx((d - nx*x1 - ny*y1) / nz, axis_z);

                // iterate the voxels range determined by z0 and z1
                //cout << "z0_id z1_id = " << z0_id << " " << z1_id << " -- ";
                for (moving_vox[axis_z]=z0_id; moving_vox[axis_z]<=z1_id; moving_vox[axis_z]++)
                {
                    p[0] = double(moving_vox[axis_y]) * voxel_dim + coordinate_min[axis_y];
                    p[1] = double(moving_vox[axis_z]) * voxel_dim + coordinate_min[axis_z];
                    if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0 &&
                        n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0 &&
                        n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                    {
                        p[0] = double(moving_vox[axis_z]) * voxel_dim + coordinate_min[axis_z];
                        p[1] = double(moving_vox[axis_x]) * voxel_dim + coordinate_min[axis_x];

                        if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0 &&
                            n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0 &&
                            n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                        {
                            markPointSpeed(moving_vox[0] + (moving_vox[1] + moving_vox[2]*grid_density) * (long long)grid_density);
                        }
                    }
                }
            }
        }
    }
}

// Setup each triangle for the scanlie based method
// and build a list sorted by their bounding box type.
void VoxelMesh::setupSortedTriangleListFLTINT()
{
    // set up the triangles in type of 0~5
    int size = sorted_triangle_list.size();
    for (int i=0; i<size; ++i)
    {
        TriangleFace *p_tri = sorted_triangle_list[i];

        // copy the vertices
        p_tri->v0[0] = p_tri->node0->ori_coordinate[0];
        p_tri->v0[1] = p_tri->node0->ori_coordinate[1];
        p_tri->v0[2] = p_tri->node0->ori_coordinate[2];
        p_tri->v1[0] = p_tri->node1->ori_coordinate[0];
        p_tri->v1[1] = p_tri->node1->ori_coordinate[1];
        p_tri->v1[2] = p_tri->node1->ori_coordinate[2];
        p_tri->v2[0] = p_tri->node2->ori_coordinate[0];
        p_tri->v2[1] = p_tri->node2->ori_coordinate[1];
        p_tri->v2[2] = p_tri->node2->ori_coordinate[2];

        // compute the voxel
        computeVoxIdx(p_tri->vox0, p_tri->v0);
        computeVoxIdx(p_tri->vox1, p_tri->v1);
        computeVoxIdx(p_tri->vox2, p_tri->v2);

        // get the voxel id for each vertex
        int *v0 = p_tri->vox0;
        int *v1 = p_tri->vox1;
        int *v2 = p_tri->vox2;

        // determine the bounding box type
        if (*v0 == *v1 && *v1 == *v2)
        {
            // must be type 1 or 2 or 4
            if (*(v0+1) == *(v1+1) && *(v1+1) == *(v2+1))
            {
                // must be type 2, i.e. only extending along z axis
                p_tri->bounding_box_type = 2;

                p_tri->voxel_min[0] = p_tri->voxel_max[0] = *v0;
                p_tri->voxel_min[1] = p_tri->voxel_max[1] = *(v0+1);
                int aa[3];
                aa[0] = *(v0+2);
                aa[1] = *(v1+2);
                aa[2] = *(v2+2);
                getMinMaxValueINT(p_tri->voxel_min[2], p_tri->voxel_max[2], aa);
            }
            else if (*(v0+2) == *(v1+2) && *(v1+2) == *(v2+2))
            {
                // must be type 1, i.e. only extending along y axis
                p_tri->bounding_box_type = 1;

                p_tri->voxel_min[0] = p_tri->voxel_max[0] = *v0;
                p_tri->voxel_min[2] = p_tri->voxel_max[2] = *(v0+2);
                int aa[3];
                aa[0] = *(v0+1);
                aa[1] = *(v1+1);
                aa[2] = *(v2+1);
                getMinMaxValueINT(p_tri->voxel_min[1], p_tri->voxel_max[1], aa);
            }
            else
            {
                // must be type 4, i.e. only extending along yz plane
                p_tri->bounding_box_type = 4;

                // sort the 2D vertices such that e2>e1>e0
                sortVertices2D(p_tri->v0, p_tri->v1, p_tri->v2, 1, 2);

                // re-compute the voxel
                computeVoxIdx(p_tri->vox0, p_tri->v0);
                computeVoxIdx(p_tri->vox1, p_tri->v1);
                computeVoxIdx(p_tri->vox2, p_tri->v2);

                // map the dominant axis to z axis
                p_tri->axis_x = 1;
                p_tri->axis_y = 2;
                p_tri->axis_z = 0;
            }
        }
        else if (*(v0+1) == *(v1+1) && *(v1+1) == *(v2+1))
        {
            // must be type 0 or 5
            if (*(v0+2) == *(v1+2) && *(v1+2) == *(v2+2))
            {
                // must be type 0, i.e. only extending along x axis
                p_tri->bounding_box_type = 0;

                p_tri->voxel_min[1] = p_tri->voxel_max[1] = *(v0+1);
                p_tri->voxel_min[2] = p_tri->voxel_max[2] = *(v0+2);
                int aa[3];
                aa[0] = *(v0);
                aa[1] = *(v1);
                aa[2] = *(v2);
                getMinMaxValueINT(p_tri->voxel_min[0], p_tri->voxel_max[0], aa);
            }
            else
            {
                // must be type 5, i.e. only extending along zx plane
                p_tri->bounding_box_type = 5;

                // sort the 2D vertices such that e2>e1>e0
                sortVertices2D(p_tri->v0, p_tri->v1, p_tri->v2, 2, 0);

                // re-compute the voxel
                computeVoxIdx(p_tri->vox0, p_tri->v0);
                computeVoxIdx(p_tri->vox1, p_tri->v1);
                computeVoxIdx(p_tri->vox2, p_tri->v2);

                // map the dominant axis to z axis
                p_tri->axis_x = 2;
                p_tri->axis_y = 0;
                p_tri->axis_z = 1;
            }
        }
        else if (*(v0+2) == *(v1+2) && *(v1+2) == *(v2+2))
        {
            // must be type 3, i.e. only extending along xy plane
            p_tri->bounding_box_type = 3;

            // sort the 2D vertices such that e2>e1>e0
            sortVertices2D(p_tri->v0, p_tri->v1, p_tri->v2, 0, 1);

            // re-compute the voxel
            computeVoxIdx(p_tri->vox0, p_tri->v0);
            computeVoxIdx(p_tri->vox1, p_tri->v1);
            computeVoxIdx(p_tri->vox2, p_tri->v2);

            // map the dominant axis to z axis
            p_tri->axis_x = 0;
            p_tri->axis_y = 1;
            p_tri->axis_z = 2;
        }

        // must be type 6, 7 or 8
        else
        {
            // compute the normal of the triangle
            zVec3 n0 = p_tri->node0->ori_coordinate;
            zVec3 n1 = p_tri->node1->ori_coordinate;
            zVec3 n2 = p_tri->node2->ori_coordinate;
            zVec3 norm = zVec3::crossproduct(n1-n0, n2-n0);
            p_tri->norm_abs[0] = abs(norm.x);
            p_tri->norm_abs[1] = abs(norm.y);
            p_tri->norm_abs[2] = abs(norm.z);

            // get the vertices
            p_tri->v0[0] = n0[0];
            p_tri->v0[1] = n0[1];
            p_tri->v0[2] = n0[2];
            p_tri->v1[0] = n1[0];
            p_tri->v1[1] = n1[1];
            p_tri->v1[2] = n1[2];
            p_tri->v2[0] = n2[0];
            p_tri->v2[1] = n2[1];
            p_tri->v2[2] = n2[2];

            // must be type 6 or 7 or 8, determined by the dominant direction.
            if (p_tri->norm_abs[0] >= p_tri->norm_abs[1])
            {
                if (p_tri->norm_abs[1] >= p_tri->norm_abs[2])
                {
                    // 0>=1>=2, must be type 6, i.e. dominant at x axis
                    p_tri->bounding_box_type = 6;

                    // map the dominant axis to z axis
                    p_tri->axis_x = 1;
                    p_tri->axis_y = 2;
                    p_tri->axis_z = 0;
                }
                else
                {
                    if (p_tri->norm_abs[0] >= p_tri->norm_abs[2])
                    {
                        // 0>=2>1, must be type 6, i.e. dominant at x axis
                        p_tri->bounding_box_type = 6;

                        // map the dominant axis to z axis
                        p_tri->axis_x = 1;
                        p_tri->axis_y = 2;
                        p_tri->axis_z = 0;
                    }
                    else
                    {
                        // 2>0>=1, must be type 8, i.e. dominant at z axis
                        p_tri->bounding_box_type = 8;

                        // map the dominant axis to z axis
                        p_tri->axis_x = 0;
                        p_tri->axis_y = 1;
                        p_tri->axis_z = 2;
                    }
                }
            }
            else
            {
                if (p_tri->norm_abs[0] >= p_tri->norm_abs[2])
                {
                    // 1>0>=2, must be type 7, i.e. dominant at y axis
                    p_tri->bounding_box_type = 7;

                    // map the dominant axis to z axis
                    p_tri->axis_x = 2;
                    p_tri->axis_y = 0;
                    p_tri->axis_z = 1;
                }
                else
                {
                    if (p_tri->norm_abs[1] >= p_tri->norm_abs[2])
                    {
                        // 1>=2>0, must be type 7, i.e. dominant at y axis
                        p_tri->bounding_box_type = 7;

                        // map the dominant axis to z axis
                        p_tri->axis_x = 2;
                        p_tri->axis_y = 0;
                        p_tri->axis_z = 1;
                    }
                    else
                    {
                        // 2>1>0, must be type 8, i.e. dominant at z axis
                        p_tri->bounding_box_type = 8;

                        // map the dominant axis to z axis
                        p_tri->axis_x = 0;
                        p_tri->axis_y = 1;
                        p_tri->axis_z = 2;
                    }
                }
            }

            // sort the vertices along dominant axis
            sortVertByDomiAxis(p_tri->v0, p_tri->v1, p_tri->v2, p_tri->axis_z);

            // re-compute the voxel
            computeVoxIdx(p_tri->vox0, p_tri->v0);
            computeVoxIdx(p_tri->vox1, p_tri->v1);
            computeVoxIdx(p_tri->vox2, p_tri->v2);
        }
    }
}

void VoxelMesh::markSortedFaceFLTSpeed(TriangleFace* facep)
{
    //cout << "FLTSpeed facep->bounding_box_type " << facep->bounding_box_type << endl;
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    else if (facep->bounding_box_type == 3 ||
             facep->bounding_box_type == 4 ||
             facep->bounding_box_type == 5)
    {
        double p0[3], p1[3], p2[3];
        p0[0] = facep->v0[0];
        p0[1] = facep->v0[1];
        p0[2] = facep->v0[2];
        p1[0] = facep->v1[0];
        p1[1] = facep->v1[1];
        p1[2] = facep->v1[2];
        p2[0] = facep->v2[0];
        p2[1] = facep->v2[1];
        p2[2] = facep->v2[2];

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        int slice_id = facep->vox0[axis_z];

        // mark the three edges
        mark2DLineSpeed(p0[axis_x], p0[axis_y], p1[axis_x], p1[axis_y],
                        axis_x, axis_y, axis_z, slice_id);
        mark2DLineSpeed(p0[axis_x], p0[axis_y], p2[axis_x], p2[axis_y],
                        axis_x, axis_y, axis_z, slice_id);
        mark2DLineSpeed(p1[axis_x], p1[axis_y], p2[axis_x], p2[axis_y],
                        axis_x, axis_y, axis_z, slice_id);

        // compute the three edge vectors p0p1, p0p2 and p1p2, and their length
        double p0p1[2], p0p2[2], p1p2[2];
        p0p1[0] = p1[axis_x]-p0[axis_x];
        p0p1[1] = p1[axis_y]-p0[axis_y];
        p0p2[0] = p2[axis_x]-p0[axis_x];
        p0p2[1] = p2[axis_y]-p0[axis_y];
        p1p2[0] = p2[axis_x]-p1[axis_x];
        p1p2[1] = p2[axis_y]-p1[axis_y];

        // compute the optimal step distance
        // along the norm direction of p1p2
        double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
        double d_hat = 0.99*voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

        // compute the step distance along p0p1 corresponding to d_hat
        double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
        double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
        double step01[2];
        step01[0] = d01 * p0p1[0] / len01;
        step01[1] = d01 * p0p1[1] / len01;

        // compute the step distance along p0p2 corresponding to d_hat
        double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
        double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
        double step02[2];
        step02[0] = d02 * p0p2[0] / len02;
        step02[1] = d02 * p0p2[1] / len02;

        // two moving points will move along p0p1 and p0p2
        double moving_p0p1[2], moving_p0p2[2];
        moving_p0p1[0] = p0[axis_x] + step01[0];
        moving_p0p1[1] = p0[axis_y] + step01[1];
        moving_p0p2[0] = p0[axis_x] + step02[0];
        moving_p0p2[1] = p0[axis_y] + step02[1];

        // iterate and voxelize all the scanlines
        int size_steps = len01 / d01;
        for (int i=0; i<size_steps; ++i)
        {
            mark2DLineSpeed(moving_p0p1[0], moving_p0p1[1], moving_p0p2[0], moving_p0p2[1],
                            axis_x, axis_y, axis_z, slice_id);
            moving_p0p1[0] += step01[0];
            moving_p0p1[1] += step01[1];
            moving_p0p2[0] += step02[0];
            moving_p0p2[1] += step02[1];
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        double p0[3], p1[3], p2[3];
        p0[0] = facep->v0[0];
        p0[1] = facep->v0[1];
        p0[2] = facep->v0[2];
        p1[0] = facep->v1[0];
        p1[1] = facep->v1[1];
        p1[2] = facep->v1[2];
        p2[0] = facep->v2[0];
        p2[1] = facep->v2[1];
        p2[2] = facep->v2[2];

        int p0_vox[3], p1_vox[3], p2_vox[3];
        p0_vox[0] = facep->vox0[0];
        p0_vox[1] = facep->vox0[1];
        p0_vox[2] = facep->vox0[2];
        p1_vox[0] = facep->vox1[0];
        p1_vox[1] = facep->vox1[1];
        p1_vox[2] = facep->vox1[2];
        p2_vox[0] = facep->vox2[0];
        p2_vox[1] = facep->vox2[1];
        p2_vox[2] = facep->vox2[2];

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        //-----------------------------------------------------------------------------
        // Mark the three edges.
        //-----------------------------------------------------------------------------
        mark3DLineSpeed(p0, p2, p0_vox, p2_vox);
        mark3DLineSpeed(p0, p1, p0_vox, p1_vox);
        mark3DLineSpeed(p1, p2, p1_vox, p2_vox);

        //-----------------------------------------------------------------------------
        // Determine the three line segments.
        //-----------------------------------------------------------------------------
        // Determine the 3D line segments of p0p2.
        double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
        double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
        double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

        // Determine the 3D line segments of p0p1.
        double a2 = p1[axis_x]-p0[axis_x], b2 = p0[axis_x];
        double c2 = p1[axis_y]-p0[axis_y], d2 = p0[axis_y];
        double e2 = p1[axis_z]-p0[axis_z], f2 = p0[axis_z];

        //-----------------------------------------------------------------------------
        // Determine the typical scanline passing p1 and pre-compute the delta steps.
        //-----------------------------------------------------------------------------
        double scp0_x, scp0_y, scp1_x, scp1_y;
        scp1_x = p1[axis_x];
        scp1_y = p1[axis_y];

        if (p0_vox[axis_z] == p2_vox[axis_z])
        {
            // The triangle is within one xy slice.
            // Simply let the typical scanline be p0p1.
            scp0_x = p0[axis_x];
            scp0_y = p0[axis_y];
        }
        else
        {
            // The triangle is within more than one xy slice.
            // Let the typical scanline be the intersection with the plane passing p1.
            double t1 = (p1[axis_z] - f1) / e1;
            scp0_x = a1 * t1 + b1;
            scp0_y = c1 * t1 + d1;
        }

        // compute the norm of the scanline.
        double nx = scp0_y - scp1_y;
        double ny = scp1_x - scp0_x;

        // make the norm direction identical to p0p2
        if (nx * a1 + ny * c1 < 0.0)
        {
            nx = - nx;
            ny = - ny;
        }

        // normalize the norm.
        double norm_length = sqrt(nx*nx + ny*ny);
        double vox_dim = 0.99*voxel_dim*(abs(nx)+abs(ny)) / norm_length;
        nx = nx / norm_length;
        ny = ny / norm_length;

        // compute the 2D step vectors along p0p2.
        //double vox_dim = 0.99 * voxel_dim;
        double step_1_x, step_1_y, step_1_length;
        double temp = vox_dim / (nx*a1 + ny*c1);
        step_1_x = a1 * temp;
        step_1_y = c1 * temp;
        step_1_length = step_1_x*step_1_x + step_1_y*step_1_y;

        // compute the 2D step vectors along p0p1.
        double step_2_x, step_2_y;
        temp = vox_dim / (nx*a2 + ny*c2);
        step_2_x = a2 * temp;
        step_2_y = c2 * temp;

        //-----------------------------------------------------------------------------
        // Voxelize the first half of the triangle.
        //-----------------------------------------------------------------------------
        // Determine the first slice.
        int slice_idx = p0_vox[axis_z];
        double plane = voxel_dim * (slice_idx + 1) + coordinate_min[axis_z];
    //    if (plane == p0[axis_z])
    //    {
    //        // the plane happened to intersect p0,
    //        // refine the slice index and move to next plane.
    //        slice_idx = computeVoxOneIdx(p0[axis_z]+0.5*voxel_dim, axis_z);

    //        plane += voxel_dim;
    //    }

        // the first scanline is a point, i.e. p0.
        double startp0_x, startp0_y, startp1_x, startp1_y;
        startp0_x = p0[axis_x];
        startp0_y = p0[axis_y];
        startp1_x = startp0_x;
        startp1_y = startp0_y;

        double endp0_x, endp0_y, endp1_x, endp1_y;
        double length;
        while  (slice_idx < p1_vox[axis_z])
        {
            double t1 = (plane - f1) / e1;
            endp0_x = a1 * t1 + b1;
            endp0_y = c1 * t1 + d1;

            double t2 = (plane - f2) / e2;
            endp1_x = a2 * t2 + b2;
            endp1_y = c2 * t2 + d2;

            length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
            while (length > 0.0)
            {
                mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                                axis_x, axis_y, axis_z, slice_idx);

                startp0_x += step_1_x;
                startp0_y += step_1_y;
                startp1_x += step_2_x;
                startp1_y += step_2_y;

                length -= step_1_length;
            }

            startp0_x = endp0_x;
            startp0_y = endp0_y;
            startp1_x = endp1_x;
            startp1_y = endp1_y;
            mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                            axis_x, axis_y, axis_z, slice_idx);

            plane += voxel_dim;
            slice_idx++;
        }

        //-----------------------------------------------------------------------------
        // Voxelize the slice containing the typical scanline passing p1.
        //-----------------------------------------------------------------------------
        endp0_x = scp0_x;
        endp0_y = scp0_y;
        endp1_x = scp1_x;
        endp1_y = scp1_y;

        length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
        while (length > 0.0)
        {
            mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                            axis_x, axis_y, axis_z, slice_idx);

            startp0_x += step_1_x;
            startp0_y += step_1_y;
            startp1_x += step_2_x;
            startp1_y += step_2_y;

            length -= step_1_length;
        }
        mark2DLineSpeed(endp0_x, endp0_y, endp1_x, endp1_y,
                        axis_x, axis_y, axis_z, slice_idx);

        // check if the plane happened to pass p1
    //    if (plane == p1[axis_z])
    //    {
    //        // the plane happened to intersect p1,
    //        // refine the slice index and move to next plane.
    //        slice_idx = computeVoxOneIdx(p1[axis_z]+0.5*voxel_dim, axis_z);

    //        plane += voxel_dim;
    //    }

        //-----------------------------------------------------------------------------
        // voxelize the second half of the triangle.
        //-----------------------------------------------------------------------------
        // Determine the 3D line segments of p0p1.
        a2 = p2[axis_x]-p1[axis_x]; b2 = p1[axis_x];
        c2 = p2[axis_y]-p1[axis_y]; d2 = p1[axis_y];
        e2 = p2[axis_z]-p1[axis_z]; f2 = p1[axis_z];

        // compute the 2D step vectors along p1p2.
        temp = vox_dim / (nx*a2 + ny*c2);
        step_2_x = a2 * temp;
        step_2_y = c2 * temp;

        // Start from the typical scanline.
        startp0_x = scp0_x;
        startp0_y = scp0_y;
        startp1_x = scp1_x;
        startp1_y = scp1_y;

        while (slice_idx < p2_vox[axis_z])
        {
            double t1 = (plane - f1) / e1;
            endp0_x = a1 * t1 + b1;
            endp0_y = c1 * t1 + d1;

            double t2 = (plane - f2) / e2;
            endp1_x = a2 * t2 + b2;
            endp1_y = c2 * t2 + d2;

            length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
            while (length > 0.0)
            {
                mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                                axis_x, axis_y, axis_z, slice_idx);

                startp0_x += step_1_x;
                startp0_y += step_1_y;
                startp1_x += step_2_x;
                startp1_y += step_2_y;

                length -= step_1_length;
            }

            startp0_x = endp0_x;
            startp0_y = endp0_y;
            startp1_x = endp1_x;
            startp1_y = endp1_y;
            mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                            axis_x, axis_y, axis_z, slice_idx);

            plane += voxel_dim;
            slice_idx++;
        }

        // voxelize the last slice.
        length = (p2[axis_x] - startp0_x)*step_1_x + (p2[axis_y] - startp0_y)*step_1_y;
        while (length > 0.0)
        {
            mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                            axis_x, axis_y, axis_z, slice_idx);

            startp0_x += step_1_x;
            startp0_y += step_1_y;
            startp1_x += step_2_x;
            startp1_y += step_2_y;

            length -= step_1_length;
        }

//        double p0[3], p1[3], p2[3];
//        p0[0] = facep->v0[0];
//        p0[1] = facep->v0[1];
//        p0[2] = facep->v0[2];
//        p1[0] = facep->v1[0];
//        p1[1] = facep->v1[1];
//        p1[2] = facep->v1[2];
//        p2[0] = facep->v2[0];
//        p2[1] = facep->v2[1];
//        p2[2] = facep->v2[2];

//        int p0_vox[3], p1_vox[3], p2_vox[3];
//        p0_vox[0] = facep->vox0[0];
//        p0_vox[1] = facep->vox0[1];
//        p0_vox[2] = facep->vox0[2];
//        p1_vox[0] = facep->vox1[0];
//        p1_vox[1] = facep->vox1[1];
//        p1_vox[2] = facep->vox1[2];
//        p2_vox[0] = facep->vox2[0];
//        p2_vox[1] = facep->vox2[1];
//        p2_vox[2] = facep->vox2[2];

//        int axis_x = facep->axis_x;
//        int axis_y = facep->axis_y;
//        int axis_z = facep->axis_z;

//        //-----------------------------------------------------------------------------
//        // Mark the three edges.
//        //-----------------------------------------------------------------------------
//        mark3DLineSpeed(p0, p2, p0_vox, p2_vox);
//        mark3DLineSpeed(p0, p1, p0_vox, p1_vox);
//        mark3DLineSpeed(p1, p2, p1_vox, p2_vox);

//        //-------------------------------------------------------------------------------
//        // compute the optimal steps along two edge p0p1 and p0p2 in 2D projection
//        //-------------------------------------------------------------------------------
//        // compute the three projected edge vectors p0p1, p0p2 and p1p2, and their length
//        double p0p1[2], p0p2[2], p1p2[2];
//        p0p1[0] = p1[axis_x]-p0[axis_x];
//        p0p1[1] = p1[axis_y]-p0[axis_y];
//        p0p2[0] = p2[axis_x]-p0[axis_x];
//        p0p2[1] = p2[axis_y]-p0[axis_y];
//        p1p2[0] = p2[axis_x]-p1[axis_x];
//        p1p2[1] = p2[axis_y]-p1[axis_y];

//        // compute the optimal step distance in 2D projections.
//        // along the norm direction of p1p2
//        double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
//        double d_hat = 0.99*voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

//        // compute the step distance along p0p1 corresponding to d_hat
//        double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
//        double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
//        double step01[2];
//        step01[0] = d01 * p0p1[0] / len01;
//        step01[1] = d01 * p0p1[1] / len01;

//        // compute the step distance along p0p2 corresponding to d_hat
//        double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
//        double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
//        double step02[2];
//        step02[0] = d02 * p0p2[0] / len02;
//        step02[1] = d02 * p0p2[1] / len02;

//        //-----------------------------------------------------------------------------
//        // Voxelize the triangle.
//        //-----------------------------------------------------------------------------
//        // Determine the 3D line segment p0p1.
//        double a1 = p1[axis_x]-p0[axis_x], b1 = p0[axis_x];
//        double c1 = p1[axis_y]-p0[axis_y], d1 = p0[axis_y];
//        double e1 = p1[axis_z]-p0[axis_z], f1 = p0[axis_z];

//        // Determine the 3D line segment p0p2.
//        double a2 = p2[axis_x]-p0[axis_x], b2 = p0[axis_x];
//        double c2 = p2[axis_y]-p0[axis_y], d2 = p0[axis_y];
//        double e2 = p2[axis_z]-p0[axis_z], f2 = p0[axis_z];

//        // voxelize according to the sub type of the triangle
//        if (facep->bounding_box_subtype == 0)
//        {
//            // Determine the index of the first slice and the first boundary plane.
//            int slice_idx = p0_vox[axis_z];
//            double plane = voxel_dim * (p0_vox[axis_z] + 1) + coordinate_min[axis_z];
//            if (plane == p0[axis_z])
//            {
//                // if the plane happened to intersect p0,
//                // refine the slice index and move to the next plane.
//                plane += voxel_dim;
//                slice_idx++;
//            }

//            // the boundary of a slice is determined by four points along edge p0p1 and p0p2.
//            // i.e. slice_point01_s, slice_point01_e, and slice_point02_s, slice_point02_e.
//            // the boundary points are the intersections of the boundary planes perpendicular to axis_z.
//            double slice_point01_s[2], slice_point01_e[2];
//            double slice_point02_s[2], slice_point02_e[2];

//            // in the first slice, slice_p0p1_0 and slice_p0p2_0 are actually p0.
//            slice_point01_s[0] = slice_point02_s[0] = p0[axis_x];
//            slice_point01_s[1] = slice_point02_s[1] = p0[axis_y];

//            // the moving point iterates the 2D scanline in the slice
//            double moving_point01[2], moving_point02[2];
//            while  (slice_idx < p1_vox[axis_z])
//            {
//                //cout << "slice idx = " << slice_idx << " " << endl;
//                // compute the boundary points slice_point01_e and slice_point02_e.
//                double t1 = (plane - f1) / e1;
//                slice_point01_e[0] = a1 * t1 + b1;
//                slice_point01_e[1] = c1 * t1 + d1;

//                double t2 = (plane - f2) / e2;
//                slice_point02_e[0] = a2 * t2 + b2;
//                slice_point02_e[1] = c2 * t2 + d2;

//                // mark two legs
//                mark2DLineSpeed(slice_point01_s[0], slice_point01_s[1],
//                           slice_point01_e[0], slice_point01_e[1],
//                           axis_x, axis_y, axis_z, slice_idx);
//                mark2DLineSpeed(slice_point02_s[0], slice_point02_s[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx);

//                // the first 2D scanline
//                moving_point01[0] = slice_point01_s[0];
//                moving_point01[1] = slice_point01_s[1];
//                moving_point02[0] = slice_point02_s[0];
//                moving_point02[1] = slice_point02_s[1];

//                // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//                double temp0 = slice_point01_e[0] - slice_point01_s[0];
//                double temp1 = slice_point01_e[1] - slice_point01_s[1];
//                double len2_slice_edge = temp0*temp0 + temp1*temp1;
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                double len2 = temp0*temp0 + temp1*temp1;
//                while (len2_slice_edge > len2)
//                {
//                    mark2DLineSpeed(moving_point01[0], moving_point01[1],
//                               moving_point02[0], moving_point02[1],
//                               axis_x, axis_y, axis_z, slice_idx);

//                    moving_point01[0] += step01[0];
//                    moving_point01[1] += step01[1];
//                    moving_point02[0] += step02[0];
//                    moving_point02[1] += step02[1];

//                    // compute len2
//                    temp0 = moving_point01[0] - slice_point01_s[0];
//                    temp1 = moving_point01[1] - slice_point01_s[1];
//                    len2 = temp0*temp0 + temp1*temp1;
//                }

//                // mark the last 2D scanline
//                mark2DLineSpeed(slice_point01_e[0], slice_point01_e[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx);

//                // let the boundary points move to the next slice.
//                slice_point01_s[0] = slice_point01_e[0];
//                slice_point01_s[1] = slice_point01_e[1];
//                slice_point02_s[0] = slice_point02_e[0];
//                slice_point02_s[1] = slice_point02_e[1];

//                plane += voxel_dim;
//                slice_idx++;
//            }

//            // deal with the last slice
//            // the boundary points slice_point01_e and slice_point02_e should be the edge p1p2.
//            slice_point01_e[0] = p1[axis_x];
//            slice_point01_e[1] = p1[axis_y];
//            slice_point02_e[0] = p2[axis_x];
//            slice_point02_e[1] = p2[axis_y];

//            // mark two legs
//            mark2DLineSpeed(slice_point01_s[0], slice_point01_s[1],
//                       slice_point01_e[0], slice_point01_e[1],
//                       axis_x, axis_y, axis_z, slice_idx);
//            mark2DLineSpeed(slice_point02_s[0], slice_point02_s[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx);

//            // the first 2D scanline
//            moving_point01[0] = slice_point01_s[0];
//            moving_point01[1] = slice_point01_s[1];
//            moving_point02[0] = slice_point02_s[0];
//            moving_point02[1] = slice_point02_s[1];

//            // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//            double temp0 = slice_point01_e[0] - slice_point01_s[0];
//            double temp1 = slice_point01_e[1] - slice_point01_s[1];
//            double len2_slice_edge = temp0*temp0 + temp1*temp1;
//            temp0 = moving_point01[0] - slice_point01_s[0];
//            temp1 = moving_point01[1] - slice_point01_s[1];
//            double len2 = temp0*temp0 + temp1*temp1;
//            while (len2_slice_edge > len2)
//            {
//                mark2DLineSpeed(moving_point01[0], moving_point01[1],
//                           moving_point02[0], moving_point02[1],
//                           axis_x, axis_y, axis_z, slice_idx);

//                moving_point01[0] += step01[0];
//                moving_point01[1] += step01[1];
//                moving_point02[0] += step02[0];
//                moving_point02[1] += step02[1];

//                // compute len2
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                len2 = temp0*temp0 + temp1*temp1;
//            }

//            // mark the last 2D scanline
//            mark2DLineSpeed(slice_point01_e[0], slice_point01_e[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx);
//        }
//        else
//        {
//            // Determine the index of the first slice and the first boundary plane.
//            int slice_idx = p0_vox[axis_z];
//            double plane = voxel_dim * p0_vox[axis_z] + coordinate_min[axis_z];
//            if (plane == p0[axis_z])
//            {
//                // if the plane happened to intersect p0,
//                // refine the slice index and move to the next plane.
//                plane -= voxel_dim;
//                slice_idx--;
//            }

//            // the boundary of a slice is determined by four points along edge p0p1 and p0p2.
//            // i.e. slice_point01_s, slice_point01_e, and slice_point02_s, slice_point02_e.
//            // the boundary points are the intersections of the boundary planes perpendicular to axis_z.
//            double slice_point01_s[2], slice_point01_e[2];
//            double slice_point02_s[2], slice_point02_e[2];

//            // in the first slice, slice_p0p1_0 and slice_p0p2_0 are actually p0.
//            slice_point01_s[0] = slice_point02_s[0] = p0[axis_x];
//            slice_point01_s[1] = slice_point02_s[1] = p0[axis_y];

//            // the moving point iterates the 2D scanline in the slice
//            double moving_point01[2], moving_point02[2];
//            while  (slice_idx > p1_vox[axis_z])
//            {
//                // compute the boundary points slice_point01_e and slice_point02_e.
//                double t1 = (plane - f1) / e1;
//                slice_point01_e[0] = a1 * t1 + b1;
//                slice_point01_e[1] = c1 * t1 + d1;

//                double t2 = (plane - f2) / e2;
//                slice_point02_e[0] = a2 * t2 + b2;
//                slice_point02_e[1] = c2 * t2 + d2;

//                // mark two legs
//                mark2DLineSpeed(slice_point01_s[0], slice_point01_s[1],
//                           slice_point01_e[0], slice_point01_e[1],
//                           axis_x, axis_y, axis_z, slice_idx);
//                mark2DLineSpeed(slice_point02_s[0], slice_point02_s[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx);

//                // the first 2D scanline
//                moving_point01[0] = slice_point01_s[0];
//                moving_point01[1] = slice_point01_s[1];
//                moving_point02[0] = slice_point02_s[0];
//                moving_point02[1] = slice_point02_s[1];

//                // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//                double temp0 = slice_point01_e[0] - slice_point01_s[0];
//                double temp1 = slice_point01_e[1] - slice_point01_s[1];
//                double len2_slice_edge = temp0*temp0 + temp1*temp1;
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                double len2 = temp0*temp0 + temp1*temp1;
//                while (len2_slice_edge >= len2)
//                {
//                    //cout << "slice idx = " << slice_idx << " " << endl;
//                    mark2DLineSpeed(moving_point01[0], moving_point01[1],
//                            moving_point02[0], moving_point02[1],
//                            axis_x, axis_y, axis_z, slice_idx);

//                    moving_point01[0] += step01[0];
//                    moving_point01[1] += step01[1];
//                    moving_point02[0] += step02[0];
//                    moving_point02[1] += step02[1];

//                    // compute len2
//                    temp0 = moving_point01[0] - slice_point01_s[0];
//                    temp1 = moving_point01[1] - slice_point01_s[1];
//                    len2 = temp0*temp0 + temp1*temp1;
//                }

//                // mark the last 2D scanline
//                mark2DLineSpeed(slice_point01_e[0], slice_point01_e[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx);

//                // let the boundary points move to the next slice.
//                slice_point01_s[0] = slice_point01_e[0];
//                slice_point01_s[1] = slice_point01_e[1];
//                slice_point02_s[0] = slice_point02_e[0];
//                slice_point02_s[1] = slice_point02_e[1];

//                plane -= voxel_dim;
//                slice_idx--;
//            }

//            // deal with the last slice
//            // the boundary points slice_point01_e and slice_point02_e should be the edge p1p2.
//            slice_point01_e[0] = p1[axis_x];
//            slice_point01_e[1] = p1[axis_y];
//            slice_point02_e[0] = p2[axis_x];
//            slice_point02_e[1] = p2[axis_y];

//            // mark two legs
//            mark2DLineSpeed(slice_point01_s[0], slice_point01_s[1],
//                       slice_point01_e[0], slice_point01_e[1],
//                       axis_x, axis_y, axis_z, slice_idx);
//            mark2DLineSpeed(slice_point02_s[0], slice_point02_s[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx);

//            // the first 2D scanline
//            moving_point01[0] = slice_point01_s[0];
//            moving_point01[1] = slice_point01_s[1];
//            moving_point02[0] = slice_point02_s[0];
//            moving_point02[1] = slice_point02_s[1];

//            // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//            double temp0 = slice_point01_e[0] - slice_point01_s[0];
//            double temp1 = slice_point01_e[1] - slice_point01_s[1];
//            double len2_slice_edge = temp0*temp0 + temp1*temp1;
//            temp0 = moving_point01[0] - slice_point01_s[0];
//            temp1 = moving_point01[1] - slice_point01_s[1];
//            double len2 = temp0*temp0 + temp1*temp1;
//            while (len2_slice_edge > len2)
//            {
//                mark2DLineSpeed(moving_point01[0], moving_point01[1],
//                           moving_point02[0], moving_point02[1],
//                           axis_x, axis_y, axis_z, slice_idx);

//                moving_point01[0] += step01[0];
//                moving_point01[1] += step01[1];
//                moving_point02[0] += step02[0];
//                moving_point02[1] += step02[1];

//                // compute len2
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                len2 = temp0*temp0 + temp1*temp1;
//            }

//            // mark the last 2D scanline
//            mark2DLineSpeed(slice_point01_e[0], slice_point01_e[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx);
//        }
    }

}

void VoxelMesh::markSortedFaceINTSpeed(TriangleFace* facep)
{
    //cout << "INT facep->bounding_box_type " << facep->bounding_box_type << endl;
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPointSpeed(active_point);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    else if (facep->bounding_box_type == 3 ||
             facep->bounding_box_type == 4 ||
             facep->bounding_box_type == 5)
    {
        double p0[3], p1[3], p2[3];
        p0[0] = facep->v0[0];
        p0[1] = facep->v0[1];
        p0[2] = facep->v0[2];
        p1[0] = facep->v1[0];
        p1[1] = facep->v1[1];
        p1[2] = facep->v1[2];
        p2[0] = facep->v2[0];
        p2[1] = facep->v2[1];
        p2[2] = facep->v2[2];

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        int slice_id = facep->vox0[axis_z];

        // mark the three edges
        mark2DLineSpeed(p0[axis_x], p0[axis_y], p1[axis_x], p1[axis_y],
                        axis_x, axis_y, axis_z, slice_id);
        mark2DLineSpeed(p0[axis_x], p0[axis_y], p2[axis_x], p2[axis_y],
                        axis_x, axis_y, axis_z, slice_id);
        mark2DLineSpeed(p1[axis_x], p1[axis_y], p2[axis_x], p2[axis_y],
                        axis_x, axis_y, axis_z, slice_id);

        // compute the three edge vectors p0p1, p0p2 and p1p2, and their length
        double p0p1[2], p0p2[2], p1p2[2];
        p0p1[0] = p1[axis_x]-p0[axis_x];
        p0p1[1] = p1[axis_y]-p0[axis_y];
        p0p2[0] = p2[axis_x]-p0[axis_x];
        p0p2[1] = p2[axis_y]-p0[axis_y];
        p1p2[0] = p2[axis_x]-p1[axis_x];
        p1p2[1] = p2[axis_y]-p1[axis_y];

        // compute the optimal step distance
        // along the norm direction of p1p2
        double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
        double d_hat = 0.99*voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

        // compute the step distance along p0p1 corresponding to d_hat
        double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
        double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
        double step01[2];
        step01[0] = d01 * p0p1[0] / len01;
        step01[1] = d01 * p0p1[1] / len01;

        // compute the step distance along p0p2 corresponding to d_hat
        double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
        double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
        double step02[2];
        step02[0] = d02 * p0p2[0] / len02;
        step02[1] = d02 * p0p2[1] / len02;

        // two moving points will move along p0p1 and p0p2
        double moving_p0p1[2], moving_p0p2[2];
        moving_p0p1[0] = p0[axis_x] + step01[0];
        moving_p0p1[1] = p0[axis_y] + step01[1];
        moving_p0p2[0] = p0[axis_x] + step02[0];
        moving_p0p2[1] = p0[axis_y] + step02[1];

        // iterate and voxelize all the scanlines
        int size_steps = len01 / d01;
        for (int i=0; i<size_steps; ++i)
        {
            mark2DLineSpeed(moving_p0p1[0], moving_p0p1[1], moving_p0p2[0], moving_p0p2[1],
                            axis_x, axis_y, axis_z, slice_id);
            moving_p0p1[0] += step01[0];
            moving_p0p1[1] += step01[1];
            moving_p0p2[0] += step02[0];
            moving_p0p2[1] += step02[1];
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        double p0[3], p1[3], p2[3];
        p0[0] = facep->v0[0];
        p0[1] = facep->v0[1];
        p0[2] = facep->v0[2];
        p1[0] = facep->v1[0];
        p1[1] = facep->v1[1];
        p1[2] = facep->v1[2];
        p2[0] = facep->v2[0];
        p2[1] = facep->v2[1];
        p2[2] = facep->v2[2];

        int p0_vox[3], p1_vox[3], p2_vox[3];
        p0_vox[0] = facep->vox0[0];
        p0_vox[1] = facep->vox0[1];
        p0_vox[2] = facep->vox0[2];
        p1_vox[0] = facep->vox1[0];
        p1_vox[1] = facep->vox1[1];
        p1_vox[2] = facep->vox1[2];
        p2_vox[0] = facep->vox2[0];
        p2_vox[1] = facep->vox2[1];
        p2_vox[2] = facep->vox2[2];

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        int grid_den = grid_density;
        double vox_dim = voxel_dim;
        double min_x = coordinate_min[0];
        double min_y = coordinate_min[1];
        double min_z = coordinate_min[2];

        long long vox_increment_x, vox_increment_y;
        if (facep->bounding_box_type == 6)
        {
            // axis_x == 1, axis_y == 2
            vox_increment_x = grid_den;
            vox_increment_y = grid_den * (long long)grid_den;
        }
        else if (facep->bounding_box_type == 7)
        {
            // axis_x == 2, axis_y == 0
            vox_increment_x = grid_den * (long long)grid_den;
            vox_increment_y = 1;
        }
        else //if (facep->bounding_box_type == 8)
        {
            // axis_x == 0, axis_y == 1
            vox_increment_x = 1;
            vox_increment_y = grid_den;
        }

        // If the case is simple, no complicate computation is needed.
//        if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
//            p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
//        {
//            markPointSpeed(p0_vox[0] + (p0_vox[1] + p0_vox[2]*grid_den) * (long long)grid_den);
//            return;
//        }

        //-----------------------------------------------------------------------------
        // Divide the triangle into two parts.
        //-----------------------------------------------------------------------------
        double p11[3];
        if (p0_vox[axis_z] == p2_vox[axis_z])
        {
            // the triangle is within one xy slice.
            p11[0] = p0[0];
            p11[1] = p0[1];
            p11[2] = p0[2];
        }
        else
        {
            // Determine the 3D line segments of p0p2.
            double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
            double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
            double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

            double t1 = (p1[axis_z] - f1) / e1;
            p11[axis_x] = a1 * t1 + b1;
            p11[axis_y] = c1 * t1 + d1;

            p11[axis_z] = p1[axis_z];
        }

        int p11_vox[3];
        p11_vox[0] = floor((p11[0] - min_x) / vox_dim);
        if (p11_vox[0] < 0)
            p11_vox[0] = 0;
        else if (p11_vox[0] >= grid_den)
            p11_vox[0] = grid_den - 1;

        p11_vox[1] = floor((p11[1] - min_y) / vox_dim);
        if (p11_vox[1] < 0)
            p11_vox[1] = 0;
        else if (p11_vox[1] >= grid_den)
            p11_vox[1] = grid_den - 1;

        p11_vox[2] = floor((p11[2] - min_z) / vox_dim);
        if (p11_vox[2] < 0)
            p11_vox[2] = 0;
        else if (p11_vox[2] >= grid_den)
            p11_vox[2] = grid_den - 1;

        //-----------------------------------------------------------------------------
        // Voxelize the first part of the triangle slice by slice.
        //-----------------------------------------------------------------------------
        int voxsequ_1[LONGEST_EDGE][3];
        int voxsequ_2[LONGEST_EDGE][3];
        int size_1 = mark3DLineSpeed(p11, p0, p11_vox, p0_vox, voxsequ_1);
        int size_2 = mark3DLineSpeed(p1, p0, p1_vox, p0_vox, voxsequ_2);

        int start_slice = p11_vox[axis_z];
        int end_slice = p0_vox[axis_z];

        int i_1 = 0, i_2 = 0;
        for (int slice_i=start_slice; slice_i>=end_slice; --slice_i)
        {
            // the flag indicate if a sequence is run out.
            bool end_flag_1 = false, end_flag_2 = false;
            while (!(end_flag_1 && end_flag_2))
            {
                int *vox_1 = voxsequ_1[i_1];
                int *vox_2 = voxsequ_2[i_2];

                // Compute the step distance parameters.
                int start_pixel_x = vox_1[axis_x];
                int start_pixel_y = vox_1[axis_y];
                int end_pixel_x = vox_2[axis_x];
                int end_pixel_y = vox_2[axis_y];

                int delta_X = end_pixel_x - start_pixel_x;
                int delta_Y = end_pixel_y - start_pixel_y;

                long long moving_vox_idx = vox_1[0] + (vox_1[1] + vox_1[2]*grid_den) * (long long)grid_den;

                int sign_x, sign_y, length_x, length_y;
                long long moving_vox_x, moving_vox_y;
                if (delta_X >= 0)
                {
                    sign_x = 1;
                    length_x = delta_X;
                    moving_vox_x = vox_increment_x;
                }
                else
                {
                    sign_x = -1;
                    length_x = -delta_X;
                    moving_vox_x = -vox_increment_x;
                }

                if (delta_Y >= 0)
                {
                    sign_y = 1;
                    length_y = delta_Y;
                    moving_vox_y = vox_increment_y;
                }
                else
                {
                    sign_y = -1;
                    length_y = -delta_Y;
                    moving_vox_y = -vox_increment_y;
                }

                // Compute the step distance parameters.
                int t_x, t_y, step_x, step_y;
                if (delta_X != 0)
                {
                    if (delta_Y != 0)
                    {
                        // v_x != 0, v_y != 0.
                        t_x = length_y;
                        step_x = length_y << 1;  // step_x = v_y*2

                        t_y = length_x;
                        step_y = length_x << 1;  // step_y = v_x*2
                    }
                    else
                    {
                        // v_x != 0, v_y == 0.
                        t_x = 1;
                        step_x = 2;
                        t_y = 1000000000;
                    }
                }
                else if (delta_Y != 0)
                {
                    // v_x == 0, v_y != 0.
                    t_y = 1;
                    step_y = 2;
                    t_x = 1000000000;
                }

                // The core loop.
                int i_x = 0;
                int i_y = 0;
                while (!(i_x == length_x && i_y == length_y))
                {
                    markPointSpeed(moving_vox_idx);

                    if (t_x <= t_y)
                    {
                        t_y = t_y - t_x;
                        t_x = step_x;

                        i_x++;
                        moving_vox_idx += moving_vox_x;
                    }
                    else
                    {
                        t_x = t_x - t_y;
                        t_y = step_y;

                        i_y++;
                        moving_vox_idx += moving_vox_y;
                    }
                }

                // mark the last pixel
                markPointSpeed(moving_vox_idx);

                // Find the next scanline.
                int temp = delta_Y * end_pixel_x - delta_X * end_pixel_y;
                int abs_X_Y = length_x + length_y;
                int c1 = temp - abs_X_Y;
                int c2 = temp + abs_X_Y;

                // find the next vox in sequ1
                if (!end_flag_1)
                {
                    i_1++;

                    if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                    {
                        int i_curr = i_1 + 1;
                        while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_1[i_curr][axis_x]
                                   - delta_X * voxsequ_1[i_curr][axis_y];

                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_1++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    else
                    {
                        end_flag_1 = true;
                        i_1--;
                    }
                }

                // find the next vox in sequ2
                if (!end_flag_2)
                {
                    i_2++;

                    if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                    {
                        int i_curr = i_2 + 1;
                        while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_2[i_curr][axis_x]
                                   - delta_X * voxsequ_2[i_curr][axis_y];

                            if (c1<=cc && cc<=c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_2++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    else
                    {
                        end_flag_2 = true;
                        i_2--;
                    }
                }

            } // end while

            // move to next slice.
            i_1++;
            i_2++;

        } // end for

        //-----------------------------------------------------------------------------
        // Voxelize the second part of the triangle slice by slice.
        //-----------------------------------------------------------------------------
        size_1 = mark3DLineSpeed(p11, p2, p11_vox, p2_vox, voxsequ_1);
        size_2 = mark3DLineSpeed(p1, p2, p1_vox, p2_vox, voxsequ_2);

        start_slice = p11_vox[axis_z];
        end_slice = p2_vox[axis_z];

        i_1 = 0;
        i_2 = 0;
        for (int slice_i=start_slice; slice_i<=end_slice; ++slice_i)
        {
            // the flag indicate if a sequence is run out.
            bool end_flag_1 = false, end_flag_2 = false;
            while (!(end_flag_1 && end_flag_2))
            {
                int *vox_1 = voxsequ_1[i_1];
                int *vox_2 = voxsequ_2[i_2];

                // Compute the step distance parameters.
                int start_pixel_x = vox_1[axis_x];
                int start_pixel_y = vox_1[axis_y];
                int end_pixel_x = vox_2[axis_x];
                int end_pixel_y = vox_2[axis_y];

                int delta_X = end_pixel_x - start_pixel_x;
                int delta_Y = end_pixel_y - start_pixel_y;

                long long moving_vox_idx = vox_1[0] + (vox_1[1] + vox_1[2]*grid_den) * (long long)grid_den;

                int sign_x, sign_y, length_x, length_y;
                long long moving_vox_x, moving_vox_y;
                if (delta_X >= 0)
                {
                    sign_x = 1;
                    length_x = delta_X;
                    moving_vox_x = vox_increment_x;
                }
                else
                {
                    sign_x = -1;
                    length_x = -delta_X;
                    moving_vox_x = -vox_increment_x;
                }

                if (delta_Y >= 0)
                {
                    sign_y = 1;
                    length_y = delta_Y;
                    moving_vox_y = vox_increment_y;
                }
                else
                {
                    sign_y = -1;
                    length_y = -delta_Y;
                    moving_vox_y = -vox_increment_y;
                }

                // Compute the step distance parameters.
                int t_x, t_y, step_x, step_y;
                if (delta_X != 0)
                {
                    if (delta_Y != 0)
                    {
                        // v_x != 0, v_y != 0.
                        t_x = length_y;
                        step_x = length_y << 1;  // step_x = v_y*2

                        t_y = length_x;
                        step_y = length_x << 1;  // step_y = v_x*2
                    }
                    else
                    {
                        // v_x != 0, v_y == 0.
                        t_x = 1;
                        step_x = 2;
                        t_y = 1000000000;
                    }
                }
                else if (delta_Y != 0)
                {
                    // v_x == 0, v_y != 0.
                    t_y = 1;
                    step_y = 2;
                    t_x = 1000000000;
                }

                // The core loop.
                int i_x = 0;
                int i_y = 0;
                while (!(i_x == length_x && i_y == length_y))
                {
                    markPointSpeed(moving_vox_idx);

                    if (t_x <= t_y)
                    {
                        t_y = t_y - t_x;
                        t_x = step_x;

                        i_x++;
                        moving_vox_idx += moving_vox_x;
                    }
                    else
                    {
                        t_x = t_x - t_y;
                        t_y = step_y;

                        i_y++;
                        moving_vox_idx += moving_vox_y;
                    }
                }

                // mark the last pixel
                markPointSpeed(moving_vox_idx);

                // Find the next scanline.
                int temp = delta_Y * end_pixel_x - delta_X * end_pixel_y;
                int abs_X_Y = length_x + length_y;
                int c1 = temp - abs_X_Y;
                int c2 = temp + abs_X_Y;

                // find the next vox in sequ1
                if (!end_flag_1)
                {
                    i_1++;

                    if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                    {
                        int i_curr = i_1 + 1;
                        while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_1[i_curr][axis_x]
                                   - delta_X * voxsequ_1[i_curr][axis_y];

                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_1++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    else
                    {
                        end_flag_1 = true;
                        i_1--;
                    }
                }

                // find the next vox in sequ2
                if (!end_flag_2)
                {
                    i_2++;

                    if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                    {
                        int i_curr = i_2 + 1;
                        while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_2[i_curr][axis_x]
                                   - delta_X * voxsequ_2[i_curr][axis_y];

                            if (c1<=cc && cc<=c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_2++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                    else
                    {
                        end_flag_2 = true;
                        i_2--;
                    }
                }

            } // end while

            // move to next slice.
            i_1++;
            i_2++;

        } // end for

    }

}

void VoxelMesh::markFaceBySATMethodSpeed(TriangleFace* facep)
{
    //Get the 3 vertices of the face.
    double v[3][3];
    v[0][0] = facep->node0->ori_coordinate[0];
    v[0][1] = facep->node0->ori_coordinate[1];
    v[0][2] = facep->node0->ori_coordinate[2];
    v[1][0] = facep->node1->ori_coordinate[0];
    v[1][1] = facep->node1->ori_coordinate[1];
    v[1][2] = facep->node1->ori_coordinate[2];
    v[2][0] = facep->node2->ori_coordinate[0];
    v[2][1] = facep->node2->ori_coordinate[1];
    v[2][2] = facep->node2->ori_coordinate[2];

//    cout << "v1 = " << v[0][0] << " " << v[0][1] << " " << v[0][2] << endl;
//    cout << "v2 = " << v[1][0] << " " << v[1][1] << " " << v[1][2] << endl;
//    cout << "v3 = " << v[2][0] << " " << v[2][1] << " " << v[2][2] << endl;

    //Check if this is invalid face, i.e. only one line or point.
    if ((v[0][0] == v[1][0] && v[0][1] == v[1][1] && v[0][2] == v[1][2]) ||
        (v[0][0] == v[2][0] && v[0][1] == v[2][1] && v[0][2] == v[2][2]) ||
        (v[1][0] == v[2][0] && v[1][1] == v[2][1] && v[1][2] == v[2][2]))
    {
        //cout << "A face contains at least two identical vetex points!" << endl;
        return;
    }

    //-------------------------------------------------------------------
    // Pre-computation for triangle plane overlapping voxel.
    //-------------------------------------------------------------------
    // compute the normal of the triangle plane.
    // compute the norm, i.e. (p1p0)x(p2p0).
    double v1[3], v2[3];
    v1[0] = v[1][0] - v[0][0];
    v1[1] = v[1][1] - v[0][1];
    v1[2] = v[1][2] - v[0][2];

    v2[0] = v[2][0] - v[0][0];
    v2[1] = v[2][1] - v[0][1];
    v2[2] = v[2][2] - v[0][2];

    double norm[3];
    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

    double diagonal_vector[3];
    diagonal_vector[0] = voxel_dim;
    diagonal_vector[1] = voxel_dim;
    diagonal_vector[2] = voxel_dim;

    // compute the critical point.
    // critical point is to adjust the minimum/maximum corner of the voxel
    // according to the triangle.
    double critical_point[3];
    assignPoint(critical_point, diagonal_vector);

    if (norm[0] <= 0.0)
        critical_point[0] = 0.0;
    if (norm[1] <= 0.0)
        critical_point[1] = 0.0;
    if (norm[2] <= 0.0)
        critical_point[2] = 0.0;

    // compute d1/d2, which are parts of the criterion.
    double d1 = norm[0]*(critical_point[0] - v[0][0]) +
                norm[1]*(critical_point[1] - v[0][1]) +
                norm[2]*(critical_point[2] - v[0][2]);

    double d2 = norm[0]*(diagonal_vector[0] - critical_point[0] - v[0][0]) +
                norm[1]*(diagonal_vector[1] - critical_point[1] - v[0][1]) +
                norm[2]*(diagonal_vector[2] - critical_point[2] - v[0][2]);

    //-------------------------------------------------------------------------
    // Pre-computation for projections of the triangle overlapping the voxel.
    //-------------------------------------------------------------------------
    // compute the edge vectors.
    double edge[3][3];
    edge[0][0] = v[1][0] - v[0][0];
    edge[0][1] = v[1][1] - v[0][1];
    edge[0][2] = v[1][2] - v[0][2];

    edge[1][0] = v[2][0] - v[1][0];
    edge[1][1] = v[2][1] - v[1][1];
    edge[1][2] = v[2][2] - v[1][2];

    edge[2][0] = v[0][0] - v[2][0];
    edge[2][1] = v[0][1] - v[2][1];
    edge[2][2] = v[0][2] - v[2][2];

    // the 2D edges.
    double n_xy_edge[3][2], n_yz_edge[3][2], n_xz_edge[3][2];
    double d_xy_edge[3], d_yz_edge[3], d_xz_edge[3];

    for (int i=0; i<3; ++i)
    {
        //----------------------------------------------------
        // for projection on xy plane.
        //----------------------------------------------------
        n_xy_edge[i][0] = -edge[i][1];
        n_xy_edge[i][1] = edge[i][0];
        if (norm[2] < 0.0)
        {
            n_xy_edge[i][0] = -n_xy_edge[i][0];
            n_xy_edge[i][1] = -n_xy_edge[i][1];
        }

        //n_xy_edge[i] = n_xy_edge[i].normalized();

        double c1 = n_xy_edge[i][0] * diagonal_vector[0];
        if (c1 < 0.0)
        {
            c1 = 0.0;
        }
        double c2 = n_xy_edge[i][1] * diagonal_vector[1];
        if (c2 < 0.0)
        {
            c2 = 0.0;
        }
        d_xy_edge[i] = c1 + c2 -
                       n_xy_edge[i][0] * v[i][0] -
                       n_xy_edge[i][1] * v[i][1];

        //----------------------------------------------------
        //for projection on yz plane.
        //----------------------------------------------------
        n_yz_edge[i][0] = -edge[i][2];
        n_yz_edge[i][1] = edge[i][1];
        if (norm[0] < 0.0)
        {
            n_yz_edge[i][0] = -n_yz_edge[i][0];
            n_yz_edge[i][1] = -n_yz_edge[i][1];
        }

        //n_yz_edge[i] = n_yz_edge[i].normalized();

        c1 = n_yz_edge[i][0] * diagonal_vector[1];
        if (c1 < 0.0)
        {
            c1 = 0.0;
        }
        c2 = n_yz_edge[i][1] * diagonal_vector[2];
        if (c2 < 0.0)
        {
            c2 = 0.0;
        }
        d_yz_edge[i] = c1 + c2 -
                       n_yz_edge[i][0] * v[i][1] -
                       n_yz_edge[i][1] * v[i][2];

        //----------------------------------------------------
        //for projection on xz plane.
        //----------------------------------------------------
        n_xz_edge[i][0] = -edge[i][0];
        n_xz_edge[i][1] = edge[i][2];
        if (norm[1] < 0.0)
        {
            n_xz_edge[i][0] = -n_xz_edge[i][0];
            n_xz_edge[i][1] = -n_xz_edge[i][1];
        }

        //n_xz_edge[i] = n_xz_edge[i].normalized();

        c1 = n_xz_edge[i][0] * diagonal_vector[2];
        if (c1 < 0.0)
        {
            c1 = 0.0;
        }
        c2 = n_xz_edge[i][1] * diagonal_vector[0];
        if (c2 < 0.0)
        {
            c2 = 0.0;
        }
        d_xz_edge[i] = c1 + c2 -
                       n_xz_edge[i][0] * v[i][2] -
                       n_xz_edge[i][1] * v[i][0];

    }

    //--------------------------------------------------------------------
    //Compute the maxima and minima of the vetices coordinates.
    //--------------------------------------------------------------------
    double v_min[3], v_max[3];
    assignPoint(v_min, v[0]);
    assignPoint(v_max, v[0]);

    for (int i=0; i<3; ++i)
    {
        if (v_min[i] > v[1][i])
            v_min[i] = v[1][i];
        if (v_min[i] > v[2][i])
            v_min[i] = v[2][i];

        if (v_max[i] < v[1][i])
            v_max[i] = v[1][i];
        if (v_max[i] < v[2][i])
            v_max[i] = v[2][i];
    }

    //---------------------------------------------------------------------
    //Compute the voxel index of v_min and v_max, i.e. the bounding box.
    //---------------------------------------------------------------------
    int vox_min[3], vox_max[3];
    computeVoxIdx(vox_min, v_min);
    computeVoxIdx(vox_max, v_max);

    //----------------------------------------------------------------------
    //Search and mark all overlapped voxels within the bounding box.
    //----------------------------------------------------------------------
    double corner_min[3];
    for (int xi = vox_min[0]; xi <= vox_max[0]; ++xi)
    {
        corner_min[0] = double(xi) * voxel_dim + coordinate_min[0];
        for (int yi = vox_min[1]; yi <= vox_max[1]; ++yi)
        {
            corner_min[1] = double(yi) * voxel_dim + coordinate_min[1];
            for (int zi = vox_min[2]; zi <= vox_max[2]; ++zi)
            {
                corner_min[2] = double(zi) * voxel_dim + coordinate_min[2];

                double np = norm[0]*corner_min[0] +
                            norm[1]*corner_min[1] +
                            norm[2]*corner_min[2];

                if ((np + d1) * (np + d2) <= 0.0)
                {
                    bool flag_overlap = true;

                    for (int i=0; i<3; i++)
                    {
                        //for projection on xy plane.
                        double temp;
                        temp = d_xy_edge[i]
                               + n_xy_edge[i][0] * corner_min[0]
                               + n_xy_edge[i][1] * corner_min[1];
                        if (temp < 0.0)
                        {
                            flag_overlap = false;
                            break;
                        }

                        //for projection on yz plane.
                        temp = d_yz_edge[i]
                               + n_yz_edge[i][0] * corner_min[1]
                               + n_yz_edge[i][1] * corner_min[2];
                        if (temp < 0.0)
                        {
                            flag_overlap = false;
                            break;
                        }

                        //for projection on zx plane.
                        temp = d_xz_edge[i]
                               + n_xz_edge[i][0] * corner_min[2]
                               + n_xz_edge[i][1] * corner_min[0];
                        if (temp < 0.0)
                        {
                            flag_overlap = false;
                            break;
                        }

                    }

                    if (flag_overlap)
                    {
                        //overlap, mark the voxel.
                        //voxel_map[xi + (yi + zi*grid_density)*(long long)grid_density] = 1;
                        markPointSpeed(xi + (yi + zi*grid_density)*(long long)grid_density);
                    }
                } // endf if
            }
        }
    }
}

void VoxelMesh::markFaceByScanlineSpeed(TriangleFace* facep)
{
    //Get the 3 vetex of the face.
    double p0[3], p1[3], p2[3];
    p0[0] = facep->node0->ori_coordinate[0];
    p0[1] = facep->node0->ori_coordinate[1];
    p0[2] = facep->node0->ori_coordinate[2];
    p1[0] = facep->node1->ori_coordinate[0];
    p1[1] = facep->node1->ori_coordinate[1];
    p1[2] = facep->node1->ori_coordinate[2];
    p2[0] = facep->node2->ori_coordinate[0];
    p2[1] = facep->node2->ori_coordinate[1];
    p2[2] = facep->node2->ori_coordinate[2];

    // Compute the normal of the triangle plane, i.e. axis_z.
    int axis_z = computeDomiAxis(p0, p1, p2);

    // Sort the three voxels by order of dominant component.
    sortVertByDomiAxis(p0, p1, p2, axis_z);

    // voxelize the 3 vertices.
    int p0_vox[3], p1_vox[3], p2_vox[3];
    computeVoxIdx(p0_vox, p0);
    computeVoxIdx(p1_vox, p1);
    computeVoxIdx(p2_vox, p2);

    // If the case is simple, no complicate computation is needed.
    if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
        p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
    {
        markPointSpeed(p0_vox[0] + (p0_vox[1] + p0_vox[2]*grid_density) * (long long)grid_density);
        return;
    }

    //-----------------------------------------------------------------------------
    // Mark the three edges.
    //-----------------------------------------------------------------------------
    mark3DLineSpeed(p0, p2, p0_vox, p2_vox);
    mark3DLineSpeed(p0, p1, p0_vox, p1_vox);
    mark3DLineSpeed(p1, p2, p1_vox, p2_vox);

    //-----------------------------------------------------------------------------
    // Redefine the x y z coordinates by the domi axis.
    //-----------------------------------------------------------------------------
    int axis_x, axis_y;
    if (axis_z == 0)
    {
        axis_x = 1;
        axis_y = 2;
    }
    else if (axis_z == 1)
    {
        axis_x = 2;
        axis_y = 0;
    }
    else
    {
        axis_x = 0;
        axis_y = 1;
    }

    //-----------------------------------------------------------------------------
    // Determine the three line segments.
    //-----------------------------------------------------------------------------
    // Determine the 3D line segments of p0p2.
    double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
    double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
    double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

    // Determine the 3D line segments of p0p1.
    double a2 = p1[axis_x]-p0[axis_x], b2 = p0[axis_x];
    double c2 = p1[axis_y]-p0[axis_y], d2 = p0[axis_y];
    double e2 = p1[axis_z]-p0[axis_z], f2 = p0[axis_z];

    //-----------------------------------------------------------------------------
    // Determine the typical scanline passing p1 and pre-compute the delta steps.
    //-----------------------------------------------------------------------------
    double scp0_x, scp0_y, scp1_x, scp1_y;
    scp1_x = p1[axis_x];
    scp1_y = p1[axis_y];

    if (p0_vox[axis_z] == p2_vox[axis_z])
    {
        // The triangle is within one xy slice.
        // Simply let the typical scanline be p0p1.
        scp0_x = p0[axis_x];
        scp0_y = p0[axis_y];
    }
    else
    {
        // The triangle is within more than one xy slice.
        // Let the typical scanline be the intersection with the plane passing p1.
        double t1 = (p1[axis_z] - f1) / e1;
        scp0_x = a1 * t1 + b1;
        scp0_y = c1 * t1 + d1;
    }

    // compute the norm of the scanline.
    double nx = scp0_y - scp1_y;
    double ny = scp1_x - scp0_x;

    // make the norm direction identical to p0p2
    if (nx * a1 + ny * c1 < 0.0)
    {
        nx = - nx;
        ny = - ny;
    }

    // normalize the norm.
    double norm_length = sqrt(nx*nx + ny*ny);
    double vox_dim = 0.99*voxel_dim*(abs(nx)+abs(ny)) / norm_length;
    nx = nx / norm_length;
    ny = ny / norm_length;

    // compute the 2D step vectors along p0p2.
    //double vox_dim = 0.99 * voxel_dim;
    double step_1_x, step_1_y, step_1_length;
    double temp = vox_dim / (nx*a1 + ny*c1);
    step_1_x = a1 * temp;
    step_1_y = c1 * temp;
    step_1_length = step_1_x*step_1_x + step_1_y*step_1_y;

    // compute the 2D step vectors along p0p1.
    double step_2_x, step_2_y;
    temp = vox_dim / (nx*a2 + ny*c2);
    step_2_x = a2 * temp;
    step_2_y = c2 * temp;

    //-----------------------------------------------------------------------------
    // Voxelize the first half of the triangle.
    //-----------------------------------------------------------------------------
    // Determine the first slice.
    int slice_idx = p0_vox[axis_z];
    double plane = voxel_dim * (slice_idx + 1) + coordinate_min[axis_z];
//    if (plane == p0[axis_z])
//    {
//        // the plane happened to intersect p0,
//        // refine the slice index and move to next plane.
//        slice_idx = computeVoxOneIdx(p0[axis_z]+0.5*voxel_dim, axis_z);

//        plane += voxel_dim;
//    }

    // the first scanline is a point, i.e. p0.
    double startp0_x, startp0_y, startp1_x, startp1_y;
    startp0_x = p0[axis_x];
    startp0_y = p0[axis_y];
    startp1_x = startp0_x;
    startp1_y = startp0_y;

    double endp0_x, endp0_y, endp1_x, endp1_y;
    double length;
    while  (slice_idx < p1_vox[axis_z])
    {
        double t1 = (plane - f1) / e1;
        endp0_x = a1 * t1 + b1;
        endp0_y = c1 * t1 + d1;

        double t2 = (plane - f2) / e2;
        endp1_x = a2 * t2 + b2;
        endp1_y = c2 * t2 + d2;

        length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
        while (length > 0.0)
        {
            mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                            axis_x, axis_y, axis_z, slice_idx);

            startp0_x += step_1_x;
            startp0_y += step_1_y;
            startp1_x += step_2_x;
            startp1_y += step_2_y;

            length -= step_1_length;
        }

        startp0_x = endp0_x;
        startp0_y = endp0_y;
        startp1_x = endp1_x;
        startp1_y = endp1_y;
        mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                        axis_x, axis_y, axis_z, slice_idx);

        plane += voxel_dim;
        slice_idx++;
    }

    //-----------------------------------------------------------------------------
    // Voxelize the slice containing the typical scanline passing p1.
    //-----------------------------------------------------------------------------
    endp0_x = scp0_x;
    endp0_y = scp0_y;
    endp1_x = scp1_x;
    endp1_y = scp1_y;

    length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
    while (length > 0.0)
    {
        mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                        axis_x, axis_y, axis_z, slice_idx);

        startp0_x += step_1_x;
        startp0_y += step_1_y;
        startp1_x += step_2_x;
        startp1_y += step_2_y;

        length -= step_1_length;
    }
    mark2DLineSpeed(endp0_x, endp0_y, endp1_x, endp1_y,
                    axis_x, axis_y, axis_z, slice_idx);

    // check if the plane happened to pass p1
//    if (plane == p1[axis_z])
//    {
//        // the plane happened to intersect p1,
//        // refine the slice index and move to next plane.
//        slice_idx = computeVoxOneIdx(p1[axis_z]+0.5*voxel_dim, axis_z);

//        plane += voxel_dim;
//    }

    //-----------------------------------------------------------------------------
    // voxelize the second half of the triangle.
    //-----------------------------------------------------------------------------
    // Determine the 3D line segments of p0p1.
    a2 = p2[axis_x]-p1[axis_x]; b2 = p1[axis_x];
    c2 = p2[axis_y]-p1[axis_y]; d2 = p1[axis_y];
    e2 = p2[axis_z]-p1[axis_z]; f2 = p1[axis_z];

    // compute the 2D step vectors along p1p2.
    temp = vox_dim / (nx*a2 + ny*c2);
    step_2_x = a2 * temp;
    step_2_y = c2 * temp;

    // Start from the typical scanline.
    startp0_x = scp0_x;
    startp0_y = scp0_y;
    startp1_x = scp1_x;
    startp1_y = scp1_y;

    while (slice_idx < p2_vox[axis_z])
    {
        double t1 = (plane - f1) / e1;
        endp0_x = a1 * t1 + b1;
        endp0_y = c1 * t1 + d1;

        double t2 = (plane - f2) / e2;
        endp1_x = a2 * t2 + b2;
        endp1_y = c2 * t2 + d2;

        length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
        while (length > 0.0)
        {
            mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                            axis_x, axis_y, axis_z, slice_idx);

            startp0_x += step_1_x;
            startp0_y += step_1_y;
            startp1_x += step_2_x;
            startp1_y += step_2_y;

            length -= step_1_length;
        }

        startp0_x = endp0_x;
        startp0_y = endp0_y;
        startp1_x = endp1_x;
        startp1_y = endp1_y;
        mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                        axis_x, axis_y, axis_z, slice_idx);

        plane += voxel_dim;
        slice_idx++;
    }

    // voxelize the last slice.
    length = (p2[axis_x] - startp0_x)*step_1_x + (p2[axis_y] - startp0_y)*step_1_y;
    while (length > 0.0)
    {
        mark2DLineSpeed(startp0_x, startp0_y, startp1_x, startp1_y,
                        axis_x, axis_y, axis_z, slice_idx);

        startp0_x += step_1_x;
        startp0_y += step_1_y;
        startp1_x += step_2_x;
        startp1_y += step_2_y;

        length -= step_1_length;
    }
}

void VoxelMesh::markFaceByScanlineIntegerSpeed(TriangleFace* facep)
{
    //Get the 3 vetex of the face.
    double p0[3], p1[3], p2[3];
    p0[0] = facep->node0->ori_coordinate[0];
    p0[1] = facep->node0->ori_coordinate[1];
    p0[2] = facep->node0->ori_coordinate[2];
    p1[0] = facep->node1->ori_coordinate[0];
    p1[1] = facep->node1->ori_coordinate[1];
    p1[2] = facep->node1->ori_coordinate[2];
    p2[0] = facep->node2->ori_coordinate[0];
    p2[1] = facep->node2->ori_coordinate[1];
    p2[2] = facep->node2->ori_coordinate[2];

    int grid_den = grid_density;
    double vox_dim = voxel_dim;
    double min_x = coordinate_min[0];
    double min_y = coordinate_min[1];
    double min_z = coordinate_min[2];

    double v1[3], v2[3];
    v1[0] = p1[0] - p0[0];
    v1[1] = p1[1] - p0[1];
    v1[2] = p1[2] - p0[2];

    v2[0] = p2[0] - p0[0];
    v2[1] = p2[1] - p0[1];
    v2[2] = p2[2] - p0[2];

    // compute the norm, i.e. (p1p0)x(p2p0).
    double norm[3];
    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

    // take the absolute value of the norm.
    if (norm[0] < 0.0)
        norm[0] = -norm[0];

    if (norm[1] < 0.0)
        norm[1] = -norm[1];

    if (norm[2] < 0.0)
        norm[2] = -norm[2];

    // find the dominant direction.
    int axis_x, axis_y, axis_z;
    long long vox_increment_x, vox_increment_y;
    if (norm[0] < norm[1])
    {
        if (norm[1] < norm[2])  // 0<1<2
        {
            axis_z = 2;
            axis_x = 0;
            axis_y = 1;

            vox_increment_x = 1;
            vox_increment_y = grid_den;
        }
        else // 0<1, 2<=1
        {
            axis_z = 1;
            axis_x = 0;
            axis_y = 2;

            vox_increment_x = 1;
            vox_increment_y = grid_den * (long long)grid_den;
        }
    }
    else if (norm[0] < norm[2]) // 1<=0, 0<2
    {
        axis_z = 2;
        axis_x = 0;
        axis_y = 1;

        vox_increment_x = 1;
        vox_increment_y = grid_den;
    }
    else
    {
        axis_z = 0;
        axis_x = 1;
        axis_y = 2;

        vox_increment_x = grid_den;
        vox_increment_y = grid_den * (long long)grid_den;
    }

    // Sort the three voxels by the order of the dominant component (axis_z).
    //sortVertByDomiAxis(p0, p1, p2, axis_z);
    if (p0[axis_z] <= p1[axis_z])
    {
        if (p1[axis_z] <= p2[axis_z])  //p0 <= p1 <= p2
        {
            ;
        }
        else
        {
            if (p0[axis_z] <= p2[axis_z])  //p0 <= p2 < p1
            {
                double temp;
                temp = p1[0]; p1[0] = p2[0]; p2[0] = temp;
                temp = p1[1]; p1[1] = p2[1]; p2[1] = temp;
                temp = p1[2]; p1[2] = p2[2]; p2[2] = temp;
            }
            else   //p2 < p0 <= p1
            {
                double temp;
                temp = p2[0]; p2[0] = p1[0]; p1[0] = p0[0]; p0[0] = temp;
                temp = p2[1]; p2[1] = p1[1]; p1[1] = p0[1]; p0[1] = temp;
                temp = p2[2]; p2[2] = p1[2]; p1[2] = p0[2]; p0[2] = temp;
            }
        }
    }
    else
    {
        if (p1[axis_z] >= p2[axis_z])  //p2 <= p1 < p0
        {
            double temp;
            temp = p0[0]; p0[0] = p2[0]; p2[0] = temp;
            temp = p0[1]; p0[1] = p2[1]; p2[1] = temp;
            temp = p0[2]; p0[2] = p2[2]; p2[2] = temp;
        }
        else
        {
            if (p0[axis_z] <= p2[axis_z])  //p1 < p0 <= p2
            {
                double temp;
                temp = p0[0]; p0[0] = p1[0]; p1[0] = temp;
                temp = p0[1]; p0[1] = p1[1]; p1[1] = temp;
                temp = p0[2]; p0[2] = p1[2]; p1[2] = temp;
            }
            else   //p1 < p2 < p0
            {
                double temp;
                temp = p0[0]; p0[0] = p1[0]; p1[0] = p2[0]; p2[0] = temp;
                temp = p0[1]; p0[1] = p1[1]; p1[1] = p2[1]; p2[1] = temp;
                temp = p0[2]; p0[2] = p1[2]; p1[2] = p2[2]; p2[2] = temp;
            }
        }
    }

    // voxelize the 3 vertices.
    int p0_vox[3], p1_vox[3], p2_vox[3];
    p0_vox[0] = floor((p0[0] - min_x) / vox_dim);
    if (p0_vox[0] < 0)
        p0_vox[0] = 0;
    else if (p0_vox[0] >= grid_den)
        p0_vox[0] = grid_den - 1;

    p0_vox[1] = floor((p0[1] - min_y) / vox_dim);
    if (p0_vox[1] < 0)
        p0_vox[1] = 0;
    else if (p0_vox[1] >= grid_den)
        p0_vox[1] = grid_den - 1;

    p0_vox[2] = floor((p0[2] - min_z) / vox_dim);
    if (p0_vox[2] < 0)
        p0_vox[2] = 0;
    else if (p0_vox[2] >= grid_den)
        p0_vox[2] = grid_den - 1;

    p1_vox[0] = floor((p1[0] - min_x) / vox_dim);
    if (p1_vox[0] < 0)
        p1_vox[0] = 0;
    else if (p1_vox[0] >= grid_den)
        p1_vox[0] = grid_den - 1;

    p1_vox[1] = floor((p1[1] - min_y) / vox_dim);
    if (p1_vox[1] < 0)
        p1_vox[1] = 0;
    else if (p1_vox[1] >= grid_den)
        p1_vox[1] = grid_den - 1;

    p1_vox[2] = floor((p1[2] - min_z) / vox_dim);
    if (p1_vox[2] < 0)
        p1_vox[2] = 0;
    else if (p1_vox[2] >= grid_den)
        p1_vox[2] = grid_den - 1;

    p2_vox[0] = floor((p2[0] - min_x) / vox_dim);
    if (p2_vox[0] < 0)
        p2_vox[0] = 0;
    else if (p2_vox[0] >= grid_den)
        p2_vox[0] = grid_den - 1;

    p2_vox[1] = floor((p2[1] - min_y) / vox_dim);
    if (p2_vox[1] < 0)
        p2_vox[1] = 0;
    else if (p2_vox[1] >= grid_den)
        p2_vox[1] = grid_den - 1;

    p2_vox[2] = floor((p2[2] - min_z) / vox_dim);
    if (p2_vox[2] < 0)
        p2_vox[2] = 0;
    else if (p2_vox[2] >= grid_den)
        p2_vox[2] = grid_den - 1;

    // If the case is simple, no complicate computation is needed.
    if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
        p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
    {
        markPointSpeed(p0_vox[0] + (p0_vox[1] + p0_vox[2]*grid_den) * (long long)grid_den);
        return;
    }

    //-----------------------------------------------------------------------------
    // Divide the triangle into two parts.
    //-----------------------------------------------------------------------------
    double p11[3];
    if (p0_vox[axis_z] == p2_vox[axis_z])
    {
        // the triangle is within one xy slice.
        p11[0] = p0[0];
        p11[1] = p0[1];
        p11[2] = p0[2];
    }
    else
    {
        // Determine the 3D line segments of p0p2.
        double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
        double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
        double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

        double t1 = (p1[axis_z] - f1) / e1;
        p11[axis_x] = a1 * t1 + b1;
        p11[axis_y] = c1 * t1 + d1;

        p11[axis_z] = p1[axis_z];
    }

    int p11_vox[3];
    p11_vox[0] = floor((p11[0] - min_x) / vox_dim);
    if (p11_vox[0] < 0)
        p11_vox[0] = 0;
    else if (p11_vox[0] >= grid_den)
        p11_vox[0] = grid_den - 1;

    p11_vox[1] = floor((p11[1] - min_y) / vox_dim);
    if (p11_vox[1] < 0)
        p11_vox[1] = 0;
    else if (p11_vox[1] >= grid_den)
        p11_vox[1] = grid_den - 1;

    p11_vox[2] = floor((p11[2] - min_z) / vox_dim);
    if (p11_vox[2] < 0)
        p11_vox[2] = 0;
    else if (p11_vox[2] >= grid_den)
        p11_vox[2] = grid_den - 1;

    //-----------------------------------------------------------------------------
    // Voxelize the first part of the triangle slice by slice.
    //-----------------------------------------------------------------------------
    int voxsequ_1[LONGEST_EDGE][3];
    int voxsequ_2[LONGEST_EDGE][3];
    int size_1 = mark3DLineSpeed(p11, p0, p11_vox, p0_vox, voxsequ_1);
    int size_2 = mark3DLineSpeed(p1, p0, p1_vox, p0_vox, voxsequ_2);

    int start_slice = p11_vox[axis_z];
    int end_slice = p0_vox[axis_z];

    int i_1 = 0, i_2 = 0;
    for (int slice_i=start_slice; slice_i>=end_slice; --slice_i)
    {
        // the flag indicate if a sequence is run out.
        bool end_flag_1 = false, end_flag_2 = false;
        while (!(end_flag_1 && end_flag_2))
        {
            int *vox_1 = voxsequ_1[i_1];
            int *vox_2 = voxsequ_2[i_2];

//            mark2DScanlineInteger(vox_1[axis_x], vox_1[axis_y],
//                                  vox_2[axis_x], vox_2[axis_y],
//                                  axis_x, axis_y, axis_z, slice_i, marker);

            // Compute the step distance parameters.
            int start_pixel_x = vox_1[axis_x];
            int start_pixel_y = vox_1[axis_y];
            int end_pixel_x = vox_2[axis_x];
            int end_pixel_y = vox_2[axis_y];

            int delta_X = end_pixel_x - start_pixel_x;
            int delta_Y = end_pixel_y - start_pixel_y;

            long long moving_vox_idx = vox_1[0] + (vox_1[1] + vox_1[2]*grid_den) * (long long)grid_den;

            int sign_x, sign_y, length_x, length_y;
            long long moving_vox_x, moving_vox_y;
            if (delta_X >= 0)
            {
                sign_x = 1;
                length_x = delta_X;
                moving_vox_x = vox_increment_x;
            }
            else
            {
                sign_x = -1;
                length_x = -delta_X;
                moving_vox_x = -vox_increment_x;
            }

            if (delta_Y >= 0)
            {
                sign_y = 1;
                length_y = delta_Y;
                moving_vox_y = vox_increment_y;
            }
            else
            {
                sign_y = -1;
                length_y = -delta_Y;
                moving_vox_y = -vox_increment_y;
            }

            // Compute the step distance parameters.
            int t_x, t_y, step_x, step_y;
            if (delta_X != 0)
            {
                if (delta_Y != 0)
                {
                    // v_x != 0, v_y != 0.
                    t_x = length_y;
                    step_x = length_y << 1;  // step_x = v_y*2

                    t_y = length_x;
                    step_y = length_x << 1;  // step_y = v_x*2
                }
                else
                {
                    // v_x != 0, v_y == 0.
                    t_x = 1;
                    step_x = 2;
                    t_y = 1000000000;
                }
            }
            else if (delta_Y != 0)
            {
                // v_x == 0, v_y != 0.
                t_y = 1;
                step_y = 2;
                t_x = 1000000000;
            }

            // The core loop.
            int i_x = 0;
            int i_y = 0;
            while (!(i_x == length_x && i_y == length_y))
            {
                markPoint(moving_vox_idx, 1);

                if (t_x <= t_y)
                {
                    t_y = t_y - t_x;
                    t_x = step_x;

                    i_x++;
                    moving_vox_idx += moving_vox_x;
                }
                else
                {
                    t_x = t_x - t_y;
                    t_y = step_y;

                    i_y++;
                    moving_vox_idx += moving_vox_y;
                }
            }

            // mark the last pixel
            markPoint(moving_vox_idx, 1);

            // Find the next scanline.
            int temp = delta_Y * end_pixel_x - delta_X * end_pixel_y;
            int abs_X_Y = length_x + length_y;
            int c1 = temp - abs_X_Y;
            int c2 = temp + abs_X_Y;

            // find the next vox in sequ1
            if (!end_flag_1)
            {
                i_1++;

                if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                {
                    int i_curr = i_1 + 1;
                    while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                    {
                        int cc = delta_Y * voxsequ_1[i_curr][axis_x]
                               - delta_X * voxsequ_1[i_curr][axis_y];

                        if (c1<cc && cc<c2)
                        {
                            // the voxel is safe.
                            i_curr++;
                            i_1++;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    end_flag_1 = true;
                    i_1--;
                }
            }

            // find the next vox in sequ2
            if (!end_flag_2)
            {
                i_2++;

                if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                {
                    int i_curr = i_2 + 1;
                    while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                    {
                        int cc = delta_Y * voxsequ_2[i_curr][axis_x]
                               - delta_X * voxsequ_2[i_curr][axis_y];

                        if (c1<=cc && cc<=c2)
                        {
                            // the voxel is safe.
                            i_curr++;
                            i_2++;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    end_flag_2 = true;
                    i_2--;
                }
            }

        } // end while

        // move to next slice.
        i_1++;
        i_2++;

    } // end for

    //-----------------------------------------------------------------------------
    // Voxelize the second part of the triangle slice by slice.
    //-----------------------------------------------------------------------------
    size_1 = mark3DLineSpeed(p11, p2, p11_vox, p2_vox, voxsequ_1);
    size_2 = mark3DLineSpeed(p1, p2, p1_vox, p2_vox, voxsequ_2);

    start_slice = p11_vox[axis_z];
    end_slice = p2_vox[axis_z];

    i_1 = 0;
    i_2 = 0;
    for (int slice_i=start_slice; slice_i<=end_slice; ++slice_i)
    {
        // the flag indicate if a sequence is run out.
        bool end_flag_1 = false, end_flag_2 = false;
        while (!(end_flag_1 && end_flag_2))
        {
            int *vox_1 = voxsequ_1[i_1];
            int *vox_2 = voxsequ_2[i_2];

//            mark2DScanlineInteger(vox_1[axis_x], vox_1[axis_y],
//                                  vox_2[axis_x], vox_2[axis_y],
//                                  axis_x, axis_y, axis_z, slice_i, marker);

            // Compute the step distance parameters.
            int start_pixel_x = vox_1[axis_x];
            int start_pixel_y = vox_1[axis_y];
            int end_pixel_x = vox_2[axis_x];
            int end_pixel_y = vox_2[axis_y];

            int delta_X = end_pixel_x - start_pixel_x;
            int delta_Y = end_pixel_y - start_pixel_y;

            long long moving_vox_idx = vox_1[0] + (vox_1[1] + vox_1[2]*grid_den) * (long long)grid_den;

            int sign_x, sign_y, length_x, length_y;
            long long moving_vox_x, moving_vox_y;
            if (delta_X >= 0)
            {
                sign_x = 1;
                length_x = delta_X;
                moving_vox_x = vox_increment_x;
            }
            else
            {
                sign_x = -1;
                length_x = -delta_X;
                moving_vox_x = -vox_increment_x;
            }

            if (delta_Y >= 0)
            {
                sign_y = 1;
                length_y = delta_Y;
                moving_vox_y = vox_increment_y;
            }
            else
            {
                sign_y = -1;
                length_y = -delta_Y;
                moving_vox_y = -vox_increment_y;
            }

            // Compute the step distance parameters.
            int t_x, t_y, step_x, step_y;
            if (delta_X != 0)
            {
                if (delta_Y != 0)
                {
                    // v_x != 0, v_y != 0.
                    t_x = length_y;
                    step_x = length_y << 1;  // step_x = v_y*2

                    t_y = length_x;
                    step_y = length_x << 1;  // step_y = v_x*2
                }
                else
                {
                    // v_x != 0, v_y == 0.
                    t_x = 1;
                    step_x = 2;
                    t_y = 1000000000;
                }
            }
            else if (delta_Y != 0)
            {
                // v_x == 0, v_y != 0.
                t_y = 1;
                step_y = 2;
                t_x = 1000000000;
            }

            // The core loop.
            int i_x = 0;
            int i_y = 0;
            while (!(i_x == length_x && i_y == length_y))
            {
                markPoint(moving_vox_idx, 1);

                if (t_x <= t_y)
                {
                    t_y = t_y - t_x;
                    t_x = step_x;

                    i_x++;
                    moving_vox_idx += moving_vox_x;
                }
                else
                {
                    t_x = t_x - t_y;
                    t_y = step_y;

                    i_y++;
                    moving_vox_idx += moving_vox_y;
                }
            }

            // mark the last pixel
            markPoint(moving_vox_idx, 1);

            // Find the next scanline.
            int temp = delta_Y * end_pixel_x - delta_X * end_pixel_y;
            int abs_X_Y = length_x + length_y;
            int c1 = temp - abs_X_Y;
            int c2 = temp + abs_X_Y;

            // find the next vox in sequ1
            if (!end_flag_1)
            {
                i_1++;

                if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                {
                    int i_curr = i_1 + 1;
                    while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                    {
                        int cc = delta_Y * voxsequ_1[i_curr][axis_x]
                               - delta_X * voxsequ_1[i_curr][axis_y];

                        if (c1<cc && cc<c2)
                        {
                            // the voxel is safe.
                            i_curr++;
                            i_1++;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    end_flag_1 = true;
                    i_1--;
                }
            }

            // find the next vox in sequ2
            if (!end_flag_2)
            {
                i_2++;

                if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                {
                    int i_curr = i_2 + 1;
                    while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                    {
                        int cc = delta_Y * voxsequ_2[i_curr][axis_x]
                               - delta_X * voxsequ_2[i_curr][axis_y];

                        if (c1<=cc && cc<=c2)
                        {
                            // the voxel is safe.
                            i_curr++;
                            i_2++;
                        }
                        else
                        {
                            break;
                        }
                    }
                }
                else
                {
                    end_flag_2 = true;
                    i_2--;
                }
            }

        } // end while

        // move to next slice.
        i_1++;
        i_2++;

    } // end for
}

int VoxelMesh::mark3DLineSpeed(const double start_point[], const double end_point[],
                               const int start_point_vox[], const int end_point_vox[],
                               int voxsequ[][3])
{
    int voxsequ_size = 0;

    // Compute l0, the distances between the start point and 3 corresponding facets.
    double vv[3];
    vv[0] = end_point[0] - start_point[0];
    vv[1] = end_point[1] - start_point[1];
    vv[2] = end_point[2] - start_point[2];

    int length[3];
    length[0] = end_point_vox[0] - start_point_vox[0];
    length[1] = end_point_vox[1] - start_point_vox[1];
    length[2] = end_point_vox[2] - start_point_vox[2];

    bool isEnd[3] = {false, false, false};
    int delta[3];
    double tt[3], step[3];

    long long increment[3];
    increment[0] = 1;
    increment[1] = grid_density;
    increment[2] = grid_density * (long long)grid_density;

    for (int i=0; i<3; i++)
    {
        if (length[i] == 0)
        {
            isEnd[i] = true;
            tt[i] = 1e20;
            step[i] = 1;
        }
        else
        {
            isEnd[i] = false;

            if (length[i] > 0)
            {
                delta[i] = 1;
                tt[i] = voxel_dim - (start_point[i] - coordinate_min[i] - double(start_point_vox[i]) * voxel_dim);
                step[i] = voxel_dim;
            }
            else
            {
                delta[i] = -1;
                tt[i] = start_point[i] - coordinate_min[i] - double(start_point_vox[i]) * voxel_dim;
                step[i] = voxel_dim;

                vv[i] = - vv[i];
                length[i] = -length[i];
                increment[i] = -increment[i];
            }
        }
    }

    //Compute final parameters (multiplied by vv[0]vv[1]vv[2])
    if (!isEnd[0])
    {
        tt[1] *= vv[0];
        step[1] *= vv[0];
        tt[2] *= vv[0];
        step[2] *= vv[0];
    }
    if (!isEnd[1])
    {
        tt[0] *= vv[1];
        step[0] *= vv[1];
        tt[2] *= vv[1];
        step[2] *= vv[1];
    }
    if (!isEnd[2])
    {
        tt[0] *= vv[2];
        step[0] *= vv[2];
        tt[1] *= vv[2];
        step[1] *= vv[2];
    }

    // The core loop.
    int moving_point_vox[3];
    moving_point_vox[0] = start_point_vox[0];
    moving_point_vox[1] = start_point_vox[1];
    moving_point_vox[2] = start_point_vox[2];

    long long moving_vox_idx = start_point_vox[0] +
                              (start_point_vox[1] + start_point_vox[2]*grid_density) * (long long)grid_density;

    int i_x = 0, i_y = 0, i_z = 0;
    while (!(i_x == length[0] && i_y == length[1] && i_z == length[2]))
    {
        markPointSpeed(moving_vox_idx);

        voxsequ[voxsequ_size][0] = moving_point_vox[0];
        voxsequ[voxsequ_size][1] = moving_point_vox[1];
        voxsequ[voxsequ_size][2] = moving_point_vox[2];
        voxsequ_size++;

        if (tt[0] <= tt[1])
        {
            if (tt[0] <= tt[2])
            {
                // 0<=1, 0<=2
                i_x++;

                tt[1] -= tt[0];
                tt[2] -= tt[0];
                tt[0] = step[0];

                moving_vox_idx += increment[0];
                moving_point_vox[0] += delta[0];

                if (i_x == length[0])
                {
                    tt[0] = 1e20;
                }
            }
            else
            {
                // 0<=1, 0>2
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

                if (i_z == length[2])
                {
                    tt[2] = 1e20;
                }
            }
        }
        else
        {
            if (tt[1] <= tt[2])
            {
                // 1<0, 1<=2
                i_y++;

                tt[0] -= tt[1];
                tt[2] -= tt[1];
                tt[1] = step[1];

                moving_vox_idx += increment[1];
                moving_point_vox[1] += delta[1];

                if (i_y == length[1])
                {
                    tt[1] = 1e20;
                }
            }
            else
            {
                // 1<0, 2<1
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

                if (i_z == length[2])
                {
                    tt[2] = 1e20;
                }
            }
        }
    }
    markPointSpeed(moving_vox_idx);

    voxsequ[voxsequ_size][0] = end_point_vox[0];
    voxsequ[voxsequ_size][1] = end_point_vox[1];
    voxsequ[voxsequ_size][2] = end_point_vox[2];
    voxsequ_size++;

    return voxsequ_size;
}

void VoxelMesh::mark3DLineSpeed(const double start_point[], const double end_point[],
                                const int start_point_vox[], const int end_point_vox[])
{
    // Compute l0, the distances between the start point and 3 corresponding facets.
    double vv[3];
    vv[0] = end_point[0] - start_point[0];
    vv[1] = end_point[1] - start_point[1];
    vv[2] = end_point[2] - start_point[2];

    int length[3];
    length[0] = end_point_vox[0] - start_point_vox[0];
    length[1] = end_point_vox[1] - start_point_vox[1];
    length[2] = end_point_vox[2] - start_point_vox[2];

    bool isEnd[3] = {false, false, false};
    int delta[3];
    double tt[3], step[3];

    long long increment[3];
    increment[0] = 1;
    increment[1] = grid_density;
    increment[2] = grid_density * (long long)grid_density;

    for (int i=0; i<3; i++)
    {
        if (length[i] == 0)
        {
            isEnd[i] = true;
            tt[i] = 1e20;
            step[i] = 1;
        }
        else
        {
            isEnd[i] = false;

            if (length[i] > 0)
            {
                delta[i] = 1;
                tt[i] = voxel_dim - (start_point[i] - coordinate_min[i] - double(start_point_vox[i]) * voxel_dim);
                step[i] = voxel_dim;
            }
            else
            {
                delta[i] = -1;
                tt[i] = start_point[i] - coordinate_min[i] - double(start_point_vox[i]) * voxel_dim;
                step[i] = voxel_dim;

                vv[i] = - vv[i];
                length[i] = -length[i];
                increment[i] = -increment[i];
            }
        }
    }

    //Compute final parameters (multiplied by vv[0]vv[1]vv[2])
    if (!isEnd[0])
    {
        tt[1] *= vv[0];
        step[1] *= vv[0];
        tt[2] *= vv[0];
        step[2] *= vv[0];
    }
    if (!isEnd[1])
    {
        tt[0] *= vv[1];
        step[0] *= vv[1];
        tt[2] *= vv[1];
        step[2] *= vv[1];
    }
    if (!isEnd[2])
    {
        tt[0] *= vv[2];
        step[0] *= vv[2];
        tt[1] *= vv[2];
        step[1] *= vv[2];
    }

    // The core loop.
    int moving_point_vox[3];
    moving_point_vox[0] = start_point_vox[0];
    moving_point_vox[1] = start_point_vox[1];
    moving_point_vox[2] = start_point_vox[2];

    long long moving_vox_idx = start_point_vox[0] +
                             (start_point_vox[1] + start_point_vox[2]*grid_density) * (long long)grid_density;

    int i_x = 0, i_y = 0, i_z = 0;
    while (!(i_x == length[0] && i_y == length[1] && i_z == length[2]))
    {
        markPointSpeed(moving_vox_idx);

        if (tt[0] <= tt[1])
        {
            if (tt[0] <= tt[2])
            {
                // 0<=1, 0<=2
                i_x++;

                tt[1] -= tt[0];
                tt[2] -= tt[0];
                tt[0] = step[0];

                moving_vox_idx += increment[0];
                moving_point_vox[0] += delta[0];

                if (i_x == length[0])
                {
                    tt[0] = 1e20;
                }
            }
            else
            {
                // 0<=1, 0>2
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

                if (i_z == length[2])
                {
                    tt[2] = 1e20;
                }
            }
        }
        else
        {
            if (tt[1] <= tt[2])
            {
                // 1<0, 1<=2
                i_y++;

                tt[0] -= tt[1];
                tt[2] -= tt[1];
                tt[1] = step[1];

                moving_vox_idx += increment[1];
                moving_point_vox[1] += delta[1];

                if (i_y == length[1])
                {
                    tt[1] = 1e20;
                }
            }
            else
            {
                // 1<0, 2<1
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

                if (i_z == length[2])
                {
                    tt[2] = 1e20;
                }
            }
        }
    }

    markPointSpeed(moving_vox_idx);
}

void VoxelMesh::mark2DLineSpeed(const double start_x, const double start_y,
                                   const double end_x, const double end_y,
                                   int axis_x, int axis_y, int axis_z, int slice_idx)
{
    double mesh_min_x = coordinate_min[axis_x];
    double mesh_min_y = coordinate_min[axis_y];
    double voxel_length = voxel_dim;
    int density = grid_density;

    // Pixelize the start/end point.
    int start_pixel_x, start_pixel_y;
    int end_pixel_x, end_pixel_y;

    start_pixel_x = floor((start_x - mesh_min_x) / voxel_length);
    if (start_pixel_x < 0)
        start_pixel_x = 0;
    if (start_pixel_x >= density)
        start_pixel_x = density - 1;

    start_pixel_y = floor((start_y - mesh_min_y) / voxel_length);
    if (start_pixel_y < 0)
        start_pixel_y = 0;
    if (start_pixel_y >= density)
        start_pixel_y = density - 1;

    end_pixel_x = floor((end_x - mesh_min_x) / voxel_length);
    if (end_pixel_x < 0)
        end_pixel_x = 0;
    if (end_pixel_x >= density)
        end_pixel_x = density - 1;

    end_pixel_y = floor((end_y - mesh_min_y) / voxel_length);
    if (end_pixel_y < 0)
        end_pixel_y = 0;
    if (end_pixel_y >= density)
        end_pixel_y = density - 1;

    // If only one pixel, record it and return.
    long long vox_idx[3];
    if (end_pixel_x == start_pixel_x && end_pixel_y == start_pixel_y)
    {
        vox_idx[axis_x] = start_pixel_x;
        vox_idx[axis_y] = start_pixel_y;
        vox_idx[axis_z] = slice_idx;

//        if (vox_idx[0] + (vox_idx[1] + vox_idx[2]*grid_density) * (long long)grid_density >=
//                grid_density * grid_density * (long long)grid_density)
//        {
//            cout << "ERROR "
//                 << vox_idx[0] << " "<< vox_idx[1] << " "<< vox_idx[2] << " " << slice_idx << " "
//                 << vox_idx[0] + (vox_idx[1] + vox_idx[2]*grid_density) * (long long)grid_density
//                    << " " << grid_density * grid_density * (long long)grid_density<< endl;
//        }
        markPointSpeed(vox_idx[0] + (vox_idx[1] + vox_idx[2]*grid_density) * (long long)grid_density);
        return;
    }

    vox_idx[0] = 1;
    vox_idx[1] = grid_density;
    vox_idx[2] = grid_density*grid_density;

    // Compute l0, the distance between start point and 3 corresponding facets.
    int delta_x, delta_y;
    double t0_x, t0_y;
    long long inc_x, inc_y;

    if (start_pixel_x <= end_pixel_x)
    {
        delta_x = 1;
        t0_x = voxel_length * double(start_pixel_x + 1) - (start_x - mesh_min_x);
        inc_x = vox_idx[axis_x];
    }
    else
    {
        delta_x = -1;
        t0_x = (start_x - mesh_min_x) - voxel_length * double(start_pixel_x);
        inc_x = -vox_idx[axis_x];
    }

    if (start_pixel_y <= end_pixel_y)
    {
        delta_y = 1;
        t0_y = voxel_length * double(start_pixel_y + 1) - (start_y - mesh_min_y);
        inc_y = vox_idx[axis_y];
    }
    else
    {
        delta_y = -1;
        t0_y = (start_y - mesh_min_y) - voxel_length * double(start_pixel_y);
        inc_y = -vox_idx[axis_y];
    }

    // Compute the step distance parameters.
    double t_x, t_y;
    double step_x, step_y;

    if (start_pixel_x != end_pixel_x)
    {
        double v_x = abs(end_x - start_x);
        t_x = t0_x / v_x;
        step_x = voxel_length / v_x;
    }
    else
    {
        t_x = 1e20;
        step_x = 0;
    }

    if (start_pixel_y != end_pixel_y)
    {
        double v_y = abs(end_y - start_y);
        t_y = t0_y / v_y;
        step_y = voxel_length / v_y;
    }
    else
    {
        t_y = 1e20;
        step_y = 0;
    }

    // The core loop.
    int moving_pixel_x = start_pixel_x;
    int moving_pixel_y = start_pixel_y;

    vox_idx[axis_x] = moving_pixel_x;
    vox_idx[axis_y] = moving_pixel_y;
    vox_idx[axis_z] = slice_idx;
    long long moving_vox_idx = vox_idx[0] + (vox_idx[1] + vox_idx[2]*grid_density) * (long long)grid_density;

    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        markPointSpeed(moving_vox_idx);

        if (t_x <= t_y)
        {
            t_y = t_y - t_x;
            t_x = step_x;

            if (moving_pixel_x != end_pixel_x)
            {
                moving_pixel_x += delta_x;
                moving_vox_idx += inc_x;
            }
        }
        else
        {
            t_x = t_x - t_y;
            t_y = step_y;
            if (moving_pixel_y != end_pixel_y)
            {
                moving_pixel_y += delta_y;
                moving_vox_idx += inc_y;
            }
        }
    }
    markPointSpeed(moving_vox_idx);
}

inline void VoxelMesh::markPointSpeed(const long long vox_idx)
{
    //clear and set the appropriate bits.
//#ifdef HIGH_GRID_DENSITY
//    unsigned char mask = marker << (7 - (vox_idx & 0x7));
//    voxel_map[vox_idx >> 3] = voxel_map[vox_idx >> 3] | mask;
//#else

//    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
//    unsigned char mask0 = ~(3 << offset);
//    unsigned char mask1 = 1 << offset;
//    voxel_map[vox_idx >> 2] = (voxel_map[vox_idx >> 2] & mask0) | mask1;

//    if (vox_idx >= grid_density*grid_density*(long long)grid_density || vox_idx < 0)
//    {
//        cout << "ERROR in 'markpointspeed'! " << vox_idx << endl;
//        return;
//    }

    voxel_map[vox_idx] = 1;

//#endif
}

//Check if the voxel is marked as the marker.
inline bool VoxelMesh::equaltoMarkerSpeed(const long long vox_idx)
{
//    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
//    unsigned char mask0 = 3 << offset;
//    unsigned char mask1 = 1 << offset;
//    return !((voxel_map[vox_idx >> 2] & mask0) ^ mask1);
    if (voxel_map[vox_idx] == 1)
        return true;
    return false;
}

//==================================================================================
// Voxelize both surface and interior of all triangle sub meshes.
// This function merges all sub obj mesh to ONE element list.
//==================================================================================
void VoxelMesh::voxelizeTriangleMesh()
{
    time_surface = 0.0;
    time_interior = 0.0;
    scanline_count = 0;
    op_atom = 0;
    op_cmp = 0;
    op_add = 0;
    op_mult = 0;

    cout <<"min x y z = " << triangle_mesh->min_x << " "
         << triangle_mesh->min_y << " " << triangle_mesh->min_z << endl
         <<"max x y z = " << triangle_mesh->max_x << " "
         << triangle_mesh->max_y << " " << triangle_mesh->max_z << endl;

    //---------------------------------------------------------------------------
    // Surface voxelization
    //---------------------------------------------------------------------------
    QElapsedTimer timer;
    if (voxelization_method == 0)
    {
        // Use SAT-based method to voxelize the surface of the sub mesh.
        timer.start();
        setupSortedTriangleListSATPan11();
        int size = sorted_triangle_list.size();
        for (int i=0; i<size; ++i)
        {
            markSortedFaceSAT(sorted_triangle_list[i], 1);
        }
        time_surface = timer.nsecsElapsed();
    }
    else if (voxelization_method == 3)
    {
        // Use SAT-based method to voxelize the surface of the sub mesh.
        timer.start();
        setupSortedTriangleListSATPan11();
        int size = sorted_triangle_list.size();
        for (int i=0; i<size; ++i)
        {
            markSortedFacePan11(sorted_triangle_list[i], 1);
        }
        time_surface = timer.nsecsElapsed();
    }
    else if (voxelization_method == 1)
    {
        // Use float scanline to voxelize the surface of the sub mesh.
        timer.start();
//        setupSortedTriangleListFLTINT();
//        int size = sorted_triangle_list.size();
//        for (int i=0; i<size; ++i)
//        {
//            markSortedFaceFLTSpeed(sorted_triangle_list[i]);
//        }
        for (int i=0; i<triangle_mesh->num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi=triangle_mesh->mesh_lists[i].begin();
                 fi!=triangle_mesh->mesh_lists[i].end(); ++fi)
            {
                markFaceByScanline(*fi, 1);
            }
        }
        time_surface = timer.nsecsElapsed();
    }
    else if (voxelization_method == 2)
    {
        // Use integer scanline to voxelize the surface of the sub mesh.
        timer.start();
//        setupSortedTriangleListFLTINT();
//        int size = sorted_triangle_list.size();
//        for (int i=0; i<size; ++i)
//        {
//            markSortedFaceINTSpeed(sorted_triangle_list[i]);
//        }
        for (int i=0; i<triangle_mesh->num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi=triangle_mesh->mesh_lists[i].begin();
                 fi!=triangle_mesh->mesh_lists[i].end(); ++fi)
            {
                markFaceByScanlineInteger(*fi, 1);
            }
        }
        time_surface = timer.nsecsElapsed();
    }

    time_surface *= 0.001;

    cout << "The operations atom, cmp, add and multi = "
         << op_atom << " " << op_cmp << " " << op_add << " " << op_mult << endl;
    cout << "The triangle mesh was voxelized using " << scanline_count << " scanlines." << endl;
//    cout << "The total #marking was " << voxel_marking_count << endl;

    //----------------------------------------------------------------------------
    // Interior voxelization.
    //----------------------------------------------------------------------------
    if (isFillingHole)
    {
        timer.start();
        fillHole(3);
        time_interior = timer.nsecsElapsed();
    }
    time_interior *= 0.001;
}

void VoxelMesh::voxelizeSubTriangleMesh(int idx)
{
    setzeroVoxelMap();

    //---------------------------------------------------------------------------
    // Surface voxelization
    //---------------------------------------------------------------------------
    if (voxelization_method == 0)
    {
        // Use SAT-based method to voxelize the surface of the sub mesh.
        for (const_iter_ptriangleface fi=triangle_mesh->mesh_lists[idx].begin();
             fi!=triangle_mesh->mesh_lists[idx].end(); ++fi)
        {
            markFaceBySATMethod(*fi, 1);
        }
    }
    else if (voxelization_method == 3)
    {
        // Use SAT-based method to voxelize the surface of the sub mesh.
        for (const_iter_ptriangleface fi=triangle_mesh->mesh_lists[idx].begin();
             fi!=triangle_mesh->mesh_lists[idx].end(); ++fi)
        {
            markFaceBySATMethod(*fi, 1);
        }
    }
    else if (voxelization_method == 1)
    {
        // Use float scanline to voxelize the surface of the sub mesh.
        for (const_iter_ptriangleface fi=triangle_mesh->mesh_lists[idx].begin();
             fi!=triangle_mesh->mesh_lists[idx].end(); ++fi)
        {
            markFaceByScanline(*fi, 1);
        }
    }
    else if (voxelization_method == 2)
    {
        // Use integer scanline to voxelize the surface of the sub mesh.
        for (const_iter_ptriangleface fi=triangle_mesh->mesh_lists[idx].begin();
             fi!=triangle_mesh->mesh_lists[idx].end(); ++fi)
        {
            markFaceByScanlineInteger(*fi, 1);
        }
    }

    //----------------------------------------------------------------------------
    // Interior voxelization.
    //----------------------------------------------------------------------------
    if (isFillingHole && grid_density <= 1024)
    {
        fillHole(3);
    }
}

void VoxelMesh::getMinMaxValueINT(int &mi, int &ma, int aa[])
{
    if (aa[0] >= aa[1])
    {
        if (aa[1] >= aa[2])
        {
            // 0>=1>=2
            ma = aa[0];
            mi = aa[2];
        }
        else
        {
            if (aa[0] >= aa[2])
            {
                // 0>=2>1
                ma = aa[0];
                mi = aa[1];
            }
            else
            {
                // 2>0>=1
                ma = aa[2];
                mi = aa[1];
            }
        }
    }
    else
    {
        if (aa[0] >= aa[2])
        {
            // 1>0>=2
            ma = aa[1];
            mi = aa[2];
        }
        else
        {
            if (aa[1] >= aa[2])
            {
                // 1>=2>0
                ma = aa[1];
                mi = aa[0];
            }
            else
            {
                // 2>1>0
                ma = aa[2];
                mi = aa[0];
            }
        }
    }
}

bool VoxelMesh::getRangePa11(double &r1, double &r2,
                             double n0, double u0, double n1, double u1, double n2, double u2)
{
    if (n0 > 0)
    {
        if (n1 > 0)
        {
            if (n2 > 0)
            {
                // n0 > 0, n1 > 0, n2 > 0
                return false;
            }
            else if (n2 < 0)
            {
                // n0 > 0, n1 > 0, n2 < 0
                if (u0 > u1)
                {
                    r1 = u0;
                }
                else
                {
                    r1 = u1;
                }
                r2 = u2;
                return true;
            }
            else
            {
                // n0 > 0, n1 > 0, n2 = 0
                return false;
            }
        }
        else if (n1 < 0)
        {
            if (n2 > 0)
            {
                // n0 > 0, n1 < 0, n2 > 0
                if (u0 > u2)
                {
                    r1 = u0;
                }
                else
                {
                    r1 = u2;
                }
                r2 = u1;
                return true;
            }
            else if (n2 < 0)
            {
                // n0 > 0, n1 < 0, n2 < 0
                if (u1 < u2)
                {
                    r2 = u1;
                }
                else
                {
                    r2 = u2;
                }
                r1 = u0;
                return true;
            }
            else
            {
                // n0 > 0, n1 < 0, n2 = 0
                r1 = u0;
                r2 = u1;
                return true;
            }
        }
        else
        {
            if (n2 > 0)
            {
                // n0 > 0, n1 = 0, n2 > 0
                return false;
            }
            else if (n2 < 0)
            {
                // n0 > 0, n1 = 0, n2 < 0
                r1 = u0;
                r2 = u2;
                return true;
            }
            else
            {
                // n0 > 0, n1 = 0, n2 = 0
                return false;
            }
        }
    }
    else if (n0 < 0)
    {
        if (n1 > 0)
        {
            if (n2 > 0)
            {
                // n0 < 0, n1 > 0, n2 > 0
                if (u1 > u2)
                {
                    r1 = u1;
                }
                else
                {
                    r1 = u2;
                }
                r2 = u0;
                return true;
            }
            else if (n2 < 0)
            {
                // n0 < 0, n1 > 0, n2 < 0
                if (u0 < u2)
                {
                    r2 = u0;
                }
                else
                {
                    r2 = u2;
                }
                r1 = u1;
                return true;
            }
            else
            {
                // n0 < 0, n1 > 0, n2 = 0
                r1 = u1;
                r2 = u0;
                return true;
            }
        }
        else if (n1 < 0)
        {
            if (n2 > 0)
            {
                // n0 < 0, n1 < 0, n2 > 0
                if (u0 < u1)
                {
                    r2 = u0;
                }
                else
                {
                    r2 = u1;
                }
                r1 = u2;
                return true;
            }
            else if (n2 < 0)
            {
                // n0 < 0, n1 < 0, n2 < 0
                return false;
            }
            else
            {
                // n0 < 0, n1 < 0, n2 = 0
                return false;
            }
        }
        else
        {
            if (n2 > 0)
            {
                // n0 < 0, n1 = 0, n2 > 0
                r1 = u2;
                r2 = u0;
                return true;
            }
            else if (n2 < 0)
            {
                // n0 < 0, n1 = 0, n2 < 0
                return false;
            }
            else
            {
                // n0 < 0, n1 = 0, n2 = 0
                return false;
            }
        }
    }
    else
    {
        if (n1 > 0)
        {
            if (n2 > 0)
            {
                // n0 = 0, n1 > 0, n2 > 0
                return false;
            }
            else if (n2 < 0)
            {
                // n0 = 0, n1 > 0, n2 < 0
                r1 = u1;
                r2 = u2;
                return true;
            }
            else
            {
                // n0 = 0, n1 > 0, n2 = 0
                return false;
            }
        }
        else if (n1 < 0)
        {
            if (n2 > 0)
            {
                // n0 = 0, n1 < 0, n2 > 0
                r1 = u2;
                r2 = u1;
                return true;
            }
            else if (n2 < 0)
            {
                // n0 = 0, n1 < 0, n2 < 0
                return false;
            }
            else
            {
                // n0 = 0, n1 < 0, n2 = 0
                return false;
            }
        }
        else
        {
            if (n2 > 0)
            {
                // n0 = 0, n1 = 0, n2 > 0
                return false;
            }
            else if (n2 < 0)
            {
                // n0 = 0, n1 = 0, n2 < 0
                return false;
            }
            else
            {
                // n0 = 0, n1 = 0, n2 = 0
                return false;
            }
        }
    }
}

void VoxelMesh::sortVertices2D(double v0[], double v1[], double v2[], int axis_x, int axis_y)
{
    double temp_0, temp_1, temp_2;

    temp_0 = v0[axis_x] - v2[axis_x];
    temp_1 = v0[axis_y] - v2[axis_y];
    double e0 = temp_0*temp_0 + temp_1*temp_1;

    temp_0 = v1[axis_x] - v0[axis_x];
    temp_1 = v1[axis_y] - v0[axis_y];
    double e1 = temp_0*temp_0 + temp_1*temp_1;

    temp_0 = v2[axis_x] - v1[axis_x];
    temp_1 = v2[axis_y] - v1[axis_y];
    double e2 = temp_0*temp_0 + temp_1*temp_1;

    if (e0 >= e1)
    {
        if (e1 >= e2)
        {
            // 0>=1>=2, v0 <-> v1, v2 nothing.
            temp_0 = v0[0];
            temp_1 = v0[1];
            temp_2 = v0[2];
            v0[0] = v1[0];
            v0[1] = v1[1];
            v0[2] = v1[2];
            v1[0] = temp_0;
            v1[1] = temp_1;
            v1[2] = temp_2;
        }
        else
        {
            if (e0 >= e2)
            {
                // 0>=2>1, v0 -> v1, v1 -> v2, v2 -> v0.
                temp_0 = v0[0];
                temp_1 = v0[1];
                temp_2 = v0[2];
                v0[0] = v1[0];
                v0[1] = v1[1];
                v0[2] = v1[2];
                v1[0] = v2[0];
                v1[1] = v2[1];
                v1[2] = v2[2];
                v2[0] = temp_0;
                v2[1] = temp_1;
                v2[2] = temp_2;
            }
            else
            {
                // 2>0>=1, v0 nothing, v1 <-> v2.
                temp_0 = v1[0];
                temp_1 = v1[1];
                temp_2 = v1[2];
                v1[0] = v2[0];
                v1[1] = v2[1];
                v1[2] = v2[2];
                v2[0] = temp_0;
                v2[1] = temp_1;
                v2[2] = temp_2;
            }
        }
    }
    else
    {
        if (e0 >= e2)
        {
            // 1>0>=2, v0 -> v2, v1 -> v0, v2 -> v1.
            temp_0 = v2[0];
            temp_1 = v2[1];
            temp_2 = v2[2];
            v2[0] = v1[0];
            v2[1] = v1[1];
            v2[2] = v1[2];
            v1[0] = v0[0];
            v1[1] = v0[1];
            v1[2] = v0[2];
            v0[0] = temp_0;
            v0[1] = temp_1;
            v0[2] = temp_2;
        }
        else
        {
            if (e1 >= e2)
            {
                // 1>=2>0, v0 <-> v2, v1 nothing.
                temp_0 = v0[0];
                temp_1 = v0[1];
                temp_2 = v0[2];
                v0[0] = v2[0];
                v0[1] = v2[1];
                v0[2] = v2[2];
                v2[0] = temp_0;
                v2[1] = temp_1;
                v2[2] = temp_2;
            }
            else
            {
                // 2>1>0, nothing
            }
        }
    }
}

//===============================================================================
// Mark a face/triangle using SAT-based (Siggraph paper SS10) method.
//===============================================================================
void VoxelMesh::markFaceBySATMethod(TriangleFace* facep, int marker)
{

}

void VoxelMesh::markSortedFaceSAT(TriangleFace* facep, int marker)
{
    //cout << "SAT facep->bounding_box_type = " << facep->bounding_box_type << endl;
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    // extend in xy plane
    else if (facep->bounding_box_type == 3)
    {
        // Setup for the 2D projection test
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[2] >= 0)
        {
            n0[0] = v0.y - v1.y;
            n0[1] = v1.x - v0.x;
            n1[0] = v1.y - v2.y;
            n1[1] = v2.x - v1.x;
            n2[0] = v2.y - v0.y;
            n2[1] = v0.x - v2.x;
        }
        else
        {
            n0[0] = v1.y - v0.y;
            n0[1] = v0.x - v1.x;
            n1[0] = v2.y - v1.y;
            n1[1] = v1.x - v2.x;
            n2[0] = v0.y - v2.y;
            n2[1] = v2.x - v0.x;
        }
        double d0 = - n0[0]*v0.x - n0[1]*v0.y;
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = - n1[0]*v1.x - n1[1]*v1.y;
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = - n2[0]*v2.x - n2[1]*v2.y;
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        int zi = facep->voxel_min[2];
        for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
        {
            for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
            {
                op_atom++;

                // get the minimum corner
                double p[2];
                p[0] = double(xi) * voxel_dim + coordinate_min[0];
                p[1] = double(yi) * voxel_dim + coordinate_min[1];
                op_add += 2;
                op_mult += 2;

                op_add += 2;
                op_mult += 2;
                op_cmp++;
                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0)
                {
                    op_add += 2;
                    op_mult += 2;
                    op_cmp++;
                    if (n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0)
                    {
                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
                        {
                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                        }
                    }
                }
            }
        }
    }

    // extend in yz plane
    else if (facep->bounding_box_type == 4)
    {
        // Setup for the 2D projection test
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[0] >= 0)
        {
            n0[0] = v0.z - v1.z;
            n0[1] = v1.y - v0.y;
            n1[0] = v1.z - v2.z;
            n1[1] = v2.y - v1.y;
            n2[0] = v2.z - v0.z;
            n2[1] = v0.y - v2.y;
        }
        else
        {
            n0[0] = v1.z - v0.z;
            n0[1] = v0.y - v1.y;
            n1[0] = v2.z - v1.z;
            n1[1] = v1.y - v2.y;
            n2[0] = v0.z - v2.z;
            n2[1] = v2.y - v0.y;
        }
        double d0 = - n0[0]*v0.y - n0[1]*v0.z;
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = - n1[0]*v1.y - n1[1]*v1.z;
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = - n2[0]*v2.y - n2[1]*v2.z;
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        int xi = facep->voxel_min[0];
        for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
        {
            for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
            {
                op_atom++;

                // get the minimum corner
                double p[2];
                p[0] = double(yi) * voxel_dim + coordinate_min[1];
                p[1] = double(zi) * voxel_dim + coordinate_min[2];
                op_add += 2;
                op_mult += 2;

                op_add += 2;
                op_mult += 2;
                op_cmp++;
                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0)
                {
                    op_add += 2;
                    op_mult += 2;
                    op_cmp++;
                    if (n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0)
                    {
                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
                        {
                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                        }
                    }
                }
            }
        }
    }

    // extend in zx plane
    else if (facep->bounding_box_type == 5)
    {
        // Setup for the 2D projection test
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[1] >= 0)
        {
            n0[0] = v0.x - v1.x;
            n0[1] = v1.z - v0.z;
            n1[0] = v1.x - v2.x;
            n1[1] = v2.z - v1.z;
            n2[0] = v2.x - v0.x;
            n2[1] = v0.z - v2.z;
        }
        else
        {
            n0[0] = v1.x - v0.x;
            n0[1] = v0.z - v1.z;
            n1[0] = v2.x - v1.x;
            n1[1] = v1.z - v2.z;
            n2[0] = v0.x - v2.x;
            n2[1] = v2.z - v0.z;
        }
        double d0 = - n0[0]*v0.z - n0[1]*v0.x;
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = - n1[0]*v1.z - n1[1]*v1.x;
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = - n2[0]*v2.z - n2[1]*v2.x;
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        int yi = facep->voxel_min[1];
        for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
        {
            for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
            {
                op_atom++;

                // get the minimum corner
                double p[2];
                p[0] = double(zi) * voxel_dim + coordinate_min[2];
                p[1] = double(xi) * voxel_dim + coordinate_min[0];
                op_add += 2;
                op_mult += 2;

                op_add += 2;
                op_mult += 2;
                op_cmp++;
                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0)
                {
                    op_add += 2;
                    op_mult += 2;
                    op_cmp++;
                    if (n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0)
                    {
                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
                        {
                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                        }
                    }
                }
            }
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // Setup for the 2D xy projection test
        double n0_xy[2], n1_xy[2], n2_xy[2];
        if (facep->norm_sign[2] >= 0)
        {
            n0_xy[0] = v0.y - v1.y;
            n0_xy[1] = v1.x - v0.x;
            n1_xy[0] = v1.y - v2.y;
            n1_xy[1] = v2.x - v1.x;
            n2_xy[0] = v2.y - v0.y;
            n2_xy[1] = v0.x - v2.x;
        }
        else
        {
            n0_xy[0] = v1.y - v0.y;
            n0_xy[1] = v0.x - v1.x;
            n1_xy[0] = v2.y - v1.y;
            n1_xy[1] = v1.x - v2.x;
            n2_xy[0] = v0.y - v2.y;
            n2_xy[1] = v2.x - v0.x;
        }
        double d0_xy = - n0_xy[0]*v0.x - n0_xy[1]*v0.y;
        if (n0_xy[0] > 0.0)
        {
            d0_xy += voxel_dim * n0_xy[0];
        }
        if (n0_xy[1] > 0.0)
        {
            d0_xy += voxel_dim * n0_xy[1];
        }
        double d1_xy = - n1_xy[0]*v1.x - n1_xy[1]*v1.y;
        if (n1_xy[0] > 0.0)
        {
            d1_xy += voxel_dim * n1_xy[0];
        }
        if (n1_xy[1] > 0.0)
        {
            d1_xy += voxel_dim * n1_xy[1];
        }
        double d2_xy = - n2_xy[0]*v2.x - n2_xy[1]*v2.y;
        if (n2_xy[0] > 0.0)
        {
            d2_xy += voxel_dim * n2_xy[0];
        }
        if (n2_xy[1] > 0.0)
        {
            d2_xy += voxel_dim * n2_xy[1];
        }

        // Setup for the 2D yz projection test
        double n0_yz[2], n1_yz[2], n2_yz[2];
        if (facep->norm_sign[0] >= 0)
        {
            n0_yz[0] = v0.z - v1.z;
            n0_yz[1] = v1.y - v0.y;
            n1_yz[0] = v1.z - v2.z;
            n1_yz[1] = v2.y - v1.y;
            n2_yz[0] = v2.z - v0.z;
            n2_yz[1] = v0.y - v2.y;
        }
        else
        {
            n0_yz[0] = v1.z - v0.z;
            n0_yz[1] = v0.y - v1.y;
            n1_yz[0] = v2.z - v1.z;
            n1_yz[1] = v1.y - v2.y;
            n2_yz[0] = v0.z - v2.z;
            n2_yz[1] = v2.y - v0.y;
        }
        double d0_yz = - n0_yz[0]*v0.y - n0_yz[1]*v0.z;
        if (n0_yz[0] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[0];
        }
        if (n0_yz[1] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[1];
        }
        double d1_yz = - n1_yz[0]*v1.y - n1_yz[1]*v1.z;
        if (n1_yz[0] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[0];
        }
        if (n1_yz[1] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[1];
        }
        double d2_yz = - n2_yz[0]*v2.y - n2_yz[1]*v2.z;
        if (n2_yz[0] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[0];
        }
        if (n2_yz[1] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[1];
        }

        // Setup for the 2D zx projection test
        double n0_zx[2], n1_zx[2], n2_zx[2];
        if (facep->norm_sign[1] >= 0)
        {
            n0_zx[0] = v0.x - v1.x;
            n0_zx[1] = v1.z - v0.z;
            n1_zx[0] = v1.x - v2.x;
            n1_zx[1] = v2.z - v1.z;
            n2_zx[0] = v2.x - v0.x;
            n2_zx[1] = v0.z - v2.z;
        }
        else
        {
            n0_zx[0] = v1.x - v0.x;
            n0_zx[1] = v0.z - v1.z;
            n1_zx[0] = v2.x - v1.x;
            n1_zx[1] = v1.z - v2.z;
            n2_zx[0] = v0.x - v2.x;
            n2_zx[1] = v2.z - v0.z;
        }
        double d0_zx = - n0_zx[0]*v0.z - n0_zx[1]*v0.x;
        if (n0_zx[0] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[0];
        }
        if (n0_zx[1] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[1];
        }
        double d1_zx = - n1_zx[0]*v1.z - n1_zx[1]*v1.x;
        if (n1_zx[0] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[0];
        }
        if (n1_zx[1] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[1];
        }
        double d2_zx = - n2_zx[0]*v2.z - n2_zx[1]*v2.x;
        if (n2_zx[0] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[0];
        }
        if (n2_zx[1] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[1];
        }

        // dominate along x axis
        if (facep->bounding_box_type == 6)
        {
            // compute the constant of the famulation of the triangle plane
            double nx = facep->norm[0];
            double ny = facep->norm[1];
            double nz = facep->norm[2];

            // multiplied by the sign of the dominant axis in order to
            // determine the range correctly.
            int sign_ny = facep->norm_sign[1] * facep->norm_sign[0];
            int sign_nz = facep->norm_sign[2] * facep->norm_sign[0];

            double d = nx*v1.x + ny*v1.y + nz*v1.z;

            // Iterate all voxels in the 2D yz projection bounding box

            for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
            {
                for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
                {
                    op_atom++;

                    // get the minimum corner
                    double p[2];
                    p[0] = double(yi) * voxel_dim + coordinate_min[1];
                    p[1] = double(zi) * voxel_dim + coordinate_min[2];
//                    op_add += 2;
//                    op_mult += 2;

                    op_add += 2;
                    op_mult += 2;
                    op_cmp++;
                    if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0)
                    {
                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0)
                        {
                            op_add += 2;
                            op_mult += 2;
                            op_cmp++;
                            if (n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                            {
                                // passed the 2D yz projection test
                                // determine the x range in the x voxel column
                                double y0,z0,y1,z1;
                                if (sign_ny > 0)
                                {
                                    y0 = p[0] + voxel_dim;
                                    y1 = p[0];
                                }
                                else
                                {
                                    y0 = p[0];
                                    y1 = p[0] + voxel_dim;
                                }
                                if (sign_nz > 0)
                                {
                                    z0 = p[1] + voxel_dim;
                                    z1 = p[1];
                                }
                                else
                                {
                                    z0 = p[1];
                                    z1 = p[1] + voxel_dim;
                                }

                                int x0_id = computeVoxOneIdx((d - ny*y0 - nz*z0) / nx, 0);
                                int x1_id = computeVoxOneIdx((d - ny*y1 - nz*z1) / nx, 0);
                                if (x0_id > x1_id)
                                {
                                    cout << "ERROR! x0_id > x1_id : " << x0_id << " " << x1_id << endl;
                                }

                                // iterate the voxels range determined by x0 and x1
                                for (int xi=x0_id; xi<=x1_id; ++xi)
                                {
                                    op_atom++;

                                    p[0] = double(xi) * voxel_dim + coordinate_min[0];
                                    p[1] = double(yi) * voxel_dim + coordinate_min[1];
//                                    op_add += 2;
//                                    op_mult += 2;

                                    op_add += 2;
                                    op_mult += 2;
                                    op_cmp++;
                                    if (n0_xy[0]*p[0] + n0_xy[1]*p[1] + d0_xy >= 0.0)
                                    {
                                        op_add += 2;
                                        op_mult += 2;
                                        op_cmp++;
                                        if (n1_xy[0]*p[0] + n1_xy[1]*p[1] + d1_xy >= 0.0)
                                        {
                                            op_add += 2;
                                            op_mult += 2;
                                            op_cmp++;
                                            if (n2_xy[0]*p[0] + n2_xy[1]*p[1] + d2_xy >= 0.0)
                                            {
                                                p[0] = double(zi) * voxel_dim + coordinate_min[2];
                                                p[1] = double(xi) * voxel_dim + coordinate_min[0];
//                                                op_add += 2;
//                                                op_mult += 2;

                                                op_add += 2;
                                                op_mult += 2;
                                                op_cmp++;
                                                if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0)
                                                {
                                                    op_add += 2;
                                                    op_mult += 2;
                                                    op_cmp++;
                                                    if (n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0)
                                                    {
                                                        op_add += 2;
                                                        op_mult += 2;
                                                        op_cmp++;
                                                        if (n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                                                        {
                                                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // dominate along y axis
        else if (facep->bounding_box_type == 7)
        {
            // compute the constant of the famulation of the triangle plane
            double nx = facep->norm[0];
            double ny = facep->norm[1];
            double nz = facep->norm[2];

            // multiplied by the sign of the dominant axis in order to
            // determine the range correctly.
            int sign_nz = facep->norm_sign[2] * facep->norm_sign[1];
            int sign_nx = facep->norm_sign[0] * facep->norm_sign[1];

            double d = nx*v2.x + ny*v2.y + nz*v2.z;

            // Iterate all voxels in the 2D zx projection bounding box
            for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
            {
                for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
                {
                    op_atom++;

                    // get the minimum corner
                    double p[2];
                    p[0] = double(zi) * voxel_dim + coordinate_min[2];
                    p[1] = double(xi) * voxel_dim + coordinate_min[0];
                    op_add += 2;
                    op_mult += 2;

                    op_add += 2;
                    op_mult += 2;
                    op_cmp++;
                    if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0)
                    {
                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0)
                        {
                            op_add += 2;
                            op_mult += 2;
                            op_cmp++;
                            if (n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                            {
                                // passed the 2D xy projection test
                                // determine the z range in the z voxel column
                                double z0,x0,z1,x1;
                                if (sign_nz > 0)
                                {
                                    z0 = p[0] + voxel_dim;
                                    z1 = p[0];
                                }
                                else
                                {
                                    z0 = p[0];
                                    z1 = p[0] + voxel_dim;
                                }
                                if (sign_nx > 0)
                                {
                                    x0 = p[1] + voxel_dim;
                                    x1 = p[1];
                                }
                                else
                                {
                                    x0 = p[1];
                                    x1 = p[1] + voxel_dim;
                                }

                                int y0_id = computeVoxOneIdx((d - nz*z0 - nx*x0) / ny, 1);
                                int y1_id = computeVoxOneIdx((d - nz*z1 - nx*x1) / ny, 1);
                                if (y0_id > y1_id)
                                {
                                    cout << "ERROR! y0_id > y1_id : " << y0_id << " " << y1_id << endl;
                                }

                                // iterate the voxels range determined by y0 and y1
                                for (int yi=y0_id; yi<=y1_id; ++yi)
                                {
                                    op_atom++;

                                    p[0] = double(xi) * voxel_dim + coordinate_min[0];
                                    p[1] = double(yi) * voxel_dim + coordinate_min[1];
                                    op_add += 2;
                                    op_mult += 2;

                                    op_add += 2;
                                    op_mult += 2;
                                    op_cmp++;
                                    if (n0_xy[0]*p[0] + n0_xy[1]*p[1] + d0_xy >= 0.0)
                                    {
                                        op_add += 2;
                                        op_mult += 2;
                                        op_cmp++;
                                        if (n1_xy[0]*p[0] + n1_xy[1]*p[1] + d1_xy >= 0.0)
                                        {
                                            op_add += 2;
                                            op_mult += 2;
                                            op_cmp++;
                                            if (n2_xy[0]*p[0] + n2_xy[1]*p[1] + d2_xy >= 0.0)
                                            {
                                                p[0] = double(yi) * voxel_dim + coordinate_min[1];
                                                p[1] = double(zi) * voxel_dim + coordinate_min[2];
                                                op_add += 2;
                                                op_mult += 2;

                                                op_add += 2;
                                                op_mult += 2;
                                                op_cmp++;
                                                if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0)
                                                {
                                                    op_add += 2;
                                                    op_mult += 2;
                                                    op_cmp++;
                                                    if(n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0)
                                                    {
                                                        op_add += 2;
                                                        op_mult += 2;
                                                        op_cmp++;
                                                        if (n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                                                        {

                                                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }

        // dominate along z axis
        else if (facep->bounding_box_type == 8)
        {
            // compute the constant of the famulation of the triangle plane
            double nx = facep->norm[0];
            double ny = facep->norm[1];
            double nz = facep->norm[2];

            // multiplied by the sign of the dominant axis in order to
            // determine the range correctly.
            int sign_nx = facep->norm_sign[0] * facep->norm_sign[2];
            int sign_ny = facep->norm_sign[1] * facep->norm_sign[2];

            double d = nx*v0.x + ny*v0.y + nz*v0.z;

            // Iterate all voxels in the 2D xy projection bounding box
            for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
            {
                for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
                {
                    op_atom++;

                    // get the minimum corner
                    double p[2];
                    p[0] = double(xi) * voxel_dim + coordinate_min[0];
                    p[1] = double(yi) * voxel_dim + coordinate_min[1];
                    op_add += 2;
                    op_mult += 2;

                    op_add += 2;
                    op_mult += 2;
                    op_cmp++;
                    if (n0_xy[0]*p[0] + n0_xy[1]*p[1] + d0_xy >= 0.0)
                    {
                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n1_xy[0]*p[0] + n1_xy[1]*p[1] + d1_xy >= 0.0)
                        {
                            op_add += 2;
                            op_mult += 2;
                            op_cmp++;
                            if (n2_xy[0]*p[0] + n2_xy[1]*p[1] + d2_xy >= 0.0)
                            {
                                // passed the 2D xy projection test
                                // determine the z range in the z voxel column
                                double x0,y0,x1,y1;
                                if (sign_nx > 0)
                                {
                                    x0 = p[0] + voxel_dim;
                                    x1 = p[0];
                                }
                                else
                                {
                                    x0 = p[0];
                                    x1 = p[0] + voxel_dim;
                                }
                                if (sign_ny > 0)
                                {
                                    y0 = p[1] + voxel_dim;
                                    y1 = p[1];
                                }
                                else
                                {
                                    y0 = p[1];
                                    y1 = p[1] + voxel_dim;
                                }

                                int z0_id = computeVoxOneIdx((d - nx*x0 - ny*y0) / nz, 2);
                                int z1_id = computeVoxOneIdx((d - nx*x1 - ny*y1) / nz, 2);
                                if (z0_id > z1_id)
                                {
                                    cout << "ERROR! z0_id > z1_id : " << z0_id << " " << z1_id << endl;
                                }

                                // iterate the voxels range determined by z0 and z1
                                //cout << "z0_id z1_id = " << z0_id << " " << z1_id << " -- ";
                                for (int zi=z0_id; zi<=z1_id; ++zi)
                                {
                                    op_atom++;

                                    p[0] = double(yi) * voxel_dim + coordinate_min[1];
                                    p[1] = double(zi) * voxel_dim + coordinate_min[2];
                                    op_add += 2;
                                    op_mult += 2;

                                    op_add += 2;
                                    op_mult += 2;
                                    op_cmp++;
                                    if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0)
                                    {
                                        op_add += 2;
                                        op_mult += 2;
                                        op_cmp++;
                                        if (n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0)
                                        {
                                            op_add += 2;
                                            op_mult += 2;
                                            op_cmp++;
                                            if (n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                                            {
                                                p[0] = double(zi) * voxel_dim + coordinate_min[2];
                                                p[1] = double(xi) * voxel_dim + coordinate_min[0];
                                                op_add += 2;
                                                op_mult += 2;

                                                op_add += 2;
                                                op_mult += 2;
                                                op_cmp++;
                                                if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0)
                                                {
                                                    op_add += 2;
                                                    op_mult += 2;
                                                    op_cmp++;
                                                    if (n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0)
                                                    {
                                                        op_add += 2;
                                                        op_mult += 2;
                                                        op_cmp++;
                                                        if (n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                                                        {
                                                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                                                        }
                                                    }
                                                }
                                            }
                                        }
                                    }
                                } // end for zi
                            }
                        }
                    }
                } // end for yi
            } // end for xi
        }
    }
}

void VoxelMesh::markSortedFacePan11(TriangleFace* facep, int marker)
{
    //cout << "Pan 11 facep->bounding_box_type = " << facep->bounding_box_type << endl;
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    // extend in xy plane
    else if (facep->bounding_box_type == 3)
    {
        //cout << "--------- Pan 11 facep->bounding_box_type = " << facep->bounding_box_type << endl;
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[2] >= 0)
        {
            n0[0] = v0.y - v1.y;
            n0[1] = v1.x - v0.x;
            n1[0] = v1.y - v2.y;
            n1[1] = v2.x - v1.x;
            n2[0] = v2.y - v0.y;
            n2[1] = v0.x - v2.x;
        }
        else
        {
            n0[0] = v1.y - v0.y;
            n0[1] = v0.x - v1.x;
            n1[0] = v2.y - v1.y;
            n1[1] = v1.x - v2.x;
            n2[0] = v0.y - v2.y;
            n2[1] = v2.x - v0.x;
        }

        // compute the minimum corner
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[0]) + coordinate_min[0];
        p[1] = voxel_dim * double(facep->voxel_min[1]) + coordinate_min[1];

        // compute the constant di
        double d0 = n0[0]*(p[0] - v0.x) + n0[1]*(p[1] - v0.y);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1.x) + n1[1]*(p[1] - v1.y);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2.x) + n2[1]*(p[1] - v2.y);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // compute constant ni
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // Iterate all voxels in the 2D xy projection bounding box
        int zi = facep->voxel_min[2];
        int size_v = facep->voxel_max[0] - facep->voxel_min[0];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*double(v)) / n0[1];
            double u1 = (- d1 - n1[0]*double(v)) / n1[1];
            double u2 = (- d2 - n2[0]*double(v)) / n2[1];
            //cout << "u0 u1 u2 = " << u0 << " " << u1 << " " << u2 << endl;

            double r1, r2;
            if (!getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2))
            {
                cout << "ERROR ! pan11 range\n";
            }
            else
            {
                int xi = facep->voxel_min[0] + v;

                // make sure rr1 rr2 are within the bounding box.
                int size_u = facep->voxel_max[1] - facep->voxel_min[1];
                int rr1 = ceil(r1);
                if (rr1 < 0)
                    rr1 = 0;
                else if (rr1 > size_u)
                    rr1 = size_u;
                int rr2 = floor(r2);
                if (rr2 < 0)
                    rr2 = 0;
                else if (rr2 > size_u)
                    rr2 = size_u;

                for (int u=rr1; u<=rr2; u++)
                {
                    int yi = facep->voxel_min[1] + u;
                    markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                }
            }
        }
//        int zi = facep->voxel_min[2];
//        for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
//        {
//            for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
//            {
//                op_atom++;

//                // get the minimum corner
//                double p[2];
//                p[0] = double(xi) * voxel_dim + coordinate_min[0];
//                p[1] = double(yi) * voxel_dim + coordinate_min[1];
//                op_add += 2;
//                op_mult += 2;

//                op_add += 2;
//                op_mult += 2;
//                op_cmp++;
//                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0)
//                {
//                    op_add += 2;
//                    op_mult += 2;
//                    op_cmp++;
//                    if (n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0)
//                    {
//                        op_add += 2;
//                        op_mult += 2;
//                        op_cmp++;
//                        if (n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
//                        {
//                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
//                        }
//                    }
//                }
//            }
//        }
    }

    // extend in yz plane
    else if (facep->bounding_box_type == 4)
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[0] >= 0)
        {
            n0[0] = v0.z - v1.z;
            n0[1] = v1.y - v0.y;
            n1[0] = v1.z - v2.z;
            n1[1] = v2.y - v1.y;
            n2[0] = v2.z - v0.z;
            n2[1] = v0.y - v2.y;
        }
        else
        {
            n0[0] = v1.z - v0.z;
            n0[1] = v0.y - v1.y;
            n1[0] = v2.z - v1.z;
            n1[1] = v1.y - v2.y;
            n2[0] = v0.z - v2.z;
            n2[1] = v2.y - v0.y;
        }

        // compute the minimum corner
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[1]) + coordinate_min[1];
        p[1] = voxel_dim * double(facep->voxel_min[2]) + coordinate_min[2];

        // compute the constant d_i
        double d0 = n0[0]*(p[0] - v0.y) + n0[1]*(p[1] - v0.z);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1.y) + n1[1]*(p[1] - v1.z);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2.y) + n2[1]*(p[1] - v2.z);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // compute constant ni
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // Iterate all voxels in the 2D projection bounding box
        int xi = facep->voxel_min[0];
        int size_v = facep->voxel_max[1] - facep->voxel_min[1];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*double(v)) / n0[1];
            double u1 = (- d1 - n1[0]*double(v)) / n1[1];
            double u2 = (- d2 - n2[0]*double(v)) / n2[1];
            //cout << "u0 u1 u2 = " << u0 << " " << u1 << " " << u2 << endl;

            double r1, r2;
            if (!getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2))
            {
                cout << "ERROR ! pan11 range\n";
            }
            else
            {
                int yi = facep->voxel_min[1] + v;

                // make sure the range is within the bounding box.
                int size_u = facep->voxel_max[2] - facep->voxel_min[2];
                int rr1 = ceil(r1);
                if (rr1 < 0)
                    rr1 = 0;
                else if (rr1 > size_u)
                    rr1 = size_u;
                int rr2 = floor(r2);
                if (rr2 < 0)
                    rr2 = 0;
                else if (rr2 > size_u)
                    rr2 = size_u;

                for (int u=rr1; u<=rr2; u++)
                {
                    int zi = facep->voxel_min[2] + u;
                    markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                }
            }
        }

//        int xi = facep->voxel_min[0];
//        for (int yi=facep->voxel_min[1]; yi<=facep->voxel_max[1]; ++yi)
//        {
//            for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
//            {
//                op_atom++;

//                // get the minimum corner
//                double p[2];
//                p[0] = double(yi) * voxel_dim + coordinate_min[1];
//                p[1] = double(zi) * voxel_dim + coordinate_min[2];
//                op_add += 2;
//                op_mult += 2;

//                op_add += 2;
//                op_mult += 2;
//                op_cmp++;
//                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0)
//                {
//                    op_add += 2;
//                    op_mult += 2;
//                    op_cmp++;
//                    if (n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0)
//                    {
//                        op_add += 2;
//                        op_mult += 2;
//                        op_cmp++;
//                        if (n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
//                        {
//                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
//                        }
//                    }
//                }
//            }
//        }
    }

    // extend in zx plane
    else if (facep->bounding_box_type == 5)
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[1] >= 0)
        {
            n0[0] = v0.x - v1.x;
            n0[1] = v1.z - v0.z;
            n1[0] = v1.x - v2.x;
            n1[1] = v2.z - v1.z;
            n2[0] = v2.x - v0.x;
            n2[1] = v0.z - v2.z;
        }
        else
        {
            n0[0] = v1.x - v0.x;
            n0[1] = v0.z - v1.z;
            n1[0] = v2.x - v1.x;
            n1[1] = v1.z - v2.z;
            n2[0] = v0.x - v2.x;
            n2[1] = v2.z - v0.z;
        }

        // compute the minimum corner
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[2]) + coordinate_min[2];
        p[1] = voxel_dim * double(facep->voxel_min[0]) + coordinate_min[0];

        // compute the constant di
        double d0 = n0[0]*(p[0] - v0.z) + n0[1]*(p[1] - v0.x);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1.z) + n1[1]*(p[1] - v1.x);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2.z) + n2[1]*(p[1] - v2.x);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Iterate all voxels in the 2D projection bounding box
        // compute constant ni
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // Iterate all voxels in the 2D zx projection bounding box
        int yi = facep->voxel_min[1];
        int size_v = facep->voxel_max[2] - facep->voxel_min[2];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*double(v)) / n0[1];
            double u1 = (- d1 - n1[0]*double(v)) / n1[1];
            double u2 = (- d2 - n2[0]*double(v)) / n2[1];
            //cout << "u0 u1 u2 = " << u0 << " " << u1 << " " << u2 << endl;

            double r1, r2;
            if (!getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2))
            {
                cout << "ERROR ! pan11 range\n";
            }
            else
            {
                int zi = facep->voxel_min[2] + v;

                // make sure the range is within the bounding box.
                int size_u = facep->voxel_max[0] - facep->voxel_min[0];
                int rr1 = ceil(r1);
                if (rr1 < 0)
                    rr1 = 0;
                else if (rr1 > size_u)
                    rr1 = size_u;
                int rr2 = floor(r2);
                if (rr2 < 0)
                    rr2 = 0;
                else if (rr2 > size_u)
                    rr2 = size_u;

                for (int u=rr1; u<=rr2; u++)
                {
                    int xi = facep->voxel_min[0] + u;
                    markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                }
            }
        }
//        int yi = facep->voxel_min[1];
//        for (int zi=facep->voxel_min[2]; zi<=facep->voxel_max[2]; ++zi)
//        {
//            for (int xi=facep->voxel_min[0]; xi<=facep->voxel_max[0]; ++xi)
//            {
//                op_atom++;

//                // get the minimum corner
//                double p[2];
//                p[0] = double(zi) * voxel_dim + coordinate_min[2];
//                p[1] = double(xi) * voxel_dim + coordinate_min[0];
//                op_add += 2;
//                op_mult += 2;

//                op_add += 2;
//                op_mult += 2;
//                op_cmp++;
//                if (n0[0]*p[0] + n0[1]*p[1] + d0 >= 0.0)
//                {
//                    op_add += 2;
//                    op_mult += 2;
//                    op_cmp++;
//                    if (n1[0]*p[0] + n1[1]*p[1] + d1 >= 0.0)
//                    {
//                        op_add += 2;
//                        op_mult += 2;
//                        op_cmp++;
//                        if (n2[0]*p[0] + n2[1]*p[1] + d2 >= 0.0)
//                        {
//                            markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
//                        }
//                    }
//                }
//            }
//        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        zVec3 v0 = facep->node0->ori_coordinate;
        zVec3 v1 = facep->node1->ori_coordinate;
        zVec3 v2 = facep->node2->ori_coordinate;

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        // compute the normals of the 2D edges
        double n0[2], n1[2], n2[2];
        if (facep->norm_sign[axis_z] >= 0)
        {
            n0[0] = v0[axis_y] - v1[axis_y];
            n0[1] = v1[axis_x] - v0[axis_x];
            n1[0] = v1[axis_y] - v2[axis_y];
            n1[1] = v2[axis_x] - v1[axis_x];
            n2[0] = v2[axis_y] - v0[axis_y];
            n2[1] = v0[axis_x] - v2[axis_x];
        }
        else
        {
            n0[0] = v1[axis_y] - v0[axis_y];
            n0[1] = v0[axis_x] - v1[axis_x];
            n1[0] = v2[axis_y] - v1[axis_y];
            n1[1] = v1[axis_x] - v2[axis_x];
            n2[0] = v0[axis_y] - v2[axis_y];
            n2[1] = v2[axis_x] - v0[axis_x];
        }

        // compute the minimum corner in the xy plane
        double p[2];
        p[0] = voxel_dim * double(facep->voxel_min[axis_x]) + coordinate_min[axis_x];
        p[1] = voxel_dim * double(facep->voxel_min[axis_y]) + coordinate_min[axis_y];

        // compute the constant di
        double d0 = n0[0]*(p[0] - v0[axis_x]) + n0[1]*(p[1] - v0[axis_y]);
        if (n0[0] > 0.0)
        {
            d0 += voxel_dim * n0[0];
        }
        if (n0[1] > 0.0)
        {
            d0 += voxel_dim * n0[1];
        }
        double d1 = n1[0]*(p[0] - v1[axis_x]) + n1[1]*(p[1] - v1[axis_y]);
        if (n1[0] > 0.0)
        {
            d1 += voxel_dim * n1[0];
        }
        if (n1[1] > 0.0)
        {
            d1 += voxel_dim * n1[1];
        }
        double d2 = n2[0]*(p[0] - v2[axis_x]) + n2[1]*(p[1] - v2[axis_y]);
        if (n2[0] > 0.0)
        {
            d2 += voxel_dim * n2[0];
        }
        if (n2[1] > 0.0)
        {
            d2 += voxel_dim * n2[1];
        }

        // Setup for the 2D yz projection test
        double n0_yz[2], n1_yz[2], n2_yz[2];
        if (facep->norm_sign[axis_x] >= 0)
        {
            n0_yz[0] = v0[axis_z] - v1[axis_z];
            n0_yz[1] = v1[axis_y] - v0[axis_y];
            n1_yz[0] = v1[axis_z] - v2[axis_z];
            n1_yz[1] = v2[axis_y] - v1[axis_y];
            n2_yz[0] = v2[axis_z] - v0[axis_z];
            n2_yz[1] = v0[axis_y] - v2[axis_y];
        }
        else
        {
            n0_yz[0] = v1[axis_z] - v0[axis_z];
            n0_yz[1] = v0[axis_y] - v1[axis_y];
            n1_yz[0] = v2[axis_z] - v1[axis_z];
            n1_yz[1] = v1[axis_y] - v2[axis_y];
            n2_yz[0] = v0[axis_z] - v2[axis_z];
            n2_yz[1] = v2[axis_y] - v0[axis_y];
        }
        double d0_yz = - n0_yz[0]*v0[axis_y] - n0_yz[1]*v0[axis_z];
        if (n0_yz[0] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[0];
        }
        if (n0_yz[1] > 0.0)
        {
            d0_yz += voxel_dim * n0_yz[1];
        }
        double d1_yz = - n1_yz[0]*v1[axis_y] - n1_yz[1]*v1[axis_z];
        if (n1_yz[0] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[0];
        }
        if (n1_yz[1] > 0.0)
        {
            d1_yz += voxel_dim * n1_yz[1];
        }
        double d2_yz = - n2_yz[0]*v2[axis_y] - n2_yz[1]*v2[axis_z];
        if (n2_yz[0] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[0];
        }
        if (n2_yz[1] > 0.0)
        {
            d2_yz += voxel_dim * n2_yz[1];
        }

        // Setup for the 2D zx projection test
        double n0_zx[2], n1_zx[2], n2_zx[2];
        if (facep->norm_sign[axis_y] >= 0)
        {
            n0_zx[0] = v0[axis_x] - v1[axis_x];
            n0_zx[1] = v1[axis_z] - v0[axis_z];
            n1_zx[0] = v1[axis_x] - v2[axis_x];
            n1_zx[1] = v2[axis_z] - v1[axis_z];
            n2_zx[0] = v2[axis_x] - v0[axis_x];
            n2_zx[1] = v0[axis_z] - v2[axis_z];
        }
        else
        {
            n0_zx[0] = v1[axis_x] - v0[axis_x];
            n0_zx[1] = v0[axis_z] - v1[axis_z];
            n1_zx[0] = v2[axis_x] - v1[axis_x];
            n1_zx[1] = v1[axis_z] - v2[axis_z];
            n2_zx[0] = v0[axis_x] - v2[axis_x];
            n2_zx[1] = v2[axis_z] - v0[axis_z];
        }
        double d0_zx = - n0_zx[0]*v0[axis_z] - n0_zx[1]*v0[axis_x];
        if (n0_zx[0] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[0];
        }
        if (n0_zx[1] > 0.0)
        {
            d0_zx += voxel_dim * n0_zx[1];
        }
        double d1_zx = - n1_zx[0]*v1[axis_z] - n1_zx[1]*v1[axis_x];
        if (n1_zx[0] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[0];
        }
        if (n1_zx[1] > 0.0)
        {
            d1_zx += voxel_dim * n1_zx[1];
        }
        double d2_zx = - n2_zx[0]*v2[axis_z] - n2_zx[1]*v2[axis_x];
        if (n2_zx[0] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[0];
        }
        if (n2_zx[1] > 0.0)
        {
            d2_zx += voxel_dim * n2_zx[1];
        }

        // compute constant ni in the xy plane
        n0[0] *= voxel_dim;
        n0[1] *= voxel_dim;
        n1[0] *= voxel_dim;
        n1[1] *= voxel_dim;
        n2[0] *= voxel_dim;
        n2[1] *= voxel_dim;

        // compute the constant of the famulation of the triangle plane
        double nx = facep->norm[axis_x];
        double ny = facep->norm[axis_y];
        double nz = facep->norm[axis_z];

        // multiplied by the sign of the dominant axis in order to
        // determine the range correctly.
        int sign_nx = facep->norm_sign[axis_x] * facep->norm_sign[axis_z];
        int sign_ny = facep->norm_sign[axis_y] * facep->norm_sign[axis_z];

        double d = nx*v0[axis_x] + ny*v0[axis_y] + nz*v0[axis_z];

        int moving_vox[3];
        int size_v = facep->voxel_max[axis_x] - facep->voxel_min[axis_x];
        for (int v=0; v<=size_v; v++)
        {
            double u0 = (- d0 - n0[0]*double(v)) / n0[1];
            double u1 = (- d1 - n1[0]*double(v)) / n1[1];
            double u2 = (- d2 - n2[0]*double(v)) / n2[1];
            //cout << "u0 u1 u2 = " << u0 << " " << u1 << " " << u2 << endl;

            double r1, r2;
            if (!getRangePa11(r1, r2, n0[1], u0, n1[1], u1, n2[1], u2))
            {
                cout << "ERROR ! pan11 range\n";
            }
            else
            {
                // determine the axis x component
                moving_vox[axis_x] = facep->voxel_min[axis_x] + v;

                // make sure the range is valid.
                int size_u = facep->voxel_max[axis_y] - facep->voxel_min[axis_y];
                int rr1 = ceil(r1);
                if (rr1 < 0)
                    rr1 = 0;
                else if (rr1 > size_u)
                    rr1 = size_u;
                int rr2 = floor(r2);
                if (rr2 < 0)
                    rr2 = 0;
                else if (rr2 > size_u)
                    rr2 = size_u;
                //cout << "rr1 rr2 = " << rr1 << " " << rr2 << endl;

                for (int u=rr1; u<=rr2; u++)
                {
                    // determine the y component
                    moving_vox[axis_y] = facep->voxel_min[axis_y] + u;

                    // passed the 2D xy projection test so far
                    // determine the z range in the z voxel column

                    // compute the min and max corner in the voxel column
                    p[0] = voxel_dim * double(moving_vox[axis_x]) + coordinate_min[axis_x];
                    p[1] = voxel_dim * double(moving_vox[axis_y]) + coordinate_min[axis_y];
                    double x0,y0,x1,y1;
                    if (sign_nx > 0)
                    {
                        x0 = p[0] + voxel_dim;
                        x1 = p[0];
                    }
                    else
                    {
                        x0 = p[0];
                        x1 = p[0] + voxel_dim;
                    }
                    if (sign_ny > 0)
                    {
                        y0 = p[1] + voxel_dim;
                        y1 = p[1];
                    }
                    else
                    {
                        y0 = p[1];
                        y1 = p[1] + voxel_dim;
                    }

                    int z0_id = computeVoxOneIdx((d - nx*x0 - ny*y0) / nz, axis_z);
                    int z1_id = computeVoxOneIdx((d - nx*x1 - ny*y1) / nz, axis_z);
                    if (z0_id > z1_id)
                    {
                        cout << "ERROR! z0_id > z1_id : " << z0_id << " " << z1_id << endl;
                    }

                    // iterate the voxels range determined by z0 and z1
                    //cout << "z0_id z1_id = " << z0_id << " " << z1_id << " -- ";
                    for (moving_vox[axis_z]=z0_id; moving_vox[axis_z]<=z1_id; moving_vox[axis_z]++)
                    {
                        op_atom++;

                        p[0] = double(moving_vox[axis_y]) * voxel_dim + coordinate_min[axis_y];
                        p[1] = double(moving_vox[axis_z]) * voxel_dim + coordinate_min[axis_z];
                        op_add += 2;
                        op_mult += 2;

                        op_add += 2;
                        op_mult += 2;
                        op_cmp++;
                        if (n0_yz[0]*p[0] + n0_yz[1]*p[1] + d0_yz >= 0.0)
                        {
                            op_add += 2;
                            op_mult += 2;
                            op_cmp++;
                            if (n1_yz[0]*p[0] + n1_yz[1]*p[1] + d1_yz >= 0.0)
                            {
                                op_add += 2;
                                op_mult += 2;
                                op_cmp++;
                                if (n2_yz[0]*p[0] + n2_yz[1]*p[1] + d2_yz >= 0.0)
                                {
                                    p[0] = double(moving_vox[axis_z]) * voxel_dim + coordinate_min[axis_z];
                                    p[1] = double(moving_vox[axis_x]) * voxel_dim + coordinate_min[axis_x];
                                    op_add += 2;
                                    op_mult += 2;

                                    op_add += 2;
                                    op_mult += 2;
                                    op_cmp++;
                                    if (n0_zx[0]*p[0] + n0_zx[1]*p[1] + d0_zx >= 0.0)
                                    {
                                        op_add += 2;
                                        op_mult += 2;
                                        op_cmp++;
                                        if (n1_zx[0]*p[0] + n1_zx[1]*p[1] + d1_zx >= 0.0)
                                        {
                                            op_add += 2;
                                            op_mult += 2;
                                            op_cmp++;
                                            if (n2_zx[0]*p[0] + n2_zx[1]*p[1] + d2_zx >= 0.0)
                                            {
                                                int xi = moving_vox[0];
                                                int yi = moving_vox[1];
                                                int zi = moving_vox[2];
                                                markPoint(xi + (yi + zi*grid_density) * (long long)grid_density, marker);
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    } // end for zi
                }
            }
        }
    }
}

//===============================================================================
// Mark a face/triangle using floating scanline-based method.
//===============================================================================
void VoxelMesh::markFaceByScanlineFLT(TriangleFace* facep, int marker)
{
    //cout << "facep->bounding_box_type " << facep->bounding_box_type << endl;
    //----------------------------------------------------------------------------------
    // In the case of 1D bounding box
    //----------------------------------------------------------------------------------
    if (facep->bounding_box_type == 0)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[0] - facep->voxel_min[0];
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point++;
        }
    }
    else if (facep->bounding_box_type == 1)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[1] - facep->voxel_min[1];
        int step = grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point += step;
        }
    }
    else if (facep->bounding_box_type == 2)
    {
        long long active_point = facep->voxel_min[0] +
                                (facep->voxel_min[1] + facep->voxel_min[2]*grid_density)
                                * (long long)grid_density;
        int len_voxel_column = facep->voxel_max[2] - facep->voxel_min[2];
        int step = grid_density * grid_density;
        for (int i=0; i<=len_voxel_column; ++i)
        {
            markPoint(active_point, marker);
            active_point += step;
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 2D bounding box
    //----------------------------------------------------------------------------------
    else if (facep->bounding_box_type == 3 ||
             facep->bounding_box_type == 4 ||
             facep->bounding_box_type == 5)
    {
        double p0[3], p1[3], p2[3];
        p0[0] = facep->v0[0];
        p0[1] = facep->v0[1];
        p0[2] = facep->v0[2];
        p1[0] = facep->v1[0];
        p1[1] = facep->v1[1];
        p1[2] = facep->v1[2];
        p2[0] = facep->v2[0];
        p2[1] = facep->v2[1];
        p2[2] = facep->v2[2];

        int axis_x = facep->axis_x;
        int axis_y = facep->axis_y;
        int axis_z = facep->axis_z;

        int slice_id = facep->vox0[axis_z];

        // mark the three edges
        markLine2D(p0[axis_x], p0[axis_y], p1[axis_x], p1[axis_y],
                   axis_x, axis_y, axis_z, slice_id, marker);
        markLine2D(p0[axis_x], p0[axis_y], p2[axis_x], p2[axis_y],
                   axis_x, axis_y, axis_z, slice_id, marker);
        markLine2D(p1[axis_x], p1[axis_y], p2[axis_x], p2[axis_y],
                   axis_x, axis_y, axis_z, slice_id, marker);

        // compute the three edge vectors p0p1, p0p2 and p1p2, and their length
        double p0p1[2], p0p2[2], p1p2[2];
        p0p1[0] = p1[axis_x]-p0[axis_x];
        p0p1[1] = p1[axis_y]-p0[axis_y];
        p0p2[0] = p2[axis_x]-p0[axis_x];
        p0p2[1] = p2[axis_y]-p0[axis_y];
        p1p2[0] = p2[axis_x]-p1[axis_x];
        p1p2[1] = p2[axis_y]-p1[axis_y];

        // compute the optimal step distance
        // along the norm direction of p1p2
        double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
        double d_hat = 0.99*voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

        // compute the step distance along p0p1 corresponding to d_hat
        double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
        double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
        double step01[2];
        step01[0] = d01 * p0p1[0] / len01;
        step01[1] = d01 * p0p1[1] / len01;

        // compute the step distance along p0p2 corresponding to d_hat
        double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
        double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
        double step02[2];
        step02[0] = d02 * p0p2[0] / len02;
        step02[1] = d02 * p0p2[1] / len02;

        // two moving points will move along p0p1 and p0p2
        double moving_p0p1[2], moving_p0p2[2];
        moving_p0p1[0] = p0[axis_x] + step01[0];
        moving_p0p1[1] = p0[axis_y] + step01[1];
        moving_p0p2[0] = p0[axis_x] + step02[0];
        moving_p0p2[1] = p0[axis_y] + step02[1];

        // iterate and voxelize all the scanlines
        int size_steps = len01 / d01;
        for (int i=0; i<size_steps; ++i)
        {
            markLine2D(moving_p0p1[0], moving_p0p1[1], moving_p0p2[0], moving_p0p2[1],
                       axis_x, axis_y, axis_z, slice_id, marker);
            moving_p0p1[0] += step01[0];
            moving_p0p1[1] += step01[1];
            moving_p0p2[0] += step02[0];
            moving_p0p2[1] += step02[1];
        }
    }

    //----------------------------------------------------------------------------------
    // In the case of 3D bounding box
    //----------------------------------------------------------------------------------
    else
    {
        markFaceByScanlineSpeed(facep);
//        double p0[3], p1[3], p2[3];
//        p0[0] = facep->v0[0];
//        p0[1] = facep->v0[1];
//        p0[2] = facep->v0[2];
//        p1[0] = facep->v1[0];
//        p1[1] = facep->v1[1];
//        p1[2] = facep->v1[2];
//        p2[0] = facep->v2[0];
//        p2[1] = facep->v2[1];
//        p2[2] = facep->v2[2];

//        int p0_vox[3], p1_vox[3], p2_vox[3];
//        p0_vox[0] = facep->vox0[0];
//        p0_vox[1] = facep->vox0[1];
//        p0_vox[2] = facep->vox0[2];
//        p1_vox[0] = facep->vox1[0];
//        p1_vox[1] = facep->vox1[1];
//        p1_vox[2] = facep->vox1[2];
//        p2_vox[0] = facep->vox2[0];
//        p2_vox[1] = facep->vox2[1];
//        p2_vox[2] = facep->vox2[2];

//        int axis_x = facep->axis_x;
//        int axis_y = facep->axis_y;
//        int axis_z = facep->axis_z;

//        //-----------------------------------------------------------------------------
//        // Mark the three edges.
//        //-----------------------------------------------------------------------------
////        mark3DLine(p0, p2, p0_vox, p2_vox, marker);
////        mark3DLine(p0, p1, p0_vox, p1_vox, marker);
////        mark3DLine(p1, p2, p1_vox, p2_vox, marker);

//        //-------------------------------------------------------------------------------
//        // compute the optimal steps along two edge p0p1 and p0p2 in 2D projection
//        //-------------------------------------------------------------------------------
//        // compute the three projected edge vectors p0p1, p0p2 and p1p2, and their length
//        double p0p1[2], p0p2[2], p1p2[2];
//        p0p1[0] = p1[axis_x]-p0[axis_x];
//        p0p1[1] = p1[axis_y]-p0[axis_y];
//        p0p2[0] = p2[axis_x]-p0[axis_x];
//        p0p2[1] = p2[axis_y]-p0[axis_y];
//        p1p2[0] = p2[axis_x]-p1[axis_x];
//        p1p2[1] = p2[axis_y]-p1[axis_y];

//        // compute the optimal step distance in 2D projections.
//        // along the norm direction of p1p2
//        double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
//        double d_hat = 0.99*voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

//        // compute the step distance along p0p1 corresponding to d_hat
//        double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
//        double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
//        double step01[2];
//        step01[0] = d01 * p0p1[0] / len01;
//        step01[1] = d01 * p0p1[1] / len01;

//        // compute the step distance along p0p2 corresponding to d_hat
//        double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
//        double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
//        double step02[2];
//        step02[0] = d02 * p0p2[0] / len02;
//        step02[1] = d02 * p0p2[1] / len02;

//        //-----------------------------------------------------------------------------
//        // Voxelize the triangle.
//        //-----------------------------------------------------------------------------
//        // Determine the 3D line segment p0p1.
//        double a1 = p1[axis_x]-p0[axis_x], b1 = p0[axis_x];
//        double c1 = p1[axis_y]-p0[axis_y], d1 = p0[axis_y];
//        double e1 = p1[axis_z]-p0[axis_z], f1 = p0[axis_z];

//        // Determine the 3D line segment p0p2.
//        double a2 = p2[axis_x]-p0[axis_x], b2 = p0[axis_x];
//        double c2 = p2[axis_y]-p0[axis_y], d2 = p0[axis_y];
//        double e2 = p2[axis_z]-p0[axis_z], f2 = p0[axis_z];

//        // voxelize according to the sub type of the triangle
//        if (facep->bounding_box_subtype == 0)
//        {
//            // Determine the index of the first slice and the first boundary plane.
//            int slice_idx = p0_vox[axis_z];
//            double plane = voxel_dim * (p0_vox[axis_z] + 1) + coordinate_min[axis_z];
//            if (plane == p0[axis_z])
//            {
//                // if the plane happened to intersect p0,
//                // refine the slice index and move to the next plane.
//                plane += voxel_dim;
//                slice_idx++;
//            }

//            // the boundary of a slice is determined by four points along edge p0p1 and p0p2.
//            // i.e. slice_point01_s, slice_point01_e, and slice_point02_s, slice_point02_e.
//            // the boundary points are the intersections of the boundary planes perpendicular to axis_z.
//            double slice_point01_s[2], slice_point01_e[2];
//            double slice_point02_s[2], slice_point02_e[2];

//            // in the first slice, slice_p0p1_0 and slice_p0p2_0 are actually p0.
//            slice_point01_s[0] = slice_point02_s[0] = p0[axis_x];
//            slice_point01_s[1] = slice_point02_s[1] = p0[axis_y];

//            // the moving point iterates the 2D scanline in the slice
//            double moving_point01[2], moving_point02[2];
//            while  (slice_idx < p1_vox[axis_z])
//            {
//                //cout << "slice idx = " << slice_idx << " " << endl;
//                // compute the boundary points slice_point01_e and slice_point02_e.
//                double t1 = (plane - f1) / e1;
//                slice_point01_e[0] = a1 * t1 + b1;
//                slice_point01_e[1] = c1 * t1 + d1;

//                double t2 = (plane - f2) / e2;
//                slice_point02_e[0] = a2 * t2 + b2;
//                slice_point02_e[1] = c2 * t2 + d2;

//                // mark two legs
//                markLine2D(slice_point01_s[0], slice_point01_s[1],
//                           slice_point01_e[0], slice_point01_e[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);
//                markLine2D(slice_point02_s[0], slice_point02_s[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);

//                // the first 2D scanline
//                moving_point01[0] = slice_point01_s[0];
//                moving_point01[1] = slice_point01_s[1];
//                moving_point02[0] = slice_point02_s[0];
//                moving_point02[1] = slice_point02_s[1];

//                // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//                double temp0 = slice_point01_e[0] - slice_point01_s[0];
//                double temp1 = slice_point01_e[1] - slice_point01_s[1];
//                double len2_slice_edge = temp0*temp0 + temp1*temp1;
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                double len2 = temp0*temp0 + temp1*temp1;
//                while (len2_slice_edge > len2)
//                {
//                    markLine2D(moving_point01[0], moving_point01[1],
//                               moving_point02[0], moving_point02[1],
//                               axis_x, axis_y, axis_z, slice_idx, marker);

//                    moving_point01[0] += step01[0];
//                    moving_point01[1] += step01[1];
//                    moving_point02[0] += step02[0];
//                    moving_point02[1] += step02[1];

//                    // compute len2
//                    temp0 = moving_point01[0] - slice_point01_s[0];
//                    temp1 = moving_point01[1] - slice_point01_s[1];
//                    len2 = temp0*temp0 + temp1*temp1;
//                }

//                // mark the last 2D scanline
//                markLine2D(slice_point01_e[0], slice_point01_e[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);

//                // let the boundary points move to the next slice.
//                slice_point01_s[0] = slice_point01_e[0];
//                slice_point01_s[1] = slice_point01_e[1];
//                slice_point02_s[0] = slice_point02_e[0];
//                slice_point02_s[1] = slice_point02_e[1];

//                plane += voxel_dim;
//                slice_idx++;
//            }

//            // deal with the last slice
//            // the boundary points slice_point01_e and slice_point02_e should be the edge p1p2.
//            slice_point01_e[0] = p1[axis_x];
//            slice_point01_e[1] = p1[axis_y];
//            slice_point02_e[0] = p2[axis_x];
//            slice_point02_e[1] = p2[axis_y];

//            // mark two legs
//            markLine2D(slice_point01_s[0], slice_point01_s[1],
//                       slice_point01_e[0], slice_point01_e[1],
//                       axis_x, axis_y, axis_z, slice_idx, marker);
//            markLine2D(slice_point02_s[0], slice_point02_s[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx, marker);

//            // the first 2D scanline
//            moving_point01[0] = slice_point01_s[0];
//            moving_point01[1] = slice_point01_s[1];
//            moving_point02[0] = slice_point02_s[0];
//            moving_point02[1] = slice_point02_s[1];

//            // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//            double temp0 = slice_point01_e[0] - slice_point01_s[0];
//            double temp1 = slice_point01_e[1] - slice_point01_s[1];
//            double len2_slice_edge = temp0*temp0 + temp1*temp1;
//            temp0 = moving_point01[0] - slice_point01_s[0];
//            temp1 = moving_point01[1] - slice_point01_s[1];
//            double len2 = temp0*temp0 + temp1*temp1;
//            while (len2_slice_edge > len2)
//            {
//                markLine2D(moving_point01[0], moving_point01[1],
//                           moving_point02[0], moving_point02[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);

//                moving_point01[0] += step01[0];
//                moving_point01[1] += step01[1];
//                moving_point02[0] += step02[0];
//                moving_point02[1] += step02[1];

//                // compute len2
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                len2 = temp0*temp0 + temp1*temp1;
//            }

//            // mark the last 2D scanline
//            markLine2D(slice_point01_e[0], slice_point01_e[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx, marker);
//        }
//        else
//        {
//            // Determine the index of the first slice and the first boundary plane.
//            int slice_idx = p0_vox[axis_z];
//            double plane = voxel_dim * p0_vox[axis_z] + coordinate_min[axis_z];
//            if (plane == p0[axis_z])
//            {
//                // if the plane happened to intersect p0,
//                // refine the slice index and move to the next plane.
//                plane -= voxel_dim;
//                slice_idx--;
//            }

//            // the boundary of a slice is determined by four points along edge p0p1 and p0p2.
//            // i.e. slice_point01_s, slice_point01_e, and slice_point02_s, slice_point02_e.
//            // the boundary points are the intersections of the boundary planes perpendicular to axis_z.
//            double slice_point01_s[2], slice_point01_e[2];
//            double slice_point02_s[2], slice_point02_e[2];

//            // in the first slice, slice_p0p1_0 and slice_p0p2_0 are actually p0.
//            slice_point01_s[0] = slice_point02_s[0] = p0[axis_x];
//            slice_point01_s[1] = slice_point02_s[1] = p0[axis_y];

//            // the moving point iterates the 2D scanline in the slice
//            double moving_point01[2], moving_point02[2];
//            while  (slice_idx > p1_vox[axis_z])
//            {
//                // compute the boundary points slice_point01_e and slice_point02_e.
//                double t1 = (plane - f1) / e1;
//                slice_point01_e[0] = a1 * t1 + b1;
//                slice_point01_e[1] = c1 * t1 + d1;

//                double t2 = (plane - f2) / e2;
//                slice_point02_e[0] = a2 * t2 + b2;
//                slice_point02_e[1] = c2 * t2 + d2;

//                // mark two legs
//                markLine2D(slice_point01_s[0], slice_point01_s[1],
//                           slice_point01_e[0], slice_point01_e[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);
//                markLine2D(slice_point02_s[0], slice_point02_s[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);

//                // the first 2D scanline
//                moving_point01[0] = slice_point01_s[0];
//                moving_point01[1] = slice_point01_s[1];
//                moving_point02[0] = slice_point02_s[0];
//                moving_point02[1] = slice_point02_s[1];

//                // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//                double temp0 = slice_point01_e[0] - slice_point01_s[0];
//                double temp1 = slice_point01_e[1] - slice_point01_s[1];
//                double len2_slice_edge = temp0*temp0 + temp1*temp1;
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                double len2 = temp0*temp0 + temp1*temp1;
//                while (len2_slice_edge >= len2)
//                {
//                    //cout << "slice idx = " << slice_idx << " " << endl;
//                    markLine2D(moving_point01[0], moving_point01[1],
//                            moving_point02[0], moving_point02[1],
//                            axis_x, axis_y, axis_z, slice_idx, marker);

//                    moving_point01[0] += step01[0];
//                    moving_point01[1] += step01[1];
//                    moving_point02[0] += step02[0];
//                    moving_point02[1] += step02[1];

//                    // compute len2
//                    temp0 = moving_point01[0] - slice_point01_s[0];
//                    temp1 = moving_point01[1] - slice_point01_s[1];
//                    len2 = temp0*temp0 + temp1*temp1;
//                }

//                // mark the last 2D scanline
//                markLine2D(slice_point01_e[0], slice_point01_e[1],
//                           slice_point02_e[0], slice_point02_e[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);

//                // let the boundary points move to the next slice.
//                slice_point01_s[0] = slice_point01_e[0];
//                slice_point01_s[1] = slice_point01_e[1];
//                slice_point02_s[0] = slice_point02_e[0];
//                slice_point02_s[1] = slice_point02_e[1];

//                plane -= voxel_dim;
//                slice_idx--;
//            }

//            // deal with the last slice
//            // the boundary points slice_point01_e and slice_point02_e should be the edge p1p2.
//            slice_point01_e[0] = p1[axis_x];
//            slice_point01_e[1] = p1[axis_y];
//            slice_point02_e[0] = p2[axis_x];
//            slice_point02_e[1] = p2[axis_y];

//            // mark two legs
//            markLine2D(slice_point01_s[0], slice_point01_s[1],
//                       slice_point01_e[0], slice_point01_e[1],
//                       axis_x, axis_y, axis_z, slice_idx, marker);
//            markLine2D(slice_point02_s[0], slice_point02_s[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx, marker);

//            // the first 2D scanline
//            moving_point01[0] = slice_point01_s[0];
//            moving_point01[1] = slice_point01_s[1];
//            moving_point02[0] = slice_point02_s[0];
//            moving_point02[1] = slice_point02_s[1];

//            // set up the comparison constant, i.e. the len2 between 01_s and 01_e
//            double temp0 = slice_point01_e[0] - slice_point01_s[0];
//            double temp1 = slice_point01_e[1] - slice_point01_s[1];
//            double len2_slice_edge = temp0*temp0 + temp1*temp1;
//            temp0 = moving_point01[0] - slice_point01_s[0];
//            temp1 = moving_point01[1] - slice_point01_s[1];
//            double len2 = temp0*temp0 + temp1*temp1;
//            while (len2_slice_edge > len2)
//            {
//                markLine2D(moving_point01[0], moving_point01[1],
//                           moving_point02[0], moving_point02[1],
//                           axis_x, axis_y, axis_z, slice_idx, marker);

//                moving_point01[0] += step01[0];
//                moving_point01[1] += step01[1];
//                moving_point02[0] += step02[0];
//                moving_point02[1] += step02[1];

//                // compute len2
//                temp0 = moving_point01[0] - slice_point01_s[0];
//                temp1 = moving_point01[1] - slice_point01_s[1];
//                len2 = temp0*temp0 + temp1*temp1;
//            }

//            // mark the last 2D scanline
//            markLine2D(slice_point01_e[0], slice_point01_e[1],
//                       slice_point02_e[0], slice_point02_e[1],
//                       axis_x, axis_y, axis_z, slice_idx, marker);
//        }
    }

}

void VoxelMesh::markLine2D(const double start_x, const double start_y,
                           const double end_x, const double end_y,
                           int axis_x, int axis_y, int axis_z, int slice_idx, int marker)
{
    double mesh_min_x = coordinate_min[axis_x];
    double mesh_min_y = coordinate_min[axis_y];
    double voxel_length = voxel_dim;
    int density = grid_density;

    // Pixelize the start/end point.
    int start_pixel_x, start_pixel_y;
    int end_pixel_x, end_pixel_y;

    start_pixel_x = floor((start_x - mesh_min_x) / voxel_length);
    if (start_pixel_x < 0)
        start_pixel_x = 0;
    if (start_pixel_x >= density)
        start_pixel_x = density - 1;

    start_pixel_y = floor((start_y - mesh_min_y) / voxel_length);
    if (start_pixel_y < 0)
        start_pixel_y = 0;
    if (start_pixel_y >= density)
        start_pixel_y = density - 1;

    end_pixel_x = floor((end_x - mesh_min_x) / voxel_length);
    if (end_pixel_x < 0)
        end_pixel_x = 0;
    if (end_pixel_x >= density)
        end_pixel_x = density - 1;

    end_pixel_y = floor((end_y - mesh_min_y) / voxel_length);
    if (end_pixel_y < 0)
        end_pixel_y = 0;
    if (end_pixel_y >= density)
        end_pixel_y = density - 1;

//    cout << start_x << " " << start_y << ", " << start_pixel_x << " " << start_pixel_y << endl;
//    cout << end_x << " " << end_y << ", " << end_pixel_x << " " << end_pixel_y << endl;

    // If only one pixel, record it and return.
    int vox_idx[3];
    if (end_pixel_x == start_pixel_x && end_pixel_y == start_pixel_y)
    {
        vox_idx[axis_z] = slice_idx;
        vox_idx[axis_x] = start_pixel_x;
        vox_idx[axis_y] = start_pixel_y;
        markPoint(vox_idx[0] + vox_idx[1]*grid_density + vox_idx[2]*grid_density*(long long)grid_density, marker);
        return;
    }

    vox_idx[0] = 1;
    vox_idx[1] = grid_density;
    vox_idx[2] = grid_density*grid_density;

    // Compute l0, the distance between start point and 3 corresponding facets.
    int delta_x, delta_y;
    double t0_x, t0_y;
    long long inc_x, inc_y;

    if (start_pixel_x <= end_pixel_x)
    {
        delta_x = 1;
        t0_x = voxel_length * (start_pixel_x + 1) - (start_x - mesh_min_x);
        inc_x = vox_idx[axis_x];
    }
    else
    {
        delta_x = -1;
        t0_x = (start_x - mesh_min_x) - voxel_length * start_pixel_x;
        inc_x = -vox_idx[axis_x];
    }

    if (start_pixel_y <= end_pixel_y)
    {
        delta_y = 1;
        t0_y = voxel_length * (start_pixel_y + 1) - (start_y - mesh_min_y);
        inc_y = vox_idx[axis_y];
    }
    else
    {
        delta_y = -1;
        t0_y = (start_y - mesh_min_y) - voxel_length * start_pixel_y;
        inc_y = -vox_idx[axis_y];
    }

    // Compute the step distance parameters.
    double t_x, t_y;
    double step_x, step_y;

    if (start_pixel_x != end_pixel_x)
    {
        double v_x = abs(end_x - start_x);
        t_x = t0_x / v_x;
        step_x = voxel_length / v_x;
    }
    else
    {
        t_x = 1e20;
        step_x = 0;
    }

    if (start_pixel_y != end_pixel_y)
    {
        double v_y = abs(end_y - start_y);
        t_y = t0_y / v_y;
        step_y = voxel_length / v_y;
    }
    else
    {
        t_y = 1e20;
        step_y = 0;
    }

    // The core loop.
    int moving_pixel_x = start_pixel_x;
    int moving_pixel_y = start_pixel_y;

    vox_idx[axis_x] = moving_pixel_x;
    vox_idx[axis_y] = moving_pixel_y;
    vox_idx[axis_z] = slice_idx;
    long long moving_vox_idx = vox_idx[0] + vox_idx[1]*grid_density +
                               vox_idx[2]*grid_density*(long long)grid_density;

    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        markPoint(moving_vox_idx, marker);

        if (t_x <= t_y)
        {
            t_y = t_y - t_x;
            t_x = step_x;

//            if (moving_pixel_x != end_pixel_x)
//            {
                moving_pixel_x += delta_x;
                moving_vox_idx += inc_x;
//            }
        }
        else
        {
            t_x = t_x - t_y;
            t_y = step_y;
//            if (moving_pixel_y != end_pixel_y)
//            {
                moving_pixel_y += delta_y;
                moving_vox_idx += inc_y;
//            }
        }
    }
    markPoint(moving_vox_idx, marker);
}

void VoxelMesh::markTriangle2DXY(const double p0[], const double p1[], const double p2[], const int zi)
{
    // mark the three edges
    markLine2DXY(p0, p1, zi);
    markLine2DXY(p0, p2, zi);
    markLine2DXY(p1, p2, zi);

    // compute the three edge vectors p0p1, p0p2 and p1p2, and their length
    double p0p1[2], p0p2[2], p1p2[2];
    p0p1[0] = p1[0]-p0[0];
    p0p1[1] = p1[1]-p0[1];
    p0p2[0] = p2[0]-p0[0];
    p0p2[1] = p2[1]-p0[1];
    p1p2[0] = p2[0]-p1[0];
    p1p2[1] = p2[1]-p1[1];

    // compute the longest possible step distance
    // along the norm direction of p1p2
    double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
    double d_hat = voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

    // compute the step distance along p0p1 corresponding to d_hat
    double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
    double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
    double step01[2];
    step01[0] = d01 * p0p1[0] / len01;
    step01[1] = d01 * p0p1[1] / len01;

    // compute the step distance along p0p2 corresponding to d_hat
    double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
    double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
    double step02[2];
    step02[0] = d02 * p0p2[0] / len02;
    step02[1] = d02 * p0p2[1] / len02;

    // two moving points will move along p0p1 and p0p2
    double moving_p0p1[2], moving_p0p2[2];
    moving_p0p1[0] = p0[0] + step01[0];
    moving_p0p1[1] = p0[1] + step01[1];
    moving_p0p2[0] = p0[0] + step02[0];
    moving_p0p2[1] = p0[1] + step02[1];

    // iterate and voxelize all the scanlines
    int size_steps = len01 / d01;
    for (int i=0; i<size_steps; ++i)
    {
        markLine2DXY(moving_p0p1, moving_p0p2, zi);
    }
}

void VoxelMesh::markTriangle2DYZ(const double p0[], const double p1[], const double p2[], const int xi)
{
    // mark the three edges
    markLine2DYZ(p0, p1, xi);
    markLine2DYZ(p0, p2, xi);
    markLine2DYZ(p1, p2, xi);

    // compute the three edge vectors p0p1, p0p2 and p1p2, and their length
    double p0p1[2], p0p2[2], p1p2[2];
    p0p1[0] = p1[0]-p0[0];
    p0p1[1] = p1[1]-p0[1];
    p0p2[0] = p2[0]-p0[0];
    p0p2[1] = p2[1]-p0[1];
    p1p2[0] = p2[0]-p1[0];
    p1p2[1] = p2[1]-p1[1];

    // compute the longest possible step distance
    // along the norm direction of p1p2
    double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
    double d_hat = voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

    // compute the step distance along p0p1 corresponding to d_hat
    double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
    double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
    double step01[2];
    step01[0] = d01 * p0p1[0] / len01;
    step01[1] = d01 * p0p1[1] / len01;

    // compute the step distance along p0p2 corresponding to d_hat
    double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
    double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
    double step02[2];
    step02[0] = d02 * p0p2[0] / len02;
    step02[1] = d02 * p0p2[1] / len02;

    // two moving points will move along p0p1 and p0p2
    double moving_p0p1[2], moving_p0p2[2];
    moving_p0p1[0] = p0[0] + step01[0];
    moving_p0p1[1] = p0[1] + step01[1];
    moving_p0p2[0] = p0[0] + step02[0];
    moving_p0p2[1] = p0[1] + step02[1];

    // iterate and voxelize all the scanlines
    int size_steps = len01 / d01;
    for (int i=0; i<size_steps; ++i)
    {
        markLine2DYZ(moving_p0p1, moving_p0p2, xi);
    }
}

void VoxelMesh::markTriangle2DZX(const double p0[], const double p1[], const double p2[], const int yi)
{
    // mark the three edges
    markLine2DZX(p0, p1, yi);
    markLine2DZX(p0, p2, yi);
    markLine2DZX(p1, p2, yi);

    // compute the three edge vectors p0p1, p0p2 and p1p2, and their length
    double p0p1[2], p0p2[2], p1p2[2];
    p0p1[0] = p1[0]-p0[0];
    p0p1[1] = p1[1]-p0[1];
    p0p2[0] = p2[0]-p0[0];
    p0p2[1] = p2[1]-p0[1];
    p1p2[0] = p2[0]-p1[0];
    p1p2[1] = p2[1]-p1[1];

    // compute the longest possible step distance
    // along the norm direction of p1p2
    double len12 = sqrt(p1p2[0]*p1p2[0] + p1p2[1]*p1p2[1]);
    double d_hat = voxel_dim * (abs(p1p2[0])+abs(p1p2[1])) / len12;

    // compute the step distance along p0p1 corresponding to d_hat
    double len01 = sqrt(p0p1[0]*p0p1[0] + p0p1[1]*p0p1[1]);
    double d01 = d_hat * len01 * len12 / abs(p0p1[1]*p1p2[0] - p0p1[0]*p1p2[1]);
    double step01[2];
    step01[0] = d01 * p0p1[0] / len01;
    step01[1] = d01 * p0p1[1] / len01;

    // compute the step distance along p0p2 corresponding to d_hat
    double len02 = sqrt(p0p2[0]*p0p2[0] + p0p2[1]*p0p2[1]);
    double d02 = d_hat * len02 * len12 / abs(p0p2[1]*p1p2[0] - p0p2[0]*p1p2[1]);
    double step02[2];
    step02[0] = d02 * p0p2[0] / len02;
    step02[1] = d02 * p0p2[1] / len02;

    // two moving points will move along p0p1 and p0p2
    double moving_p0p1[2], moving_p0p2[2];
    moving_p0p1[0] = p0[0] + step01[0];
    moving_p0p1[1] = p0[1] + step01[1];
    moving_p0p2[0] = p0[0] + step02[0];
    moving_p0p2[1] = p0[1] + step02[1];

    // iterate and voxelize all the scanlines
    int size_steps = len01 / d01;
    for (int i=0; i<size_steps; ++i)
    {
        markLine2DZX(moving_p0p1, moving_p0p2, yi);
    }
}

void VoxelMesh::markLine2DXY(const double start_point[], const double end_point[], const int zi)
{
    double mesh_min_0 = coordinate_min[0];
    double mesh_min_1 = coordinate_min[1];
    double voxel_length = voxel_dim;
    int d = grid_density;
    long long d2 = d*d;

    // Pixelize the start/end point.
    int start_pixel_0, start_pixel_1;
    int end_pixel_0, end_pixel_1;
    start_pixel_0 = floor((start_point[0] - mesh_min_0) / voxel_length);
    if (start_pixel_0 < 0)
        start_pixel_0 = 0;
    if (start_pixel_0 >= d)
        start_pixel_0 = d - 1;
    start_pixel_1 = floor((start_point[1] - mesh_min_1) / voxel_length);
    if (start_pixel_1 < 0)
        start_pixel_1 = 0;
    if (start_pixel_1 >= d)
        start_pixel_1 = d - 1;
    end_pixel_0 = floor((end_point[0] - mesh_min_0) / voxel_length);
    if (end_pixel_0 < 0)
        end_pixel_0 = 0;
    if (end_pixel_0 >= d)
        end_pixel_0 = d - 1;
    end_pixel_1 = floor((end_point[1] - mesh_min_1) / voxel_length);
    if (end_pixel_1 < 0)
        end_pixel_1 = 0;
    if (end_pixel_1 >= d)
        end_pixel_1 = d - 1;

    // Compute t0_0 and t0_1, the distance between start point and 2 corresponding facets.
    int delta_0, delta_1;
    double t0_0, t0_1;
    if (start_pixel_0 <= end_pixel_0)
    {
        delta_0 = 1;
        t0_0 = voxel_length * (start_pixel_0 + 1) - (start_point[0] - mesh_min_0);
    }
    else
    {
        delta_0 = -1;
        t0_0 = (start_point[0] - mesh_min_0) - voxel_length * start_pixel_0;
    }
    if (start_pixel_1 <= end_pixel_1)
    {
        delta_1 = 1;
        t0_1 = voxel_length * (start_pixel_1 + 1) - (start_point[1] - mesh_min_1);
    }
    else
    {
        delta_1 = -1;
        t0_1 = (start_point[1] - mesh_min_1) - voxel_length * start_pixel_1;
    }

    // Compute the step distance parameters.
    double t_0, t_1;
    double step_0, step_1;
    if (start_pixel_0 != end_pixel_0)
    {
        double v_0 = abs(end_point[0] - start_point[0]);
        t_0 = t0_0 / v_0;
        step_0 = voxel_length / v_0;
    }
    else
    {
        t_0 = 1e20;
        step_0 = 0;
    }
    if (start_pixel_1 != end_pixel_1)
    {
        double v_1 = abs(end_point[1] - start_point[1]);
        t_1 = t0_1 / v_1;
        step_1 = voxel_length / v_1;
    }
    else
    {
        t_1 = 1e20;
        step_1 = 0;
    }

    // The core loop.
    int moving_pixel_0 = start_pixel_0;
    int moving_pixel_1 = start_pixel_1;
    int zzi = zi * d2; // pre-compte to save in the loop
    while (!(moving_pixel_0 == end_pixel_0 && moving_pixel_1 == end_pixel_1))
    {
        markPoint(moving_pixel_0 + moving_pixel_1*d + zzi, 1);

        if (t_0 <= t_1)
        {
            t_1 = t_1 - t_0;
            t_0 = step_0;

            if (moving_pixel_0 != end_pixel_0)
            {
                moving_pixel_0 += delta_0;
            }
        }
        else
        {
            t_0 = t_0 - t_1;
            t_1 = step_1;
            if (moving_pixel_1 != end_pixel_1)
            {
                moving_pixel_1 += delta_1;
            }
        }
    }
    markPoint(moving_pixel_0 + moving_pixel_1*d + zzi, 1);
}

void VoxelMesh::markLine2DYZ(const double start_point[], const double end_point[], const int xi)
{
    double mesh_min_0 = coordinate_min[1];
    double mesh_min_1 = coordinate_min[2];
    double voxel_length = voxel_dim;
    int d = grid_density;
    long long d2 = d*d;

    cout << "mesh_min = "
         << mesh_min_0 << " " << mesh_min_1 << " voxel_dim = " << voxel_dim << endl;

    // Pixelize the start/end point.
    int start_pixel_0, start_pixel_1;
    int end_pixel_0, end_pixel_1;
    start_pixel_0 = floor((start_point[0] - mesh_min_0) / voxel_length);
    if (start_pixel_0 < 0)
        start_pixel_0 = 0;
    if (start_pixel_0 >= d)
        start_pixel_0 = d - 1;
    start_pixel_1 = floor((start_point[1] - mesh_min_1) / voxel_length);
    if (start_pixel_1 < 0)
        start_pixel_1 = 0;
    if (start_pixel_1 >= d)
        start_pixel_1 = d - 1;
    end_pixel_0 = floor((end_point[0] - mesh_min_0) / voxel_length);
    if (end_pixel_0 < 0)
        end_pixel_0 = 0;
    if (end_pixel_0 >= d)
        end_pixel_0 = d - 1;
    end_pixel_1 = floor((end_point[1] - mesh_min_1) / voxel_length);
    if (end_pixel_1 < 0)
        end_pixel_1 = 0;
    if (end_pixel_1 >= d)
        end_pixel_1 = d - 1;

    // Compute t0_0 and t0_1, the distance between start point and 2 corresponding facets.
    int delta_0, delta_1;
    double t0_0, t0_1;
    if (start_pixel_0 <= end_pixel_0)
    {
        delta_0 = 1;
        t0_0 = voxel_length * (start_pixel_0 + 1) - (start_point[0] - mesh_min_0);
    }
    else
    {
        delta_0 = -1;
        t0_0 = (start_point[0] - mesh_min_0) - voxel_length * start_pixel_0;
    }
    if (start_pixel_1 <= end_pixel_1)
    {
        delta_1 = 1;
        t0_1 = voxel_length * (start_pixel_1 + 1) - (start_point[1] - mesh_min_1);
    }
    else
    {
        delta_1 = -1;
        t0_1 = (start_point[1] - mesh_min_1) - voxel_length * start_pixel_1;
    }

    // Compute the step distance parameters.
    double t_0, t_1;
    double step_0, step_1;
    if (start_pixel_0 != end_pixel_0)
    {
        double v_0 = abs(end_point[0] - start_point[0]);
        t_0 = t0_0 / v_0;
        step_0 = voxel_length / v_0;
    }
    else
    {
        t_0 = 1e20;
        step_0 = 0;
    }
    if (start_pixel_1 != end_pixel_1)
    {
        double v_1 = abs(end_point[1] - start_point[1]);
        t_1 = t0_1 / v_1;
        step_1 = voxel_length / v_1;
    }
    else
    {
        t_1 = 1e20;
        step_1 = 0;
    }

    cout << "mark 2d line yz "
         << start_point[0] << " " << start_point[1] << " "
         << end_point[0] << " " << end_point[1] << " "
         << start_pixel_0 << " " << start_pixel_1 << " "
         << end_pixel_0 << " " << end_pixel_1 << endl;

    // The core loop.
    int moving_pixel_0 = start_pixel_0;
    int moving_pixel_1 = start_pixel_1;
    while (!(moving_pixel_0 == end_pixel_0 && moving_pixel_1 == end_pixel_1))
    {
        markPoint(xi + moving_pixel_0*d + moving_pixel_1*d2, 1);

        if (t_0 <= t_1)
        {
            t_1 = t_1 - t_0;
            t_0 = step_0;

            if (moving_pixel_0 != end_pixel_0)
            {
                moving_pixel_0 += delta_0;
            }
        }
        else
        {
            t_0 = t_0 - t_1;
            t_1 = step_1;
            if (moving_pixel_1 != end_pixel_1)
            {
                moving_pixel_1 += delta_1;
            }
        }
    }
    markPoint(xi + moving_pixel_0*d + moving_pixel_1*d2, 1);
}

void VoxelMesh::markLine2DZX(const double start_point[], const double end_point[], const int yi)
{
    double mesh_min_0 = coordinate_min[2];
    double mesh_min_1 = coordinate_min[0];
    double voxel_length = voxel_dim;
    int d = grid_density;
    long long d2 = d*d;

    // Pixelize the start/end point.
    int start_pixel_0, start_pixel_1;
    int end_pixel_0, end_pixel_1;
    start_pixel_0 = floor((start_point[0] - mesh_min_0) / voxel_length);
    if (start_pixel_0 < 0)
        start_pixel_0 = 0;
    if (start_pixel_0 >= d)
        start_pixel_0 = d - 1;
    start_pixel_1 = floor((start_point[1] - mesh_min_1) / voxel_length);
    if (start_pixel_1 < 0)
        start_pixel_1 = 0;
    if (start_pixel_1 >= d)
        start_pixel_1 = d - 1;
    end_pixel_0 = floor((end_point[0] - mesh_min_0) / voxel_length);
    if (end_pixel_0 < 0)
        end_pixel_0 = 0;
    if (end_pixel_0 >= d)
        end_pixel_0 = d - 1;
    end_pixel_1 = floor((end_point[1] - mesh_min_1) / voxel_length);
    if (end_pixel_1 < 0)
        end_pixel_1 = 0;
    if (end_pixel_1 >= d)
        end_pixel_1 = d - 1;

    // Compute t0_0 and t0_1, the distance between start point and 2 corresponding facets.
    int delta_0, delta_1;
    double t0_0, t0_1;
    if (start_pixel_0 <= end_pixel_0)
    {
        delta_0 = 1;
        t0_0 = voxel_length * (start_pixel_0 + 1) - (start_point[0] - mesh_min_0);
    }
    else
    {
        delta_0 = -1;
        t0_0 = (start_point[0] - mesh_min_0) - voxel_length * start_pixel_0;
    }
    if (start_pixel_1 <= end_pixel_1)
    {
        delta_1 = 1;
        t0_1 = voxel_length * (start_pixel_1 + 1) - (start_point[1] - mesh_min_1);
    }
    else
    {
        delta_1 = -1;
        t0_1 = (start_point[1] - mesh_min_1) - voxel_length * start_pixel_1;
    }

    // Compute the step distance parameters.
    double t_0, t_1;
    double step_0, step_1;
    if (start_pixel_0 != end_pixel_0)
    {
        double v_0 = abs(end_point[0] - start_point[0]);
        t_0 = t0_0 / v_0;
        step_0 = voxel_length / v_0;
    }
    else
    {
        t_0 = 1e20;
        step_0 = 0;
    }
    if (start_pixel_1 != end_pixel_1)
    {
        double v_1 = abs(end_point[1] - start_point[1]);
        t_1 = t0_1 / v_1;
        step_1 = voxel_length / v_1;
    }
    else
    {
        t_1 = 1e20;
        step_1 = 0;
    }

    // The core loop.
    int moving_pixel_0 = start_pixel_0;
    int moving_pixel_1 = start_pixel_1;
    int yyi = yi * d;
    while (!(moving_pixel_0 == end_pixel_0 && moving_pixel_1 == end_pixel_1))
    {
        markPoint(moving_pixel_1 + yyi + moving_pixel_0*d2, 1);

        if (t_0 <= t_1)
        {
            t_1 = t_1 - t_0;
            t_0 = step_0;

            if (moving_pixel_0 != end_pixel_0)
            {
                moving_pixel_0 += delta_0;
            }
        }
        else
        {
            t_0 = t_0 - t_1;
            t_1 = step_1;
            if (moving_pixel_1 != end_pixel_1)
            {
                moving_pixel_1 += delta_1;
            }
        }
    }
    markPoint(moving_pixel_1 + yyi + moving_pixel_0*d2, 1);
}

//========================================================================================
// This function marks a triangle using floating scanlines.
//========================================================================================
void VoxelMesh::markFaceByScanline1D(TriangleFace* facep, int marker)
{
    //Get the 3 vetex of the face.
    double p0[3], p1[3], p2[3];
    p0[0] = facep->node0->ori_coordinate[0];
    p0[1] = facep->node0->ori_coordinate[1];
    p0[2] = facep->node0->ori_coordinate[2];
    p1[0] = facep->node1->ori_coordinate[0];
    p1[1] = facep->node1->ori_coordinate[1];
    p1[2] = facep->node1->ori_coordinate[2];
    p2[0] = facep->node2->ori_coordinate[0];
    p2[1] = facep->node2->ori_coordinate[1];
    p2[2] = facep->node2->ori_coordinate[2];

    // Compute the normal of the triangle plane, i.e. axis_z.
    int axis_z = computeDomiAxis(p0, p1, p2);
    int axis_x, axis_y;
    if (axis_z == 0)
    {
        axis_x = 1;
        axis_y = 2;
    }
    else if (axis_z == 1)
    {
        axis_x = 0;
        axis_y = 2;
    }
    else
    {
        axis_x = 0;
        axis_y = 1;
    }

    // Sort the three voxels by order of dominant component.
    sortVertByDomiAxis(p0, p1, p2, axis_z);

    // voxelize the 3 vertices.
    int p0_vox[3], p1_vox[3], p2_vox[3];
    computeVoxIdx(p0_vox, p0);
    computeVoxIdx(p1_vox, p1);
    computeVoxIdx(p2_vox, p2);

    // If the case is simple, no complicate computation is needed.
    if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
        p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
    {
        markPoint(p0_vox[0] + p0_vox[1]*grid_density + p0_vox[2]*grid_density*(long long)grid_density, marker);
        return;
    }

    //-----------------------------------------------------------------------------
    // If only one slice, easily mark and return.
    //-----------------------------------------------------------------------------
    if (p0_vox[axis_z] == p2_vox[axis_z])
    {
        // the triangle is within one xy slice.
        markSliceTriangle(p0[axis_x], p0[axis_y], p1[axis_x], p1[axis_y], p2[axis_x], p2[axis_y],
                          axis_x, axis_y, axis_z, p0_vox[axis_z], marker);
        return;
    }

    //-----------------------------------------------------------------------------
    // Mark the three edges.
    //-----------------------------------------------------------------------------
//    mark3DLine(p0, p2, p0_vox, p2_vox, marker);
//    mark3DLine(p0, p1, p0_vox, p1_vox, marker);
//    mark3DLine(p1, p2, p1_vox, p2_vox, marker);

    //-----------------------------------------------------------------------------
    // Determine the line segments.
    //-----------------------------------------------------------------------------
    // Determine the 3D line segments of p0p2.
    double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
    double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
    double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];
    double t1;

    // Determine the 3D line segments of p0p1.
    double a2 = p1[axis_x]-p0[axis_x], b2 = p0[axis_x];
    double c2 = p1[axis_y]-p0[axis_y], d2 = p0[axis_y];
    double e2 = p1[axis_z]-p0[axis_z], f2 = p0[axis_z];
    double t2;

    // If the three lines are colinear, return.
//    if (a1 != 0.0)
//    {
//        t1 = (p1[axis_x] - b1)/a1;
//        if (c1*t1+d1 == p1[axis_y] && e1*t1+f1 == p1[axis_z])
//        {
//            cout << "Bad face detected!\n";
//            return;
//        }
//    }
//    else if (c1 != 0.0)
//    {
//        t1 = (p1[axis_y] - d1)/c1;
//        if (a1*t1+b1 == p1[axis_x] && e1*t1+f1 == p1[axis_z])
//        {
//            cout << "Bad face detected!\n";
//            return;
//        }
//    }
//    else if (e1 != 0.0)
//    {
//        t1 = (p1[axis_x] - f1)/e1;
//        if (c1*t1+d1 == p1[axis_y] && a1*t1+b1 == p1[axis_x])
//        {
//            cout << "Bad face detected!\n";
//            return;
//        }
//    }

    //-----------------------------------------------------------------------------
    // Voxelize the first half of the triangle.
    //-----------------------------------------------------------------------------
    // Determine the first slice.
    int slice_idx = p0_vox[axis_z];
    double plane = voxel_dim * (p0_vox[axis_z] + 1) + coordinate_min[axis_z];
    if (plane == p0[axis_z])
    {
        // the plane happened to intersect p0,
        // refine the slice index and move to next plane.
        slice_idx = computeVoxOneIdx(p0[axis_z]+0.5*voxel_dim, axis_z);

        plane += voxel_dim;
    }

    double startp0_x, startp0_y, startp1_x, startp1_y;
    startp0_x = p0[axis_x];
    startp0_y = p0[axis_y];
    startp1_x = startp0_x;
    startp1_y = startp0_y;

    double endp0_x, endp0_y, endp1_x, endp1_y;
    while (plane < p1[axis_z])
    {
        t1 = (plane - f1) / e1;
        endp0_x = a1 * t1 + b1;
        endp0_y = c1 * t1 + d1;

        t2 = (plane - f2) / e2;
        endp1_x = a2 * t2 + b2;
        endp1_y = c2 * t2 + d2;

        markSliceTriangle(startp0_x, startp0_y, startp1_x, startp1_y, endp1_x, endp1_y,
                          axis_x, axis_y, axis_z, slice_idx, marker);
        markSliceTriangle(startp0_x, startp0_y, endp0_x, endp0_y, endp1_x, endp1_y,
                          axis_x, axis_y, axis_z, slice_idx, marker);

        startp0_x = endp0_x;
        startp0_y = endp0_y;
        startp1_x = endp1_x;
        startp1_y = endp1_y;

        plane += voxel_dim;
        slice_idx++;
    }

    //-----------------------------------------------------------------------------
    // Voxelize the slice containing the typical scanline passing p1.
    //-----------------------------------------------------------------------------
    t1 = (p1[axis_z] - f1) / e1;
    endp0_x = a1 * t1 + b1;
    endp0_y = c1 * t1 + d1;

    endp1_x = p1[axis_x];
    endp1_y = p1[axis_y];

    markSliceTriangle(startp0_x, startp0_y, startp1_x, startp1_y, endp1_x, endp1_y,
                      axis_x, axis_y, axis_z, slice_idx, marker);
    markSliceTriangle(startp0_x, startp0_y, endp0_x, endp0_y, endp1_x, endp1_y,
                      axis_x, axis_y, axis_z, slice_idx, marker);

    startp0_x = endp0_x;
    startp0_y = endp0_y;
    startp1_x = endp1_x;
    startp1_y = endp1_y;

    // check if the plane happened to pass p1
    if (plane == p1[axis_z])
    {
        // the plane happened to intersect p1,
        // refine the slice index and move to next plane.
        slice_idx = computeVoxOneIdx(p1[axis_z]+0.5*voxel_dim, axis_z);

        plane += voxel_dim;
    }

    //-----------------------------------------------------------------------------
    // voxelize the second half of the triangle.
    //-----------------------------------------------------------------------------
    // Determine the 3D line segments of p1p2.
    a2 = p2[axis_x]-p1[axis_x]; b2 = p1[axis_x];
    c2 = p2[axis_y]-p1[axis_y]; d2 = p1[axis_y];
    e2 = p2[axis_z]-p1[axis_z]; f2 = p1[axis_z];

    while (plane < p2[axis_z])
    {
        t1 = (plane - f1) / e1;
        endp0_x = a1 * t1 + b1;
        endp0_y = c1 * t1 + d1;

        t2 = (plane - f2) / e2;
        endp1_x = a2 * t2 + b2;
        endp1_y = c2 * t2 + d2;

        markSliceTriangle(startp0_x, startp0_y, startp1_x, startp1_y, endp1_x, endp1_y,
                          axis_x, axis_y, axis_z, slice_idx, marker);
        markSliceTriangle(startp0_x, startp0_y, endp0_x, endp0_y, endp1_x, endp1_y,
                          axis_x, axis_y, axis_z, slice_idx, marker);

        startp0_x = endp0_x;
        startp0_y = endp0_y;
        startp1_x = endp1_x;
        startp1_y = endp1_y;

        plane += voxel_dim;
        slice_idx++;
    }

    // voxelize the last slice.
    endp0_x = p2[axis_x];
    endp0_y = p2[axis_y];

    markSliceTriangle(startp0_x, startp0_y, startp1_x, startp1_y, endp1_x, endp1_y,
                      axis_x, axis_y, axis_z, slice_idx, marker);
    markSliceTriangle(startp0_x, startp0_y, endp0_x, endp0_y, endp1_x, endp1_y,
                      axis_x, axis_y, axis_z, slice_idx, marker);
}

void VoxelMesh::markSliceTrapezoid(const double pp0_x, const double pp0_y,
                                   const double pp1_x, const double pp1_y,
                                   const double pp2_x, const double pp2_y,
                                   const double pp3_x, const double pp3_y,
                                   int axis_x, int axis_y, int axis_z, int slice_idx, int marker)
{
    double pp[4][3];
    int sequ_pix_1[LONGEST_EDGE][3], sequ_pix_2[LONGEST_EDGE][3];
    int size_1, size_2;

        // This is a trapezoid.
        pp[0][axis_x] = pp0_x; pp[0][axis_y] = pp0_y;
        pp[1][axis_x] = pp1_x; pp[1][axis_y] = pp1_y;
        pp[2][axis_x] = pp2_x; pp[2][axis_y] = pp2_y;
        pp[3][axis_x] = pp3_x; pp[3][axis_y] = pp3_y;

        // sort the four points by axis_y value such that 0<=1<=2<=3.
        for (int i=0; i<4; ++i)
        {
            for (int j=i+1; j<4; ++j)
            {
                if (pp[i][axis_y] > pp[j][axis_y])
                {
                    // switch them
                    double temp = pp[i][axis_x];
                    pp[i][axis_x] = pp[j][axis_x];
                    pp[j][axis_x] = temp;

                    temp = pp[i][axis_y];
                    pp[i][axis_y] = pp[j][axis_y];
                    pp[j][axis_y] = temp;
                }
            }
        }
        // switch p2 and p3
        double temp = pp[2][axis_x];
        pp[2][axis_x] = pp[3][axis_x];
        pp[3][axis_x] = temp;

        temp = pp[2][axis_y];
        pp[2][axis_y] = pp[3][axis_y];
        pp[3][axis_y] = temp;

        // get the two pixel sequences, i.e. p0->p1->p2 and p0->p3->p2.
//        size_1 = mark2DScanline(pp[0], pp[1], sequ_pix_1, 0,
//                                axis_x, axis_y, axis_z, slice_idx, marker);
//        cout << "size 1 ==== " << size_1 << " " << sequ_pix_1[size_1-1][axis_y] << endl;

//        size_1 = mark2DScanline(pp[1], pp[2], sequ_pix_1, size_1 - 1,
//                                axis_x, axis_y, axis_z, slice_idx, marker);
//        cout << "size 1 ==== " << size_1 << " " << sequ_pix_1[size_1-1][axis_y] << endl;

//        size_2 = mark2DScanline(pp[0], pp[3], sequ_pix_2, 0,
//                                axis_x, axis_y, axis_z, slice_idx, marker);
//        cout << "size 2 ==== " << size_2 << " " << sequ_pix_2[size_2-1][axis_y] << endl;
//        size_2 = mark2DScanline(pp[3], pp[2], sequ_pix_2, size_2 - 1,
//                                axis_x, axis_y, axis_z, slice_idx, marker);
//        cout << "size 2 ==== " << size_2 << " " << sequ_pix_2[size_2-1][axis_y] << endl;

//cout << "size 1 2 = " << size_1 << " " << size_2 << " "
//     << sequ_pix_1[size_1-1][axis_y] << " " << sequ_pix_2[size_2-1][axis_y] << endl;

    // sort p1 and p2 by axis_x value such that 1<=2.
//    double aa = pp[3][axis_x]-pp[0][axis_x], bb = pp[0][axis_x];
//    double cc = pp[3][axis_y]-pp[0][axis_y], dd = pp[0][axis_y];
//    if (cc != 0.0)
//    {
//        if (aa * (pp[1][axis_y] - dd) / cc + bb < pp[1][axis_x])
//        {
//            // switch pp1 and pp2
//            double temp = pp[1][0];
//            pp[1][0] = pp[2][0];
//            pp[2][0] = temp;

//            temp = pp[1][1];
//            pp[1][1] = pp[2][1];
//            pp[2][1] = temp;

//            temp = pp[1][2];
//            pp[1][2] = pp[2][2];
//            pp[2][2] = temp;
//        }
//    }
//    else
//    {
//        if (pp[2][axis_x] < pp[1][axis_x])
//        {
//            // switch pp1 and pp2
//            double temp = pp[1][0];
//            pp[1][0] = pp[2][0];
//            pp[2][0] = temp;

//            temp = pp[1][1];
//            pp[1][1] = pp[2][1];
//            pp[2][1] = temp;

//            temp = pp[1][2];
//            pp[1][2] = pp[2][2];
//            pp[2][2] = temp;
//        }
//    }

    //-----------------------------------------------------------------------------
    // voxelize the slice per 1D stripe along axis_y.
    //-----------------------------------------------------------------------------
    int start_stripe = computeVoxOneIdx(pp[0][axis_y], axis_y);
    int end_stripe = computeVoxOneIdx(pp[2][axis_y], axis_y);

//cout << "slice " << slice_idx << " " << start_stripe << " " << end_stripe << endl;

    int voxidx[3];
    voxidx[axis_z] = slice_idx;

    int i_1 = 0, i_2 = 0;
    for (int stripe_i=start_stripe; stripe_i<=end_stripe; ++stripe_i)
    {

        voxidx[axis_y] = stripe_i;

        while (i_1 < size_1 && sequ_pix_1[i_1][axis_y] == stripe_i)
        {
            i_1++;
        }

        while (i_2 < size_2 && sequ_pix_2[i_2][axis_y] == stripe_i)
        {
            i_2++;
        }

        int s_x = sequ_pix_1[i_1 - 1][axis_x];
        int e_x = sequ_pix_2[i_2 - 1][axis_x];

//cout << " here is slice 7 = " << s_x << " " << e_x  << " - " << sequ_pix_1[i_1 - 1][axis_y] << " " << sequ_pix_2[i_2 - 1][axis_y] << endl;

        // voxelize the stripe.
        if (s_x <= e_x)
        {
            for (voxidx[axis_x]=s_x+1; voxidx[axis_x]<e_x; ++voxidx[axis_x])
            {
                markPoint(voxidx[0] + voxidx[1]*grid_density + voxidx[2]*grid_density*(long long)grid_density, marker);
            }
        }
        else
        {
            for (voxidx[axis_x]=s_x-1; voxidx[axis_x]>e_x; --voxidx[axis_x])
            {
                markPoint(voxidx[0] + voxidx[1]*grid_density + voxidx[2]*grid_density*(long long)grid_density, marker);
            }
        }
    }
}

void VoxelMesh::markSliceTriangle(const double pp0_x, const double pp0_y,
                                  const double pp1_x, const double pp1_y,
                                  const double pp2_x, const double pp2_y,
                                  int axis_x, int axis_y, int axis_z, int slice_idx, int marker)
{
    double pp[3][3];
    pp[0][axis_x] = pp0_x; pp[0][axis_y] = pp0_y;
    pp[1][axis_x] = pp1_x; pp[1][axis_y] = pp1_y;
    pp[2][axis_x] = pp2_x; pp[2][axis_y] = pp2_y;

    // sort the three points by axis_y value such that 0<=1<=2.
    for (int i=0; i<3; ++i)
    {
        for (int j=i+1; j<3; ++j)
        {
            if (pp[i][axis_y] > pp[j][axis_y])
            {
                // switch them
                double temp = pp[i][axis_x];
                pp[i][axis_x] = pp[j][axis_x];
                pp[j][axis_x] = temp;

                temp = pp[i][axis_y];
                pp[i][axis_y] = pp[j][axis_y];
                pp[j][axis_y] = temp;
            }
        }
    }

    // get the two pixel sequences, i.e. p0->p2 and p0->p1->p2.
    int sequ_pix_1[LONGEST_EDGE][3], sequ_pix_2[LONGEST_EDGE][3];
    int size_1, size_2;
    size_1 = mark2DLine(pp[0], pp[2], axis_x, axis_y, axis_z, slice_idx, marker,
                        sequ_pix_1, 0);

    size_2 = mark2DLine(pp[0], pp[1], axis_x, axis_y, axis_z, slice_idx, marker,
                        sequ_pix_2, 0);
    size_2 = mark2DLine(pp[1], pp[2], axis_x, axis_y, axis_z, slice_idx, marker,
                        sequ_pix_2, size_2 - 1);

    //-----------------------------------------------------------------------------
    // voxelize the slice per 1D stripe along axis_y.
    //-----------------------------------------------------------------------------
    int start_stripe = computeVoxOneIdx(pp[0][axis_y], axis_y);
    int end_stripe = computeVoxOneIdx(pp[2][axis_y], axis_y);

    int voxidx[3];
    voxidx[axis_z] = slice_idx;

    int i_1 = 0, i_2 = 0;
    for (int stripe_i=start_stripe; stripe_i<=end_stripe; ++stripe_i)
    {
        voxidx[axis_y] = stripe_i;

        while (i_1 < size_1 && sequ_pix_1[i_1][axis_y] == stripe_i)
        {
            i_1++;
        }

        while (i_2 < size_2 && sequ_pix_2[i_2][axis_y] == stripe_i)
        {
            i_2++;
        }

        // voxelize the stripe.
        int s_x = sequ_pix_1[i_1 - 1][axis_x];
        int e_x = sequ_pix_2[i_2 - 1][axis_x];
        //scanline_count++;

        //cout << "stripe_i s_x e_x = " << stripe_i << " " << s_x << " " << e_x << endl;

        if (s_x <= e_x)
        {
            for (voxidx[axis_x]=s_x+1; voxidx[axis_x]<e_x; ++voxidx[axis_x])
            {
                markPoint(voxidx[0] + voxidx[1]*grid_density + voxidx[2]*grid_density*(long long)grid_density, marker);
            }
        }
        else
        {
            for (voxidx[axis_x]=s_x-1; voxidx[axis_x]>e_x; --voxidx[axis_x])
            {
                markPoint(voxidx[0] + voxidx[1]*grid_density + voxidx[2]*grid_density*(long long)grid_density, marker);
            }
        }
    }
}

void VoxelMesh::markFaceByScanline(TriangleFace* facep, int marker)
{
    //Get the 3 vetex of the face.
    double p0[3], p1[3], p2[3];
    p0[0] = facep->node0->ori_coordinate[0];
    p0[1] = facep->node0->ori_coordinate[1];
    p0[2] = facep->node0->ori_coordinate[2];
    p1[0] = facep->node1->ori_coordinate[0];
    p1[1] = facep->node1->ori_coordinate[1];
    p1[2] = facep->node1->ori_coordinate[2];
    p2[0] = facep->node2->ori_coordinate[0];
    p2[1] = facep->node2->ori_coordinate[1];
    p2[2] = facep->node2->ori_coordinate[2];

    // compute the norm, i.e. (p1p0)x(p2p0).
    double v1[3], v2[3];
    v1[0] = p1[0] - p0[0];
    v1[1] = p1[1] - p0[1];
    v1[2] = p1[2] - p0[2];

    v2[0] = p2[0] - p0[0];
    v2[1] = p2[1] - p0[1];
    v2[2] = p2[2] - p0[2];

    double norm[3];
    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

    facep->norm[0] = norm[0];
    facep->norm[1] = norm[1];
    facep->norm[2] = norm[2];

    // take the absolute value of the norm.
    if (norm[0] < 0.0)
        norm[0] = -norm[0];

    if (norm[1] < 0.0)
        norm[1] = -norm[1];

    if (norm[2] < 0.0)
        norm[2] = -norm[2];

    // find the dominant direction.
    int axis_z = 0;
    if (norm[0] < norm[1])
    {
        if (norm[1] < norm[2])  // 0<1<2
        {
            axis_z = 2;
        }
        else // 0<1, 2<=1
        {
            axis_z = 1;
        }
    }
    else if (norm[0] < norm[2]) // 1<=0, 0<2
    {
        axis_z = 2;
    }

    // Sort the three voxels by order of dominant component.
    sortVertByDomiAxis(p0, p1, p2, axis_z);

    // voxelize the 3 vertices.
    int p0_vox[3], p1_vox[3], p2_vox[3];
    computeVoxIdx(p0_vox, p0);
    computeVoxIdx(p1_vox, p1);
    computeVoxIdx(p2_vox, p2);

    // If the case is simple, no complicate computation is needed.
    if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
        p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
    {
        markPoint(p0_vox[0] + p0_vox[1]*grid_density + p0_vox[2]*grid_density*(long long)grid_density, marker);
        return;
    }

    //-----------------------------------------------------------------------------
    // Mark the three edges.
    //-----------------------------------------------------------------------------
    mark3DLine(p0, p2, p0_vox, p2_vox, marker);
    mark3DLine(p0, p1, p0_vox, p1_vox, marker);
    mark3DLine(p1, p2, p1_vox, p2_vox, marker);

    //-----------------------------------------------------------------------------
    // Redefine the x y z coordinates by the domi axis.
    //-----------------------------------------------------------------------------
    int axis_x, axis_y;
    if (axis_z == 0)
    {
        axis_x = 1;
        axis_y = 2;
    }
    else if (axis_z == 1)
    {
        axis_x = 0;
        axis_y = 2;
    }
    else
    {
        axis_x = 0;
        axis_y = 1;
    }

    facep->axis_x = axis_x;
    facep->axis_y = axis_y;
    facep->axis_z = axis_z;

    //-----------------------------------------------------------------------------
    // Determine the three line segments.
    //-----------------------------------------------------------------------------
    // Determine the 3D line segments of p0p2.
    double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
    double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
    double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

    // Determine the 3D line segments of p0p1.
    double a2 = p1[axis_x]-p0[axis_x], b2 = p0[axis_x];
    double c2 = p1[axis_y]-p0[axis_y], d2 = p0[axis_y];
    double e2 = p1[axis_z]-p0[axis_z], f2 = p0[axis_z];

    // If the three lines are colinear, return.
//    if (a1 != 0.0)
//    {
//        t1 = (p1[axis_x] - b1)/a1;
//        if (c1*t1+d1 == p1[axis_y] && e1*t1+f1 == p1[axis_z])
//        {
//            cout << "Bad face detected!\n";
//            return;
//        }
//    }
//    else if (c1 != 0.0)
//    {
//        t1 = (p1[axis_y] - d1)/c1;
//        if (a1*t1+b1 == p1[axis_x] && e1*t1+f1 == p1[axis_z])
//        {
//            cout << "Bad face detected!\n";
//            return;
//        }
//    }
//    else if (e1 != 0.0)
//    {
//        t1 = (p1[axis_x] - f1)/e1;
//        if (c1*t1+d1 == p1[axis_y] && a1*t1+b1 == p1[axis_x])
//        {
//            cout << "Bad face detected!\n";
//            return;
//        }
//    }

    //-----------------------------------------------------------------------------
    // Determine the typical scanline passing p1 and pre-compute the delta steps.
    //-----------------------------------------------------------------------------
    double scp0_x, scp0_y, scp1_x, scp1_y;
    scp1_x = p1[axis_x];
    scp1_y = p1[axis_y];

    if (p0_vox[axis_z] == p2_vox[axis_z])
    {
        // the triangle is within one xy slice.
        // Simply let the typical scanline be p0p1.
        scp0_x = p0[axis_x];
        scp0_y = p0[axis_y];
    }
    else
    {
        double t1 = (p1[axis_z] - f1) / e1;
        scp0_x = a1 * t1 + b1;
        scp0_y = c1 * t1 + d1;
    }

    // compute the norm of the scanline.
    double nx = scp0_y - scp1_y;
    double ny = scp1_x - scp0_x;

    // make the norm direction identical to p0p2
    double vx = p2[axis_x]-p0[axis_x];
    double vy = p2[axis_y]-p0[axis_y];
    if (nx * vx + ny * vy < 0.0)
    {
        nx = - nx;
        ny = - ny;
    }

    // normalize the norm.
    double norm_length = sqrt(nx*nx + ny*ny);
    double vox_dim = 0.99*voxel_dim*(abs(nx)+abs(ny)) / norm_length;
    nx = nx / norm_length;
    ny = ny / norm_length;

    // compute the 2D step vectors along p0p2.
    //double vox_dim = 0.99*voxel_dim;
    double step_1_x, step_1_y, step_1_length;
    double temp = vox_dim / (nx*vx + ny*vy);
    step_1_x = vx * temp;
    step_1_y = vy * temp;
    step_1_length = step_1_x*step_1_x + step_1_y*step_1_y;

    // compute the 2D step vectors along p0p1.
    vx = p1[axis_x]-p0[axis_x];
    vy = p1[axis_y]-p0[axis_y];
    double step_2_x, step_2_y;
    temp = vox_dim / (nx*vx + ny*vy);
    step_2_x = vx * temp;
    step_2_y = vy * temp;

    //-----------------------------------------------------------------------------
    // Voxelize the first half of the triangle.
    //-----------------------------------------------------------------------------
    // The array use to record the scanlines.
    facep->sequ_primscanline = new vector<double*>;
    facep->sequ_scanline = new vector<double*>;
    facep->sequ_primscanvoxel = new vector<vector<int*>*>;
    facep->sequ_scanvoxel = new vector<vector<int*>*>;

    // Determine the first slice.
    int slice_idx = p0_vox[axis_z];
    double plane = voxel_dim * (p0_vox[axis_z] + 1) + coordinate_min[axis_z];
    if (plane == p0[axis_z])
    {
        // the plane happened to intersect p0,
        // refine the slice index and move to next plane.
        slice_idx = computeVoxOneIdx(p0[axis_z]+0.5*voxel_dim, axis_z);

        plane += voxel_dim;
    }

    // the first scanline is a point, i.e. p0.
    double startp0_x, startp0_y, startp1_x, startp1_y;
    startp0_x = p0[axis_x];
    startp0_y = p0[axis_y];
    startp1_x = startp0_x;
    startp1_y = startp0_y;

    double endp0_x, endp0_y, endp1_x, endp1_y;
    double movep0_x, movep0_y, movep1_x, movep1_y;
    double length;
    while  (plane < p1[axis_z])
    {
        double t1 = (plane - f1) / e1;
        endp0_x = a1 * t1 + b1;
        endp0_y = c1 * t1 + d1;

        double t2 = (plane - f2) / e2;
        endp1_x = a2 * t2 + b2;
        endp1_y = c2 * t2 + d2;

        movep0_x = startp0_x;
        movep0_y = startp0_y;
        movep1_x = startp1_x;
        movep1_y = startp1_y;
        length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
        while (length > 0.0)
        {
            vector<int*>* p_sequ_voxel = new vector<int*>;
            mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
                       axis_x, axis_y, axis_z, slice_idx, marker,
                       p_sequ_voxel);
            scanline_count++;

            // record the scanlines (optional).
            double* pp = new double[6];
            pp[axis_x] = movep0_x;
            pp[axis_y] = movep0_y;
            pp[axis_z] = coordinate_min[axis_z];
            pp[axis_x+3] = movep1_x;
            pp[axis_y+3] = movep1_y;
            pp[axis_z+3] = coordinate_min[axis_z];
            if (movep0_x == startp0_x && movep0_y == startp0_y)
            {
                // record as the prime scanline.
                facep->sequ_primscanline->push_back(pp);
                facep->sequ_primscanvoxel->push_back(p_sequ_voxel);
            }
            else
            {
                // record the scanline within slice.
                facep->sequ_scanline->push_back(pp);
                facep->sequ_scanvoxel->push_back(p_sequ_voxel);
            }

            movep0_x += step_1_x;
            movep0_y += step_1_y;
            movep1_x += step_2_x;
            movep1_y += step_2_y;

            length -= step_1_length;
        }

        movep0_x = endp0_x;
        movep0_y = endp0_y;
        movep1_x = endp1_x;
        movep1_y = endp1_y;

        vector<int*>* p_sequ_voxel = new vector<int*>;
        mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
                   axis_x, axis_y, axis_z, slice_idx, marker,
                   p_sequ_voxel);
        scanline_count++;

        // record the scanlines (optional).
        double* pp = new double[6];
        pp[axis_x] = movep0_x;
        pp[axis_y] = movep0_y;
        pp[axis_z] = coordinate_min[axis_z];
        pp[axis_x+3] = movep1_x;
        pp[axis_y+3] = movep1_y;
        pp[axis_z+3] = coordinate_min[axis_z];

        // record as the prime scanline.
        facep->sequ_primscanline->push_back(pp);
        facep->sequ_primscanvoxel->push_back(p_sequ_voxel);

        // move to next slice.
        startp0_x = endp0_x;
        startp0_y = endp0_y;
        startp1_x = endp1_x;
        startp1_y = endp1_y;
        plane += voxel_dim;
        slice_idx++;
    }

    //-----------------------------------------------------------------------------
    // Voxelize the slice containing the typical scanline passing p1.
    //-----------------------------------------------------------------------------
    endp0_x = scp0_x;
    endp0_y = scp0_y;
    endp1_x = scp1_x;
    endp1_y = scp1_y;

    movep0_x = startp0_x;
    movep0_y = startp0_y;
    movep1_x = startp1_x;
    movep1_y = startp1_y;

    length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
    while (length > 0.0)
    {
        vector<int*>* p_sequ_voxel = new vector<int*>;
        mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
                   axis_x, axis_y, axis_z, slice_idx, marker,
                   p_sequ_voxel);
        scanline_count++;

        // record the scanlines (optional).
        double* pp = new double[6];
        pp[axis_x] = movep0_x;
        pp[axis_y] = movep0_y;
        pp[axis_z] = coordinate_min[axis_z];
        pp[axis_x+3] = movep1_x;
        pp[axis_y+3] = movep1_y;
        pp[axis_z+3] = coordinate_min[axis_z];
        if (movep0_x == startp0_x && movep0_y == startp0_y)
        {
            // record as the prime scanline.
            facep->sequ_primscanline->push_back(pp);
            facep->sequ_primscanvoxel->push_back(p_sequ_voxel);
        }
        else
        {
            // record the scanline within slice.
            facep->sequ_scanline->push_back(pp);
            facep->sequ_scanvoxel->push_back(p_sequ_voxel);
        }

        movep0_x += step_1_x;
        movep0_y += step_1_y;
        movep1_x += step_2_x;
        movep1_y += step_2_y;

        length -= step_1_length;
    }

    movep0_x = endp0_x;
    movep0_y = endp0_y;
    movep1_x = endp1_x;
    movep1_y = endp1_y;

    vector<int*>* p_sequ_voxel = new vector<int*>;
    mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
               axis_x, axis_y, axis_z, slice_idx, marker,
               p_sequ_voxel);
    scanline_count++;

    // record the scanlines (optional).
    double* pp = new double[6];
    pp[axis_x] = movep0_x;
    pp[axis_y] = movep0_y;
    pp[axis_z] = coordinate_min[axis_z];
    pp[axis_x+3] = movep1_x;
    pp[axis_y+3] = movep1_y;
    pp[axis_z+3] = coordinate_min[axis_z];

    // record as the prime scanline.
    facep->sequ_primscanline->push_back(pp);
    facep->sequ_primscanvoxel->push_back(p_sequ_voxel);

    if (plane == p1[axis_z])
    {
        // the plane happened to intersect p1,
        // refine the slice index and move to next plane.
        slice_idx = computeVoxOneIdx(p1[axis_z]+0.5*voxel_dim, axis_z);

        plane += voxel_dim;
    }

    //-----------------------------------------------------------------------------
    // voxelize the second half of the triangle.
    //-----------------------------------------------------------------------------
    // compute the 2D step vectors along p1p2.
    vx = p2[axis_x]-p1[axis_x];
    vy = p2[axis_y]-p1[axis_y];
    temp = vox_dim / (nx*vx + ny*vy);
    step_2_x = vx * temp;
    step_2_y = vy * temp;

    // Determine the 3D line segments of p0p1.
    a2 = p2[axis_x]-p1[axis_x]; b2 = p1[axis_x];
    c2 = p2[axis_y]-p1[axis_y]; d2 = p1[axis_y];
    e2 = p2[axis_z]-p1[axis_z]; f2 = p1[axis_z];

    // Start from the typical scanline.
    startp0_x = scp0_x;
    startp0_y = scp0_y;
    startp1_x = scp1_x;
    startp1_y = scp1_y;

    while (plane < p2[axis_z])
    {
        double t1 = (plane - f1) / e1;
        endp0_x = a1 * t1 + b1;
        endp0_y = c1 * t1 + d1;

        double t2 = (plane - f2) / e2;
        endp1_x = a2 * t2 + b2;
        endp1_y = c2 * t2 + d2;

        movep0_x = startp0_x;
        movep0_y = startp0_y;
        movep1_x = startp1_x;
        movep1_y = startp1_y;
        length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
        while (length > 0.0)
        {
            vector<int*>* p_sequ_voxel = new vector<int*>;
            mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
                       axis_x, axis_y, axis_z, slice_idx, marker,
                       p_sequ_voxel);
            scanline_count++;

            // record the scanlines (optional).
            double* pp = new double[6];
            pp[axis_x] = movep0_x;
            pp[axis_y] = movep0_y;
            pp[axis_z] = coordinate_min[axis_z];
            pp[axis_x+3] = movep1_x;
            pp[axis_y+3] = movep1_y;
            pp[axis_z+3] = coordinate_min[axis_z];
            if (movep0_x == startp0_x && movep0_y == startp0_y)
            {
                // record as the prime scanline.
                facep->sequ_primscanline->push_back(pp);
                facep->sequ_primscanvoxel->push_back(p_sequ_voxel);
            }
            else
            {
                // record the scanline within slice.
                facep->sequ_scanline->push_back(pp);
                facep->sequ_scanvoxel->push_back(p_sequ_voxel);
            }

            movep0_x += step_1_x;
            movep0_y += step_1_y;
            movep1_x += step_2_x;
            movep1_y += step_2_y;

            length -= step_1_length;
        }

        movep0_x = endp0_x;
        movep0_y = endp0_y;
        movep1_x = endp1_x;
        movep1_y = endp1_y;

        p_sequ_voxel = new vector<int*>;
        mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
                   axis_x, axis_y, axis_z, slice_idx, marker,
                   p_sequ_voxel);
        scanline_count++;

        // record the scanlines (optional).
        double* pp = new double[6];
        pp[axis_x] = movep0_x;
        pp[axis_y] = movep0_y;
        pp[axis_z] = coordinate_min[axis_z];
        pp[axis_x+3] = movep1_x;
        pp[axis_y+3] = movep1_y;
        pp[axis_z+3] = coordinate_min[axis_z];

        // record as the prime scanline.
        facep->sequ_primscanline->push_back(pp);
        facep->sequ_primscanvoxel->push_back(p_sequ_voxel);

        // move to next slice
        startp0_x = endp0_x;
        startp0_y = endp0_y;
        startp1_x = endp1_x;
        startp1_y = endp1_y;
        plane += voxel_dim;
        slice_idx++;
    }

    // voxelize the last slice.
    endp0_x = p2[axis_x];
    endp0_y = p2[axis_y];
    endp1_x = endp0_x;
    endp1_y = endp0_y;

    length = (endp0_x - startp0_x)*step_1_x + (endp0_y - startp0_y)*step_1_y;
    while (length > 0.0)
    {
        vector<int*>* p_sequ_voxel = new vector<int*>;
        mark2DLine(movep0_x, movep0_y, movep1_x, movep1_y,
                   axis_x, axis_y, axis_z, slice_idx, marker,
                   p_sequ_voxel);
        scanline_count++;

        // record the scanlines (optional).
        double* pp = new double[6];
        pp[axis_x] = movep0_x;
        pp[axis_y] = movep0_y;
        pp[axis_z] = coordinate_min[axis_z];
        pp[axis_x+3] = movep1_x;
        pp[axis_y+3] = movep1_y;
        pp[axis_z+3] = coordinate_min[axis_z];
        if (movep0_x == startp0_x && movep0_y == startp0_y)
        {
            // record as the prime scanline.
            facep->sequ_primscanline->push_back(pp);
            facep->sequ_primscanvoxel->push_back(p_sequ_voxel);
        }
        else
        {
            // record the scanline within slice.
            facep->sequ_scanline->push_back(pp);
            facep->sequ_scanvoxel->push_back(p_sequ_voxel);
        }

        movep0_x += step_1_x;
        movep0_y += step_1_y;
        movep1_x += step_2_x;
        movep1_y += step_2_y;

        length -= step_1_length;
    }
}

void VoxelMesh::markFaceByScanlineInteger(TriangleFace* facep, int marker)
{
    //Get the 3 vetex of the face.
    double p0[3], p1[3], p2[3];
    p0[0] = facep->node0->ori_coordinate[0];
    p0[1] = facep->node0->ori_coordinate[1];
    p0[2] = facep->node0->ori_coordinate[2];
    p1[0] = facep->node1->ori_coordinate[0];
    p1[1] = facep->node1->ori_coordinate[1];
    p1[2] = facep->node1->ori_coordinate[2];
    p2[0] = facep->node2->ori_coordinate[0];
    p2[1] = facep->node2->ori_coordinate[1];
    p2[2] = facep->node2->ori_coordinate[2];

    // compute the norm, i.e. (p1p0)x(p2p0).
    double v1[3], v2[3];
    v1[0] = p1[0] - p0[0];
    v1[1] = p1[1] - p0[1];
    v1[2] = p1[2] - p0[2];

    v2[0] = p2[0] - p0[0];
    v2[1] = p2[1] - p0[1];
    v2[2] = p2[2] - p0[2];

    double norm[3];
    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

    facep->norm[0] = norm[0];
    facep->norm[1] = norm[1];
    facep->norm[2] = norm[2];

    // take the absolute value of the norm.
    if (norm[0] < 0.0)
        norm[0] = -norm[0];

    if (norm[1] < 0.0)
        norm[1] = -norm[1];

    if (norm[2] < 0.0)
        norm[2] = -norm[2];

    // find the dominant direction.
    int axis_z = 0;
    if (norm[0] < norm[1])
    {
        if (norm[1] < norm[2])  // 0<1<2
        {
            axis_z = 2;
        }
        else // 0<1, 2<=1
        {
            axis_z = 1;
        }
    }
    else if (norm[0] < norm[2]) // 1<=0, 0<2
    {
        axis_z = 2;
    }

    // Sort the three voxels by order of dominant component.
    sortVertByDomiAxis(p0, p1, p2, axis_z);

    // voxelize the 3 vertices.
    int p0_vox[3], p1_vox[3], p2_vox[3];
    computeVoxIdx(p0_vox, p0);
    computeVoxIdx(p1_vox, p1);
    computeVoxIdx(p2_vox, p2);

    // If the case is simple, no complicate computation is needed.
    if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
        p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
    {
        markPoint(p0_vox[0] + p0_vox[1]*grid_density + p0_vox[2]*grid_density*(long long)grid_density, marker);
        return;
    }

    //-----------------------------------------------------------------------------
    // Redefine the x y z coordinates by the domi axis.
    //-----------------------------------------------------------------------------
    int axis_x, axis_y;
    if (axis_z == 0)
    {
        axis_x = 1;
        axis_y = 2;
    }
    else if (axis_z == 1)
    {
        axis_x = 0;
        axis_y = 2;
    }
    else
    {
        axis_x = 0;
        axis_y = 1;
    }

    facep->axis_x = axis_x;
    facep->axis_y = axis_y;
    facep->axis_z = axis_z;

    //-----------------------------------------------------------------------------
    // Divide the triangle into two parts.
    //-----------------------------------------------------------------------------
    double p11[3];
    if (p0_vox[axis_z] == p2_vox[axis_z])
    {
        // the triangle is within one xy slice.
        p11[0] = p0[0];
        p11[1] = p0[1];
        p11[2] = p0[2];
    }
    else
    {
        // Determine the 3D line segments of p0p2.
        double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
        double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
        double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

        double t1 = (p1[axis_z] - f1) / e1;
        p11[axis_x] = a1 * t1 + b1;
        p11[axis_y] = c1 * t1 + d1;

        p11[axis_z] = p1[axis_z];
    }

    int p11_vox[3];
    computeVoxIdx(p11_vox, p11);

    // The array use to record the scanlines.
    facep->sequ_primscanline = new vector<double*>;
    facep->sequ_primscanvoxel = new vector<vector<int*>*>;
    facep->sequ_scanline = new vector<double*>;
    facep->sequ_scanvoxel = new vector<vector<int*>*>;

    int voxsequ_1[LONGEST_EDGE][3];
    int voxsequ_2[LONGEST_EDGE][3];
    int size_1, size_2;

    //-----------------------------------------------------------------------------
    // Voxelize the first half of the triangle slice by slice.
    //-----------------------------------------------------------------------------
    size_1 = mark3DLine(p11, p0, p11_vox, p0_vox, voxsequ_1, marker);
    size_2 = mark3DLine(p1, p0, p1_vox, p0_vox, voxsequ_2, marker);

    int start_slice = p11_vox[axis_z];
    int end_slice = p0_vox[axis_z];

    int start_voxel_1 = 0, start_voxel_2 = 0;
    for (int slice_i=start_slice; slice_i>=end_slice; --slice_i)
    {
        int i_1 = start_voxel_1;
        int i_2 = start_voxel_2;

        // the flag indicate if a sequence has run out.
        bool end_flag_1 = false, end_flag_2 = false;
        while (!end_flag_1 || !end_flag_2)
        {
            int *vox_1 = voxsequ_1[i_1];
            int *vox_2 = voxsequ_2[i_2];

            vector<int*>* p_sequ_voxel = new vector<int*>;
            mark2DLineInteger(vox_1[axis_x], vox_1[axis_y], vox_2[axis_x], vox_2[axis_y],
                              axis_x, axis_y, axis_z, slice_i, marker,
                              p_sequ_voxel);
            scanline_count++;

            // record the scanlines (optional).
            if (i_1 == start_voxel_1 && i_2 == start_voxel_2)
            {
                // record the prime scanline.
                double *pp = new double[6];
                pp[axis_x] = (vox_1[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y] = (vox_1[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z] = coordinate_min[axis_z];
                pp[axis_x+3] = (vox_2[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y+3] = (vox_2[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z+3] = coordinate_min[axis_z];
                facep->sequ_primscanline->push_back(pp);
                facep->sequ_primscanvoxel->push_back(p_sequ_voxel);
            }
            else
            {
                // record the scanline within slice.
                double *pp = new double[6];
                pp[axis_x] = (vox_1[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y] = (vox_1[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z] = coordinate_min[axis_z];
                pp[axis_x+3] = (vox_2[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y+3] = (vox_2[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z+3] = coordinate_min[axis_z];
                facep->sequ_scanline->push_back(pp);
                facep->sequ_scanvoxel->push_back(p_sequ_voxel);
            }

            // Find the next scanline.
            int delta_X = (vox_1[axis_x] - vox_2[axis_x]);
            int delta_Y = (vox_1[axis_y] - vox_2[axis_y]);
            int axis_xx = axis_x;
            int axis_yy = axis_y;
            int abs_X_Y = abs(delta_X) + abs(delta_Y);
            //int abs_X_Y = delta_X + delta_Y;

//            if (abs(delta_X) < abs(delta_Y))
//            //if (delta_X < delta_Y)
//            {
//                int temp = delta_X;
//                delta_X = delta_Y;
//                delta_Y = temp;

//                axis_xx = axis_y;
//                axis_yy = axis_x;
//            }

            // find the next vox in sequ1
            if (!end_flag_1)
            {
                i_1++;

                if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int temp = delta_Y * vox_1[axis_xx] - delta_X * vox_1[axis_yy];
                        int c1 = temp - abs_X_Y;
                        int c2 = temp + abs_X_Y;

                        int i_curr = i_1 + 1;
                        while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_1[i_curr][axis_xx]
                                    - delta_X * voxsequ_1[i_curr][axis_yy];

                            //if (c1<=cc && cc<=c2)
                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_1++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_1 = true;
                    i_1--;
                }
            }

            // find the next vox in sequ2
            if (!end_flag_2)
            {
                i_2++;

                if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int temp = delta_Y * vox_2[axis_xx] - delta_X * vox_2[axis_yy];
                        int c1 = temp - abs_X_Y;
                        int c2 = temp + abs_X_Y;

                        int i_curr = i_2 + 1;
                        while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_2[i_curr][axis_xx]
                                   - delta_X * voxsequ_2[i_curr][axis_yy];

                            if (c1<=cc && cc<=c2)
                                //if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_2++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_2 = true;
                    i_2--;
                }
            }

        } // end while, scanlines within one slice

        // move to next slice.
        start_voxel_1 = i_1 + 1;
        start_voxel_2 = i_2 + 1;

    } // end for

    //-----------------------------------------------------------------------------
    // Voxelize the second half of the triangle slice by slice.
    //-----------------------------------------------------------------------------
    size_1 = mark3DLine(p11, p2, p11_vox, p2_vox, voxsequ_1, marker);
    size_2 = mark3DLine(p1, p2, p1_vox, p2_vox, voxsequ_2, marker);

    start_slice = p11_vox[axis_z];
    end_slice = p2_vox[axis_z];

    start_voxel_1 = 0;
    start_voxel_2 = 0;
    for (int slice_i=start_slice; slice_i<=end_slice; ++slice_i)
    {
        int i_1 = start_voxel_1;
        int i_2 = start_voxel_2;

        // the flag indicate if a sequence has run out.
        bool end_flag_1 = false, end_flag_2 = false;
        while (!end_flag_1 || !end_flag_2)
        {
            int *vox_1 = voxsequ_1[i_1];
            int *vox_2 = voxsequ_2[i_2];

            vector<int*>* p_sequ_voxel = new vector<int*>;
            mark2DLineInteger(vox_1[axis_x], vox_1[axis_y], vox_2[axis_x], vox_2[axis_y],
                              axis_x, axis_y, axis_z, slice_i, marker,
                              p_sequ_voxel);
            scanline_count++;

            // record the scanlines (optional).
            if (i_1 == start_voxel_1 && i_2 == start_voxel_2)
            {
                // record the prime scanline.
                double *pp = new double[6];
                pp[axis_x] = (vox_1[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y] = (vox_1[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z] = coordinate_min[axis_z];
                pp[axis_x+3] = (vox_2[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y+3] = (vox_2[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z+3] = coordinate_min[axis_z];
                facep->sequ_primscanline->push_back(pp);
                facep->sequ_primscanvoxel->push_back(p_sequ_voxel);
            }
            else
            {
                // record the scanline within slice.
                double *pp = new double[6];
                pp[axis_x] = (vox_1[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y] = (vox_1[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z] = coordinate_min[axis_z];
                pp[axis_x+3] = (vox_2[axis_x]+0.5)*voxel_dim + coordinate_min[axis_x];
                pp[axis_y+3] = (vox_2[axis_y]+0.5)*voxel_dim + coordinate_min[axis_y];
                pp[axis_z+3] = coordinate_min[axis_z];
                facep->sequ_scanline->push_back(pp);
                facep->sequ_scanvoxel->push_back(p_sequ_voxel);
            }

            // Find the next scanline.
            int delta_X = (vox_1[axis_x] - vox_2[axis_x]);
            int delta_Y = (vox_1[axis_y] - vox_2[axis_y]);
            int axis_xx = axis_x;
            int axis_yy = axis_y;
            int abs_X_Y = abs(delta_X) + abs(delta_Y);
            //int abs_X_Y = delta_X + delta_Y;

//            if (abs(delta_X) < abs(delta_Y))
//            //if (delta_X < delta_Y)
//            {
//                int temp = delta_X;
//                delta_X = delta_Y;
//                delta_Y = temp;

//                axis_xx = axis_y;
//                axis_yy = axis_x;
//            }

            // find the next vox in sequ1
            if (!end_flag_1)
            {
                i_1++;

                if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int temp = delta_Y * vox_1[axis_xx] - delta_X * vox_1[axis_yy];
                        int c1 = temp - abs_X_Y;
                        int c2 = temp + abs_X_Y;

                        int i_curr = i_1 + 1;
                        while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_1[i_curr][axis_xx]
                                   - delta_X * voxsequ_1[i_curr][axis_yy];

                            //if (c1<=cc && cc<=c2)
                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_1++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_1 = true;
                    i_1--;
                }
            }

            // find the next vox in sequ2
            if (!end_flag_2)
            {
                i_2++;

                if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int temp = delta_Y * vox_2[axis_xx] - delta_X * vox_2[axis_yy];
                        int c1 = temp - abs_X_Y;
                        int c2 = temp + abs_X_Y;

                        int i_curr = i_2 + 1;
                        while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_2[i_curr][axis_xx]
                                   - delta_X * voxsequ_2[i_curr][axis_yy];

                            if (c1<=cc && cc<=c2)
                            //if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_2++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_2 = true;
                    i_2--;
                }
            }

        } // end while, scanlines within one slice

        // move to next slice.
        start_voxel_1 = i_1 + 1;
        start_voxel_2 = i_2 + 1;

    } // end for

}

void VoxelMesh::markFaceByScanlineIntegerDetailed(TriangleFace* facep, int marker)
{
    //Get the 3 vetex of the face.
    double p0[3], p1[3], p2[3];
    p0[0] = facep->node0->ori_coordinate[0];
    p0[1] = facep->node0->ori_coordinate[1];
    p0[2] = facep->node0->ori_coordinate[2];
    p1[0] = facep->node1->ori_coordinate[0];
    p1[1] = facep->node1->ori_coordinate[1];
    p1[2] = facep->node1->ori_coordinate[2];
    p2[0] = facep->node2->ori_coordinate[0];
    p2[1] = facep->node2->ori_coordinate[1];
    p2[2] = facep->node2->ori_coordinate[2];

    int grid_den = grid_density;
    double vox_dim = voxel_dim;
    double min_x = coordinate_min[0];
    double min_y = coordinate_min[1];
    double min_z = coordinate_min[2];

    double v1[3], v2[3];
    v1[0] = p1[0] - p0[0];
    v1[1] = p1[1] - p0[1];
    v1[2] = p1[2] - p0[2];

    v2[0] = p2[0] - p0[0];
    v2[1] = p2[1] - p0[1];
    v2[2] = p2[2] - p0[2];

    // compute the norm, i.e. (p1p0)x(p2p0).
    double norm[3];
    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

    // take the absolute value of the norm.
    if (norm[0] < 0.0)
        norm[0] = -norm[0];

    if (norm[1] < 0.0)
        norm[1] = -norm[1];

    if (norm[2] < 0.0)
        norm[2] = -norm[2];

    // find the dominant direction.
    int axis_x, axis_y, axis_z;
    long long vox_increment_x, vox_increment_y;
    if (norm[0] < norm[1])
    {
        if (norm[1] < norm[2])  // 0<1<2
        {
            axis_z = 2;
            axis_x = 0;
            axis_y = 1;

            vox_increment_x = 1;
            vox_increment_y = grid_den;
        }
        else // 0<1, 2<=1
        {
            axis_z = 1;
            axis_x = 0;
            axis_y = 2;

            vox_increment_x = 1;
            vox_increment_y = grid_den * (long long)grid_den;
        }
    }
    else if (norm[0] < norm[2]) // 1<=0, 0<2
    {
        axis_z = 2;
        axis_x = 0;
        axis_y = 1;

        vox_increment_x = 1;
        vox_increment_y = grid_den;
    }
    else
    {
        axis_z = 0;
        axis_x = 1;
        axis_y = 2;

        vox_increment_x = grid_den;
        vox_increment_y = grid_den * (long long)grid_den;
    }

    // Sort the three voxels by the order of the dominant component (axis_z).
    //sortVertByDomiAxis(p0, p1, p2, axis_z);
    if (p0[axis_z] <= p1[axis_z])
    {
        if (p1[axis_z] <= p2[axis_z])  //p0 <= p1 <= p2
        {
            ;
        }
        else
        {
            if (p0[axis_z] <= p2[axis_z])  //p0 <= p2 < p1
            {
                double temp;
                temp = p1[0]; p1[0] = p2[0]; p2[0] = temp;
                temp = p1[1]; p1[1] = p2[1]; p2[1] = temp;
                temp = p1[2]; p1[2] = p2[2]; p2[2] = temp;
            }
            else   //p2 < p0 <= p1
            {
                double temp;
                temp = p2[0]; p2[0] = p1[0]; p1[0] = p0[0]; p0[0] = temp;
                temp = p2[1]; p2[1] = p1[1]; p1[1] = p0[1]; p0[1] = temp;
                temp = p2[2]; p2[2] = p1[2]; p1[2] = p0[2]; p0[2] = temp;
            }
        }
    }
    else
    {
        if (p1[axis_z] >= p2[axis_z])  //p2 <= p1 < p0
        {
            double temp;
            temp = p0[0]; p0[0] = p2[0]; p2[0] = temp;
            temp = p0[1]; p0[1] = p2[1]; p2[1] = temp;
            temp = p0[2]; p0[2] = p2[2]; p2[2] = temp;
        }
        else
        {
            if (p0[axis_z] <= p2[axis_z])  //p1 < p0 <= p2
            {
                double temp;
                temp = p0[0]; p0[0] = p1[0]; p1[0] = temp;
                temp = p0[1]; p0[1] = p1[1]; p1[1] = temp;
                temp = p0[2]; p0[2] = p1[2]; p1[2] = temp;
            }
            else   //p1 < p2 < p0
            {
                double temp;
                temp = p0[0]; p0[0] = p1[0]; p1[0] = p2[0]; p2[0] = temp;
                temp = p0[1]; p0[1] = p1[1]; p1[1] = p2[1]; p2[1] = temp;
                temp = p0[2]; p0[2] = p1[2]; p1[2] = p2[2]; p2[2] = temp;
            }
        }
    }

    // voxelize the 3 vertices.
    int p0_vox[3], p1_vox[3], p2_vox[3];
    p0_vox[0] = floor((p0[0] - min_x) / vox_dim);
    if (p0_vox[0] < 0)
        p0_vox[0] = 0;
    else if (p0_vox[0] >= grid_den)
        p0_vox[0] = grid_den - 1;

    p0_vox[1] = floor((p0[1] - min_y) / vox_dim);
    if (p0_vox[1] < 0)
        p0_vox[1] = 0;
    else if (p0_vox[1] >= grid_den)
        p0_vox[1] = grid_den - 1;

    p0_vox[2] = floor((p0[2] - min_z) / vox_dim);
    if (p0_vox[2] < 0)
        p0_vox[2] = 0;
    else if (p0_vox[2] >= grid_den)
        p0_vox[2] = grid_den - 1;

    p1_vox[0] = floor((p1[0] - min_x) / vox_dim);
    if (p1_vox[0] < 0)
        p1_vox[0] = 0;
    else if (p1_vox[0] >= grid_den)
        p1_vox[0] = grid_den - 1;

    p1_vox[1] = floor((p1[1] - min_y) / vox_dim);
    if (p1_vox[1] < 0)
        p1_vox[1] = 0;
    else if (p1_vox[1] >= grid_den)
        p1_vox[1] = grid_den - 1;

    p1_vox[2] = floor((p1[2] - min_z) / vox_dim);
    if (p1_vox[2] < 0)
        p1_vox[2] = 0;
    else if (p1_vox[2] >= grid_den)
        p1_vox[2] = grid_den - 1;

    p2_vox[0] = floor((p2[0] - min_x) / vox_dim);
    if (p2_vox[0] < 0)
        p2_vox[0] = 0;
    else if (p2_vox[0] >= grid_den)
        p2_vox[0] = grid_den - 1;

    p2_vox[1] = floor((p2[1] - min_y) / vox_dim);
    if (p2_vox[1] < 0)
        p2_vox[1] = 0;
    else if (p2_vox[1] >= grid_den)
        p2_vox[1] = grid_den - 1;

    p2_vox[2] = floor((p2[2] - min_z) / vox_dim);
    if (p2_vox[2] < 0)
        p2_vox[2] = 0;
    else if (p2_vox[2] >= grid_den)
        p2_vox[2] = grid_den - 1;

    // If the case is simple, no complicate computation is needed.
    if (p0_vox[0] == p1_vox[0] && p0_vox[1] == p1_vox[1] && p0_vox[2] == p1_vox[2] &&
        p0_vox[0] == p2_vox[0] && p0_vox[1] == p2_vox[1] && p0_vox[2] == p2_vox[2])
    {
        markPoint(p0_vox[0] + (p0_vox[1] + p0_vox[2]*grid_den) * (long long)grid_den, 1);
        return;
    }

    //-----------------------------------------------------------------------------
    // Divide the triangle into two parts.
    //-----------------------------------------------------------------------------
    double p11[3];
    if (p0_vox[axis_z] == p2_vox[axis_z])
    {
        // the triangle is within one xy slice.
        p11[0] = p0[0];
        p11[1] = p0[1];
        p11[2] = p0[2];
    }
    else
    {
        // Determine the 3D line segments of p0p2.
        double a1 = p2[axis_x]-p0[axis_x], b1 = p0[axis_x];
        double c1 = p2[axis_y]-p0[axis_y], d1 = p0[axis_y];
        double e1 = p2[axis_z]-p0[axis_z], f1 = p0[axis_z];

        double t1 = (p1[axis_z] - f1) / e1;
        p11[axis_x] = a1 * t1 + b1;
        p11[axis_y] = c1 * t1 + d1;

        p11[axis_z] = p1[axis_z];
    }

    int p11_vox[3];
    p11_vox[0] = floor((p11[0] - min_x) / vox_dim);
    if (p11_vox[0] < 0)
        p11_vox[0] = 0;
    else if (p11_vox[0] >= grid_den)
        p11_vox[0] = grid_den - 1;

    p11_vox[1] = floor((p11[1] - min_y) / vox_dim);
    if (p11_vox[1] < 0)
        p11_vox[1] = 0;
    else if (p11_vox[1] >= grid_den)
        p11_vox[1] = grid_den - 1;

    p11_vox[2] = floor((p11[2] - min_z) / vox_dim);
    if (p11_vox[2] < 0)
        p11_vox[2] = 0;
    else if (p11_vox[2] >= grid_den)
        p11_vox[2] = grid_den - 1;

    //-----------------------------------------------------------------------------
    // Voxelize the first part of the triangle slice by slice.
    //-----------------------------------------------------------------------------
    int voxsequ_1[LONGEST_EDGE][3];
    int voxsequ_2[LONGEST_EDGE][3];
    int size_1 = mark3DLine(p0, p11, p0_vox, p11_vox, voxsequ_1, marker);
    int size_2 = mark3DLine(p0, p1, p0_vox, p1_vox, voxsequ_2, marker);

    int start_slice = p0_vox[axis_z];
    int end_slice = p11_vox[axis_z];

    int i_1 = 0, i_2 = 0;
    for (int slice_i=start_slice; slice_i<=end_slice; ++slice_i)
    {
        // the flag indicate if a sequence is run out.
        bool end_flag_1 = false, end_flag_2 = false;
        while (!(end_flag_1 && end_flag_2))
        {
            int *vox_1 = voxsequ_1[i_1];
            int *vox_2 = voxsequ_2[i_2];

            //---------------------------------------------------------------------------------
            // Voxelize the center scanline
            //---------------------------------------------------------------------------------
//            mark2DScanlineInteger(vox_1[axis_x], vox_1[axis_y],
//                                  vox_2[axis_x], vox_2[axis_y],
//                                  axis_x, axis_y, axis_z, slice_i, marker);

            // Compute the step distance parameters.
            int start_pixel_x = vox_1[axis_x];
            int start_pixel_y = vox_1[axis_y];
            int end_pixel_x = vox_2[axis_x];
            int end_pixel_y = vox_2[axis_y];

            int delta_X = end_pixel_x - start_pixel_x;
            int delta_Y = end_pixel_y - start_pixel_y;

            int sign_x, sign_y, length_x, length_y;
            long long moving_vox_x, moving_vox_y;
            if (delta_X >= 0)
            {
                sign_x = 1;
                length_x = delta_X;
                moving_vox_x = vox_increment_x;
            }
            else
            {
                sign_x = -1;
                length_x = -delta_X;
                moving_vox_x = -vox_increment_x;
            }

            if (delta_Y >= 0)
            {
                sign_y = 1;
                length_y = delta_Y;
                moving_vox_y = vox_increment_y;
            }
            else
            {
                sign_y = -1;
                length_y = -delta_Y;
                moving_vox_y = -vox_increment_y;
            }

            // Compute the step distance parameters.
            int t_x, t_y, step_x, step_y;
            if (delta_X != 0)
            {
                if (delta_Y != 0)
                {
                    // v_x != 0, v_y != 0.
                    t_x = length_y;
                    step_x = length_y << 1;  // step_x = v_y*2

                    t_y = length_x;
                    step_y = length_x << 1;  // step_y = v_x*2
                }
                else
                {
                    // v_x != 0, v_y == 0.
                    t_x = 1;
                    step_x = 2;
                    t_y = 1000000000;
                }
            }
            else if (delta_Y != 0)
            {
                // v_x == 0, v_y != 0.
                t_y = 1;
                step_y = 2;
                t_x = 1000000000;
            }

            // The core loop.
            int i_x = 0;
            int i_y = 0;
            long long moving_vox_idx = vox_1[0] + (vox_1[1] + vox_1[2]*grid_den) * (long long)grid_den;
            while (!(i_x == length_x && i_y == length_y))
            {
                markPoint(moving_vox_idx, 1);
                //voxel_map[moving_vox_idx] = 1;

                if (t_x <= t_y)
                {
                    t_y = t_y - t_x;
                    t_x = step_x;

                    i_x++;
                    moving_vox_idx += moving_vox_x;
                }
                else
                {
                    t_x = t_x - t_y;
                    t_y = step_y;

                    i_y++;
                    moving_vox_idx += moving_vox_y;
                }
            }

            // mark the last pixel
            markPoint(moving_vox_idx, 1);
            //voxel_map[moving_vox_idx] = 1;

            //---------------------------------------------------------------------------------
            // Determine the next scanline
            //---------------------------------------------------------------------------------
            int temp = delta_Y * end_pixel_x - delta_X * end_pixel_y;
            int abs_X_Y = length_x + length_y;
            int c1 = temp - abs_X_Y;
            int c2 = temp + abs_X_Y;

            // find the next vox in sequ1
            if (!end_flag_1)
            {
                i_1++;

                if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int i_curr = i_1 + 1;
                        while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_1[i_curr][axis_x]
                                    - delta_X * voxsequ_1[i_curr][axis_y];

                            //if (c1<=cc && cc<=c2)
                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_1++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_1 = true;
                    i_1--;
                }
            }

            // find the next vox in sequ2
            if (!end_flag_2)
            {
                i_2++;

                if (i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int i_curr = i_2 + 1;
                        while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_2[i_curr][axis_x]
                                    - delta_X * voxsequ_2[i_curr][axis_y];

                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_2++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_2 = true;
                    i_2--;
                }
            }

        } // end while

        // move to next slice.
        i_1++;
        i_2++;

    } // end for

    //-----------------------------------------------------------------------------
    // Voxelize the second part of the triangle slice by slice.
    //-----------------------------------------------------------------------------
    size_1 = mark3DLine(p11, p2, p11_vox, p2_vox, voxsequ_1, marker);
    size_2 = mark3DLine(p1, p2, p1_vox, p2_vox, voxsequ_2, marker);

    start_slice = p11_vox[axis_z];
    end_slice = p2_vox[axis_z];

    i_1 = 0;
    i_2 = 0;
    for (int slice_i=start_slice; slice_i<=end_slice; ++slice_i)
    {
        // the flag indicate if a sequence is run out.
        bool end_flag_1 = false, end_flag_2 = false;
        while (!(end_flag_1 && end_flag_2))
        {
            int *vox_1 = voxsequ_1[i_1];
            int *vox_2 = voxsequ_2[i_2];

            //---------------------------------------------------------------------------------
            // Voxelize the center scanline
            //---------------------------------------------------------------------------------
//            mark2DScanlineInteger(vox_1[axis_x], vox_1[axis_y],
//                                  vox_2[axis_x], vox_2[axis_y],
//                                  axis_x, axis_y, axis_z, slice_i, marker);

            // Compute the step distance parameters.
            int start_pixel_x = vox_1[axis_x];
            int start_pixel_y = vox_1[axis_y];
            int end_pixel_x = vox_2[axis_x];
            int end_pixel_y = vox_2[axis_y];

            int delta_X = end_pixel_x - start_pixel_x;
            int delta_Y = end_pixel_y - start_pixel_y;

            int sign_x, sign_y, length_x, length_y;
            long long moving_vox_x, moving_vox_y;
            if (delta_X >= 0)
            {
                sign_x = 1;
                length_x = delta_X;
                moving_vox_x = vox_increment_x;
            }
            else
            {
                sign_x = -1;
                length_x = -delta_X;
                moving_vox_x = -vox_increment_x;
            }

            if (delta_Y >= 0)
            {
                sign_y = 1;
                length_y = delta_Y;
                moving_vox_y = vox_increment_y;
            }
            else
            {
                sign_y = -1;
                length_y = -delta_Y;
                moving_vox_y = -vox_increment_y;
            }

            // Compute the step distance parameters.
            int t_x, t_y, step_x, step_y;
            if (delta_X != 0)
            {
                if (delta_Y != 0)
                {
                    // v_x != 0, v_y != 0.
                    t_x = length_y;
                    step_x = length_y << 1;  // step_x = v_y*2

                    t_y = length_x;
                    step_y = length_x << 1;  // step_y = v_x*2
                }
                else
                {
                    // v_x != 0, v_y == 0.
                    t_x = 1;
                    step_x = 2;
                    t_y = 1000000000;
                }
            }
            else if (delta_Y != 0)
            {
                // v_x == 0, v_y != 0.
                t_y = 1;
                step_y = 2;
                t_x = 1000000000;
            }

            // The core loop.
            int i_x = 0;
            int i_y = 0;
            long long moving_vox_idx = vox_1[0] + (vox_1[1] + vox_1[2]*grid_den) * (long long)grid_den;
            while (!(i_x == length_x && i_y == length_y))
            {
                markPoint(moving_vox_idx, 1);
                //voxel_map[moving_vox_idx] = 1;

                if (t_x <= t_y)
                {
                    t_y = t_y - t_x;
                    t_x = step_x;

                    i_x++;
                    moving_vox_idx += moving_vox_x;
                }
                else
                {
                    t_x = t_x - t_y;
                    t_y = step_y;

                    i_y++;
                    moving_vox_idx += moving_vox_y;
                }
            }

            // mark the last pixel
            markPoint(moving_vox_idx, 1);
            //voxel_map[moving_vox_idx] = 1;

            //---------------------------------------------------------------------------------
            // Determine the next scanline
            //---------------------------------------------------------------------------------
            int temp = delta_Y * end_pixel_x - delta_X * end_pixel_y;
            int abs_X_Y = length_x + length_y;
            int c1 = temp - abs_X_Y;
            int c2 = temp + abs_X_Y;

            // find the next vox in sequ1
            if (!end_flag_1)
            {
                i_1++;

                if (i_1 < size_1 && voxsequ_1[i_1][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int i_curr = i_1 + 1;
                        while (i_curr < size_1 && voxsequ_1[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_1[i_curr][axis_x]
                                    - delta_X * voxsequ_1[i_curr][axis_y];

                            //if (c1<=cc && cc<=c2)
                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_1++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_1 = true;
                    i_1--;
                }
            }

            // find the next vox in sequ2
            if (!end_flag_2)
            {
                i_2++;

                if ((delta_X != 0 || delta_Y != 0) &&
                    i_2 < size_2 && voxsequ_2[i_2][axis_z] == slice_i)
                {
                    // do not test if the previous scanline is one voxel.
                    //if (delta_X != 0 || delta_Y != 0)
                    {
                        int i_curr = i_2 + 1;
                        while (i_curr < size_2 && voxsequ_2[i_curr][axis_z] == slice_i)
                        {
                            int cc = delta_Y * voxsequ_2[i_curr][axis_x]
                                   - delta_X * voxsequ_2[i_curr][axis_y];

                            if (c1<cc && cc<c2)
                            {
                                // the voxel is safe.
                                i_curr++;
                                i_2++;
                            }
                            else
                            {
                                break;
                            }
                        }
                    }
                }
                else
                {
                    end_flag_2 = true;
                    i_2--;
                }
            }

        } // end while

        // move to next slice.
        i_1++;
        i_2++;

    } // end for
}

//=========================================================
// Mark a 2D line.
//=========================================================
// Voxelize a 2D line segment from start pixel to end pixel.
int VoxelMesh::mark2DLineInteger(const int start_pixel_x, const int start_pixel_y,
                                 const int end_pixel_x, const int end_pixel_y,
                                 int sequ_pixel[][2])
{
    // Compute l0, the distance between start point and 3 corresponding facets.
    int delta_x, delta_y, v_x, v_y;
    if (start_pixel_x <= end_pixel_x)
    {
        delta_x = 1;
        v_x = end_pixel_x - start_pixel_x;
    }
    else
    {
        delta_x = -1;
        v_x = start_pixel_x - end_pixel_x;
    }

    if (start_pixel_y <= end_pixel_y)
    {
        delta_y = 1;
        v_y = end_pixel_y - start_pixel_y;
    }
    else
    {
        delta_y = -1;
        v_y = start_pixel_y - end_pixel_y;
    }

    // Compute the step distance parameters.
    int t_x, t_y, step_x, step_y;
    if (v_x != 0)
    {
        if (v_y != 0)
        {
            // v_x != 0, v_y != 0.
            t_x = v_y;
            step_x = v_y << 1;  // step_x = v_y*2

            t_y = v_x;
            step_y = v_x << 1;  // step_y = v_x*2
        }
        else
        {
            // v_x != 0, v_y == 0.
            t_x = 1;
            step_x = 2;
            t_y = 1000000000;
        }
    }
    else
    {
        if (v_y != 0)
        {
            // v_x == 0, v_y != 0.
            t_y = 1;
            step_y = 2;
            t_x = 1000000000;
        }
        else
        {
            // v_x == 0, v_y == 0.
            sequ_pixel[0][0] = start_pixel_x;
            sequ_pixel[0][1] = start_pixel_y;
            return 1;
        }
    }

    // The core loop.
    int moving_pixel_x = start_pixel_x;
    int moving_pixel_y = start_pixel_y;
    int i = 0;
    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        sequ_pixel[i][0] = moving_pixel_x;
        sequ_pixel[i][1] = moving_pixel_y;
        i++;

        if (t_x <= t_y)
        {
            t_y = t_y - t_x;
            t_x = step_x;
            if (moving_pixel_x != end_pixel_x)
            {
                moving_pixel_x += delta_x;
            }
        }
        else
        {
            t_x = t_x - t_y;
            t_y = step_y;
            if (moving_pixel_y != end_pixel_y)
            {
                moving_pixel_y += delta_y;
            }
        }
    }

    sequ_pixel[i][0] = end_pixel_x;
    sequ_pixel[i][1] = end_pixel_y;
    i++;

    return i;
}


//=========================================================
//Mark a 3D line starting from startpoint to endpoint,
//which could be coordinates or the index of the voxels.
//=========================================================
// This funtion precisely mark a line segment.
// Input: voxel index of two end points of the line segment as well as the direction.
// Output: the voxel sequence in voxsequ[].
int VoxelMesh::mark3DLine(const double start_point[], const double end_point[],
                          const int start_point_vox[], const int end_point_vox[],
                          int voxsequ[][3], int marker)
{
    //If start_point and end_point are in the same vox, simply mark start_point then return.
//    if (start_point_vox[0] == end_point_vox[0] &&
//        start_point_vox[1] == end_point_vox[1] &&
//        start_point_vox[2] == end_point_vox[2])
//    {
//        markPoint(start_point_vox, marker);

//        voxsequ[0][0] = start_point_vox[0];
//        voxsequ[0][1] = start_point_vox[1];
//        voxsequ[0][2] = start_point_vox[2];

//        return 1;
//    }

    int voxsequ_size = 0;

    // Compute l0, the distances between the start point and 3 corresponding facets.
    double vv[3];
    vv[0] = end_point[0] - start_point[0];
    vv[1] = end_point[1] - start_point[1];
    vv[2] = end_point[2] - start_point[2];

    int length[3];
    length[0] = end_point_vox[0] - start_point_vox[0];
    length[1] = end_point_vox[1] - start_point_vox[1];
    length[2] = end_point_vox[2] - start_point_vox[2];

    bool isEnd[3] = {false, false, false};
    int delta[3];
    double tt[3], step[3];

    long long increment[3];
    increment[0] = 1;
    increment[1] = grid_density;
    increment[2] = grid_density * (long long)grid_density;

    for (int i=0; i<3; i++)
    {
        if (length[i] == 0)
        {
            isEnd[i] = true;
            tt[i] = 1e20;
            step[i] = 1;
        }
        else
        {
            isEnd[i] = false;

            if (length[i] > 0)
            {
                delta[i] = 1;
                tt[i] = voxel_dim - (start_point[i] - coordinate_min[i] - start_point_vox[i] * voxel_dim);
                step[i] = voxel_dim;
            }
            else
            {
                delta[i] = -1;
                tt[i] = start_point[i] - coordinate_min[i] - start_point_vox[i] * voxel_dim;
                step[i] = voxel_dim;

                vv[i] = - vv[i];
                length[i] = -length[i];
                increment[i] = -increment[i];
            }
        }
    }

    //Compute final parameters (multiplied by vv[0]vv[1]vv[2])
    if (!isEnd[0])
    {
        tt[1] *= vv[0];
        step[1] *= vv[0];
        tt[2] *= vv[0];
        step[2] *= vv[0];
    }
    if (!isEnd[1])
    {
        tt[0] *= vv[1];
        step[0] *= vv[1];
        tt[2] *= vv[1];
        step[2] *= vv[1];
    }
    if (!isEnd[2])
    {
        tt[0] *= vv[2];
        step[0] *= vv[2];
        tt[1] *= vv[2];
        step[1] *= vv[2];
    }

    // The core loop.
    int moving_point_vox[3];
    moving_point_vox[0] = start_point_vox[0];
    moving_point_vox[1] = start_point_vox[1];
    moving_point_vox[2] = start_point_vox[2];

    long long moving_vox_idx = start_point_vox[0] +
            (start_point_vox[1] + start_point_vox[2]*grid_density) * (long long)grid_density;

    int i_x = 0, i_y = 0, i_z = 0;
    while (!(i_x == length[0] && i_y == length[1] && i_z == length[2]))
    {
        markPoint(moving_vox_idx, marker);
        //voxel_map[moving_vox_idx] = 1;

        voxsequ[voxsequ_size][0] = moving_point_vox[0];
        voxsequ[voxsequ_size][1] = moving_point_vox[1];
        voxsequ[voxsequ_size][2] = moving_point_vox[2];
        voxsequ_size++;

        if (tt[0] <= tt[1])
        {
            if (tt[0] <= tt[2])
            {
                // 0<=1, 0<=2
                i_x++;

                tt[1] -= tt[0];
                tt[2] -= tt[0];
                tt[0] = step[0];

                moving_vox_idx += increment[0];
                moving_point_vox[0] += delta[0];

//                if (i_x == length[0])
//                {
//                    //tt[0] = 1e20;
//                }
            }
            else
            {
                // 0<=1, 0>2
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

//                if (i_z == length[2])
//                {
//                    //tt[2] = 1e20;
//                }
            }
        }
        else
        {
            if (tt[1] <= tt[2])
            {
                // 1<0, 1<=2
                i_y++;

                tt[0] -= tt[1];
                tt[2] -= tt[1];
                tt[1] = step[1];

                moving_vox_idx += increment[1];
                moving_point_vox[1] += delta[1];

//                if (i_y == length[1])
//                {
//                    //tt[1] = 1e20;
//                }
            }
            else
            {
                // 1<0, 2<1
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

//                if (i_z == length[2])
//                {
//                    //tt[2] = 1e20;
//                }
            }
        }
    }
    markPoint(moving_vox_idx, marker);
    //voxel_map[moving_vox_idx] = 1;

    voxsequ[voxsequ_size][0] = end_point_vox[0];
    voxsequ[voxsequ_size][1] = end_point_vox[1];
    voxsequ[voxsequ_size][2] = end_point_vox[2];
    voxsequ_size++;

    return voxsequ_size;
}

// This funtion precisely mark a line segment.
// Input: voxel index of two end points of the line segment as well as the direction.
// Output: the voxel sequence in voxsequ[].
void VoxelMesh::mark3DLine(const double start_point[], const double end_point[],
                           const int start_point_vox[], const int end_point_vox[],
                           int marker)
{
    //If start_point and end_point are in the same vox, simply mark start_point then return.
//    if (start_point_vox[0] == end_point_vox[0] &&
//        start_point_vox[1] == end_point_vox[1] &&
//        start_point_vox[2] == end_point_vox[2])
//    {
//        markPoint(start_point_vox, marker);

//        voxsequ[0][0] = start_point_vox[0];
//        voxsequ[0][1] = start_point_vox[1];
//        voxsequ[0][2] = start_point_vox[2];

//        return 1;
//    }

    // Compute l0, the distances between the start point and 3 corresponding facets.
    double vv[3];
    vv[0] = end_point[0] - start_point[0];
    vv[1] = end_point[1] - start_point[1];
    vv[2] = end_point[2] - start_point[2];

    int length[3];
    length[0] = end_point_vox[0] - start_point_vox[0];
    length[1] = end_point_vox[1] - start_point_vox[1];
    length[2] = end_point_vox[2] - start_point_vox[2];

    bool isEnd[3] = {false, false, false};
    int delta[3];
    double tt[3], step[3];

    long long increment[3];
    increment[0] = 1;
    increment[1] = grid_density;
    increment[2] = grid_density * (long long)grid_density;

    for (int i=0; i<3; i++)
    {
        if (length[i] == 0)
        {
            isEnd[i] = true;
            tt[i] = 1e20;
            step[i] = 1;
        }
        else
        {
            isEnd[i] = false;

            if (length[i] > 0)
            {
                delta[i] = 1;
                tt[i] = voxel_dim - (start_point[i] - coordinate_min[i] - start_point_vox[i] * voxel_dim);
                step[i] = voxel_dim;
            }
            else
            {
                delta[i] = -1;
                tt[i] = start_point[i] - coordinate_min[i] - start_point_vox[i] * voxel_dim;
                step[i] = voxel_dim;

                vv[i] = - vv[i];
                length[i] = -length[i];
                increment[i] = -increment[i];
            }
        }
    }

    //Compute final parameters (multiplied by vv[0]vv[1]vv[2])
    if (!isEnd[0])
    {
        tt[1] *= vv[0];
        step[1] *= vv[0];
        tt[2] *= vv[0];
        step[2] *= vv[0];
    }
    if (!isEnd[1])
    {
        tt[0] *= vv[1];
        step[0] *= vv[1];
        tt[2] *= vv[1];
        step[2] *= vv[1];
    }
    if (!isEnd[2])
    {
        tt[0] *= vv[2];
        step[0] *= vv[2];
        tt[1] *= vv[2];
        step[1] *= vv[2];
    }

    // The core loop.
    int moving_point_vox[3];
    moving_point_vox[0] = start_point_vox[0];
    moving_point_vox[1] = start_point_vox[1];
    moving_point_vox[2] = start_point_vox[2];

    long long moving_vox_idx = start_point_vox[0] +
            (start_point_vox[1] + start_point_vox[2]*grid_density) * (long long)grid_density;

    int i_x = 0, i_y = 0, i_z = 0;
    while (!(i_x == length[0] && i_y == length[1] && i_z == length[2]))
    {
        markPoint(moving_vox_idx, marker);

        if (tt[0] <= tt[1])
        {
            if (tt[0] <= tt[2])
            {
                // 0<=1, 0<=2
                i_x++;

                tt[1] -= tt[0];
                tt[2] -= tt[0];
                tt[0] = step[0];

                moving_vox_idx += increment[0];
                moving_point_vox[0] += delta[0];

//                if (i_x == length[0])
//                {
//                    //tt[0] = 1e20;
//                }
            }
            else
            {
                // 0<=1, 0>2
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

//                if (i_z == length[2])
//                {
//                    //tt[2] = 1e20;
//                }
            }
        }
        else
        {
            if (tt[1] <= tt[2])
            {
                // 1<0, 1<=2
                i_y++;

                tt[0] -= tt[1];
                tt[2] -= tt[1];
                tt[1] = step[1];

                moving_vox_idx += increment[1];
                moving_point_vox[1] += delta[1];

//                if (i_y == length[1])
//                {
//                    //tt[1] = 1e20;
//                }
            }
            else
            {
                // 1<0, 2<1
                i_z++;

                tt[0] -= tt[2];
                tt[1] -= tt[2];
                tt[2] = step[2];

                moving_vox_idx += increment[2];
                moving_point_vox[2] += delta[2];

//                if (i_z == length[2])
//                {
//                    //tt[2] = 1e20;
//                }
            }
        }
    }
    markPoint(moving_vox_idx, marker);
}

int VoxelMesh::mark3DLineInteger(const int start_point_vox[], const int end_point_vox[],
                                 int voxsequ[][3], int marker)
{
    int moving_vox_i = 0;

    //If start_point and end_point are in the same vox, simply mark start_point then return.
    if (start_point_vox[0] == end_point_vox[0] &&
        start_point_vox[1] == end_point_vox[1] &&
        start_point_vox[2] == end_point_vox[2])
    {
        markPoint(start_point_vox[0] + start_point_vox[1]*grid_density +
                  start_point_vox[2]*grid_density*(long long)grid_density, marker);

        voxsequ[0][0] = start_point_vox[0];
        voxsequ[0][1] = start_point_vox[1];
        voxsequ[0][2] = start_point_vox[2];

        return 1;
    }

    // Compute l0, the distance between start point and 3 corresponding facets.
    double vv[3];
    vv[0] = end_point_vox[0] - start_point_vox[0];
    vv[1] = end_point_vox[1] - start_point_vox[1];
    vv[2] = end_point_vox[2] - start_point_vox[2];

    bool isEnd[3] = {false, false, false};
    int delta[3];
    //double l0[3];
    for (int i=0; i<3; i++)
    {
        if (end_point_vox[i] == start_point_vox[i])
        {
            isEnd[i] = true;
        }
        else
        {
            isEnd[i] = false;

            if (end_point_vox[i] > start_point_vox[i])
            {
                delta[i] = 1;
            }
            else
            {
                delta[i] = -1;

                vv[i] = - vv[i];
            }
        }
    }

    //Compute 3 slopes (multiplied by vv[0]vv[1]vv[2])
    double kk[3] = {1.0, 1.0, 1.0};
    if (!isEnd[0])
    {
        kk[1] *= vv[0];
        kk[2] *= vv[0];
    }
    if (!isEnd[1])
    {
        kk[0] *= vv[1];
        kk[2] *= vv[1];
    }
    if (!isEnd[2])
    {
        kk[0] *= vv[2];
        kk[1] *= vv[2];
    }

    //Compute the step parameters.
    double tt[3], step[3];
    for (int i=0; i<3; i++)
    {
        if  (!isEnd[i])
        {
            tt[i] = kk[i];
            step[i] = 2 * kk[i];
        }
    }

    // The core loop.
    int moving_point_vox[3];
    moving_point_vox[0] = start_point_vox[0];
    moving_point_vox[1] = start_point_vox[1];
    moving_point_vox[2] = start_point_vox[2];

    while (!(isEnd[0] && isEnd[1] && isEnd[2]))
    {
        markPoint(moving_point_vox[0] + moving_point_vox[1]*grid_density +
                  moving_point_vox[2]*grid_density*(long long)grid_density, marker);

        voxsequ[moving_vox_i][0] = moving_point_vox[0];
        voxsequ[moving_vox_i][1] = moving_point_vox[1];
        voxsequ[moving_vox_i][2] = moving_point_vox[2];
        moving_vox_i++;

        // give initial value;.
        int tt_idx;
        double tt_min;
        if (!isEnd[0])
        {
            tt_idx = 0;
            tt_min = tt[0];
            if (!isEnd[1] && tt_min > tt[1])
            {
                tt_idx = 1;
                tt_min = tt[1];
            }
            if (!isEnd[2] && tt_min > tt[2])
            {
                tt_idx = 2;
                tt_min = tt[2];
            }
        }
        else if (!isEnd[1])
        {
            tt_idx = 1;
            tt_min = tt[1];
            if (!isEnd[2] && tt_min > tt[2])
            {
                tt_idx = 2;
                tt_min = tt[2];
            }
        }
        else
        {
            tt_idx = 2;
            tt_min = tt[2];
        }

        //update the parametric steps along each direction.
        for (int j=0; j<3; j++)
        {
            if (!isEnd[j])
            {
                if (j == tt_idx)
                {
                    tt[j] = step[j];
                    moving_point_vox[j] += delta[j];

                    // set termination flag if reach the end point.
                    if (moving_point_vox[j] == end_point_vox[j])
                    {
                        isEnd[j] = true;
                    }
                }
                else
                {
                    tt[j] -= tt_min;
                }
            }
        }
    }
    markPoint(end_point_vox[0] + end_point_vox[1]*grid_density +
              end_point_vox[2]*grid_density*(long long)grid_density, marker);

    voxsequ[moving_vox_i][0] = end_point_vox[0];
    voxsequ[moving_vox_i][1] = end_point_vox[1];
    voxsequ[moving_vox_i][2] = end_point_vox[2];
    moving_vox_i++;

    return moving_vox_i;
}

//================================================================
// Mark internal voxels after finished surface marking.
//================================================================
void VoxelMesh::fillHole(int marker)
{

    markExteriorPart(marker);

    // while all 6 faces have not been marked bfs fill the outside.
    /*long extiror_idx = -1;
    while ( (extiror_idx = boundaryMarked()) != -1)
    {
        bfsMark(extiror_idx, marker);
    }*/

    // iterate over entire grid and mark the inside with 2.
    //ui.progressMessage->setText("Voxelizing interiorly ...");
    //ui.progressMessage->show();
    //ui.progressBar->show();
    //double t = 100.0 / double(grid_density);
    int d = grid_density;
    long long d2 = d*(long long)d;

    for (int xi = 0; xi < d; ++xi)
    {
        for (int yi = 0; yi < d; ++yi)
        {
            for (int zi = 0; zi < d; ++zi)
            {
                long long vox_idx = xi + yi*d + zi*d2;
                if (equaltoMarker(vox_idx, 0))
                {
                    //Mark the voxel as interior one.
                    markPoint(vox_idx, 2);
                }
            }
        }
        //ui.progressBar->setValue(t * double(xi));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    //ui.progressBar->hide();
    //ui.progressMessage->hide();
}

//==============================================================================
//Mark the exterior part of the object.
//==============================================================================
void VoxelMesh::markExteriorPart(int marker)
{
    //ui.progressMessage->setText("Marking exteriorly ...");
    //ui.progressMessage->show();
    //ui.progressBar->show();
    //double t = 100.0 / 6.0 / double(grid_density);

    int d = grid_density;
    long long d2 = d * d;

    //Scan the first time to pre-mark the exterior voxels
    //from 6 bounding surfaces toward the surfaces of the object.
    // x-y planes.
    for (int xi = 0; xi<d; ++xi)
    {
        for (int yi = 0; yi<d; ++yi)
        {
            int zi = 0;
            long long vox_idx = xi + yi*d;
            while (zi < d && equaltoMarker(vox_idx, 0))
            {
                markPoint(vox_idx, marker);
                zi++;
                vox_idx += d2;
            }

            zi = d - 1;
            vox_idx = xi + yi*d + zi*d2;
            while (zi >= 0 && equaltoMarker(vox_idx, 0))
            {
                markPoint(vox_idx, marker);
                zi--;
                vox_idx -= d2;
            }
        }
        //ui.progressBar->setValue(t * double(xi));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    // x-z planes.
    for (int xi = 0; xi<d; ++xi)
    {
        for (int zi = 0; zi<d; ++zi)
        {
            int yi = 0;
            long long vox_idx = xi + zi*d2;
            while (yi < d && equaltoMarker(vox_idx, 0))
            {
                markPoint(vox_idx, marker);
                yi++;
                vox_idx += d;

            }

            yi = d - 1;
            vox_idx = xi + yi*d + zi*d2;
            while (yi >= 0 && equaltoMarker(vox_idx, 0))
            {
                markPoint(vox_idx, marker);
                yi--;
                vox_idx -= d;
            }
        }
        //ui.progressBar->setValue(t * double(xi + grid_density));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    // y-z planes.
    for (int yi = 0; yi<d; ++yi)
    {
        for (int zi = 0; zi<d; ++zi)
        {
            int xi = 0;
            long long vox_idx = yi*d + zi*d2;
            while (xi < d && equaltoMarker(vox_idx, 0))
            {
                markPoint(vox_idx, marker);
                xi++;
                vox_idx++;
            }

            xi = d - 1;
            vox_idx = xi + yi*d + zi*d2;
            while (xi >= 0 && equaltoMarker(vox_idx, 0))
            {
                markPoint(vox_idx, marker);
                xi--;
                vox_idx--;
            }
        }
        //ui.progressBar->setValue(t * double(yi + 2*grid_density));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    //Scan the second time to mark concave part by BFS.
    // x-y planes.
    for (int xi = 0; xi<d; ++xi)
    {
        for (int yi = 0; yi<d; ++yi)
        {
            int zi = 0;
            while (zi < d && equaltoMarker(xi + yi*d + zi*d2, marker))
            {
                //up
                long long vox_idx = zi * (long long)d2 + yi * d + xi;
                if (yi + 1 < d && equaltoMarker(vox_idx + d, 0))
                {
                    bfsMark(vox_idx + d, marker);
                }

                //down
                if (yi - 1 >= 0 && equaltoMarker(vox_idx - d, 0))
                {
                    bfsMark(vox_idx - d, marker);
                }

                //left
                if (xi + 1 < d && equaltoMarker(vox_idx + 1, 0))
                {
                    bfsMark(vox_idx + 1, marker);
                }

                //right
                if (xi - 1 >= 0 && equaltoMarker(vox_idx - 1, 0))
                {
                    bfsMark(vox_idx - 1, marker);
                }

                ++zi;
            }

            zi = d - 1;
            while (zi >= 0 && equaltoMarker(xi + yi*d + zi*d2, marker))
            {
                //up
                long long vox_idx = zi * (long long)d2 + yi * d + xi;
                if (yi + 1 < d && equaltoMarker(vox_idx + d, 0))
                {
                    bfsMark(vox_idx + d, marker);
                }

                //down
                if (yi - 1 >= 0 && equaltoMarker(vox_idx - d, 0))
                {
                    bfsMark(vox_idx - d, marker);
                }

                //left
                if (xi + 1 < d && equaltoMarker(vox_idx + 1, 0))
                {
                    bfsMark(vox_idx + 1, marker);
                }

                //right
                if (xi - 1 >= 0 && equaltoMarker(vox_idx - 1, 0))
                {
                    bfsMark(vox_idx - 1, marker);
                }

                --zi;
            }
        }
        //ui.progressBar->setValue(t * double(xi + 3*grid_density));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    // x-z planes.
    for (int xi = 0; xi<d; ++xi)
    {
        for (int zi = 0; zi<d; ++zi)
        {
            int yi = 0;
            while (yi < d && equaltoMarker(xi + yi*d + zi*d2, marker))
            {
                //up
                long long vox_idx = zi * (long long)d2 + yi * d + xi;
                if (zi + 1 < d && equaltoMarker(vox_idx + d2, 0))
                {
                    bfsMark(vox_idx + d2, marker);
                }

                //down
                if (zi - 1 >= 0 && equaltoMarker(vox_idx - d2, 0))
                {
                    bfsMark(vox_idx - d2, marker);
                }

                //left
                if (xi + 1 < d && equaltoMarker(vox_idx + 1, 0))
                {
                    bfsMark(vox_idx + 1, marker);
                }

                //right
                if (xi - 1 >= 0 && equaltoMarker(vox_idx - 1, 0))
                {
                    bfsMark(vox_idx - 1, marker);
                }

                ++yi;
            }

            yi = d - 1;
            while (yi >= 0 && equaltoMarker(xi + yi*d + zi*d2, marker))
            {
                //up
                long long vox_idx = zi * (long long)d2 + yi * d + xi;
                if (zi + 1 < d && equaltoMarker(vox_idx + d2, 0))
                {
                    bfsMark(vox_idx + d2, marker);
                }

                //down
                if (zi - 1 >= 0 && equaltoMarker(vox_idx - d2, 0))
                {
                    bfsMark(vox_idx - d2, marker);
                }

                //left
                if (xi + 1 < d && equaltoMarker(vox_idx + 1, 0))
                {
                    bfsMark(vox_idx + 1, marker);
                }

                //right
                if (xi - 1 >= 0 && equaltoMarker(vox_idx - 1, 0))
                {
                    bfsMark(vox_idx - 1, marker);
                }

                --yi;
            }
        }
        //ui.progressBar->setValue(t * double(xi + 4*grid_density));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    // y-z planes.
    for (int yi = 0; yi<d; ++yi)
    {
        for (int zi = 0; zi<d; ++zi)
        {
            int xi = 0;
            while (xi < d && equaltoMarker(xi + yi*d + zi*d2, marker))
            {
                //up
                long long vox_idx = zi * (long long)d2 + yi * d + xi;
                if (yi + 1 < d && equaltoMarker(vox_idx + d, 0))
                {
                    bfsMark(vox_idx + d, marker);
                }

                //down
                if (yi - 1 >= 0 && equaltoMarker(vox_idx - d, 0))
                {
                    bfsMark(vox_idx - d, marker);
                }

                //left
                if (zi + 1 < d && equaltoMarker(vox_idx + d2, 0))
                {
                    bfsMark(vox_idx + d2, marker);
                }

                //right
                if (zi - 1 >= 0 && equaltoMarker(vox_idx - d2, 0))
                {
                    bfsMark(vox_idx - d2, marker);
                }

                ++xi;
            }

            xi = d - 1;
            while (xi >= 0 && equaltoMarker(xi + yi*d + zi*d2, marker))
            {
                //up
                long long vox_idx = zi * (long long)d2 + yi * d + xi;
                if (yi + 1 < d && equaltoMarker(vox_idx + d, 0))
                {
                    bfsMark(vox_idx + d, marker);
                }

                //down
                if (yi - 1 >= 0 && equaltoMarker(vox_idx - d, 0))
                {
                    bfsMark(vox_idx - d, marker);
                }

                //left
                if (zi + 1 < d && equaltoMarker(vox_idx + d2, 0))
                {
                    bfsMark(vox_idx + d2, marker);
                }

                //right
                if (zi - 1 >= 0 && equaltoMarker(vox_idx - d2, 0))
                {
                    bfsMark(vox_idx - d2, marker);
                }

                --xi;
            }
        }
        //ui.progressBar->setValue(t * double(yi + 5*grid_density));

        // To keep GUI resposive.
        //QCoreApplication::processEvents();
    }

    //ui.progressBar->hide();
    //ui.progressMessage->hide();
}

//=================================================================
//BFS marking the exterior voxels.
//=================================================================
void VoxelMesh::bfsMark(long long idx, int marker)
{
    int d = grid_density;
    int d2 = d * d;

    stack<long long> vox_unvisit;

    markPoint(idx, marker);

    vox_unvisit.push(idx);

    while(!vox_unvisit.empty())
    {
        long long vox_idx = vox_unvisit.top();
        vox_unvisit.pop();

        int zi = vox_idx / d2;
        int yi = (vox_idx - zi * (long long)d2) / d;
        int xi = vox_idx - zi * (long long)d2 - yi * d;

        assert (zi * (long long)d2 + yi * d + xi == vox_idx);

        // up
        if (yi + 1 < d)
        {
            if (equaltoMarker(vox_idx + d, 0))
            {
                markPoint(vox_idx + d, marker);
                vox_unvisit.push(vox_idx + d);
            }
        }

        // down
        if ( yi - 1 >=0 )
        {
            if (equaltoMarker(vox_idx - d, 0))
            {
                markPoint(vox_idx - d, marker);
                vox_unvisit.push(vox_idx - d);
            }
        }

        // left
        if (xi - 1 >= 0)
        {
            if (equaltoMarker(vox_idx - 1, 0))
            {
                markPoint(vox_idx - 1, marker);
                vox_unvisit.push(vox_idx - 1);
            }
        }

        // right
        if (xi + 1 < d)
        {
            if (equaltoMarker(vox_idx + 1, 0))
            {
                markPoint(vox_idx + 1, marker);
                vox_unvisit.push(vox_idx + 1);
            }
        }

        // front
        if (zi + 1 < d)
        {
            if (equaltoMarker(vox_idx + d2, 0))
            {
                markPoint(vox_idx + d2, marker);
                vox_unvisit.push(vox_idx + d2);
            }
        }

        // back
        if (zi - 1 >= 0)
        {
            if (equaltoMarker(vox_idx - d2, 0))
            {
                markPoint(vox_idx - d2, marker);
                vox_unvisit.push(vox_idx - d2);
            }
        }
    } // end while
}

//===========================================================================
// returns 0 if any of the faces of the mesh bounding box are not marked
//===========================================================================
long long VoxelMesh::boundaryMarked()
{
    // simple check voxels on 6 surfaces of the bounding box
    int d = grid_density;
    long long d2 = d * d;
    long long vox_idx;

    // x-y planes, z = 0.
    for (int xi = 0; xi<d; ++xi)
    for (int yi = 0; yi<d; ++yi)
    {
        vox_idx = yi*d + xi;
        if (equaltoMarker(vox_idx, 0))
        {
            return vox_idx;
        }
    }

    // x-y planes, z = grid_density-1.
    for (int xi = 0; xi<d; ++xi)
    for (int yi = 0; yi<d; ++yi)
    {
        vox_idx = (d - 1) * d2 + yi*d + xi;
        if (equaltoMarker(vox_idx, 0))
        {
            return vox_idx;
        }
    }

    // z-y planes, x = 0.
    for (int zi = 0; zi<d; ++zi)
    for (int yi = 0; yi<d; ++yi)
    {
        vox_idx = zi * d2 + yi * d;
        if (equaltoMarker(vox_idx, 0))
        {
            return vox_idx;
        }
    }

    // z-y planes, x = grid_density - 1.
    for (int zi = 0; zi<d; ++zi)
    for (int yi = 0; yi<d; ++yi)
    {
        vox_idx = zi * d2 + yi * d + d - 1;
        if (equaltoMarker(vox_idx, 0))
        {
            return vox_idx;
        }
    }

    // x-z planes, y = 0.
    for (int xi = 0; xi<d; ++xi)
    for (int zi = 0; zi<d; ++zi)
    {
        vox_idx = zi * d2 + xi;
        if (equaltoMarker(vox_idx, 0))
        {
            return vox_idx;
        }
    }

    // x-z planes, y = grid_density - 1.
    for (int xi = 0; xi<d; ++xi)
    for (int zi = 0; zi<d; ++zi)
    {
        vox_idx = zi * d2 + (d - 1) * d + xi;
        if (equaltoMarker(vox_idx, 0))
        {
            return vox_idx;
        }
    }

    // all 6 bounding surfaces are marked.
    return -1;
}

//=======================================================================
// Given the 'voxel_map', this function generates or adds nodes to
// the global 'node_list'. A new sub cubic element list is generated
// and add to the 'mesh_list'.
//=======================================================================
void VoxelMesh::generateELEMesh(vector<Node*> *node_list_p,
                                vector<CubicElement*> *element_list_p)
{
//    // Build a new element list and add the 'mesh_list'.
//    vector<CubicElement*> elelist;
//    volume_mesh.element_group_lists.push_back(elelist);

//    vector<CubicElement*> *element_list_p = &volume_mesh.element_group_lists[0];
//    vector<Node*> *node_list_p = &volume_mesh.node_list;

    int d = grid_density;
    long long d2 = d * d;

    // Use two slices to store all the possible neighbors of one voxel.
    long long* idx_slice_0 = new long long[d2];
    long long* idx_slice_1 = new long long[d2];

    memset(idx_slice_0, 0, sizeof(long long) * d2);
    memset(idx_slice_1, 0, sizeof(long long) * d2);

    // the index counters for elements.
    long long idx_count = 0;
    long long idx_surface_count = 0;

//    num_ele = 0;

    // Go over the voxel map.
    for (int xi = 0; xi < d; ++xi)
    {
        for (int yi = 0; yi < d; ++yi)
            for (int zi = 0; zi < d; ++zi)
            {
                long long vox_idx = xi + yi*d + zi*d2;

                if (equaltoMarker(vox_idx, 1) ||
                    equaltoMarker(vox_idx, 2) )
                {
                    //cout << "xyz= " << xi <<" " << yi <<" " << zi << endl;
                    CubicElement* cub_ele = new CubicElement;
                    cub_ele->node0 = NULL;
                    cub_ele->node1 = NULL;
                    cub_ele->node2 = NULL;
                    cub_ele->node3 = NULL;
                    cub_ele->node4 = NULL;
                    cub_ele->node5 = NULL;
                    cub_ele->node6 = NULL;
                    cub_ele->node7 = NULL;

                    //Clear the neighbor pointers.
                    cub_ele->x_neg = NULL;
                    cub_ele->x_pos = NULL;
                    cub_ele->y_neg = NULL;
                    cub_ele->y_pos = NULL;
                    cub_ele->z_neg = NULL;
                    cub_ele->z_pos = NULL;

                    cub_ele->hitting_times = 0;//voxel_hitting_array[vox_idx];

                    //Check the neighbor sharing node 0, i.e.(zi-1, yi-1, xi-1)
                    if (zi - 1 >= 0 && yi - 1 >= 0 && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d2 - d - 1, 1) ||
                            equaltoMarker(vox_idx - d2 - d - 1, 2))
                        {
                            long long i = idx_slice_0[zi - 1 + yi*d - d];
                            cub_ele->node0 = element_list_p->at(i)->node7;
                        }
                    }

                    //Check the neighbor sharing node 0 and 2, i.e.(zi, yi-1, xi-1)
                    if (yi - 1 >= 0 && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d - 1, 1) ||
                            equaltoMarker(vox_idx - d - 1, 2))
                        {
                            long long i = idx_slice_0[zi + yi*d - d];
                            cub_ele->node0 = element_list_p->at(i)->node5;
                            cub_ele->node2 = element_list_p->at(i)->node7;
                        }
                    }

                    //Check the neighbor sharing node 2, i.e.(zi+1, yi-1, xi-1)
                    if (zi + 1 < d && yi - 1 >= 0 && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx + d2 - d - 1, 1) ||
                            equaltoMarker(vox_idx + d2 - d - 1, 2))
                        {
                            long long i = idx_slice_0[zi + 1 + yi*d - d];
                            cub_ele->node2 = element_list_p->at(i)->node5;
                        }
                    }

                    //Check the neighbor sharing node 0 and 4, i.e.(zi-1, yi, xi-1).
                    if (zi - 1 >= 0 && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d2 - 1, 1) ||
                            equaltoMarker(vox_idx - d2 - 1, 2))
                        {
                            long long i = idx_slice_0[zi - 1 + yi*d];
                            cub_ele->node4 = element_list_p->at(i)->node7;
                            cub_ele->node0 = element_list_p->at(i)->node3;
                        }
                    }

                    //Check the neighbor sharing node 0,2,4 and 6, i.e.(zi, yi, xi-1)
                    //This a face connected neighbor.
                    if (xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - 1, 1) ||
                            equaltoMarker(vox_idx - 1, 2) )
                        {
                            long long i = idx_slice_0[zi + yi*d];
                            cub_ele->node4 = element_list_p->at(i)->node5;
                            cub_ele->node0 = element_list_p->at(i)->node1;
                            cub_ele->node6 = element_list_p->at(i)->node7;
                            cub_ele->node2 = element_list_p->at(i)->node3;

                            //Link to this neighbor
                            cub_ele->x_neg = element_list_p->at(i);
                            element_list_p->at(i)->x_pos = cub_ele;
                        }
                    }

                    //Check the neighbor sharing node 2 and 6, i.e.(zi+1, yi, xi-1)
                    if (zi + 1 < d && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx + d2 - 1, 1) ||
                            equaltoMarker(vox_idx + d2 - 1, 2) )
                        {
                            long long i = idx_slice_0[zi + 1 + yi*d];
                            cub_ele->node6 = element_list_p->at(i)->node5;
                            cub_ele->node2 = element_list_p->at(i)->node1;
                        }
                    }

                    //Check the neighbor sharing node 4, i.e.(zi-1, yi+1, xi-1)
                    if (zi - 1 >=0 && yi + 1 <d && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d2 + d - 1, 1) ||
                            equaltoMarker(vox_idx - d2 + d - 1, 2) )
                        {
                            long long i = idx_slice_0[zi - 1 + yi*d + d];
                            cub_ele->node4 = element_list_p->at(i)->node3;
                        }
                    }

                    //Check the neighbor sharing node 4 and 6, i.e.(zi, yi+1, xi-1)
                    if (yi + 1 < d && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx + d - 1, 1) ||
                            equaltoMarker(vox_idx + d - 1, 2) )
                        {
                            long long i = idx_slice_0[zi + yi*d + d];
                            cub_ele->node4 = element_list_p->at(i)->node1;
                            cub_ele->node6 = element_list_p->at(i)->node3;
                        }
                    }

                    //Check the neighbor sharing node 6, i.e.(zi+1, yi+1, xi-1)
                    if (zi + 1 < d && yi + 1 < d && xi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx + d2 + d - 1, 1) ||
                            equaltoMarker(vox_idx + d2 + d - 1, 2) )
                        {
                            long long i = idx_slice_0[zi + 1 + yi*d + d];
                            cub_ele->node6 = element_list_p->at(i)->node1;
                        }
                    }

                    //Check the neighbor sharing node 0 and 1, i.e.(zi-1, yi-1, xi)
                    if (zi - 1 >= 0 && yi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d2 - d, 1) ||
                            equaltoMarker(vox_idx - d2 - d, 2) )
                        {
                            long long i = idx_slice_1[zi - 1 + yi*d - d];
                            cub_ele->node0 = element_list_p->at(i)->node6;
                            cub_ele->node1 = element_list_p->at(i)->node7;
                        }
                    }

                    //Check the neighbor sharing node 0,1,2 and 3, i.e.(zi, yi-1, xi)
                    //This a face connected neighbor.
                    if (yi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d, 1) ||
                            equaltoMarker(vox_idx - d, 2) )
                        {
                            long long i = idx_slice_1[zi + yi*d - d];
                            cub_ele->node0 = element_list_p->at(i)->node4;
                            cub_ele->node1 = element_list_p->at(i)->node5;
                            cub_ele->node2 = element_list_p->at(i)->node6;
                            cub_ele->node3 = element_list_p->at(i)->node7;

                            //Link to this neighbor
                            cub_ele->y_neg = element_list_p->at(i);
                            element_list_p->at(i)->y_pos = cub_ele;
                        }
                    }

                    //Check the neighbor sharing node 2 and 3, i.e.(zi+1, yi-1, xi)
                    if (zi + 1 < d && yi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx + d2 - d, 1) ||
                            equaltoMarker(vox_idx + d2 - d, 2) )
                        {
                            long long i = idx_slice_1[zi + 1 + yi*d - d];
                            cub_ele->node2 = element_list_p->at(i)->node4;
                            cub_ele->node3 = element_list_p->at(i)->node5;
                        }
                    }

                    //Check the neighbor sharing node 0,1,4 and 5, i.e.(zi-1, yi, xi)
                    //This a face connected neighbor.
                    if (zi - 1 >= 0)
                    {
                        if (equaltoMarker(vox_idx - d2, 1) ||
                            equaltoMarker(vox_idx - d2, 2) )
                        {
                            long long i = idx_slice_1[zi - 1 + yi*d];

                            cub_ele->node4 = element_list_p->at(i)->node6;
                            cub_ele->node5 = element_list_p->at(i)->node7;
                            cub_ele->node0 = element_list_p->at(i)->node2;
                            cub_ele->node1 = element_list_p->at(i)->node3;

                            //Link to this neighbor
                            cub_ele->z_neg = element_list_p->at(i);
                            element_list_p->at(i)->z_pos = cub_ele;
                        }
                    }

                    double x = (double)xi * voxel_dim + triangle_mesh->min_x;
                    double y = (double)yi * voxel_dim + triangle_mesh->min_y;
                    double z = (double)zi * voxel_dim + triangle_mesh->min_z;

                    // create new node if the node pointer is still NULL
                    if (!cub_ele->node0)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x;
                        pnode->ori_coordinate[1] = y;
                        pnode->ori_coordinate[2] = z;
                        cub_ele->node0 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node1)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x + voxel_dim;
                        pnode->ori_coordinate[1] = y;
                        pnode->ori_coordinate[2] = z;
                        cub_ele->node1 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node2)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x;
                        pnode->ori_coordinate[1] = y;
                        pnode->ori_coordinate[2] = z + voxel_dim;
                        cub_ele->node2 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node3)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x + voxel_dim;
                        pnode->ori_coordinate[1] = y;
                        pnode->ori_coordinate[2] = z + voxel_dim;
                        cub_ele->node3 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node4)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x;
                        pnode->ori_coordinate[1] = y + voxel_dim;
                        pnode->ori_coordinate[2] = z;
                        cub_ele->node4 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node5)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x + voxel_dim;
                        pnode->ori_coordinate[1] = y + voxel_dim;
                        pnode->ori_coordinate[2] = z;
                        cub_ele->node5 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node6)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x;
                        pnode->ori_coordinate[1] = y + voxel_dim;
                        pnode->ori_coordinate[2] = z + voxel_dim;
                        cub_ele->node6 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    if (!cub_ele->node7)
                    {
                        Node* pnode = new Node;
                        pnode->ori_coordinate[0] = x + voxel_dim;
                        pnode->ori_coordinate[1] = y + voxel_dim;
                        pnode->ori_coordinate[2] = z + voxel_dim;
                        cub_ele->node7 = pnode;
                        node_list_p->push_back(pnode);
                        pnode->index = node_list_p->size() - 1;

                        // Set as not suface node, ready for generate surface mesh.
                        pnode->isOnSurface = false;
                        pnode->idx_surface = -1;
                    }

                    // Set some properties of the element.
                    cub_ele->vox_idx = vox_idx;
                    cub_ele->index = idx_count++;
                    if (equaltoMarker(vox_idx, 1))
                    {
                        cub_ele->isOnSurfaceVoxelization = true;
                        cub_ele->index_surface = idx_surface_count++;

                        // set the eight nodes on surface voxelization.
                        cub_ele->node0->isOnSurfaceVoxelization = true;
                        cub_ele->node1->isOnSurfaceVoxelization = true;
                        cub_ele->node2->isOnSurfaceVoxelization = true;
                        cub_ele->node3->isOnSurfaceVoxelization = true;
                        cub_ele->node4->isOnSurfaceVoxelization = true;
                        cub_ele->node5->isOnSurfaceVoxelization = true;
                        cub_ele->node6->isOnSurfaceVoxelization = true;
                        cub_ele->node7->isOnSurfaceVoxelization = true;
                    }
                    else
                    {
                        cub_ele->isOnSurfaceVoxelization = false;

                        // set the eight nodes not on surface voxelization.
                        cub_ele->node0->isOnSurfaceVoxelization = false;
                        cub_ele->node1->isOnSurfaceVoxelization = false;
                        cub_ele->node2->isOnSurfaceVoxelization = false;
                        cub_ele->node3->isOnSurfaceVoxelization = false;
                        cub_ele->node4->isOnSurfaceVoxelization = false;
                        cub_ele->node5->isOnSurfaceVoxelization = false;
                        cub_ele->node6->isOnSurfaceVoxelization = false;
                        cub_ele->node7->isOnSurfaceVoxelization = false;
                    }

                    // Add the element to the list.
                    element_list_p->push_back(cub_ele);

                    // Record the element idx in slice 1.
                    idx_slice_1[zi + yi*d] = cub_ele->index;

                } // end if
            } // end for loops

        // move slice_1 to silce_0, then clear slice_1.
        memcpy(idx_slice_0, idx_slice_1, sizeof(long long) * d2);
        memset(idx_slice_1, 0, sizeof(long long) * d2);

    } // end of for loop of xi

    delete idx_slice_0;
    delete idx_slice_1;
}

//==========================================================================
// Given the volume mesh, this functions generates the corresponding
// surface meshes.
//==========================================================================
void VoxelMesh::buildOBJMeshList()
{
    // Determine the number of sub meshes.
    int mesh_size = volume_mesh.element_group_lists.size();
    volume_mesh.surface_mesh_lists.resize(mesh_size);

    vector<Node*> *node_list_p = &volume_mesh.surface_node_list;

    // Go over all sub element meshes.
    for (int mesh_i=0; mesh_i<mesh_size; ++mesh_i)
    {
        // Get the sub element and mesh list.
        vector<CubicElement*>* element_list_p = &volume_mesh.element_group_lists[mesh_i];

//        // Build a new quad face list for the current sub element mesh.
//        vector<QuadFace*> fflist;
//        volu_mesh.mesh_lists.push_back(fflist);

        vector<QuadFace*> *face_list_p = &volume_mesh.surface_mesh_lists[mesh_i];

        // Go through all the elements in the current sub ele mesh.
        //long long face_count = 0;
        long long node_idx = 0;
        long long size = element_list_p->size();
        for (long long k = 0; k < size; ++k)
        {
            // Get an element.
            CubicElement* p_ele = element_list_p->at(k);

            //Check if the face 0145 is on surface.
            if (!p_ele->z_neg)
            {
                //No neighbor is on z_neg, 0145 is on surface.
                Node *n0 = p_ele->node0;
                Node *n1 = p_ele->node1;
                Node *n4 = p_ele->node4;
                Node *n5 = p_ele->node5;

                //Update the 4 nodes as surface node and the idx on surface.
                if (!n0->isOnSurface)
                {
                    n0->isOnSurface = true;
                    n0->idx_surface = node_idx++;
                    node_list_p->push_back(n0);
                }

                if (!n1->isOnSurface)
                {
                    n1->isOnSurface = true;
                    n1->idx_surface = node_idx++;
                    node_list_p->push_back(n1);
                }

                if (!n4->isOnSurface)
                {
                    n4->isOnSurface = true;
                    n4->idx_surface = node_idx++;
                    node_list_p->push_back(n4);
                }

                if (!n5->isOnSurface)
                {
                    n5->isOnSurface = true;
                    n5->idx_surface = node_idx++;
                    node_list_p->push_back(n5);
                }

                // Add the face 1045 to the quad face list.
                QuadFace* nface = new QuadFace;
                nface->node0 = n1;
                nface->node1 = n0;
                nface->node2 = n4;
                nface->node3 = n5;
                nface->idx0 = n1->idx_surface;
                nface->idx1 = n0->idx_surface;
                nface->idx2 = n4->idx_surface;
                nface->idx3 = n5->idx_surface;
                face_list_p->push_back(nface);
                //face_count++;
            }

            //Check the face 2367.
            if (!p_ele->z_pos)
            {
                //No neighbor is on z_pos, 2367 is on surface.
                Node *n2 = p_ele->node2;
                Node *n3 = p_ele->node3;
                Node *n6 = p_ele->node6;
                Node *n7 = p_ele->node7;

                //Update the 4 nodes as surface node and the idx on surface.
                if (!n2->isOnSurface)
                {
                    n2->isOnSurface = true;
                    n2->idx_surface = node_idx++;
                    node_list_p->push_back(n2);
                }

                if (!n3->isOnSurface)
                {
                    n3->isOnSurface = true;
                    n3->idx_surface = node_idx++;
                    node_list_p->push_back(n3);
                }

                if (!n6->isOnSurface)
                {
                    n6->isOnSurface = true;
                    n6->idx_surface = node_idx++;
                    node_list_p->push_back(n6);
                }

                if (!n7->isOnSurface)
                {
                    n7->isOnSurface = true;
                    n7->idx_surface = node_idx++;
                    node_list_p->push_back(n7);
                }

                //face 3762.
                QuadFace* nface = new QuadFace;
                nface->node0 = n3;
                nface->node1 = n7;
                nface->node2 = n6;
                nface->node3 = n2;
                nface->idx0 = n3->idx_surface;
                nface->idx1 = n7->idx_surface;
                nface->idx2 = n6->idx_surface;
                nface->idx3 = n2->idx_surface;
                face_list_p->push_back(nface);
                //face_count++;
            }

            //Check the face 0123.
            if (!p_ele->y_neg)
            {
                //The neighbor vox is unmarked, 0123 is on surface.
                Node *n0 = p_ele->node0;
                Node *n1 = p_ele->node1;
                Node *n2 = p_ele->node2;
                Node *n3 = p_ele->node3;

                //Update the 4 nodes as surface node and the idx on surface.
                if (!n0->isOnSurface) {
                    n0->isOnSurface = true;
                    n0->idx_surface = node_idx++;
                    node_list_p->push_back(n0);
                }

                if (!n1->isOnSurface) {
                    n1->isOnSurface = true;
                    n1->idx_surface = node_idx++;
                    node_list_p->push_back(n1);
                }

                if (!n2->isOnSurface) {
                    n2->isOnSurface = true;
                    n2->idx_surface = node_idx++;
                    node_list_p->push_back(n2);
                }

                if (!n3->isOnSurface) {
                    n3->isOnSurface = true;
                    n3->idx_surface = node_idx++;
                    node_list_p->push_back(n3);
                }

                //face 0132.
                QuadFace* nface = new QuadFace;
                nface->node0 = n0;
                nface->node1 = n1;
                nface->node2 = n3;
                nface->node3 = n2;
                nface->idx0 = n0->idx_surface;
                nface->idx1 = n1->idx_surface;
                nface->idx2 = n3->idx_surface;
                nface->idx3 = n2->idx_surface;
                face_list_p->push_back(nface);
                //face_count++;
            }

            //Check the face 4567.
            if (!p_ele->y_pos)
            {
                //The neighbor vox is unmarked, 4567 is on surface.
                Node *n4 = p_ele->node4;
                Node *n5 = p_ele->node5;
                Node *n6 = p_ele->node6;
                Node *n7 = p_ele->node7;

                //Update the 4 nodes as surface node and the idx on surface.
                if (!n4->isOnSurface) {
                    n4->isOnSurface = true;
                    n4->idx_surface = node_idx++;
                    node_list_p->push_back(n4);
                }

                if (!n5->isOnSurface) {
                    n5->isOnSurface = true;
                    n5->idx_surface = node_idx++;
                    node_list_p->push_back(n5);
                }

                if (!n6->isOnSurface) {
                    n6->isOnSurface = true;
                    n6->idx_surface = node_idx++;
                    node_list_p->push_back(n6);
                }

                if (!n7->isOnSurface) {
                    n7->isOnSurface = true;
                    n7->idx_surface = node_idx++;
                    node_list_p->push_back(n7);
                }

                //face 4675.
                QuadFace* nface = new QuadFace;
                nface->node0 = n4;
                nface->node1 = n6;
                nface->node2 = n7;
                nface->node3 = n5;
                nface->idx0 = n4->idx_surface;
                nface->idx1 = n6->idx_surface;
                nface->idx2 = n7->idx_surface;
                nface->idx3 = n5->idx_surface;
                face_list_p->push_back(nface);
                //face_count++;
            }

            //Check the face 0246.
            if (!p_ele->x_neg)
            {
                //The neighbor vox is unmarked, 0246 is on surface.
                Node *n0 = p_ele->node0;
                Node *n2 = p_ele->node2;
                Node *n4 = p_ele->node4;
                Node *n6 = p_ele->node6;

                //Update the 4 nodes as surface node and the idx on surface.
                if (!n0->isOnSurface) {
                    n0->isOnSurface = true;
                    n0->idx_surface = node_idx++;
                    node_list_p->push_back(n0);
                }

                if (!n2->isOnSurface) {
                    n2->isOnSurface = true;
                    n2->idx_surface = node_idx++;
                    node_list_p->push_back(n2);
                }

                if (!n4->isOnSurface) {
                    n4->isOnSurface = true;
                    n4->idx_surface = node_idx++;
                    node_list_p->push_back(n4);
                }

                if (!n6->isOnSurface) {
                    n6->isOnSurface = true;
                    n6->idx_surface = node_idx++;
                    node_list_p->push_back(n6);
                }

                //face 0264.
                QuadFace* nface = new QuadFace;
                nface->node0 = n0;
                nface->node1 = n2;
                nface->node2 = n6;
                nface->node3 = n4;
                nface->idx0 = n0->idx_surface;
                nface->idx1 = n2->idx_surface;
                nface->idx2 = n6->idx_surface;
                nface->idx3 = n4->idx_surface;
                face_list_p->push_back(nface);
                //face_count++;
            }

            //Check the face 1357.
            if (!p_ele->x_pos)
            {
                //The neighbor vox is unmarked, 1357 is on surface.
                Node *n1 = p_ele->node1;
                Node *n3 = p_ele->node3;
                Node *n5 = p_ele->node5;
                Node *n7 = p_ele->node7;

                //Update the 4 nodes as surface node and the idx on surface.
                if (!n1->isOnSurface) {
                    n1->isOnSurface = true;
                    n1->idx_surface = node_idx++;
                    node_list_p->push_back(n1);
                }

                if (!n3->isOnSurface) {
                    n3->isOnSurface = true;
                    n3->idx_surface = node_idx++;
                    node_list_p->push_back(n3);
                }

                if (!n5->isOnSurface) {
                    n5->isOnSurface = true;
                    n5->idx_surface = node_idx++;
                    node_list_p->push_back(n5);
                }

                if (!n7->isOnSurface) {
                    n7->isOnSurface = true;
                    n7->idx_surface = node_idx++;
                    node_list_p->push_back(n7);
                }

                //face 1573.
                QuadFace* nface = new QuadFace;
                nface->node0 = n1;
                nface->node1 = n5;
                nface->node2 = n7;
                nface->node3 = n3;
                nface->idx0 = n1->idx_surface;
                nface->idx1 = n5->idx_surface;
                nface->idx2 = n7->idx_surface;
                nface->idx3 = n3->idx_surface;
                face_list_p->push_back(nface);
                //face_count++;
            }

        } //end of k element.

//        num_obj_quadface += face_list->size();
    }

//    num_obj_node = obj_node_list.size();
}

//================================================================================================
// This function computes the normalized coordinates for each node.
//================================================================================================
void VoxelMesh::computeNormalizedCoordinates()
{
    //Compute the normalized voxel mesh.
    double shift_x = 0.5* (triangle_mesh->max_x + triangle_mesh->min_x);
    double shift_y = 0.5* (triangle_mesh->max_y + triangle_mesh->min_y);
    double shift_z = 0.5* (triangle_mesh->max_z + triangle_mesh->min_z);
    double max_dim = triangle_mesh->max_dimension;

    int number_node = volume_mesh.node_list.size();
    for (int i=0; i<number_node; ++i)
    {
        Node* p_node = volume_mesh.node_list[i];
        p_node->norm_coordinate[0] = (p_node->ori_coordinate[0] - shift_x) / max_dim;
        p_node->norm_coordinate[1] = (p_node->ori_coordinate[1] - shift_y) / max_dim;
        p_node->norm_coordinate[2] = (p_node->ori_coordinate[2] - shift_z) / max_dim;
    }

    //Compute the normalized scanlines of the triangle mesh.
    int num_mesh = triangle_mesh->num_mesh;
    for (int i=0; i<num_mesh; ++i)
    {
        for (const_iter_ptriangleface fi = triangle_mesh->mesh_lists[i].begin();
             fi!=triangle_mesh->mesh_lists[i].end(); ++fi)
        {
            // the primary scanlines
            if ((*fi)->sequ_primscanline)
            {
                (*fi)->norm_sequ_primscanline = new vector<double*>;

                for (int j=0; j<(*fi)->sequ_primscanline->size(); ++j)
                {
                    double* pp = new double[6];
                    pp[0] = ((*fi)->sequ_primscanline->at(j)[0] - shift_x) / max_dim;
                    pp[1] = ((*fi)->sequ_primscanline->at(j)[1] - shift_y) / max_dim;
                    pp[2] = ((*fi)->sequ_primscanline->at(j)[2] - shift_z) / max_dim;

                    pp[3] = ((*fi)->sequ_primscanline->at(j)[3] - shift_x) / max_dim;
                    pp[4] = ((*fi)->sequ_primscanline->at(j)[4] - shift_y) / max_dim;
                    pp[5] = ((*fi)->sequ_primscanline->at(j)[5] - shift_z) / max_dim;

                    (*fi)->norm_sequ_primscanline->push_back(pp);
                }
            }

            // the scanlines within a slice.
            if ((*fi)->sequ_scanline)
            {
                (*fi)->norm_sequ_scanline = new vector<double*>;

                for (int j=0; j<(*fi)->sequ_scanline->size(); ++j)
                {
                    double* pp = new double[6];
                    pp[0] = ((*fi)->sequ_scanline->at(j)[0] - shift_x) / max_dim;
                    pp[1] = ((*fi)->sequ_scanline->at(j)[1] - shift_y) / max_dim;
                    pp[2] = ((*fi)->sequ_scanline->at(j)[2] - shift_z) / max_dim;

                    pp[3] = ((*fi)->sequ_scanline->at(j)[3] - shift_x) / max_dim;
                    pp[4] = ((*fi)->sequ_scanline->at(j)[4] - shift_y) / max_dim;
                    pp[5] = ((*fi)->sequ_scanline->at(j)[5] - shift_z) / max_dim;

                    (*fi)->norm_sequ_scanline->push_back(pp);
                }
            }
        }
    }
}

//================================================================================================
// Heavily used inline functions.
//================================================================================================
inline void VoxelMesh::computeVoxIdx(int voxidx[], const double pp[])
{
    voxidx[0] = floor((pp[0] - coordinate_min[0]) / voxel_dim);
    if (voxidx[0] < 0)
        voxidx[0] = 0;
    else if (voxidx[0] >= grid_density)
        voxidx[0] = grid_density - 1;

    voxidx[1] = floor((pp[1] - coordinate_min[1]) / voxel_dim);
    if (voxidx[1] < 0)
        voxidx[1] = 0;
    else if (voxidx[1] >= grid_density)
        voxidx[1] = grid_density - 1;

    voxidx[2] = floor((pp[2] - coordinate_min[2]) / voxel_dim);
    if (voxidx[2] < 0)
        voxidx[2] = 0;
    else if (voxidx[2] >= grid_density)
        voxidx[2] = grid_density - 1;
}

inline int VoxelMesh::computeVoxOneIdx(double coor, int axis_idx)
{

    int voxidx = floor((coor - coordinate_min[axis_idx]) / voxel_dim);

    if (voxidx < 0)
        voxidx = 0;
    else if (voxidx >= grid_density)
        voxidx = grid_density - 1;

    return voxidx;
}

inline int VoxelMesh::computeDomiAxis(const double pp0[3], const double pp1[3], const double pp2[3])
{
    // compute the norm, i.e. (p1p0)x(p2p0).
    double v1[3], v2[3];
    v1[0] = pp1[0] - pp0[0];
    v1[1] = pp1[1] - pp0[1];
    v1[2] = pp1[2] - pp0[2];

    v2[0] = pp2[0] - pp0[0];
    v2[1] = pp2[1] - pp0[1];
    v2[2] = pp2[2] - pp0[2];

    double norm[3];
    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

    // take the absolute value of the norm.
    if (norm[0] < 0.0)
        norm[0] = -norm[0];

    if (norm[1] < 0.0)
        norm[1] = -norm[1];

    if (norm[2] < 0.0)
        norm[2] = -norm[2];

    // find the dominant direction.
    int domi_direction = 0;
    if (norm[0] < norm[1])
    {
        if (norm[1] < norm[2])  // 0<1<2
        {
            domi_direction = 2;
        }
        else // 0<1, 2<=1
        {
            domi_direction = 1;
        }
    }
    else if (norm[0] < norm[2]) // 1<=0, 0<2
    {
        domi_direction = 2;
    }

    return domi_direction;
}

//inline void VoxelMesh::computeDomiAxis(int &domi_x, int &domi_y, int &domi_z,
//                                       const double pp0[3], const double pp1[3], const double pp2[3])
//{
//    // compute the norm, i.e. (p1p0)x(p2p0).
//    double v1[3], v2[3];
//    v1[0] = pp1[0] - pp0[0];
//    v1[1] = pp1[1] - pp0[1];
//    v1[2] = pp1[2] - pp0[2];

//    v2[0] = pp2[0] - pp0[0];
//    v2[1] = pp2[1] - pp0[1];
//    v2[2] = pp2[2] - pp0[2];

//    double norm[3];
//    norm[0] = v1[1]*v2[2] - v1[2]*v2[1];
//    norm[1] = v1[2]*v2[0] - v1[0]*v2[2];
//    norm[2] = v1[0]*v2[1] - v1[1]*v2[0];

//    // take the absolute value of the norm.
//    if (norm[0] < 0.0)
//        norm[0] = -norm[0];

//    if (norm[1] < 0.0)
//        norm[1] = -norm[1];

//    if (norm[2] < 0.0)
//        norm[2] = -norm[2];

//    // find the dominant direction.
//    if (norm[0] <= norm[1])
//    {
//        if (norm[1] <= norm[2])  // 0<=1<=2
//        {
//            domi_x = 0;
//            domi_y = 1;
//            domi_z = 2;
//        }
//        else if (norm[0] <= norm[2]) // 0<=2, 2<1
//        {
//            domi_x = 0;
//            domi_y = 2;
//            domi_z = 1;
//        }
//        else // 2<0, 0<=1
//        {
//            domi_x = 2;
//            domi_y = 0;
//            domi_z = 1;
//        }
//    }
//    else
//    {
//        if (norm[0] <= norm[2]) // 1<0, 0<=2
//        {
//            domi_x = 1;
//            domi_y = 0;
//            domi_z = 2;
//        }
//        else if (norm[1] < norm[2]) // 1<2, 2<0
//        {
//            domi_x = 1;
//            domi_y = 2;
//            domi_z = 0;
//        }
//        else // 2<=1, 1<0
//        {
//            domi_x = 2;
//            domi_y = 1;
//            domi_z = 0;
//        }
//    }

//    return;
//}

inline void VoxelMesh::sortVertByDomiAxis(double pp0[], double pp1[], double pp2[], int dd)
{
    double temp[3];

    if (pp0[dd] <= pp1[dd])
    {
        if (pp1[dd] <= pp2[dd])  //p0 <= p1 <= p2
        {
            ;
        }
        else
        {
            if (pp0[dd] <= pp2[dd])  //p0 <= p2 < p1
            {
                assignPoint(temp, pp1);
                assignPoint(pp1, pp2);
                assignPoint(pp2, temp);
            }
            else   //p2 < p0 <= p1
            {
                assignPoint(temp, pp2);
                assignPoint(pp2, pp1);
                assignPoint(pp1, pp0);
                assignPoint(pp0, temp);
            }
        }
    }
    else
    {
        if (pp1[dd] >= pp2[dd])  //p2 <= p1 < p0
        {
            assignPoint(temp, pp0);
            assignPoint(pp0, pp2);
            assignPoint(pp2, temp);
        }
        else
        {
            if (pp0[dd] <= pp2[dd])  //p1 < p0 <= p2
            {
                assignPoint(temp, pp0);
                assignPoint(pp0, pp1);
                assignPoint(pp1, temp);
            }
            else   //p1 < p2 < p0
            {
                assignPoint(temp, pp0);
                assignPoint(pp0, pp1);
                assignPoint(pp1, pp2);
                assignPoint(pp2, temp);
            }
        }
    }
}

// This function assign pp1 to pp0, i.e. pp0 = pp1.
inline void VoxelMesh::assignPoint(double pp0[], const double pp1[])
{
    pp0[0] = pp1[0];
    pp0[1] = pp1[1];
    pp0[2] = pp1[2];
}

// Voxelize a 2D line segment.
int VoxelMesh::mark2DLine(const double start_point[], const double end_point[],
                          int axis_x, int axis_y, int axis_z, int slice_idx, int marker,
                          int pixsequ[][3], int start_pix_idx)
{
    double mesh_min_x = coordinate_min[axis_x];
    double mesh_min_y = coordinate_min[axis_y];
    double voxel_length = voxel_dim;
    int d = grid_density;
    long long d2 = d*d;

    // Pixelize the start/end point.
    int start_pixel_x, start_pixel_y;
    int end_pixel_x, end_pixel_y;

    start_pixel_x = floor((start_point[axis_x] - mesh_min_x) / voxel_length);
    if (start_pixel_x < 0)
        start_pixel_x = 0;
    if (start_pixel_x >= d)
        start_pixel_x = d - 1;

    start_pixel_y = floor((start_point[axis_y] - mesh_min_y) / voxel_length);
    if (start_pixel_y < 0)
        start_pixel_y = 0;
    if (start_pixel_y >= d)
        start_pixel_y = d - 1;

    end_pixel_x = floor((end_point[axis_x] - mesh_min_x) / voxel_length);
    if (end_pixel_x < 0)
        end_pixel_x = 0;
    if (end_pixel_x >= d)
        end_pixel_x = d - 1;

    end_pixel_y = floor((end_point[axis_y] - mesh_min_y) / voxel_length);
    if (end_pixel_y < 0)
        end_pixel_y = 0;
    if (end_pixel_y >= d)
        end_pixel_y = d - 1;

    // count the size of the sequ.
    int moving_pix_idx = start_pix_idx;

    // If only one pixel, record it and return.
    int vox_idx[3];
    vox_idx[axis_z] = slice_idx;
    if (end_pixel_x == start_pixel_x && end_pixel_y == start_pixel_y)
    {
        vox_idx[axis_x] = start_pixel_x;
        vox_idx[axis_y] = start_pixel_y;
        markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

        pixsequ[moving_pix_idx][axis_x] = start_pixel_x;
        pixsequ[moving_pix_idx][axis_y] = start_pixel_y;
        moving_pix_idx++;

        // return the new size of the sequence.
        return moving_pix_idx;
    }

    // Compute l0, the distance between start point and 3 corresponding facets.
    int delta_x, delta_y;
    double t0_x, t0_y;

    if (start_pixel_x <= end_pixel_x)
    {
        delta_x = 1;
        t0_x = voxel_length * (start_pixel_x + 1) - (start_point[axis_x] - mesh_min_x);
        //l0_x = voxel_length - (start_x - mesh_min_x - start_pixel_x * voxel_length);
    }
    else
    {
        delta_x = -1;
        t0_x = (start_point[axis_x] - mesh_min_x) - voxel_length * start_pixel_x;
    }

    if (start_pixel_y <= end_pixel_y)
    {
        delta_y = 1;
        t0_y = voxel_length * (start_pixel_y + 1) - (start_point[axis_y] - mesh_min_y);
    }
    else
    {
        delta_y = -1;
        t0_y = (start_point[axis_y] - mesh_min_y) - voxel_length * start_pixel_y;
    }

    // Compute the step distance parameters.
    double t_x, t_y;
    double step_x, step_y;

    if (start_pixel_x != end_pixel_x)
    {
        double v_x = abs(end_point[axis_x] - start_point[axis_x]);
        t_x = t0_x / v_x;
        step_x = voxel_length / v_x;
    }
    else
    {
        t_x = 1e20;
        step_x = 0;
    }

    if (start_pixel_y != end_pixel_y)
    {
        double v_y = abs(end_point[axis_y] - start_point[axis_y]);
        t_y = t0_y / v_y;
        step_y = voxel_length / v_y;
    }
    else
    {
        t_y = 1e20;
        step_y = 0;
    }

    // The core loop.
    int moving_pixel_x = start_pixel_x;
    int moving_pixel_y = start_pixel_y;
    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        vox_idx[axis_x] = moving_pixel_x;
        vox_idx[axis_y] = moving_pixel_y;
        markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

        pixsequ[moving_pix_idx][axis_x] = moving_pixel_x;
        pixsequ[moving_pix_idx][axis_y] = moving_pixel_y;
        moving_pix_idx++;

        if (t_x <= t_y)
        {
            t_y = t_y - t_x;
            t_x = step_x;

            if (moving_pixel_x != end_pixel_x)
            {
                moving_pixel_x += delta_x;
            }
        }
        else
        {
            t_x = t_x - t_y;
            t_y = step_y;
            if (moving_pixel_y != end_pixel_y)
            {
                moving_pixel_y += delta_y;
            }
        }
    }

    vox_idx[axis_x] = end_pixel_x;
    vox_idx[axis_y] = end_pixel_y;
    markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

    pixsequ[moving_pix_idx][axis_x] = end_pixel_x;
    pixsequ[moving_pix_idx][axis_y] = end_pixel_y;
    moving_pix_idx++;

    return moving_pix_idx;
}


// Voxelize a 2D YZ line segment from start point to end point.
void VoxelMesh::mark2DLine(const double start_x, const double start_y, const double end_x, const double end_y,
                           int axis_x, int axis_y, int axis_z, int slice_idx, int marker,
                           vector<int*>* p_sequ_voxel)
{
    double mesh_min_x = coordinate_min[axis_x];
    double mesh_min_y = coordinate_min[axis_y];
    double voxel_length = voxel_dim;
    int d = grid_density;
    long long d2 = d*d;

    // Pixelize the start/end point.
    int start_pixel_x, start_pixel_y;
    int end_pixel_x, end_pixel_y;

    start_pixel_x = floor((start_x - mesh_min_x) / voxel_length);
    if (start_pixel_x < 0)
        start_pixel_x = 0;
    if (start_pixel_x >= d)
        start_pixel_x = d - 1;

    start_pixel_y = floor((start_y - mesh_min_y) / voxel_length);
    if (start_pixel_y < 0)
        start_pixel_y = 0;
    if (start_pixel_y >= d)
        start_pixel_y = d - 1;

    end_pixel_x = floor((end_x - mesh_min_x) / voxel_length);
    if (end_pixel_x < 0)
        end_pixel_x = 0;
    if (end_pixel_x >= d)
        end_pixel_x = d - 1;

    end_pixel_y = floor((end_y - mesh_min_y) / voxel_length);
    if (end_pixel_y < 0)
        end_pixel_y = 0;
    if (end_pixel_y >= d)
        end_pixel_y = d - 1;

    // If only one pixel, record it and return.
    int vox_idx[3];
    vox_idx[axis_z] = slice_idx;
    if (end_pixel_x == start_pixel_x && end_pixel_y == start_pixel_y)
    {
        vox_idx[axis_x] = start_pixel_x;
        vox_idx[axis_y] = start_pixel_y;
        markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

        int *p_voxel = new int[3];
        p_voxel[axis_x] = start_pixel_x;
        p_voxel[axis_y] = start_pixel_y;
        p_voxel[axis_z] = slice_idx;
        p_sequ_voxel->push_back(p_voxel);

        return;
    }

    // Compute l0, the distance between start point and 3 corresponding facets.
    int delta_x, delta_y;
    double t0_x, t0_y;

    if (start_pixel_x <= end_pixel_x)
    {
        delta_x = 1;
        t0_x = voxel_length * (start_pixel_x + 1) - (start_x - mesh_min_x);
        //l0_x = voxel_length - (start_x - mesh_min_x - start_pixel_x * voxel_length);
    }
    else
    {
        delta_x = -1;
        t0_x = (start_x - mesh_min_x) - voxel_length * start_pixel_x;
    }

    if (start_pixel_y <= end_pixel_y)
    {
        delta_y = 1;
        t0_y = voxel_length * (start_pixel_y + 1) - (start_y - mesh_min_y);
    }
    else
    {
        delta_y = -1;
        t0_y = (start_y - mesh_min_y) - voxel_length * start_pixel_y;
    }

    // Compute the step distance parameters.
    double t_x, t_y;
    double step_x, step_y;

    if (start_pixel_x != end_pixel_x)
    {
        double v_x = abs(end_x - start_x);
        t_x = t0_x / v_x;
        step_x = voxel_length / v_x;
    }
    else
    {
        t_x = 1e20;
        step_x = 0;
    }

    if (start_pixel_y != end_pixel_y)
    {
        double v_y = abs(end_y - start_y);
        t_y = t0_y / v_y;
        step_y = voxel_length / v_y;
    }
    else
    {
        t_y = 1e20;
        step_y = 0;
    }

    // The core loop.
    int moving_pixel_x = start_pixel_x;
    int moving_pixel_y = start_pixel_y;
    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        vox_idx[axis_x] = moving_pixel_x;
        vox_idx[axis_y] = moving_pixel_y;
        markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

        int *p_voxel = new int[3];
        p_voxel[axis_x] = moving_pixel_x;
        p_voxel[axis_y] = moving_pixel_y;
        p_voxel[axis_z] = slice_idx;
        p_sequ_voxel->push_back(p_voxel);

        if (t_x <= t_y)
        {
            t_y = t_y - t_x;
            t_x = step_x;

//            if (moving_pixel_x != end_pixel_x)
            {
                moving_pixel_x += delta_x;
            }
        }
        else
        {
            t_x = t_x - t_y;
            t_y = step_y;
//            if (moving_pixel_y != end_pixel_y)
            {
                moving_pixel_y += delta_y;
            }
        }
    }

    vox_idx[axis_x] = end_pixel_x;
    vox_idx[axis_y] = end_pixel_y;
    markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

    int *p_voxel = new int[3];
    p_voxel[axis_x] = end_pixel_x;
    p_voxel[axis_y] = end_pixel_y;
    p_voxel[axis_z] = slice_idx;
    p_sequ_voxel->push_back(p_voxel);
}


// Voxelize a 2D central line from start pixel to end pixel.
void VoxelMesh::mark2DLineInteger(const int start_pixel_x, const int start_pixel_y,
                                  const int end_pixel_x, const int end_pixel_y,
                                  int axis_x, int axis_y, int axis_z, int slice_idx, int marker,
                                  vector<int*>* p_sequ_voxel)
{
    // Compute the step distance parameters.
    int delta_x, delta_y, v_x, v_y;
    if (start_pixel_x <= end_pixel_x)
    {
        delta_x = 1;
        v_x = end_pixel_x - start_pixel_x;
    }
    else
    {
        delta_x = -1;
        v_x = start_pixel_x - end_pixel_x;
    }

    if (start_pixel_y <= end_pixel_y)
    {
        delta_y = 1;
        v_y = end_pixel_y - start_pixel_y;
    }
    else
    {
        delta_y = -1;
        v_y = start_pixel_y - end_pixel_y;
    }

    // Compute the step distance parameters.
    int d = grid_density;
    long long d2 = d*d;
    int t_x, t_y, step_x, step_y, vox_idx[3];
    vox_idx[axis_z] = slice_idx;
    if (v_x != 0)
    {
        if (v_y != 0)
        {
            // v_x != 0, v_y != 0.
            t_x = v_y;
            step_x = v_y << 1;  // step_x = v_y*2

            t_y = v_x;
            step_y = v_x << 1;  // step_y = v_x*2
        }
        else
        {
            // v_x != 0, v_y == 0.
            t_x = 1;
            step_x = 2;
            t_y = 1000000000;
        }
    }
    else
    {
        if (v_y != 0)
        {
            // v_x == 0, v_y != 0.
            t_y = 1;
            step_y = 2;
            t_x = 1000000000;
        }
        else
        {
            // v_x == 0, v_y == 0.
            vox_idx[axis_x] = start_pixel_x;
            vox_idx[axis_y] = start_pixel_y;
            markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

            int *p_voxel = new int[3];
            p_voxel[axis_x] = start_pixel_x;
            p_voxel[axis_y] = start_pixel_y;
            p_voxel[axis_z] = slice_idx;
            p_sequ_voxel->push_back(p_voxel);

            return;
        }
    }

    // The core loop.
    int moving_pixel_x = start_pixel_x;
    int moving_pixel_y = start_pixel_y;
    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        vox_idx[axis_x] = moving_pixel_x;
        vox_idx[axis_y] = moving_pixel_y;
        markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

        int *p_voxel = new int[3];
        p_voxel[axis_x] = moving_pixel_x;
        p_voxel[axis_y] = moving_pixel_y;
        p_voxel[axis_z] = slice_idx;
        p_sequ_voxel->push_back(p_voxel);

//        if (t_x == t_y)
//        {
//            vox_idx[axis_x] = moving_pixel_x+delta_x;
//            vox_idx[axis_y] = moving_pixel_y;
//            markPoint(vox_idx, marker);

//            vox_idx[axis_x] = moving_pixel_x;
//            vox_idx[axis_y] = moving_pixel_y+delta_y;
//            markPoint(vox_idx, marker);

//            cout << "tx == ty \n";
//        }

        if (t_x <= t_y)
        {
            t_y = t_y - t_x;
            t_x = step_x;

            if (moving_pixel_x != end_pixel_x)
            {
                moving_pixel_x += delta_x;
            }
        }
        else
        {
            t_x = t_x - t_y;
            t_y = step_y;
            if (moving_pixel_y != end_pixel_y)
            {
                moving_pixel_y += delta_y;
            }
        }
    }

    // mark the last pixel
    vox_idx[axis_x] = end_pixel_x;
    vox_idx[axis_y] = end_pixel_y;
    markPoint(vox_idx[0] + vox_idx[1]*d + vox_idx[2]*d2, marker);

    int *p_voxel = new int[3];
    p_voxel[axis_x] = end_pixel_x;
    p_voxel[axis_y] = end_pixel_y;
    p_voxel[axis_z] = slice_idx;
    p_sequ_voxel->push_back(p_voxel);
}

//================================================================================
// Mark one voxel or test if it is marked.
//================================================================================
inline bool VoxelMesh::equaltoMarkerByteMap(const long long vox_idx, int marker)
{
    if (voxel_map[vox_idx] == marker)
    {
        return true;
    }
    return false;
}

inline bool VoxelMesh::equaltoMarker(const long long vox_idx, int marker)
{
//    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
//    unsigned char mask0 = 3 << offset;
//    unsigned char mask1 = marker << offset;
//    return !((voxel_map[vox_idx >> 2] & mask0) ^ mask1);

    unsigned char target_bits = voxel_map[vox_idx >> 2] >> ((vox_idx & 3) << 1);
    return !((target_bits ^ marker) & 3);  // only the last two bits matter.
}

inline bool VoxelMesh::equaltoMarkerOneBitMap(const long long vox_idx, int marker)
{
//    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
//    unsigned char mask0 = 3 << offset;
//    unsigned char mask1 = marker << offset;
//    return !((voxel_map[vox_idx >> 2] & mask0) ^ mask1);
      unsigned char target_bit = (voxel_map[vox_idx >> 3] >> (vox_idx & 7)) & 1;
      return !(target_bit ^ (marker & 1));  // only the last bit matters.
}

inline void VoxelMesh::markPointByteMap(const long long vox_idx, int marker)
{
    //clear and set the appropriate bits.
//    if (vox_idx >= grid_density*grid_density*(long long)grid_density || vox_idx < 0)
//    {
//        cout << "ERROR in 'markpoint'! " << vox_idx << endl;
//        return;
//    }
    voxel_map[vox_idx] == marker;
}

inline void VoxelMesh::markPoint(const long long vox_idx, int marker)
{
    //clear and set the appropriate bits.
//    if (vox_idx >= grid_density*grid_density*(long long)grid_density || vox_idx < 0)
//    {
//        cout << "ERROR in 'markpoint'! " << vox_idx << endl;
//        return;
//    }
//    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
//    unsigned char mask0 = ~(3 << offset);
//    unsigned char mask1 = marker << offset;
//    voxel_map[vox_idx >> 2] = (voxel_map[vox_idx >> 2] & mask0) | mask1;

    unsigned char offset = (vox_idx & 0x3) << 1;
    voxel_map[vox_idx >> 2] = (voxel_map[vox_idx >> 2] & ~(3 << offset)) |
                              ((marker & 3) << offset);
}

inline void VoxelMesh::markPointOneBitMap(const long long vox_idx, int marker)
{
    //clear and set the appropriate bits.
//    if (vox_idx >= grid_density*grid_density*(long long)grid_density || vox_idx < 0)
//    {
//        cout << "ERROR in 'markpoint'! " << vox_idx << endl;
//        return;
//    }
//    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
//    unsigned char mask0 = ~(3 << offset);
//    unsigned char mask1 = marker << offset;
//    voxel_map[vox_idx >> 2] = (voxel_map[vox_idx >> 2] & mask0) | mask1;
    unsigned char offset = vox_idx & 0x7;
    voxel_map[vox_idx >> 3] = (voxel_map[vox_idx >> 3] & ~(1 << offset)) |
                              ((marker & 1) << offset);
}

