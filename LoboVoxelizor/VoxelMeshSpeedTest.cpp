#include "VoxelMeshSpeedTest.h"

VoxelMeshSpeedTest::VoxelMeshSpeedTest()
{

}

//===============================================================
// For speed test of the surface voxelization.
// 0: center scanlines (markFaceByScanlineInteger)
// 1: parallel scanlines (markFaceByScanline)
// 2: SAT-based
//===============================================================
void VoxelMeshSpeedTest::computeVoxelMeshSpeed(int method_idx)
{
    // return if no triangle mesh or invalid grid density.
    if (!triangle_mesh)
    {
        cout << "Error: No triangle mesh!\n";
        return;
    }
    if (grid_density <= 0)
    {
        cout << "Error: Invalid grid density!\n";
        return;
    }

    // Compute the length of the edge of a voxel.
    voxel_dim = triangle_mesh->max_dimension / double(grid_density);

    // Clear the node/element lists.
    cleanAll();

    //--------------------------------------------------------------------------
    //Allocate memory for the voxel map.
    //--------------------------------------------------------------------------
    //The voxel map is a unsigned char array. One byte for one voxel.
    long long sizearray = grid_density * grid_density *(long long)grid_density;
    long long voxel_map_size = sizearray;

    // The map should be big, try memory allocation.
    cout << "Grid density = " << grid_density << endl;
    try
    {
        voxel_map = new unsigned char[voxel_map_size];
    }
    catch(...)
    {
        cout << "Initializing the voxel map of " << voxel_map_size << " bytes failed!" << endl;
        return;
    }
    cout << "Initializing the voxel map of " << voxel_map_size << " bytes succeeded." << endl;

    // Clear the map.
    memset(voxel_map, 0, sizeof(unsigned char) * voxel_map_size);

    //-----------------------------------------------------------------------
    // Surface voxelization by 1000 times.
    //-----------------------------------------------------------------------
    time_surface = 0.0;

    QElapsedTimer timer;
    switch (method_idx) {
    case 0:
        //-----------------------------------------------------------------------
        // Use center scanlines to voxelize.
        //-----------------------------------------------------------------------
        timer.start();
        for (int k=0; k<1000; k++)
        {
            for (int i=0; i<triangle_mesh->num_mesh; ++i)
            {
                for (const_iter_ptriangleface fi=triangle_mesh->mesh_list.at(i)->begin();
                     fi!=triangle_mesh->mesh_list.at(i)->end(); ++fi)
                {
                    markFaceByScanlineIntegerSpeed(*fi);
                }
            }
        }
        time_surface = timer.nsecsElapsed();
        break;

    case 1:
        //-----------------------------------------------------------------------
        // Use parallel scanlines to voxelize.
        //-----------------------------------------------------------------------
        timer.start();
        for (int k=0; k<1000; k++)
        {
            for (int i=0; i<triangle_mesh->num_mesh; ++i)
            {
                for (const_iter_ptriangleface fi=triangle_mesh->mesh_list.at(i)->begin();
                     fi!=triangle_mesh->mesh_list.at(i)->end(); ++fi)
                {
                    markFaceByScanlineSpeed(*fi);
                }
            }
        }
        time_surface = timer.nsecsElapsed();
        break;

    case 2:
        //-----------------------------------------------------------------------
        // Use SAT-based to voxelize the surface of the sub mesh.
        //-----------------------------------------------------------------------
        timer.start();
        for (int k=0; k<1000; k++)
        {
            for (int i=0; i<triangle_mesh->num_mesh; ++i)
            {
                for (const_iter_ptriangleface fi=triangle_mesh->mesh_list.at(i)->begin();
                     fi!=triangle_mesh->mesh_list.at(i)->end(); ++fi)
                {
                    markFaceBySATMethodSpeed(*fi);
                }
            }
        }
        time_surface = timer.nsecsElapsed();
        break;

    default:
        break;
    }

    // Set the unit to 'us' and average by 1000 times.
    time_surface = time_surface * 0.001 * 0.001;

    //-----------------------------------------------------------------------
    // Interior voxelization.
    //-----------------------------------------------------------------------
//    time_interior = 0.0;
//    if (isFillingHole)
//    {
//        QElapsedTimer timer;
//        timer.start();
//        //fillHole(3);
//        time_interior = timer.nsecsElapsed();
//    }
//    time_interior = time_interior * 0.001;

    //-----------------------------------------------------------------------
    // Statistic data.
    //-----------------------------------------------------------------------
    num_ele_surface = 0;
    for (int i=0; i<voxel_map_size; ++i)
    {
        if (voxel_map[i] == 1)
        {
            num_ele_surface++;
        }
    }
}

void VoxelMeshSpeedTest::markFaceBySATMethodSpeed(TriangleFace* facep)
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

                op_op++;

                // inner product
                op_mul += 3;
                op_add += 2;
                double np = norm[0]*corner_min[0] +
                            norm[1]*corner_min[1] +
                            norm[2]*corner_min[2];

                op_mul += 1;
                op_add += 2;
                op_cmp += 1;
                if ((np + d1) * (np + d2) <= 0.0)
                {
                    bool flag_overlap = true;

                    for (int i=0; i<3; i++)
                    {
                        //for projection on xy plane.
                        op_mul += 2;
                        op_add += 2;
                        op_cmp += 1;
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
                        op_mul += 2;
                        op_add += 2;
                        op_cmp += 1;
                        temp = d_yz_edge[i]
                               + n_yz_edge[i][0] * corner_min[1]
                               + n_yz_edge[i][1] * corner_min[2];
                        if (temp < 0.0)
                        {
                            flag_overlap = false;
                            break;
                        }

                        //for projection on zx plane.
                        op_mul += 2;
                        op_add += 2;
                        op_cmp += 1;
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
                        voxel_map[xi + (yi + zi*grid_density)*(long long)grid_density] = 1;
                    }
                } // endf if
            }
        }
    }
}

void VoxelMeshSpeedTest::markFaceByScanlineIntegerSpeed(TriangleFace* facep)
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
        //markPoint(p0_vox[0] + p0_vox[1]*grid_den + p0_vox[2]*grid_den*(long long)grid_den, marker);
        voxel_map[p0_vox[0] + (p0_vox[1] + p0_vox[2]*grid_den) * (long long)grid_den] = 1;
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
                //markPoint(moving_vox_idx, marker);
                voxel_map[moving_vox_idx] = 1;

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
            //markPoint(moving_vox_idx, marker);
            voxel_map[moving_vox_idx] = 1;

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
                //markPoint(moving_vox_idx, marker);
                voxel_map[moving_vox_idx] = 1;

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
            //markPoint(moving_vox_idx, marker);
            voxel_map[moving_vox_idx] = 1;

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

void VoxelMeshSpeedTest::markFaceByScanlineSpeed(TriangleFace* facep)
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
        //markPoint(p0_vox[0] + p0_vox[1]*grid_density + p0_vox[2]*grid_density*(long long)grid_density, marker);
        voxel_map[p0_vox[0] + (p0_vox[1] + p0_vox[2]*grid_density) * (long long)grid_density] = 1;
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
        axis_x = 0;
        axis_y = 2;
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
    double length;
    while  (plane < p1[axis_z])
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

    while (plane < p2[axis_z])
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

int VoxelMeshSpeedTest::mark3DLineSpeed(const double start_point[], const double end_point[],
                                  const int start_point_vox[], const int end_point_vox[],
                                  int voxsequ[][3])
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
        //markPoint(moving_vox_idx, marker);
        voxel_map[moving_vox_idx] = 1;

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
    //markPoint(moving_vox_idx, marker);
    voxel_map[moving_vox_idx] = 1;

    voxsequ[voxsequ_size][0] = end_point_vox[0];
    voxsequ[voxsequ_size][1] = end_point_vox[1];
    voxsequ[voxsequ_size][2] = end_point_vox[2];
    voxsequ_size++;

    return voxsequ_size;
}

void VoxelMeshSpeedTest::mark3DLineSpeed(const double start_point[], const double end_point[],
                                   const int start_point_vox[], const int end_point_vox[])
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
        //markPoint(moving_vox_idx, marker);
        voxel_map[moving_vox_idx] = 1;

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

    //markPoint(moving_vox_idx, marker);
    voxel_map[moving_vox_idx] = 1;
}

void VoxelMeshSpeedTest::mark2DLineSpeed(const double start_x, const double start_y,
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
    int vox_idx[3];
    if (end_pixel_x == start_pixel_x && end_pixel_y == start_pixel_y)
    {
        vox_idx[axis_z] = slice_idx;
        vox_idx[axis_x] = start_pixel_x;
        vox_idx[axis_y] = start_pixel_y;
        //markPoint(vox_idx[0] + vox_idx[1]*grid_density + vox_idx[2]*grid_density*(long long)grid_density, marker);
        voxel_map[vox_idx[0] + (vox_idx[1] + vox_idx[2]*grid_density) * (long long)grid_density] = 1;
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
    long long moving_vox_idx = vox_idx[0] + (vox_idx[1] + vox_idx[2]*grid_density) * (long long)grid_density;

    while (!(moving_pixel_x == end_pixel_x && moving_pixel_y == end_pixel_y))
    {
        //markPoint(moving_vox_idx, marker);
        voxel_map[moving_vox_idx] = 1;

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
    //markPoint(moving_vox_idx, marker);
    voxel_map[moving_vox_idx] = 1;
}

inline void VoxelMeshSpeedTest::computeVoxIdx(int voxidx[], const double pp[])
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

inline int VoxelMeshSpeedTest::computeVoxOneIdx(double coor, int axis_idx)
{

    int voxidx = floor((coor - coordinate_min[axis_idx]) / voxel_dim);

    if (voxidx < 0)
        voxidx = 0;
    else if (voxidx >= grid_density)
        voxidx = grid_density - 1;

    return voxidx;
}

inline int VoxelMeshSpeedTest::computeDomiAxis(const double pp0[3], const double pp1[3], const double pp2[3])
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

double VoxelMeshSpeedTest::computeTriangleArea(double pp0[], double pp1[], double pp2[])
{
    double v1[3], v2[3], v3[3];
    v1[0] = pp1[0] - pp0[0];
    v1[1] = pp1[1] - pp0[1];
    v1[2] = pp1[2] - pp0[2];

    v2[0] = pp2[0] - pp0[0];
    v2[1] = pp2[1] - pp0[1];
    v2[2] = pp2[2] - pp0[2];

    v3[0] = pp2[0] - pp1[0];
    v3[1] = pp2[1] - pp1[1];
    v3[2] = pp2[2] - pp1[2];

    double ll1 = sqrt(v1[0]*v1[0]+v1[1]*v1[1]+v1[2]*v1[2]);
    double ll2 = sqrt(v2[0]*v2[0]+v2[1]*v2[1]+v2[2]*v2[2]);
    double ll3 = sqrt(v3[0]*v3[0]+v3[1]*v3[1]+v3[2]*v3[2]);
    double ss = 0.5 * (ll1+ll2+ll3);

    return sqrt(ss*(ss-ll1)*(ss-ll2)*(ss-ll3));    ;
}

inline void VoxelMeshSpeedTest::sortVertByDomiAxis(double pp0[], double pp1[], double pp2[], int dd)
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
inline void VoxelMeshSpeedTest::assignPoint(double pp0[], const double pp1[])
{
    pp0[0] = pp1[0];
    pp0[1] = pp1[1];
    pp0[2] = pp1[2];
}

//Mark a voxel with 1 index.
inline void VoxelMeshSpeedTest::markPoint(const long long vox_idx, int marker)
{
    //clear and set the appropriate bits.
//#ifdef HIGH_GRID_DENSITY
//    unsigned char mask = marker << (7 - (vox_idx & 0x7));
//    voxel_map[vox_idx >> 3] = voxel_map[vox_idx >> 3] | mask;
//#else
    unsigned char offset = (3 - (vox_idx & 0x3)) << 1;
    unsigned char mask0 = ~(3 << offset);
    unsigned char mask1 = marker << offset;
    voxel_map[vox_idx >> 2] = (voxel_map[vox_idx >> 2] & mask0) | mask1;
//#endif
}

// record the index of the voxel.
void VoxelMeshSpeedTest::recordPoint(const int xx, const int yy, const int zz, vector<long long>* p_scan)
{
    long long vox_idx = xx + (yy + zz*grid_density) * (long long)grid_density;
    p_scan->push_back(vox_idx);
}
