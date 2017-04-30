#include "voxelmeshspeed.h"

VoxelMeshSpeed::VoxelMeshSpeed(TriangleMesh* p_mesh, int grid_dens, int method_)
    : VoxelMesh(p_mesh, grid_dens, method_)
{

}

bool VoxelMeshSpeed::updateGridDensity(int new_density, int new_method)
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

void VoxelMeshSpeed::computeVoxelMesh()
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

    // the speed test takes average of many times of voxelization for surface only.
    long long sizearray = grid_density * grid_density *(long long)grid_density;
    voxel_map_size = (sizearray>>3) + 1;
    //voxel_map_size = sizearray;
    allocateVoxelMap();

    // determine the repeat times for testing by the grid dentsity.
    // around 2000 at 128, 1000 at 512, 1 at above 2048
    int test_times = 5500.0 - 500.0 * log2(grid_density);
    if (test_times <= 0)
    {
        test_times = 1;
    }
    cout << "Speed test times " << test_times << endl;

    if (isMultiThread)
    {
        time_surface_speedtest = 0.0;
        QElapsedTimer timer;
        switch (voxelization_method)
        {
        case 0:
        {
            // Use SAT-based to voxelize the surface of the sub mesh.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListSATPan11();

                th[0] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedSAT, this, 0);
                th[1] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedSAT, this, 1);
                th[2] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedSAT, this, 2);
                th[3] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedSAT, this, 3);

                for (int t=0; t<4; ++t)
                {
                    if (th[t].joinable())
                    {
                        th[t].join();
                    }
                }

                //                int size = sorted_triangle_list.size();
                //                for (int i=0; i<size; ++i)
                //                {
                //                    markSortedFaceSATSpeed(sorted_triangle_list[i], 0);
                //                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        case 3:
        {
            // Use SAT-based method Pan11 to voxelize the surface of the sub mesh.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListSATPan11();
                th[0] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedPan11, this, 0);
                th[1] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedPan11, this, 1);
                th[2] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedPan11, this, 2);
                th[3] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedPan11, this, 3);

                for (int t=0; t<4; ++t)
                {
                    if (th[t].joinable())
                    {
                        th[t].join();
                    }
                }

                //                int size = sorted_triangle_list.size();
                //                for (int i=0; i<size; ++i)
                //                {
                //                    markSortedFacePan11Speed(sorted_triangle_list[i]);
                //                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        case 1:
        {
            // Use parallel scanlines to voxelize.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListFLTINT();

                th[0] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedFLT, this, 0);
                th[1] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedFLT, this, 1);
                th[2] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedFLT, this, 2);
                th[3] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedFLT, this, 3);

                for (int t=0; t<4; ++t)
                {
                    if (th[t].joinable())
                    {
                        th[t].join();
                    }
                }

                //                int size = sorted_triangle_list.size();
                //                for (int i=0; i<size; ++i)
                //                {
                //                    markSortedFaceFLTSpeed(sorted_triangle_list[i]);
                //                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        case 2:
        {
            // Use center scanlines to voxelize.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListFLTINT();

                th[0] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedINT, this, 0);
                th[1] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedINT, this, 1);
                th[2] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedINT, this, 2);
                th[3] = thread(&VoxelMesh::voxelizeOneFourthSortedTrianglesSpeedINT, this, 3);

                for (int t=0; t<4; ++t)
                {
                    if (th[t].joinable())
                    {
                        th[t].join();
                    }
                }

                //                int size = sorted_triangle_list.size();
                //                for (int i=0; i<size; ++i)
                //                {
                //                    markSortedFaceINTSpeed(sorted_triangle_list[i]);
                //                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        default:
            break;
        }

        // Set the unit to 'us' and average by test times.
        time_surface_speedtest *= 0.001 / double(test_times);
    }
    else
    {
        time_surface_speedtest = 0.0;
        QElapsedTimer timer;
        switch (voxelization_method)
        {
        case 0:
        {
            // Use SAT-based to voxelize the surface of the sub mesh.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListSATPan11();
                int size = sorted_triangle_list.size();
                for (int i=0; i<size; ++i)
                {
                    markSortedFaceSATSpeed(sorted_triangle_list[i]);
                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        case 3:
        {
            // Use SAT-based method Pan11 to voxelize the surface of the sub mesh.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListSATPan11();
                int size = sorted_triangle_list.size();
                for (int i=0; i<size; ++i)
                {
                    markSortedFacePan11Speed(sorted_triangle_list[i]);
                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        case 1:
        {
            // Use parallel scanlines to voxelize.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListFLTINT();
                int size = sorted_triangle_list.size();
                for (int i=0; i<size; ++i)
                {
                    markSortedFaceFLTSpeed(sorted_triangle_list[i]);
                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        case 2:
        {
            // Use center scanlines to voxelize.
            timer.start();
            for (int k=0; k<test_times; k++)
            {
                setupSortedTriangleListFLTINT();
                int size = sorted_triangle_list.size();
                for (int i=0; i<size; ++i)
                {
                    markSortedFaceINTSpeed(sorted_triangle_list[i]);
                }
            }
            time_surface_speedtest = timer.nsecsElapsed();
            break;
        }

        default:
            break;
        }

        // Set the unit to 'us' and average by test times.
        time_surface_speedtest *= 0.001 / double(test_times);
    }

    // Compute the number of voxels
    num_element_surface_speedtest = 0;
    for (int xi=0; xi<grid_density; ++xi)
    {
        for (int yi=0; yi<grid_density; ++yi)
        {
            for (int zi=0; zi<grid_density; ++zi)
            {
                if (equaltoMarkerSpeed(xi + (yi + zi*grid_density)*(long long)grid_density))
                {
                    num_element_surface_speedtest++;
                }
            }
        }
    }
}

bool VoxelMeshSpeed::allocateVoxelMap()
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

inline void VoxelMeshSpeed::markPointSpeed(const long long vox_idx)
{
    voxel_map[vox_idx >> 3] = voxel_map[vox_idx >> 3] | (1 << (vox_idx & 7));

//    voxel_map[vox_idx] = 1;
}

//Check if the voxel is marked as the marker.
inline bool VoxelMeshSpeed::equaltoMarkerSpeed(const long long vox_idx)
{
    return (voxel_map[vox_idx >> 3] >> (vox_idx & 7)) & 1;  // only the last bit matters.

//    if (voxel_map[vox_idx] == 1)
//        return true;
//    return false;
}
