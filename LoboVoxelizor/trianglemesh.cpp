#include "geometry.h"

//===============================================================
// Constructor and destructor
//===============================================================
TriangleMesh::TriangleMesh()
{
	num_face = 0;
	num_node = 0;
    num_mesh = 0;

    max_x = max_y = max_z = 0.0;
    min_x = min_y = min_z = 0.0;
    max_dimension = 0.0;
}

TriangleMesh::~TriangleMesh()
{

}

//===================================================================================
// Read OBJ file. Only vertices and faces are loaded.
//===================================================================================
// The node list and mesh list are built by the input obj file.
// The polygon face is divied into several triangle faces.
// Multiple objects are loaded into separated sub mesh list.
void TriangleMesh::loadOBJFile(const char* obj_filename)
{
    // Clear the previous OBJ file if needed.
	if (num_face || num_node)
	{
        mesh_lists.clear();
		num_face = 0;
		node_list.clear();
		num_node = 0;
	}

    // open the obj file.
	ifstream ifs(obj_filename);
    if (ifs.fail())
    {
        cout << "Obj file opening failed" << endl;
        return;
    }

	char line[1024];
    char delimiters[] = " \r\n";

    char* token;
    char commt[] = "#";
    char vertex[] = "v";
    char face[] = "f";
    char object[] = "o";  // symble for sub mesh
    char group[] = "g";  // symble for sub mesh

    // the first group/sub object is default
    mesh_lists.clear();
    vector<TriangleFace*> submeshlist;
    mesh_lists.push_back(submeshlist);
    int current_submesh_id = 0;
    bool is_first_group = true;
    while (!ifs.eof())
	{
		ifs.getline(line, 1024);
        token = strtok(line, delimiters);

		if (strlen(line) == 0)
		{
			continue;
		}
		else if (strcmp(token, commt) == 0)
		{
            continue;
		}

        // found a new group
        else if (strcmp(token, group) == 0 || strcmp(token, object) == 0)
        {
            if (is_first_group)
            {
                // ignore the first group with "g".
                is_first_group = false;
            }
            else
            {
                mesh_lists.push_back(submeshlist);
                current_submesh_id++;
            }
        }

		// a line for node index
		else if (strcmp(token, vertex) == 0 )
		{
			Node* p_new_node = new Node;

            //Generate a node (vertex) then add to the tail of the node list.
			token = strtok(NULL, " ");
            p_new_node->ori_coordinate[0] = atof(token);
			token = strtok(NULL, " ");
            p_new_node->ori_coordinate[1] = atof(token);
			token = strtok(NULL, " ");
            p_new_node->ori_coordinate[2] = atof(token);

			node_list.push_back(p_new_node);
		}

        // a line for face index
		else if (strcmp (token, face) == 0)
		{
            TriangleFace* p_new_face = new TriangleFace;

            //Generate a face then add to the tail of the face list.
            //A face contains 3 index of the node_list.
            token = strtok(NULL, " ");
            p_new_face->idx0 = atoi(token) - 1;
            token = strtok(NULL, " ");
            p_new_face->idx1 = atoi(token) - 1;
            token = strtok(NULL, " ");
            p_new_face->idx2 = atoi(token) - 1;
            p_new_face->sequ_primscanline = NULL;
            p_new_face->norm_sequ_primscanline = NULL;
            p_new_face->sequ_scanline = NULL;
            p_new_face->norm_sequ_scanline = NULL;
            p_new_face->sequ_primscanvoxel = NULL;
            p_new_face->sequ_scanvoxel = NULL;
            mesh_lists[current_submesh_id].push_back(p_new_face);

            //If the face is a polygon, keep getting the rest indices
            //and convert to a couple of triangles.
            long long vv0 = p_new_face->idx0;
            long long vv1 = p_new_face->idx2;
            while ((token = strtok(NULL, " \r\n")) != NULL)
            {
                int vv2 = atoi(token) - 1;

                p_new_face = new TriangleFace;
                p_new_face->idx0 = vv0;
                p_new_face->idx1 = vv1;
                p_new_face->idx2 = vv2;
                p_new_face->sequ_primscanline = NULL;
                p_new_face->norm_sequ_primscanline = NULL;
                p_new_face->sequ_scanline = NULL;
                p_new_face->norm_sequ_scanline = NULL;
                p_new_face->sequ_primscanvoxel = NULL;
                p_new_face->sequ_scanvoxel = NULL;
                mesh_lists[current_submesh_id].push_back(p_new_face);

                vv1 = vv2;
            }
		}
	}
	ifs.close();

    num_node = node_list.size();
    num_mesh = mesh_lists.size();

    // Deal with negative-face-index case.
    bool isNegativeFaceIdx = false;
    for (int i=0; i<num_mesh; ++i)
    {
        for (iter_ptriangleface fi = mesh_lists[i].begin(); fi!=mesh_lists[i].end(); ++fi)
        {
            if ((*fi)->idx0 < 0)
            {
                isNegativeFaceIdx = true;
                break;
            }
        }
    }
    if (isNegativeFaceIdx)
    {
        cout << "Negative face index. " << endl;
        for (int i=0; i<num_mesh; ++i)
        {
            for (iter_ptriangleface fi = mesh_lists[i].begin(); fi!=mesh_lists[i].end(); ++fi)
            {
                (*fi)->idx0 += num_node + 1;
                (*fi)->idx1 += num_node + 1;
                (*fi)->idx2 += num_node + 1;
            }
        }
    }

    //Point 3 node-pointers in the face to the coresponding nodes in the node_list.
    num_face = 0;
    for (int i=0; i<num_mesh; ++i)
    {
        num_face += mesh_lists[i].size();
        for (iter_ptriangleface fi = mesh_lists[i].begin(); fi!=mesh_lists[i].end(); ++fi)
        {
            (*fi)->node0 = node_list[(*fi)->idx0];
            (*fi)->node1 = node_list[(*fi)->idx1];
            (*fi)->node2 = node_list[(*fi)->idx2];

            if ((*fi)->idx0 < 0 || (*fi)->idx0 >= num_node ||
                (*fi)->idx1 < 0 || (*fi)->idx1 >= num_node ||
                (*fi)->idx2 < 0 || (*fi)->idx2 >= num_node)
            {
                cout << "Wrong face ! f "
                     << (*fi)->idx0 << " "
                     << (*fi)->idx1 << " "
                     << (*fi)->idx2 << endl;
            }
        }
    }
}

//========================================================================
// Find the max and min coordinates and compute the normalized mesh.
//========================================================================
// Set the default global bounding box determined by the obj mesh.
void TriangleMesh::setBoundingBox()
{
    if (node_list.empty() || mesh_lists.empty())
    {
        return;
    }

    //Find the max and min coordinates.
    min_x = max_x = node_list[0]->ori_coordinate[0];
    min_y = max_y = node_list[0]->ori_coordinate[1];
    min_z = max_z = node_list[0]->ori_coordinate[2];

    for (int i=1; i<num_node; i++)
    {
        double x = node_list[i]->ori_coordinate[0];
        double y = node_list[i]->ori_coordinate[1];
        double z = node_list[i]->ori_coordinate[2];

        max_x = x > max_x ? x : max_x;
        max_y = y > max_y ? y : max_y;
        max_z = z > max_z ? z : max_z;

        min_x = x < min_x ? x : min_x;
        min_y = y < min_y ? y : min_y;
        min_z = z < min_z ? z : min_z;
    }

    double sx = max_x - min_x;
    double sy = max_y - min_y;
    double sz = max_z - min_z;

    if (sx >= sy)
    {
        if (sx >= sz)
        {
            // x is the max axis
            max_dimension = max_x - min_x;
            max_axis = 0;
        }
        else
        {
            // z is the max axis
            max_dimension = max_z - min_z;
            max_axis = 2;
        }
    }
    else
    {
        if (sy >= sz)
        {
            // y is the max axis
            max_dimension = max_y - min_y;
            max_axis = 1;
        }
        else
        {
            // z is the max axis
            max_dimension = max_z - min_z;
            max_axis = 2;
        }
    }

    //Compute the normalized mesh,
    //i.e. the original mesh scaled into unit box centering at (0,0,0), dim of 1.
    double shift_x = 0.5* (max_x + min_x);
    double shift_y = 0.5* (max_y + min_y);
    double shift_z = 0.5* (max_z + min_z);

    for (int i=0; i<num_node; i++)
    {
        node_list[i]->norm_coordinate[0] =
                (node_list[i]->ori_coordinate[0] - shift_x) / max_dimension;
        node_list[i]->norm_coordinate[1] =
                (node_list[i]->ori_coordinate[1] - shift_y) / max_dimension;
        node_list[i]->norm_coordinate[2] =
                (node_list[i]->ori_coordinate[2] - shift_z) / max_dimension;
    }

    norm_max_x = (max_x - shift_x) / max_dimension;
    norm_max_y = (max_y - shift_y) / max_dimension;
    norm_max_z = (max_z - shift_z) / max_dimension;

    norm_min_x = (min_x - shift_x) / max_dimension;
    norm_min_y = (min_y - shift_y) / max_dimension;
    norm_min_z = (min_z - shift_z) / max_dimension;

    norm_max_dimension = 1.0;
}

// Set the custermized global bounding box.
void TriangleMesh::setBoundingBox(const double ma_x, const double ma_y, const double ma_z,
                                  const double mi_x, const double mi_y, const double mi_z)
{
    if (node_list.empty() || mesh_lists.empty())
    {
        return;
    }

    if (ma_x <= mi_x || ma_y <= mi_y || ma_z <= mi_z)
    {
        cout << "Wrong setting of bounding box. Use default one.\n";
        setBoundingBox();
        return;
    }

    max_x = ma_x;
    max_y = ma_y;
    max_z = ma_z;

    min_x = mi_x;
    min_y = mi_y;
    min_z = mi_z;

    double sx = max_x - min_x;
    double sy = max_y - min_y;
    double sz = max_z - min_z;

    if (sx >= sy)
    {
        if (sx >= sz)
        {
            // x is the max axis
            max_dimension = max_x - min_x;
            max_axis = 0;
        }
        else
        {
            // z is the max axis
            max_dimension = max_z - min_z;
            max_axis = 2;
        }
    }
    else
    {
        if (sy >= sz)
        {
            // y is the max axis
            max_dimension = max_y - min_y;
            max_axis = 1;
        }
        else
        {
            // z is the max axis
            max_dimension = max_z - min_z;
            max_axis = 2;
        }
    }

    //Compute the normalized mesh,
    //i.e. the original mesh scaled into unit box centering at (0,0,0).
    double shift_x = 0.5* (max_x + min_x);
    double shift_y = 0.5* (max_y + min_y);
    double shift_z = 0.5* (max_z + min_z);

    for (int i=0; i<num_node; i++)
    {
        node_list[i]->norm_coordinate[0] =
                (node_list[i]->ori_coordinate[0] - shift_x) / max_dimension;
        node_list[i]->norm_coordinate[1] =
                (node_list[i]->ori_coordinate[1] - shift_y) / max_dimension;
        node_list[i]->norm_coordinate[2] =
                (node_list[i]->ori_coordinate[2] - shift_z) / max_dimension;
    }

    norm_max_x = (max_x - shift_x) / max_dimension;
    norm_max_y = (max_y - shift_y) / max_dimension;
    norm_max_z = (max_z - shift_z) / max_dimension;

    norm_min_x = (min_x - shift_x) / max_dimension;
    norm_min_y = (min_y - shift_y) / max_dimension;
    norm_min_z = (min_z - shift_z) / max_dimension;

    norm_max_dimension = 1.0;
}
