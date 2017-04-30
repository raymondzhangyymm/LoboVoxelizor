#include "mainviewer.h"

//===================================================================================
//Constructor and destructor.
//===================================================================================
mainviewer::mainviewer(QWidget *parent) : QGLViewer(QGLFormat(QGL::SampleBuffers), parent)
{
    //p_tri_mesh = NULL;
    p_vox_mesh = NULL;

    show_triangle_mesh = true;
    show_surface_voxel = true;
    show_interior_voxel = true;
    show_bounding_box = false;
    show_scanline = false;
    show_scanpixel = false;
    show_2D_projection = false;

    view_mode = 0;

    //Initial Colors.
    mesh_color[0] = BLUE0;
    mesh_color[1] = BLUE1;
    mesh_color[2] = BLUE2;

    line_color[0] = RED0;
    line_color[1] = RED1;
    line_color[2] = RED2;

    voxel_color[0] = 0.8;
    voxel_color[1] = 0.8;
    voxel_color[2] = 0.8;

//    voxel_color[0] = GREEN0;
//    voxel_color[1] = GREEN1;
//    voxel_color[2] = GREEN2;

    material_ambient[0] = 0.5;
    material_ambient[1] = 0.5;
    material_ambient[2] = 0.5;
    material_ambient[3] = 1.0;

    material_diffuse[0] = 0.5;
    material_diffuse[1] = 0.5;
    material_diffuse[2] = 0.5;
    material_diffuse[3] = 1.0;

    material_specular[0] = 0.5;
    material_specular[1] = 0.5;
    material_specular[2] = 0.5;
    material_specular[3] = 1.0;

    material_shininess[0] = 100.0;

    screenshot = NULL;
}

mainviewer::~mainviewer()
{

}

//===========================================================================
//Qwidge slots
//===========================================================================
void mainviewer::setShowTriangleMesh(bool b)
{
    show_triangle_mesh = b;
    updateGL();
}

void mainviewer::setShowSurfaceVoxels(bool b)
{
    show_surface_voxel = b;
    updateGL();
}

void mainviewer::setShowInteriorVoxels(bool b)
{
    show_interior_voxel = b;
    updateGL();
}

void mainviewer::setShowBoundingBox(bool b)
{
    show_bounding_box = b;
    updateGL();
}

void mainviewer::setShowScanlines(bool b)
{
    show_scanline = b;
    updateGL();
}

void mainviewer::setShowScanpixels(bool b)
{
    show_scanpixel = b;
    updateGL();
}

void mainviewer::setShow2DProjections(bool b)
{
    show_2D_projection = b;
    updateGL();
}

void mainviewer::setViewMode(int mm)
{
    view_mode = mm;
    updateGL();
}

//=============================================================================
//Get meshs and corresponding parameters from outside.
//=============================================================================
void mainviewer::setVoxelMesh(VoxelMesh *p_mesh)
{
    p_vox_mesh = p_mesh;

    mesh_color[0] = BLUE0;
    mesh_color[1] = BLUE1;
    mesh_color[2] = BLUE2;

    if (!p_vox_mesh)
    {
        return;
    }
    TriangleMesh* p_tri_mesh = p_vox_mesh->triangle_mesh;

    // copy the geometry information of the voxel mesh.
    grid_density = p_vox_mesh->grid_density;
    if (view_mode == 0)
    {
        max_coordinate[0] = p_tri_mesh->norm_max_x;
        max_coordinate[1] = p_tri_mesh->norm_max_y;
        max_coordinate[2] = p_tri_mesh->norm_max_z;

        min_coordinate[0] = p_tri_mesh->norm_min_x;
        min_coordinate[1] = p_tri_mesh->norm_min_y;
        min_coordinate[2] = p_tri_mesh->norm_min_z;

        voxel_dim = 1.0 / grid_density;
    }
    else
    {
        max_coordinate[0] = p_tri_mesh->max_x;
        max_coordinate[1] = p_tri_mesh->max_y;
        max_coordinate[2] = p_tri_mesh->max_z;

        min_coordinate[0] = p_tri_mesh->min_x;
        min_coordinate[1] = p_tri_mesh->min_y;
        min_coordinate[2] = p_tri_mesh->min_z;

        voxel_dim = p_vox_mesh->voxel_dim;
    }

    // to align the two shorter dimensions.
    if (p_tri_mesh->max_axis == 0)
    {
        // max along x axis remains the same.
        max_coordinate[1] = (floor((max_coordinate[1] - min_coordinate[1]) / voxel_dim) + 1)
                          * voxel_dim + min_coordinate[1];
        max_coordinate[2] = (floor((max_coordinate[2] - min_coordinate[2]) / voxel_dim) + 1)
                          * voxel_dim + min_coordinate[2];
    }
    else if (p_tri_mesh->max_axis == 1)
    {
        // max along y axis remains the same.
        max_coordinate[0] = (floor((max_coordinate[0] - min_coordinate[0]) / voxel_dim) + 1)
                          * voxel_dim + min_coordinate[0];
        max_coordinate[2] = (floor((max_coordinate[2] - min_coordinate[2]) / voxel_dim) + 1)
                          * voxel_dim + min_coordinate[2];
    }
    else
    {
        // max along z axis remains the same.
        max_coordinate[0] = (floor((max_coordinate[0] - min_coordinate[0]) / voxel_dim) + 1)
                          * voxel_dim + min_coordinate[0];
        max_coordinate[1] = (floor((max_coordinate[1] - min_coordinate[1]) / voxel_dim) + 1)
                          * voxel_dim + min_coordinate[1];
    }
}

//void mainviewer::setGreenColor()
//{
//    mesh_color[0] = GREEN0;
//    mesh_color[1] = GREEN1;
//    mesh_color[2] = GREEN2;
//}


//================================================================================
//Initialize the mainviewer.
//================================================================================
void mainviewer::init()
{
//    // change default key+mouse binding
//    setMouseBinding(Qt::NoModifier, Qt::RightButton, NO_CLICK_ACTION);
//    setMouseBinding(Qt::ControlModifier, Qt::LeftButton, CAMERA, TRANSLATE);
//    setMouseBinding(Qt::AltModifier, Qt::LeftButton, CAMERA, ROTATE);
//    setMouseBinding(Qt::ShiftModifier, Qt::LeftButton, CAMERA, ZOOM);

//    //Initialize OpenGL
//    glPolygonMode(GL_FRONT, GL_FILL);
//    glEnable(GL_POLYGON_OFFSET_LINE);
//    glPolygonOffset(-1.0, -1.0);

//    glShadeModel(GL_SMOOTH);

//    glEnable(GL_DEPTH_TEST);

//    glFrontFace(GL_CCW);
//    glCullFace(GL_BACK);
//    glEnable(GL_CULL_FACE);

////    glEnable(GL_COLOR_MATERIAL);

//    glEnable (GL_BLEND);
//    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
//    glEnable (GL_LINE_SMOOTH);

//    // non-integer line width makes the line smoother
//    glLineWidth(1.3);

//    glHint (GL_LINE_SMOOTH_HINT, GL_NICEST);
//    glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);

//    //------------------------------------------------------------------------------
//    // Light setup
//    //------------------------------------------------------------------------------
////    glDisable(GL_LIGHT0);
////    glEnable(GL_LIGHT1);

////    // Light default parameters
////    const GLfloat light_ambient[4]  = {0.2, 0.2, 0.2, 1.0};
////    const GLfloat light_specular[4] = {1.0, 1.0, 1.0, 1.0};
////    const GLfloat light_diffuse[4]  = {1.0, 1.0, 1.0, 1.0};

////    glLightf( GL_LIGHT1, GL_SPOT_EXPONENT, 3.0);
////    glLightf( GL_LIGHT1, GL_SPOT_CUTOFF,   10.0);
////    glLightf( GL_LIGHT1, GL_CONSTANT_ATTENUATION,  0.1f);
////    glLightf( GL_LIGHT1, GL_LINEAR_ATTENUATION,    0.3f);
////    glLightf( GL_LIGHT1, GL_QUADRATIC_ATTENUATION, 0.3f);
////    glLightfv(GL_LIGHT1, GL_AMBIENT,  light_ambient);
////    glLightfv(GL_LIGHT1, GL_SPECULAR, light_specular);
////    glLightfv(GL_LIGHT1, GL_DIFFUSE,  light_diffuse);

//    // Place light position
//    //const GLfloat pos[4] = {0.5, 1, -1, 1.0};
//    //glLightfv(GL_LIGHT1, GL_POSITION, pos);

//    // Orientate light along view direction
//    //glLightfv(GL_LIGHT1, GL_SPOT_DIRECTION, camera()->viewDirection());

//    glDisable(GL_LIGHTING);

//    //help();
//    setBackgroundColor(QColor(255, 255, 255));

    glCullFace(GL_BACK);
    glEnable(GL_CULL_FACE);

//    glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);

//    glEnable(GL_LINE_SMOOTH);
//    glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
//    glEnable(GL_POLYGON_SMOOTH);
//    glHint(GL_POLYGON_SMOOTH_HINT, GL_NICEST);

    //glEnable(GL_COLOR_MATERIAL);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

//    glEnable(GL_MULTISAMPLE);
//    GLint  iMultiSample = 0;
//    GLint  iNumSamples = 0;
//    glGetIntegerv(GL_SAMPLE_BUFFERS, &iMultiSample);
//    glGetIntegerv(GL_SAMPLES, &iNumSamples);
//    cout << "Sample buffers = " << iMultiSample << ", Samples = " << iNumSamples << endl;

    // set the light
    GLfloat lightAmbient[] = {0.5, 0.5, 0.5, 1};
    GLfloat lightDiffuse[] = {0.5, 0.5, 0.5, 1};
    GLfloat lightPosition[] = {1.0, -1.0, 50.0, 0};

    glLightfv(GL_LIGHT0, GL_AMBIENT, lightAmbient);	//Setup The Ambient Light
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightDiffuse);	//Setup The Diffuse Light
    glLightfv(GL_LIGHT0, GL_POSITION, lightPosition);	//Position The Light
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHTING);

    glClearColor(1.0, 1.0, 1.0, 0.0);
    glClear(GL_COLOR_BUFFER_BIT);

    glEnable(GL_DEPTH_TEST);

    // set clipping planes
    this->camera()->setZNearCoefficient(0.00001);
    this->camera()->setZClippingCoefficient(1000.0);
}

//===============================================================================
//Draw in the mainviewer widget.
//===============================================================================
void mainviewer::draw()
{
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);

    if (show_triangle_mesh)
    {
        // Setting Lighting materials
        glMaterialfv(GL_FRONT, GL_AMBIENT, material_ambient);
        glMaterialfv(GL_FRONT, GL_DIFFUSE, material_diffuse);
        glMaterialfv(GL_FRONT, GL_SPECULAR, material_specular);
        glMaterialfv(GL_FRONT, GL_SHININESS, material_shininess);
        GLfloat color[3] = {0,0,1};
        drawTriangleMesh(color, 0.5);
    }

    if (show_surface_voxel)
    {
        drawSurfaceVoxelMesh(voxel_color, 0.5);
    }

    if (show_interior_voxel)
    {
        drawInteriorVoxelMesh(voxel_color, 0.2);
    }

    if (show_bounding_box)
    {
        drawBoundingBox();
    }

    if (show_scanline)
    {
        drawScanline();
    }

    if (show_scanpixel)
    {
        drawScanvoxel(voxel_color, 0.2);
    }

    if (show_2D_projection)
    {
        draw2DProjection(mesh_color, 0.2);
    }
}

//==================================================================================
// Draw triangle mesh.
//==================================================================================
void mainviewer::drawTriangleMesh(GLfloat m_color[], GLfloat alpha_)
{
    if (!p_vox_mesh)
    {
        return;
    }
    TriangleMesh* p_tri_mesh = p_vox_mesh->triangle_mesh;
    int num_mesh = p_tri_mesh->num_mesh;
    zVec3 n0, n1, n2, n01, n02, n;

    glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
    glLineWidth(3);

    switch (view_mode)
    {
    case 0:
        glBegin(GL_TRIANGLES);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                n0 = (*fi)->node0->norm_coordinate;
                n1 = (*fi)->node1->norm_coordinate;
                n2 = (*fi)->node2->norm_coordinate;

//                n01 = n0 - n1;
//                n02 = n0 - n2;

//                n = zVec3::crossproduct(n01, n02);
//                n.normalize();

//                glColor4f(0.1+bb, 0, 0, alpha_);
//                int aa = aa + 1;

//                double bb = double(aa%4) * 0.3;

//                glNormal3d(n[0], n[1], n[2]);
                glVertex3d(n0[0], n0[1], n0[2]);
                glVertex3d(n1[0], n1[1], n1[2]);
                glVertex3d(n2[0], n2[1], n2[2]);
            }
        }
        glEnd();
        break;

    case 1:
        glBegin(GL_TRIANGLES);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                n0 = (*fi)->node0->ori_coordinate;
                n1 = (*fi)->node1->ori_coordinate;
                n2 = (*fi)->node2->ori_coordinate;

//                n01 = n0 - n1;
//                n02 = n0 - n2;

//                n = zVec3::crossproduct(n01, n02);
//                n.normalize();

//                glNormal3d(n[0], n[1], n[2]);
                glVertex3d(n0[0], n0[1], n0[2]);
                glVertex3d(n1[0], n1[1], n1[2]);
                glVertex3d(n2[0], n2[1], n2[2]);
            }
        }
        glEnd();
        break;

    case 2:
        break;
    }
}

void mainviewer::drawScanline()
{
    if (!p_vox_mesh)
    {
        return;
    }

    TriangleMesh* p_tri_mesh = p_vox_mesh->triangle_mesh;
    int num_mesh = p_tri_mesh->num_mesh;

    switch (view_mode)
    {
    case 0:
        glColor4f(1.0, 0.0, 0.0, 1.0);
        glLineWidth(2);
        glBegin(GL_LINES);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                if ((*fi)->norm_sequ_scanline)
                {
                    for (int j=0; j<(*fi)->norm_sequ_scanline->size(); ++j)
                    {
                        double* nn = (*fi)->norm_sequ_scanline->at(j);

                        glVertex3d(nn[0], nn[1], nn[2]);
                        glVertex3d(nn[3], nn[4], nn[5]);
                    }
                }
            }
        }
        glEnd();

        glColor4f(0.0, 0.0, 1.0, 1.0);
        glLineWidth(4);
        glBegin(GL_LINES);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                if ((*fi)->norm_sequ_primscanline)
                {
                    for (int j=0; j<(*fi)->norm_sequ_primscanline->size(); ++j)
                    {
                        double* nn = (*fi)->norm_sequ_primscanline->at(j);

                        glVertex3d(nn[0], nn[1], nn[2]);
                        glVertex3d(nn[3], nn[4], nn[5]);
                    }
                }
            }
        }
        glEnd();
        break;

    case 1:
        glColor4f(1.0, 0.0, 0.0, 1.0);
        glLineWidth(2);
        glBegin(GL_LINES);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                if ((*fi)->sequ_scanline)
                {
                    for (int j=0; j<(*fi)->sequ_scanline->size(); ++j)
                    {
                        double* nn = (*fi)->sequ_scanline->at(j);

                        glVertex3d(nn[0], nn[1], nn[2]);
                        glVertex3d(nn[3], nn[4], nn[5]);
                    }
                }
            }
        }
        glEnd();

        glLineWidth(4);
        glColor4f(0.0, 0.0, 1.0, 1.0);
        glBegin(GL_LINES);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                if ((*fi)->sequ_primscanline)
                {
                    for (int j=0; j<(*fi)->sequ_primscanline->size(); ++j)
                    {
                        double* nn = (*fi)->sequ_primscanline->at(j);

                        glVertex3d(nn[0], nn[1], nn[2]);
                        glVertex3d(nn[3], nn[4], nn[5]);
                    }
                }
            }
        }
        glEnd();
        break;
    case 2:
        break;
    }
}

void mainviewer::drawScanvoxel(float* m_color, float alpha_)
{
    if (!p_vox_mesh)
    {
        return;
    }

    TriangleMesh* p_tri_mesh = p_vox_mesh->triangle_mesh;
    int num_mesh = p_tri_mesh->num_mesh;

    glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
    glLineWidth(2);
    glPolygonMode(GL_FRONT, GL_FILL);
    glBegin(GL_QUADS);
    for (int i=0; i<num_mesh; ++i)
    {
        for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
             fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
        {
            int axis_x = (*fi)->axis_x;
            int axis_y = (*fi)->axis_y;
            int axis_z = (*fi)->axis_z;

            if ((*fi)->sequ_primscanvoxel)
            {
                for (int j=0; j<(*fi)->sequ_primscanvoxel->size(); ++j)
                {
                    for (int k=0; k<(*fi)->sequ_primscanvoxel->at(j)->size(); ++k)
                    {
                        drawOneVoxelByIDX((*fi)->sequ_primscanvoxel->at(j)->at(k));
                        drawOnePixelByIDX((*fi)->sequ_primscanvoxel->at(j)->at(k),
                                          axis_x, axis_y, axis_z);
                    }
                }
            }
        }
    }
    glEnd();
}

//================================================================================
// Draw the voxel mesh.
//================================================================================
void mainviewer::drawSurfaceVoxelMesh(float* m_color, float alpha_)
{
    if (!p_vox_mesh)
    {
        return;
    }
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();

    glLineWidth(1);

    switch (view_mode)
    {
    case 0:
        glPolygonMode(GL_FRONT, GL_LINE);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_+0.2);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {
            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if ((*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelNormalized(*pelement);
                }
            }
        }
        glEnd();

        glPolygonMode(GL_FRONT, GL_FILL);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {

            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if ((*pelement)->isOnSurfaceVoxelization)
                {
                    if ((*pelement)->hitting_times == 2)
                    {
                        glColor4f(0.5, 0.5, 0.0, alpha_);
                    }
                    else if ((*pelement)->hitting_times == 3)
                    {
                        glColor4f(0.4, 0.8, 0.0, alpha_);
                    }
                    else if ((*pelement)->hitting_times == 4)
                    {
                        glColor4f(0.6, 0.0, 0.0, alpha_);
                    }
                    else if ((*pelement)->hitting_times == 5)
                    {
                        glColor4f(0.8, 0.0, 0.8, alpha_);
                    }
                    else
                    {
                        glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
                        //glColor4f(1.0, 0.0, 1.0, alpha_);
                    }



                    drawOneVoxelNormalized(*pelement);
                }
            }
        }
        glEnd();
        break;

    case 1:
        glPolygonMode(GL_FRONT, GL_LINE);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_+0.2);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {
            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if ((*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelOriginal(*pelement);
                }
            }
        }
        glEnd();

        glPolygonMode(GL_FRONT, GL_FILL);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {
            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if ((*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelOriginal(*pelement);
                }
            }
        }
        glEnd();
        break;

    case 2:
        break;
    }

}

void mainviewer::drawInteriorVoxelMesh(float* m_color, float alpha_)
{
    if (!p_vox_mesh)
    {
        return;
    }
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();

    glLineWidth(1);

    switch (view_mode)
    {
    case 0:
        glPolygonMode(GL_FRONT, GL_LINE);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_+0.2);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {
            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if (!(*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelNormalized(*pelement);
                }
            }
        }
        glEnd();

        glPolygonMode(GL_FRONT, GL_FILL);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {

            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if (!(*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelNormalized(*pelement);
                }
            }
        }
        glEnd();
        break;

    case 1:
        glPolygonMode(GL_FRONT, GL_LINE);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_+0.2);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {
            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if (!(*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelOriginal(*pelement);
                }
            }
        }
        glEnd();

        glPolygonMode(GL_FRONT, GL_FILL);
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
        glBegin(GL_QUADS);
        for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
        {
            vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];

            for (vector<CubicElement*>::const_iterator
                 pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
            {
                if (!(*pelement)->isOnSurfaceVoxelization)
                {
                    drawOneVoxelOriginal(*pelement);
                }
            }
        }
        glEnd();
        break;

    case 2:
        break;
    }
}

void mainviewer::drawBoundingBox()
{
    if (!p_vox_mesh)
    {
        return;
    }
    TriangleMesh* p_tri_mesh = p_vox_mesh->triangle_mesh;

    glLineWidth(1);
    glColor4f(0.8, 0.8, 0.8, 0.3);
    //glPolygonMode(GL_FRONT, GL_LINE);
    glBegin(GL_LINES);
    for (double x = min_coordinate[0]; x <= max_coordinate[0]; x += voxel_dim)
    {
        for (double y = min_coordinate[1]; y <= max_coordinate[1]; y += voxel_dim)
        {
            for (double z = min_coordinate[2]; z <= max_coordinate[2]; z += voxel_dim)
            {
                glVertex3d(min_coordinate[0], y, z);
                glVertex3d(max_coordinate[0], y, z);

                glVertex3d(x, min_coordinate[1], z);
                glVertex3d(x, max_coordinate[1], z);

                glVertex3d(x, y, min_coordinate[2]);
                glVertex3d(x, y, min_coordinate[2]);
            }
        }
    }
    glEnd();
}


//==================================================================================
// Draw 2D projections.
//==================================================================================
void mainviewer::draw2DProjection(float* m_color, float alpha_)
{
    if (!p_vox_mesh)
    {
        return;
    }
    TriangleMesh* p_tri_mesh = p_vox_mesh->triangle_mesh;
    int num_mesh = p_tri_mesh->num_mesh;

    double ma_x, ma_y, ma_z, mi_x, mi_y, mi_z, vox_dim;
    ma_x = max_coordinate[0];
    ma_y = max_coordinate[1];
    ma_z = max_coordinate[2];

    mi_x = min_coordinate[0];
    mi_y = min_coordinate[1];
    mi_z = min_coordinate[2];

    vox_dim = voxel_dim;

    // draw 2D grid
    glColor4f(m_color[0], m_color[1], m_color[2], 0.3);
    glBegin(GL_LINES);
    glLineWidth(1);
    for (double x = mi_x; x <= ma_x; x += vox_dim)
    {
        for (double y = mi_y; y <= ma_y; y += vox_dim)
        {
                glVertex3d(x, mi_y, mi_z);
                glVertex3d(x, ma_y, mi_z);

                glVertex3d(mi_x, y, mi_z);
                glVertex3d(ma_x, y, mi_z);
        }
    }
    for (double z = mi_z; z <= ma_z; z += vox_dim)
    {
        for (double y = mi_y; y <= ma_y; y += vox_dim)
        {
                glVertex3d(mi_x, y, mi_z);
                glVertex3d(mi_x, y, ma_z);

                glVertex3d(mi_x, mi_y, z);
                glVertex3d(mi_x, ma_y, z);
        }
    }
    for (double x = mi_x; x <= ma_x; x += vox_dim)
    {
        for (double z = mi_z; z <= ma_z; z += vox_dim)
        {
                glVertex3d(x, mi_y, mi_z);
                glVertex3d(x, mi_y, ma_z);

                glVertex3d(mi_x, mi_y, z);
                glVertex3d(ma_x, mi_y, z);
        }
    }
    glEnd();

    // draw 2D mesh
    switch (view_mode)
    {
    case 0:
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
        glBegin(GL_TRIANGLES);
        glLineWidth(1);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                zVec3 n0 = (*fi)->node0->norm_coordinate;
                zVec3 n1 = (*fi)->node1->norm_coordinate;
                zVec3 n2 = (*fi)->node2->norm_coordinate;

                if ((*fi)->axis_z == 0)
                {
                    n0[0] = mi_x;
                    n1[0] = mi_x;
                    n2[0] = mi_x;
                }
                else if ((*fi)->axis_z == 1)
                {
                    n0[1] = mi_y;
                    n1[1] = mi_y;
                    n2[1] = mi_y;
                }
                else
                {
                    n0[2] = mi_z;
                    n1[2] = mi_z;
                    n2[2] = mi_z;
                }

                glNormal3d((*fi)->norm[0], (*fi)->norm[1], (*fi)->norm[2]);
                glVertex3d(n0[0], n0[1], n0[2]);
                glVertex3d(n1[0], n1[1], n1[2]);
                glVertex3d(n2[0], n2[1], n2[2]);

                glNormal3d(-(*fi)->norm[0], -(*fi)->norm[1], -(*fi)->norm[2]);
                glVertex3d(n0[0], n0[1], n0[2]);
                glVertex3d(n2[0], n2[1], n2[2]);
                glVertex3d(n1[0], n1[1], n1[2]);
            }
        }
        glEnd();
        break;

    case 1:
        glColor4f(m_color[0], m_color[1], m_color[2], alpha_);
        glBegin(GL_TRIANGLES);
        glLineWidth(1);
        for (int i=0; i<num_mesh; ++i)
        {
            for (const_iter_ptriangleface fi = p_tri_mesh->mesh_lists[i].begin();
                 fi!=p_tri_mesh->mesh_lists[i].end(); ++fi)
            {
                zVec3 n0 = (*fi)->node0->ori_coordinate;
                zVec3 n1 = (*fi)->node1->ori_coordinate;
                zVec3 n2 = (*fi)->node2->ori_coordinate;

                if ((*fi)->axis_z == 0)
                {
                    n0[0] = mi_x;
                    n1[0] = mi_x;
                    n2[0] = mi_x;
                }
                else if ((*fi)->axis_z == 1)
                {
                    n0[1] = mi_y;
                    n1[1] = mi_y;
                    n2[1] = mi_y;
                }
                else
                {
                    n0[2] = mi_z;
                    n1[2] = mi_z;
                    n2[2] = mi_z;
                }

                glNormal3d((*fi)->norm[0], (*fi)->norm[1], (*fi)->norm[2]);
                glVertex3d(n0[0], n0[1], n0[2]);
                glVertex3d(n1[0], n1[1], n1[2]);
                glVertex3d(n2[0], n2[1], n2[2]);

                glNormal3d(-(*fi)->norm[0], -(*fi)->norm[1], -(*fi)->norm[2]);
                glVertex3d(n0[0], n0[1], n0[2]);
                glVertex3d(n2[0], n2[1], n2[2]);
                glVertex3d(n1[0], n1[1], n1[2]);
            }
        }
        glEnd();
        break;

    case 2:
        break;
    }
}

void mainviewer::drawOneVoxelOriginal(CubicElement* pelement)
{
    //Use Garcia's order in the element file.
    //            Eigen::zVec3 n0 = (*pelement)->node7->ori_coordinate;
    //            Eigen::zVec3 n1 = (*pelement)->node5->ori_coordinate;
    //            Eigen::zVec3 n2 = (*pelement)->node4->ori_coordinate;
    //            Eigen::zVec3 n3 = (*pelement)->node6->ori_coordinate;
    //            Eigen::zVec3 n4 = (*pelement)->node3->ori_coordinate;
    //            Eigen::zVec3 n5 = (*pelement)->node1->ori_coordinate;
    //            Eigen::zVec3 n6 = (*pelement)->node0->ori_coordinate;
    //            Eigen::zVec3 n7 = (*pelement)->node2->ori_coordinate;

    zVec3 n0 = pelement->node0->ori_coordinate;
    zVec3 n1 = pelement->node1->ori_coordinate;
    zVec3 n2 = pelement->node2->ori_coordinate;
    zVec3 n3 = pelement->node3->ori_coordinate;
    zVec3 n4 = pelement->node4->ori_coordinate;
    zVec3 n5 = pelement->node5->ori_coordinate;
    zVec3 n6 = pelement->node6->ori_coordinate;
    zVec3 n7 = pelement->node7->ori_coordinate;

    //0132
    glNormal3d(0, -1, 0);
    glVertex3d(n0.x, n0.y, n0.z);
    glVertex3d(n1.x, n1.y, n1.z);
    glVertex3d(n3.x, n3.y, n3.z);
    glVertex3d(n2.x, n2.y, n2.z);

    //4675
    glNormal3d(0, 1, 0);
    glVertex3d(n4.x, n4.y, n4.z);
    glVertex3d(n6.x, n6.y, n6.z);
    glVertex3d(n7.x, n7.y, n7.z);
    glVertex3d(n5.x, n5.y, n5.z);

    //6237
    glNormal3d(0, 0, 1);
    glVertex3d(n6.x, n6.y, n6.z);
    glVertex3d(n2.x, n2.y, n2.z);
    glVertex3d(n3.x, n3.y, n3.z);
    glVertex3d(n7.x, n7.y, n7.z);

    //4510
    glNormal3d(0, 0, -1);
    glVertex3d(n4.x, n4.y, n4.z);
    glVertex3d(n5.x, n5.y, n5.z);
    glVertex3d(n1.x, n1.y, n1.z);
    glVertex3d(n0.x, n0.y, n0.z);

    //5731
    glNormal3d(1, 0, 0);
    glVertex3d(n5.x, n5.y, n5.z);
    glVertex3d(n7.x, n7.y, n7.z);
    glVertex3d(n3.x, n3.y, n3.z);
    glVertex3d(n1.x, n1.y, n1.z);

    //4026
    glNormal3d(-1, 0, 0);
    glVertex3d(n4.x, n4.y, n4.z);
    glVertex3d(n0.x, n0.y, n0.z);
    glVertex3d(n2.x, n2.y, n2.z);
    glVertex3d(n6.x, n6.y, n6.z);
}

void mainviewer::drawOneVoxelNormalized(CubicElement* pelement)
{
    //Use Garcia's order in the element file.
    //            Eigen::zVec3 n0 = (*pelement)->node7->ori_coordinate;
    //            Eigen::zVec3 n1 = (*pelement)->node5->ori_coordinate;
    //            Eigen::zVec3 n2 = (*pelement)->node4->ori_coordinate;
    //            Eigen::zVec3 n3 = (*pelement)->node6->ori_coordinate;
    //            Eigen::zVec3 n4 = (*pelement)->node3->ori_coordinate;
    //            Eigen::zVec3 n5 = (*pelement)->node1->ori_coordinate;
    //            Eigen::zVec3 n6 = (*pelement)->node0->ori_coordinate;
    //            Eigen::zVec3 n7 = (*pelement)->node2->ori_coordinate;

    zVec3 n0 = pelement->node0->norm_coordinate;
    zVec3 n1 = pelement->node1->norm_coordinate;
    zVec3 n2 = pelement->node2->norm_coordinate;
    zVec3 n3 = pelement->node3->norm_coordinate;
    zVec3 n4 = pelement->node4->norm_coordinate;
    zVec3 n5 = pelement->node5->norm_coordinate;
    zVec3 n6 = pelement->node6->norm_coordinate;
    zVec3 n7 = pelement->node7->norm_coordinate;

    //0132
    glNormal3d(0, -1, 0);
    glVertex3d(n0.x, n0.y, n0.z);
    glVertex3d(n1.x, n1.y, n1.z);
    glVertex3d(n3.x, n3.y, n3.z);
    glVertex3d(n2.x, n2.y, n2.z);

    //4675
    glNormal3d(0, 1, 0);
    glVertex3d(n4.x, n4.y, n4.z);
    glVertex3d(n6.x, n6.y, n6.z);
    glVertex3d(n7.x, n7.y, n7.z);
    glVertex3d(n5.x, n5.y, n5.z);

    //6237
    glNormal3d(0, 0, 1);
    glVertex3d(n6.x, n6.y, n6.z);
    glVertex3d(n2.x, n2.y, n2.z);
    glVertex3d(n3.x, n3.y, n3.z);
    glVertex3d(n7.x, n7.y, n7.z);

    //4510
    glNormal3d(0, 0, -1);
    glVertex3d(n4.x, n4.y, n4.z);
    glVertex3d(n5.x, n5.y, n5.z);
    glVertex3d(n1.x, n1.y, n1.z);
    glVertex3d(n0.x, n0.y, n0.z);

    //5731
    glNormal3d(1, 0, 0);
    glVertex3d(n5.x, n5.y, n5.z);
    glVertex3d(n7.x, n7.y, n7.z);
    glVertex3d(n3.x, n3.y, n3.z);
    glVertex3d(n1.x, n1.y, n1.z);

    //4026
    glNormal3d(-1, 0, 0);
    glVertex3d(n4.x, n4.y, n4.z);
    glVertex3d(n0.x, n0.y, n0.z);
    glVertex3d(n2.x, n2.y, n2.z);
    glVertex3d(n6.x, n6.y, n6.z);
}

void mainviewer::drawOneVoxelByIDX(int *vox_idx)
{
    double n0[3];
    n0[0] = vox_idx[0] * voxel_dim + min_coordinate[0];
    n0[1] = vox_idx[1] * voxel_dim + min_coordinate[1];
    n0[2] = vox_idx[2] * voxel_dim + min_coordinate[2];

    double n1[3],n2[3],n3[3],n4[3],n5[3],n6[3],n7[3];
    n1[0] = n0[0];
    n1[1] = n0[1];
    n1[2] = n0[2] + voxel_dim;

    n2[0] = n0[0] + voxel_dim;
    n2[1] = n0[1];
    n2[2] = n0[2] + voxel_dim;

    n3[0] = n0[0] + voxel_dim;
    n3[1] = n0[1];
    n3[2] = n0[2];

    n4[0] = n0[0];
    n4[1] = n0[1] + voxel_dim;
    n4[2] = n0[2];

    n5[0] = n0[0];
    n5[1] = n0[1] + voxel_dim;
    n5[2] = n0[2] + voxel_dim;

    n6[0] = n0[0] + voxel_dim;
    n6[1] = n0[1] + voxel_dim;
    n6[2] = n0[2] + voxel_dim;

    n7[0] = n0[0] + voxel_dim;
    n7[1] = n0[1] + voxel_dim;
    n7[2] = n0[2];

    glNormal3d(0, -1, 0);
    glVertex3dv(n0);
    glVertex3dv(n3);
    glVertex3dv(n2);
    glVertex3dv(n1);

    glNormal3d(0, 1, 0);
    glVertex3dv(n4);
    glVertex3dv(n5);
    glVertex3dv(n6);
    glVertex3dv(n7);

    glNormal3d(0, 0, 1);
    glVertex3dv(n1);
    glVertex3dv(n2);
    glVertex3dv(n6);
    glVertex3dv(n5);

    glNormal3d(0, 0, -1);
    glVertex3dv(n0);
    glVertex3dv(n4);
    glVertex3dv(n7);
    glVertex3dv(n3);

    glNormal3d(1, 0, 0);
    glVertex3dv(n2);
    glVertex3dv(n3);
    glVertex3dv(n7);
    glVertex3dv(n6);

    glNormal3d(-1, 0, 0);
    glVertex3dv(n0);
    glVertex3dv(n1);
    glVertex3dv(n5);
    glVertex3dv(n4);
}

void mainviewer::drawOnePixelByIDX(int *vox_idx, int axis_x, int axis_y, int axis_z)
{
    double n0[3];
    n0[axis_x] = vox_idx[axis_x] * voxel_dim + min_coordinate[axis_x];
    n0[axis_y] = vox_idx[axis_y] * voxel_dim + min_coordinate[axis_y];
    n0[axis_z] = min_coordinate[axis_z];

    double n1[3],n2[3],n3[3];
    n1[axis_x] = n0[axis_x] + voxel_dim;
    n1[axis_y] = n0[axis_y];
    n1[axis_z] = n0[axis_z];

    n2[axis_x] = n0[axis_x] + voxel_dim;
    n2[axis_y] = n0[axis_y] + voxel_dim;
    n2[axis_z] = n0[axis_z];

    n3[axis_x] = n0[axis_x];
    n3[axis_y] = n0[axis_y] + voxel_dim;
    n3[axis_z] = n0[axis_z];

    glNormal3d(0, 0, 1);
    glVertex3dv(n0);
    glVertex3dv(n1);
    glVertex3dv(n2);
    glVertex3dv(n3);

    glNormal3d(0, 0, -1);
    glVertex3dv(n0);
    glVertex3dv(n3);
    glVertex3dv(n2);
    glVertex3dv(n1);
}

void mainviewer::updateScreenShot()
{
    if (screenshot)
    {
        delete screenshot;
    }

    screenshot = new unsigned char[4 * width() * height()];
    glReadPixels(0, 0, width(), height(), GL_RGBA, GL_UNSIGNED_BYTE, screenshot);
}
