#ifndef mainviewer_H
#define mainviewer_H

#include <QGLViewer/qglviewer.h>
#include <QObject>
#include "voxelmesh.h"

class mainviewer : public QGLViewer
{
public:
    mainviewer(QWidget *parent = 0);
    ~mainviewer();

    Q_OBJECT
private Q_SLOTS:
    void setShowTriangleMesh(bool b);
    void setShowSurfaceVoxels(bool b);
    void setShowInteriorVoxels(bool b);
    void setShowBoundingBox(bool b);
    void setShowScanlines(bool b);
    void setShowScanpixels(bool b);
    void setShow2DProjections(bool b);
	
public:
    //void setTriangleMesh(TriangleMesh* p_mesh);
    void setVoxelMesh(VoxelMesh *p_mesh);
    void setGreenColor();
    void setViewMode(int mm);

    unsigned char *screenshot;
    void updateScreenShot();

protected :
    void draw();
    void init();

    void drawTriangleMesh(GLfloat m_color[], GLfloat alpha_);
    void drawSurfaceVoxelMesh(float* m_color, float alpha_);
    void drawInteriorVoxelMesh(float* m_color, float alpha_);
    void drawBoundingBox();
    void drawScanline();
    void drawScanvoxel(float* m_color, float alpha_);
    void draw2DProjection(float* m_color, float alpha_);

    void drawOneVoxelOriginal(CubicElement* pelement);
    void drawOneVoxelNormalized(CubicElement* pelement);
    void drawOneVoxelByIDX(int* vox_idx);
    void drawOnePixelByIDX(int* vox_idx, int axis_x, int axis_y, int axis_z);

private :
    VoxelMesh *p_vox_mesh;

	bool show_triangle_mesh;
    bool show_surface_voxel;
    bool show_interior_voxel;
    bool show_bounding_box;
    bool show_scanline;
    bool show_scanpixel;
    bool show_2D_projection;

    // 0 for normalized, 1 for original.
    int view_mode;

    GLfloat mesh_color[3];
    GLfloat line_color[3];
    GLfloat voxel_color[3];

    GLfloat material_ambient[4];
    GLfloat material_diffuse[4];
    GLfloat material_specular[4];
    GLfloat material_shininess[1];

    // the geometry information copy of voxel mesh for drawing
    double max_coordinate[3];
    double min_coordinate[3];
    double voxel_dim;
    int grid_density;
};

#endif
