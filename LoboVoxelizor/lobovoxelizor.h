#ifndef LoboVoxelizor_H
#define LoboVoxelizor_H

#include <QtGui>
#include <QMainWindow>
#include <QFileDialog>
#include <stack>
#include "voxelmesh.h"
#include "voxelmeshspeed.h"
#include "ui_lobovoxelizor.h"
#include <sstream>

class LoboVoxelizor : public QMainWindow
{
    Q_OBJECT

public:
    LoboVoxelizor(QWidget *parent = 0, Qt::WindowFlags flags = 0);
    ~LoboVoxelizor();

private slots:
    void on_actionOpen_OBJ_file_triggered();

    void on_pushButtonVoxelize_clicked();

    void on_actionSave_OBJ_file_Quad_triggered();

    void on_actionSave_OBJ_file_Triangle_triggered();

    void on_spinBoxGridDensity_editingFinished();

    void on_actionSave_ELE_Cubic_All_Elements_triggered();

    void on_actionSave_ELE_Cubic_Surface_Only_triggered();

    void on_actionSave_ELE_Tet_All_Elements_triggered();

    void on_actionSave_ELE_Tet_Surface_Only_triggered();

    void on_actionMultiOBJ_AllElement_triggered();

    void on_actionMultiOBJ_AllElement_collision_triggered();

    void on_actionSave_OBJ_File_All_Quad_Face_triggered();

    void on_actionMultiOBJ_File_Collisions_triggered();

    void on_actionMultiOBJ_File_Add_Container_triggered();

    void on_actionMultiOBJ_File_voxelization_triggered();

    void on_actionSingleOBJ_File_voxelization_triggered();

    void on_actionNormalized_Mesh_changed();

    void on_actionOriginal_Mesh_changed();

//    void on_action2D_Projections_changed();

    void on_actionSpeed_Test_Center_Scanlines_triggered();

    void on_actionSpeed_Test_Parallel_Scanlines_triggered();

    void on_actionSpeed_Test_SAT_based_triggered();

    void on_actionTriangle_triggered();

    void on_radioButton_SAT_clicked(bool checked);

    void on_radioButton_FLT_clicked(bool checked);

    void on_radioButton_INT_clicked(bool checked);

    void on_radioButton_ViewNormalized_clicked(bool checked);

    void on_radioButton_ViewOriginal_clicked(bool checked);

    void on_checkBox_setSpeedTest_clicked(bool checked);

    void on_radioButton_Pan11_clicked(bool checked);

    void on_pushButtonLoad_clicked();

    void on_checkBox_setFillHoles_clicked(bool checked);

    void on_checkBox_setMultipleThreads_clicked(bool checked);

    void on_pushButton_Screenshot_clicked();

private:
    Ui::LoboVoxelizorClass ui;

    VoxelMesh *p_voxel_mesh;
    VoxelMeshSpeed *p_voxel_mesh_speed;

    //Working directory.
    QString workingdir;

    // to determine which mesh to be shown.
    bool isShowingMeshEXP;

    // to select the method of voxelization. 0 for SS10, 1 for our floating method, 2 for our int method.
    int voxelization_method;

    bool speed_test_mode;

    void cleanAllVoxelMesh();

    // For element files
    void exportAllNode(VoxelMesh *p_vox_mesh, const char* filename);
    void exportAllCubicELE(VoxelMesh *p_vox_mesh, const char* filename);
    void exportAllTetELE(VoxelMesh *p_vox_mesh, const char* filename);

    void exportSurfaceNode(VoxelMesh *p_vox_mesh, const char* filename);
    void exportSurfaceCubicELE(VoxelMesh *p_vox_mesh, const char* filename);
    void exportSurfaceTetELE(VoxelMesh *p_vox_mesh, const char* filename);

    // For object files
    void exportOBJFileQuad(VoxelMesh *p_vox_mesh, const char* filename);
    void exportOBJFileTriangle(VoxelMesh *p_vox_mesh, const char* filename);
    void exportOBJFileQuadAllElement(VoxelMesh *p_vox_mesh, const char* filename);
    //void exportNormalizedOBJFiles(const char* filename);

    // Show message in the text window.
    void displayMessage(QString& msg);

    //----------------------------------------------------------------------------------
    // For experiments.
    //----------------------------------------------------------------------------------
//    VoxelMeshEXP *p_voxel_meshEXP;
    void exportMulOBJFileQuadAllElement(const char* filename);
    void exportMulOBJFileQuadAllElementWithCollision(const char* filename);
    void exportMulOBJFileQuadCollision(const char* filename);

    void detectCollision();
    vector<CubicElement*> collision_ele_list;

    // speed test current triangle mesh, return the result message.
    QString speedTest();

    // screen shot
    void saveScreenShotPPM(const char *filename);
    void saveScreenShotPNG(const char *filename);
};

#endif // LoboVoxelizor_H
