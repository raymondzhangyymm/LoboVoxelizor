#include "lobovoxelizor.h"

LoboVoxelizor::LoboVoxelizor(QWidget *parent, Qt::WindowFlags flags)
    : QMainWindow(parent, flags)
{
    ui.setupUi(this);
    p_voxel_mesh = NULL;
    p_voxel_mesh_speed = NULL;

    QString windowtitle("LOBO VOXELIZOR");
    //windowtitle.append(MAX_GRID_DENSITY);
    this->setWindowTitle(windowtitle);

//    ui.spinBoxGridDensity->setMaximum(HIGH_GRID_DENSITY);
    ui.spinBoxGridDensity->setValue(INIT_DENSITY);

    //let the plain text be read only.
    ui.MessageText->setReadOnly(true);

    //set up the progress bar.
//    ui.progressBar->setRange(0, 100);
//    ui.progressBar->hide();
//    ui.progressMessage->hide();

    // set flag for showing default mesh, i.e. p_voxel_mesh.
    isShowingMeshEXP = false;

    speed_test_mode = false;

    // use SS10 as default method.
    voxelization_method = 0;

    //Create and set working directory path
    workingdir = WORKING_DIR;
    //QString wd = QString::fromStdString(workingdir);
    if (!QDir(workingdir).exists())
    {
        QDir().mkdir(workingdir);
    }
    QDir::setCurrent(workingdir);
}
LoboVoxelizor::~LoboVoxelizor()
{
}

// Show message in the text window.
void LoboVoxelizor::displayMessage(QString& msg)
{
    //Change the color of the old context to black.
    QTextCursor cursor = ui.MessageText->textCursor();
    ui.MessageText->selectAll();
    ui.MessageText->setTextColor(Qt::black);
    ui.MessageText->setTextCursor(cursor);

    //Show the message.
    ui.MessageText->setTextColor(Qt::blue);
    ui.MessageText->append(msg);
}

// Clean the voxel meshs.
void LoboVoxelizor::cleanAllVoxelMesh()
{
    if (p_voxel_mesh)
    {
        delete p_voxel_mesh;
    }
    p_voxel_mesh = NULL;
}

//============================================================================================
// The actions respond to the main window.
//============================================================================================
void LoboVoxelizor::on_pushButtonVoxelize_clicked()
{
    if (p_voxel_mesh)
    {
        p_voxel_mesh->voxelization_method = voxelization_method;
        p_voxel_mesh->grid_density = ui.spinBoxGridDensity->value();
        p_voxel_mesh->computeVoxelMesh();

        //Output information of the obj file.
        QString msg;
        msg = "\nMethod " + QString::number(p_voxel_mesh->voxelization_method) +
                "\nGrid Density: " + QString::number(p_voxel_mesh->grid_density) +
                "\n#surface voxels: " + QString::number(p_voxel_mesh->volume_mesh.num_element_surface) +
                "\nVoxelizing surface: " + QString::number(p_voxel_mesh->time_surface,'g') + " us." +
                "\n#interior voxels : " + QString::number(p_voxel_mesh->volume_mesh.num_element_interior) +
                "\nVoxelizing interior: " + QString::number(p_voxel_mesh->time_interior,'g') + " us.";

        // Speed test of the voxel mesh.
        if (ui.checkBox_setSpeedTest->isChecked())
        {
            msg += speedTest();
        }

        displayMessage(msg);

        ui.viewer->setVoxelMesh(p_voxel_mesh);
        ui.viewer->updateGL();
    }
}

//-------------------------------------------------------------------------------------------
// Change the grid density.
//-------------------------------------------------------------------------------------------
void LoboVoxelizor::on_spinBoxGridDensity_editingFinished()
{
    int new_density = ui.spinBoxGridDensity->value();
    if (new_density < 0)
    {
        return;
    }

    if (!isShowingMeshEXP)
    {
        if (p_voxel_mesh && p_voxel_mesh->updateGridDensity(new_density, voxelization_method))
        {
            //Output information of the obj file.
            QString msg;
            msg = "\nMethod " + QString::number(p_voxel_mesh->voxelization_method) +
                  "\nGrid Density: " + QString::number(p_voxel_mesh->grid_density) +
                  "\n#surface voxels: " + QString::number(p_voxel_mesh->volume_mesh.num_element_surface) +
                  "\nVoxelizing surface: " + QString::number(p_voxel_mesh->time_surface,'g') + " us." +
                  "\n#interior voxels : " + QString::number(p_voxel_mesh->volume_mesh.num_element_interior) +
                  "\nVoxelizing interior: " + QString::number(p_voxel_mesh->time_interior,'g') + " us.";

            // Speed test of the voxel mesh.
            if (ui.checkBox_setSpeedTest->isChecked())
            {
                msg += speedTest();
            }

            displayMessage(msg);

            ui.viewer->setVoxelMesh(p_voxel_mesh);
            ui.viewer->updateGL();
        }
    }
    else
    {
//        if (p_voxel_meshEXP && p_voxel_meshEXP->updateGridDensity(new_density, voxelization_method, speed_test_mode))
//        {
//            //Output information of the obj file.
//            QString msg;
//            msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//                       "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//                       "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us." +
//                       "\n#interior voxels : " + QString::number(p_voxel_meshEXP->num_ele_interior) +
//                       "\nVoxelizing interior: " + QString::number(p_voxel_meshEXP->time_interior,'g') + " us.");
//            displayMessage(msg);

//            ui.viewer->updateGL();
//        }
    }
}

void LoboVoxelizor::on_actionNormalized_Mesh_changed()
{
    if (ui.actionNormalized_Mesh->isChecked())
    {
//        ui.actionOriginal_Mesh->setChecked(false);
//        ui.action2D_Projections->setChecked(false);
//        ui.viewer->setViewMode(0);
    }
}

void LoboVoxelizor::on_actionOriginal_Mesh_changed()
{
    if (ui.actionOriginal_Mesh->isChecked())
    {
//        ui.actionNormalized_Mesh->setChecked(false);
//        ui.action2D_Projections->setChecked(false);
//        ui.viewer->setViewMode(1);
    }
}

//-------------------------------------------------------------------------------------------
// Operations for the input and output files.
//-------------------------------------------------------------------------------------------
// Open the input obj file that could be triangle mesh or quad mesh.
void LoboVoxelizor::on_actionOpen_OBJ_file_triggered()
{
//    QString obj_file = QFileDialog::getOpenFileName(this, tr("Open OBJ mesh"), ".", tr("*.obj"));
//    if (!obj_file.length())
//    {
//        return;
//    }
//    cout << "obj file = " << obj_file.toUtf8().constData() << endl;

//    // Build a triangle mesh.
//    TriangleMesh* p_mesh = new TriangleMesh;
//    p_mesh->loadOBJFile(obj_file.toLatin1());
//    //p_mesh->setBoundingBox(10.0, 10.0, 10.0, -10.0, -10.0, -10.0);
//    p_mesh->setBoundingBox();

//    // Build the voxel mesh.
//    cleanAllVoxelMesh();
//    p_voxel_mesh = new VoxelMesh(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//    p_voxel_mesh->isSpeedTest = ui.checkBox_setSpeedTest->isChecked();
//    p_voxel_mesh->isFillingHole = ui.checkBox_setFillHoles->isChecked();
//    p_voxel_mesh->isMultiThread = ui.checkBox_setMultipleThreads->isChecked();
//    p_voxel_mesh->computeVoxelMesh();

//    //output information of the obj file.
//    QString msg;
//    msg = "---------------------------------------\nOBJ file: " + QString(obj_file.toLatin1()) +
//            "\n#Vertices: " + QString::number(p_voxel_mesh->triangle_mesh->num_node) +
//            "\n#Faces: " + QString::number(p_voxel_mesh->triangle_mesh->num_face) +
//            "\n#SubMeshes: " + QString::number(p_voxel_mesh->triangle_mesh->num_mesh) +
//            "\nGrid Density: " + QString::number(p_voxel_mesh->grid_density) +
//            "\n#surface voxels: " + QString::number(p_voxel_mesh->volume_mesh.num_element_surface) +
//            "\nVoxelizing surface: " + QString::number(p_voxel_mesh->time_surface,'g') +
//            " us. \n#interior voxels : " + QString::number(p_voxel_mesh->volume_mesh.num_element_interior) +
//            "\nVoxelizing interior: " + QString::number(p_voxel_mesh->time_interior,'g') + " us.";
//    displayMessage(msg);

//    // Pass to the viewer.
//    ui.viewer->setVoxelMesh(p_voxel_mesh);
//    ui.viewer->updateGL();

//    isShowingMeshEXP = false;
}

// Save the cubic element mesh files, i.e. '.ele' and '.node'.
void LoboVoxelizor::on_actionSave_ELE_Cubic_All_Elements_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to cubic format"), ".", tr("*.ele"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportAllNode(p_voxel_mesh, path.toLatin1());
        exportAllCubicELE(p_voxel_mesh, path.toLatin1());
        long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting Cubic ELE file (all elements) took " +
                   QString::number(timetoken) + "ms."
                   "\n#Submeshs: " + QString::number(p_voxel_mesh->volume_mesh.num_group) +
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node) +
                   "\n#Elements: " + QString::number(p_voxel_mesh->volume_mesh.num_element));
        displayMessage(msg);
    }
}

// Save the cubic element mesh files, i.e. '.ele' and '.node'.
// Only elements on the surface are counted.
void LoboVoxelizor::on_actionSave_ELE_Cubic_Surface_Only_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to cubic format"), ".", tr("*.ele"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportSurfaceNode(p_voxel_mesh, path.toLatin1());
        exportSurfaceCubicELE(p_voxel_mesh, path.toLatin1());
        long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting Cubic ELE file (surface elements only) took " +
                   QString::number(timetoken) + "ms."
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node_surface) +
                   "\n#Elements: " + QString::number(p_voxel_mesh->volume_mesh.num_element_surface));
        displayMessage(msg);
    }
}

// Save the tetrahedron element mesh files, i.e. '.ele' and '.node'.
// One cubic element generate 6 tetrahedron elements.
void LoboVoxelizor::on_actionSave_ELE_Tet_All_Elements_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to tetrahedron format"), ".", tr("*.ele"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportAllNode(p_voxel_mesh, path.toLatin1());
        exportAllTetELE(p_voxel_mesh, path.toLatin1());
        long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting Tet ELE file (all elements) took " +
                   QString::number(timetoken) + "ms."
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node) +
                   "\n#Elements: " + QString::number(p_voxel_mesh->volume_mesh.num_element*6));
        displayMessage(msg);
    }
}

// Save the tetrahedron element mesh files, i.e. '.ele' and '.node'.
// One cubic element generate 6 tetrahedron elements.
// Only elemets on the surface are counted.
void LoboVoxelizor::on_actionSave_ELE_Tet_Surface_Only_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to tetrahedron format"), ".", tr("*.ele"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportSurfaceNode(p_voxel_mesh, path.toLatin1());
        exportSurfaceTetELE(p_voxel_mesh, path.toLatin1());
        long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting Tet ELE file (surface elements only) took " +
                   QString::number(timetoken) + "ms."
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node_surface) +
                   "\n#Elements: " + QString::number(p_voxel_mesh->volume_mesh.num_element_surface*6));
        displayMessage(msg);
    }
}

//void LoboVoxelizor::on_actionSave_OBJ_file_Normalized_triggered()
//{
//    QString path = QFileDialog::getSaveFileName
//            (this, tr("Export to surface normalized (OBJ) format"), ".", tr("*.obj"));

//    if (path.length())
//    {
//        QElapsedTimer timer;
//        timer.start();
//        exportNormalizedOBJFiles(path.toLatin1());
//        long timetoken = timer.elapsed();

//        //output information.
//        QString msg;
//        msg.append("\nNormalized triangle file is generated successfully."
//                   "\n#Vertices: " + QString::number(p_voxel_mesh->triangle_mesh->num_node) +
//                   "\n#Faces: " + QString::number(p_voxel_mesh->triangle_mesh->num_face) +
//                   "\nExporting file took " + QString::number(timetoken) + "ms.");
//        displayMessage(msg);
//    }
//}

// Save the quad mesh file of the surface of the object, i.e. '.obj'.
void LoboVoxelizor::on_actionSave_OBJ_file_Quad_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to surface quad (OBJ) format"), ".", tr("*.obj"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportOBJFileQuad(p_voxel_mesh, path.toLatin1());
        long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting OBJ file (quad face) took " + QString::number(timetoken) + "ms."
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node_surface) +
                   "\n#Faces: " + QString::number(p_voxel_mesh->volume_mesh.num_face));
        displayMessage(msg);
    }
}

// Save the triangle mesh file of the surface of the object, i.e. '.obj'.
// One quad face generates 2 triangle faces.
void LoboVoxelizor::on_actionSave_OBJ_file_Triangle_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to surface triangle (OBJ) format"), ".", tr("*.obj"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportOBJFileTriangle(p_voxel_mesh, path.toLatin1());
        long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting OBJ file (triangle face) took " + QString::number(timetoken) + "ms."
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node_surface) +
                   "\n#Faces: " + QString::number(p_voxel_mesh->volume_mesh.num_face * 2));
        displayMessage(msg);
    }
}

// Save the quad mesh file of the entire voxelized object, i.e. '.obj'.
// One cubic element generates 6 quad faces.
void LoboVoxelizor::on_actionSave_OBJ_File_All_Quad_Face_triggered()
{
    // Return if voxel mesh is not ready.
    if (!p_voxel_mesh)
    {
        return;
    }

    QString path = QFileDialog::getSaveFileName
            (this, tr("Export to surface quad (OBJ) format"), ".", tr("*.obj"));

    if (path.length())
    {
        QElapsedTimer timer;
        timer.start();
        exportOBJFileQuadAllElement(p_voxel_mesh, path.toLatin1());
        long long timetoken = timer.elapsed();

        //output information.
        QString msg;
        msg.append("\nExporting OBJ file (quad face) took " + QString::number(timetoken) + "ms."
                   "\n#Vertices: " + QString::number(p_voxel_mesh->volume_mesh.num_node_surface) +
                   "\n#Faces: " + QString::number(p_voxel_mesh->volume_mesh.num_face));
        displayMessage(msg);
    }

}


//============================================================================================
// The primitive file IO functions.
//============================================================================================
//---------------------------------------------------------------------------------
//Export all nodes in node_list to node file 'filename.node'.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportAllNode(VoxelMesh *p_vox_mesh, const char* filename)
{
    string nodefilename;
    nodefilename.append(filename);
    int pos = nodefilename.find_first_of('.');
    if (pos != string::npos)
    {
        nodefilename.erase(pos, nodefilename.length());
    }
    nodefilename += ".node";

    ofstream ofs(nodefilename, ios::binary);
    long long num_node = p_vox_mesh->volume_mesh.num_node;
    ofs << num_node << " " << 3 << " " << 0 << " " << 0 << endl;

    // Show progree bar.
//    ui.progressMessage->setText("Exporting .node file...");
//    ui.progressMessage->show();
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_node);

    // Output the node list.
    stringstream buffer;
    buffer.str("");
    int line_count = 0;
    for (long k = 0; k < num_node; ++k)
    {
        Node* pnode = p_vox_mesh->volume_mesh.node_list[k];
        buffer << k << " "
               << pnode->ori_coordinate[0] << " "
               << pnode->ori_coordinate[1] << " "
               << pnode->ori_coordinate[2] << endl;
        line_count++;

        //put 10000 lines in the buffer.
        if (line_count >= 10000)
        {
            //write the buffer into the file.
            ofs.write(buffer.str().c_str(), buffer.str().length());
            buffer.str("");
            line_count = 0;
        }

        // Update the bar and keep GUI resposive.
//        ui.progressBar->setValue(bar_ratio * double(k));
        QCoreApplication::processEvents();
    }

    // write the last buffer to the file.
    ofs.write(buffer.str().c_str(), buffer.str().length());
    ofs.close();
}

//---------------------------------------------------------------------------------
//Export all SURFACE nodes in node_list to node file 'filename.node'.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportSurfaceNode(VoxelMesh *p_vox_mesh, const char* filename)
{
    string nodefilename;
    nodefilename.append(filename);
    int pos = nodefilename.find_first_of('.');
    if (pos != string::npos)
    {
        nodefilename.erase(pos, nodefilename.length());
    }
    nodefilename += ".node";

    ofstream ofs(nodefilename, ios::binary);
    long long num_node = p_vox_mesh->volume_mesh.num_node;
    //ofs << num_node << " " << 3 << " " << 0 << " " << 0 << endl;

    // Show progress bar.
//    ui.progressMessage->setText("Exporting .node file...");
//    ui.progressMessage->show();
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_node);

    // Output the node list.
    stringstream buffer;
    buffer.str("");
    int line_count = 0;
    for (long long k = 0; k < num_node; ++k)
    {
        Node* pnode = p_vox_mesh->volume_mesh.node_list[k];
        if (pnode->isOnSurfaceVoxelization)
        {
            buffer << pnode->idx_surface_ele << " "
                   << pnode->ori_coordinate[0] << " "
                   << pnode->ori_coordinate[1] << " "
                   << pnode->ori_coordinate[2] << endl;
            line_count++;

            //put 10000 lines in the buffer.
            if (line_count >= 10000)
            {
                //write the buffer into the file.
                ofs.write(buffer.str().c_str(), buffer.str().length());
                buffer.str("");
                line_count = 0;
            }

            // Update the bar and keep GUI resposive.
//            ui.progressBar->setValue(bar_ratio * double(k));
            QCoreApplication::processEvents();
        }
    }

    // write the last buffer to the file.
    ofs.write(buffer.str().c_str(), buffer.str().length());
    ofs.close();
}

//---------------------------------------------------------------------------------
//Export all cubic elements in mesh_list to element file 'filename.ele'.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportAllCubicELE(VoxelMesh *p_vox_mesh, const char* filename)
{
    string elefilename;
    elefilename.append(filename);
    int pos = elefilename.find_first_of('.');
    if (pos != string::npos)
    {
        elefilename.erase(pos, elefilename.length());
    }
    elefilename += ".ele";

    // Open the file as binary format.
    ofstream ofs(elefilename, ios::binary);
    long long num_element = p_vox_mesh->volume_mesh.num_element;
    //ofs << "#Total number of elements, 8 vertices and 6 neighbor elements" << endl;
    ofs << num_element << " " << 8 << " " << 6 << endl;

    // Show the progress bar.
//    ui.progressMessage->setText("Exporting .ele file...");
//    ui.progressBar->setValue(0);
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_element);

    // Output the ele file.
    stringstream buffer;
    int line_count, element_count = 0;
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g ele_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g ele_mesh" << mesh_idx << endl;
        }

        vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];
        for (vector<CubicElement*>::const_iterator
             pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
        {
            //Use Garcia's order in the element file.
            int n0 = (*pelement)->node7->index;
            int n1 = (*pelement)->node5->index;
            int n2 = (*pelement)->node4->index;
            int n3 = (*pelement)->node6->index;
            int n4 = (*pelement)->node3->index;
            int n5 = (*pelement)->node1->index;
            int n6 = (*pelement)->node0->index;
            int n7 = (*pelement)->node2->index;

            buffer << element_count++ << " "
                   << n0 << " " << n1 << " " << n2 << " " << n3 << " "
                   << n4 << " " << n5 << " " << n6 << " " << n7 << " ";

            //The neighbors' index(started from 0) are provided. -1 for no neighbor.
            if ((*pelement)->x_neg)
                buffer << (*pelement)->x_neg->index << " ";
            else
                buffer << -1 << " ";

            if ((*pelement)->x_pos)
                buffer << (*pelement)->x_pos->index << " ";
            else
                buffer << -1 << " ";

            if ((*pelement)->y_neg)
                buffer << (*pelement)->y_neg->index << " ";
            else
                buffer << -1 << " ";

            if ((*pelement)->y_pos)
                buffer << (*pelement)->y_pos->index << " ";
            else
                buffer << -1 << " ";

            if ((*pelement)->z_neg)
                buffer << (*pelement)->z_neg->index << " ";
            else
                buffer << -1 << " ";

            if ((*pelement)->z_pos)
                buffer << (*pelement)->z_pos->index << endl;
            else
                buffer << -1 << endl;

            //increment
            line_count++;

            if (line_count >= 10000)
            {
                //write the buffer into the file.
                ofs.write(buffer.str().c_str(), buffer.str().length());
                buffer.str("");
                line_count = 0;
            }

            // Update the bar and keep GUI resposive.
//            ui.progressBar->setValue(bar_ratio * double(element_count));
            QCoreApplication::processEvents();
        }

        //write the last buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}

//---------------------------------------------------------------------------------
//Export all SURFACE cubic elements in mesh_list to element file 'filename.ele'.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportSurfaceCubicELE(VoxelMesh *p_vox_mesh, const char* filename)
{
    string elefilename;
    elefilename.append(filename);
    int pos = elefilename.find_first_of('.');
    if (pos != string::npos)
    {
        elefilename.erase(pos, elefilename.length());
    }
    elefilename += ".ele";

    // Open the file as binary format.
    ofstream ofs(elefilename, ios::binary);
    long num_element = p_vox_mesh->volume_mesh.num_element_surface;
    //ofs << "#Total number of elements, 8 vertices and 6 neighbor elements" << endl
    ofs << num_element << " " << 8 << " " << 6 << endl;

    // Show the progress bar.
//    ui.progressMessage->setText("Exporting .ele file...");
//    ui.progressBar->setValue(0);
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_element);

    // Output the ele file.
    stringstream buffer;
    int line_count, element_count = 0;
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g ele_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g ele_mesh" << mesh_idx << endl;
        }

        vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];
        for (vector<CubicElement*>::const_iterator
             pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
        {
            if ((*pelement)->isOnSurfaceVoxelization)
            {
                //Use Garcia's order in the element file.
                int n0 = (*pelement)->node7->idx_surface_ele;
                int n1 = (*pelement)->node5->idx_surface_ele;
                int n2 = (*pelement)->node4->idx_surface_ele;
                int n3 = (*pelement)->node6->idx_surface_ele;
                int n4 = (*pelement)->node3->idx_surface_ele;
                int n5 = (*pelement)->node1->idx_surface_ele;
                int n6 = (*pelement)->node0->idx_surface_ele;
                int n7 = (*pelement)->node2->idx_surface_ele;

                buffer << element_count++ << " "
                       << n0 << " " << n1 << " " << n2 << " " << n3 << " "
                       << n4 << " " << n5 << " " << n6 << " " << n7 << " ";

                //The neighbors' index(started from 0) are provided. -1 for no neighbor.
                if ((*pelement)->x_neg)
                    buffer << (*pelement)->x_neg->index << " ";
                else
                    buffer << -1 << " ";

                if ((*pelement)->x_pos)
                    buffer << (*pelement)->x_pos->index << " ";
                else
                    buffer << -1 << " ";

                if ((*pelement)->y_neg)
                    buffer << (*pelement)->y_neg->index << " ";
                else
                    buffer << -1 << " ";

                if ((*pelement)->y_pos)
                    buffer << (*pelement)->y_pos->index << " ";
                else
                    buffer << -1 << " ";

                if ((*pelement)->z_neg)
                    buffer << (*pelement)->z_neg->index << " ";
                else
                    buffer << -1 << " ";

                if ((*pelement)->z_pos)
                    buffer << (*pelement)->z_pos->index << endl;
                else
                    buffer << -1 << endl;

                //increment
                line_count++;

                if (line_count >= 10000)
                {
                    //write the buffer into the file.
                    ofs.write(buffer.str().c_str(), buffer.str().length());
                    buffer.str("");
                    line_count = 0;
                }

                // Update the bar and keep GUI resposive.
//                ui.progressBar->setValue(bar_ratio * double(element_count));
                QCoreApplication::processEvents();
            }
        }

        //write the last buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}

//---------------------------------------------------------------------------------
//Export all tet elements in mesh_list to element file 'filename.ele'.
//One cubic element generates 6 tetrahedron elements.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportAllTetELE(VoxelMesh *p_vox_mesh, const char* filename)
{
    string elefilename;
    elefilename.append(filename);
    int pos = elefilename.find_first_of('.');
    if (pos != string::npos)
    {
        elefilename.erase(pos, elefilename.length());
    }
    elefilename += ".ele";

    ofstream ofs(elefilename, ios::binary);
    int num_element = p_vox_mesh->volume_mesh.num_element;
    ofs << num_element * 6 << " " << 4 << " " << 0 << endl;

    // Show the progress bar.
//    ui.progressMessage->setText("Exporting .ele file...");
//    ui.progressBar->setValue(0);
    double bar_ratio = 100.0 / double(num_element);

    //put 10000 lines in the buffer.
    stringstream buffer;
    int line_count, element_count = 0;
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g ele_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g ele_mesh" << mesh_idx << endl;
        }

        vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];
        for (vector<CubicElement*>::const_iterator
             pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
        {
            int n0 = (*pelement)->node0->index;
            int n1 = (*pelement)->node1->index;
            int n2 = (*pelement)->node2->index;
            int n3 = (*pelement)->node3->index;
            int n4 = (*pelement)->node4->index;
            int n5 = (*pelement)->node5->index;
            int n6 = (*pelement)->node6->index;
            int n7 = (*pelement)->node7->index;

            // Tet No. 1
            buffer << element_count++ << " "
                   << n0 << " " << n2 << " " << n3 << " " << n6 << endl;

            // Tet No. 2
            buffer << element_count++ << " "
                   << n0 << " " << n3 << " " << n7 << " " << n6 << endl;

            // Tet No. 3
            buffer << element_count++ << " "
                   << n0 << " " << n4 << " " << n6 << " " << n7 << endl;

            // Tet No. 4
            buffer << element_count++ << " "
                   << n1 << " " << n4 << " " << n7 << " " << n5 << endl;

            // Tet No. 5
            buffer << element_count++ << " "
                   << n1 << " " << n0 << " " << n7 << " " << n4 << endl;

            // Tet No. 6
            buffer << element_count++ << " "
                   << n1 << " " << n0 << " " << n3 << " " << n7 << endl;

            //increment
            line_count += 6;

            if (line_count >= 10000)
            {
                //write the buffer into the file.
                ofs.write(buffer.str().c_str(), buffer.str().length());
                buffer.str("");
                line_count = 0;
            }

            // Update the bar and keep GUI resposive.
//            ui.progressBar->setValue(bar_ratio * double(element_count));
            QCoreApplication::processEvents();
        }

        //write the last buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}

//---------------------------------------------------------------------------------
//Export all SURFACE tet elements in mesh_list to element file 'filename.ele'.
//One cubic element generates 6 tetrahedron elements.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportSurfaceTetELE(VoxelMesh *p_vox_mesh, const char* filename)
{
    string elefilename;
    elefilename.append(filename);
    int pos = elefilename.find_first_of('.');
    if (pos != string::npos)
    {
        elefilename.erase(pos, elefilename.length());
    }
    elefilename += ".ele";

    ofstream ofs(elefilename, ios::binary);
    int num_element = p_vox_mesh->volume_mesh.num_element_surface;
    ofs << num_element * 6 << " " << 4 << " " << 0 << endl;

    // Show the progress bar.
//    ui.progressMessage->setText("Exporting .ele file...");
//    ui.progressBar->setValue(0);
    double bar_ratio = 100.0 / double(num_element);

    //put 10000 lines in the buffer.
    stringstream buffer;
    int line_count, element_count = 0;
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g ele_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g ele_mesh" << mesh_idx << endl;
        }

        vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];
        for (vector<CubicElement*>::const_iterator
             pelement = p_elegroup->begin(); pelement != p_elegroup->end(); ++pelement)
        {
            if ((*pelement)->isOnSurfaceVoxelization)
            {
                int n0 = (*pelement)->node0->idx_surface_ele;
                int n1 = (*pelement)->node1->idx_surface_ele;
                int n2 = (*pelement)->node2->idx_surface_ele;
                int n3 = (*pelement)->node3->idx_surface_ele;
                int n4 = (*pelement)->node4->idx_surface_ele;
                int n5 = (*pelement)->node5->idx_surface_ele;
                int n6 = (*pelement)->node6->idx_surface_ele;
                int n7 = (*pelement)->node7->idx_surface_ele;

                // Tet No. 1
                buffer << element_count++ << " "
                       << n0 << " " << n2 << " " << n3 << " " << n6 << endl;

                // Tet No. 2
                buffer << element_count++ << " "
                       << n0 << " " << n3 << " " << n7 << " " << n6 << endl;

                // Tet No. 3
                buffer << element_count++ << " "
                       << n0 << " " << n4 << " " << n6 << " " << n7 << endl;

                // Tet No. 4
                buffer << element_count++ << " "
                       << n1 << " " << n4 << " " << n7 << " " << n5 << endl;

                // Tet No. 5
                buffer << element_count++ << " "
                       << n1 << " " << n0 << " " << n7 << " " << n4 << endl;

                // Tet No. 6
                buffer << element_count++ << " "
                       << n1 << " " << n0 << " " << n3 << " " << n7 << endl;

                //increment
                line_count += 6;

                if (line_count >= 10000)
                {
                    //write the buffer into the file.
                    ofs.write(buffer.str().c_str(), buffer.str().length());
                    buffer.str("");
                    line_count = 0;
                }

                // Update the bar and keep GUI resposive.
//                ui.progressBar->setValue(bar_ratio * double(element_count));
                QCoreApplication::processEvents();
            }
        }

        //write the last buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}

//---------------------------------------------------------------------------------
//Export surface quad mesh to obj file 'filename.obj'.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportOBJFileQuad(VoxelMesh *p_vox_mesh, const char* filename)
{
    string objfilename;
    objfilename.append(filename);
    int pos = objfilename.find_first_of('.');
    if (pos != string::npos)
    {
        objfilename.erase(pos, objfilename.length());
    }
    objfilename += ".obj";

    ofstream ofs(objfilename, ios::binary);
    int num_node = p_vox_mesh->volume_mesh.num_node_surface;
    int num_face = p_vox_mesh->volume_mesh.num_face;
    ofs << "#Vertices : " << num_node << endl;
    ofs << "#Quad faces : " << num_face << endl;

    // Use stringstream as a buffer.
    stringstream buffer;

    // Show  progress bar.
//    ui.progressMessage->setText("Exporting .obj file...");
//    ui.progressMessage->show();
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_node + num_face);

    //---------------------------------------------------------
    // Output vertices.
    //---------------------------------------------------------
    buffer.str("");
    int line_count = 0;
    for (int k=0; k < num_node; ++k)
    {
        Node* pnode = p_vox_mesh->volume_mesh.surface_node_list[k];
        buffer << "v "
               << pnode->ori_coordinate[0] << " "
               << pnode->ori_coordinate[1] << " "
               << pnode->ori_coordinate[2] << endl;
        line_count++;

        // Output to file every 10000 lines.
        if (line_count >= 10000)
        {
            //write the buffer into the file.
            ofs.write(buffer.str().c_str(), buffer.str().length());
            buffer.str("");
            line_count = 0;
        }

        // Update the progree bar and keep GUI resposive.
        //        ui.progressBar->setValue(bar_ratio * double(k));
        QCoreApplication::processEvents();
    }

    // Write the last buffer to the file.
    ofs.write(buffer.str().c_str(), buffer.str().length());

    //----------------------------------------------------------
    // Output faces.
    //----------------------------------------------------------
    int bar_count = num_node;
    int num_mesh = p_vox_mesh->volume_mesh.surface_mesh_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g obj_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g obj_mesh" << mesh_idx << endl;
        }

        vector<QuadFace*> *p_quadlist = &p_vox_mesh->volume_mesh.surface_mesh_lists[mesh_idx];
        for (vector<QuadFace*>::const_iterator
             pface = p_quadlist->begin(); pface != p_quadlist->end(); ++pface)
        {
            buffer << "f "
                   << (*pface)->idx0 + 1 << " "
                   << (*pface)->idx1 + 1 << " "
                   << (*pface)->idx2 + 1 << " "
                   << (*pface)->idx3 + 1 << endl;

            line_count++;

            // Output every 10000 lines.
            if (line_count >= 10000)
            {
                //write the buffer into the file.
                ofs.write(buffer.str().c_str(), buffer.str().length());
                buffer.str("");
                line_count = 0;
            }

            // Update the bar and keep GUI resposive.
//            ui.progressBar->setValue(bar_ratio * double(bar_count++));
            QCoreApplication::processEvents();
        }

        //write the last buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

    // Hide the bar.
//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}

//---------------------------------------------------------------------------------
//Export surface triangle mesh to obj file 'filename.obj'.
//One quad generates 2 triangles.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportOBJFileTriangle(VoxelMesh *p_vox_mesh, const char* filename)
{
    string objfilename;
    objfilename.append(filename);
    int pos = objfilename.find_first_of('.');
    if (pos != string::npos)
    {
        objfilename.erase(pos, objfilename.length());
    }
    objfilename += ".obj";

    ofstream ofs(objfilename, ios::binary);
    int num_node = p_vox_mesh->volume_mesh.num_node_surface;
    int num_face = p_vox_mesh->volume_mesh.num_face;
    ofs << "#Vertices : " << num_node << endl;
    ofs << "#Triangle faces : " << num_face * 2 << endl;

    // Show the progress bar.
//    ui.progressMessage->setText("Exporting .obj file...");
//    ui.progressMessage->show();
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_node + num_face);

    //-----------------------------------------------------------------------
    // Output vertices.
    //-----------------------------------------------------------------------
    stringstream buffer;
    buffer.str("");
    int line_count = 0;
    for (int k=0; k < num_node; ++k)
    {
        Node* pnode = p_vox_mesh->volume_mesh.surface_node_list[k];
        buffer << "v "
               << pnode->ori_coordinate[0] << " "
               << pnode->ori_coordinate[1] << " "
               << pnode->ori_coordinate[2] << endl;
        line_count++;

        // ouput to file every 10000 lines.
        if (line_count >= 10000)
        {
            //write the buffer into the file.
            ofs.write(buffer.str().c_str(), buffer.str().length());
            buffer.str("");
            line_count = 0;
        }

        // Update the bar and keep GUI resposive.
        //        ui.progressBar->setValue(bar_ratio * double(k));
        QCoreApplication::processEvents();
    }
    ofs.write(buffer.str().c_str(), buffer.str().length());

    //---------------------------------------------------------------------
    // Output faces.
    //---------------------------------------------------------------------
    int bar_count = num_node;
    int num_mesh = p_vox_mesh->volume_mesh.surface_mesh_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g obj_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g obj_mesh" << mesh_idx << endl;
        }

        vector<QuadFace*> *p_quadlist = &p_vox_mesh->volume_mesh.surface_mesh_lists[mesh_idx];
        for (vector<QuadFace*>::const_iterator
             pface = p_quadlist->begin(); pface != p_quadlist->end(); ++pface)
        {
            // divided the quad face to two triangle faces.
            buffer << "f "
                   << (*pface)->idx0 + 1 << " "
                   << (*pface)->idx1 + 1 << " "
                   << (*pface)->idx2 + 1 << endl;

            buffer << "f "
                   << (*pface)->idx2 + 1 << " "
                   << (*pface)->idx3 + 1 << " "
                   << (*pface)->idx0 + 1 << endl;

            line_count += 2;

            // Output every 10000 lines.
            if (line_count >= 10000)
            {
                //write the buffer into the file.
                ofs.write(buffer.str().c_str(), buffer.str().length());
                buffer.str("");
                line_count = 0;
            }

            // Update the bar and keep GUI resposive.
//            ui.progressBar->setValue(bar_ratio * double(bar_count++));
            QCoreApplication::processEvents();
        }

        //write the buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}

//---------------------------------------------------------------------------------
//Export all quad faces of all cubic elements to obj file 'filename.obj'.
//One cubic element has 6 quad faces.
//---------------------------------------------------------------------------------
void LoboVoxelizor::exportOBJFileQuadAllElement(VoxelMesh *p_vox_mesh, const char* filename)
{
    string objfilename;
    objfilename.append(filename);
    int pos = objfilename.find_first_of('.');
    if (pos != string::npos)
    {
        objfilename.erase(pos, objfilename.length());
    }
    objfilename += ".obj";

    ofstream ofs(objfilename, ios::binary);
    int num_node = p_vox_mesh->volume_mesh.num_node;
    int num_face = p_vox_mesh->volume_mesh.num_element * 6; //each element has 6 quad faces
    ofs << "#Vertices : " << num_node << endl;
    ofs << "#Quad faces : " << num_face << endl;

    // Use stringstream as a buffer.
    stringstream buffer;

    // Show  progress bar.
//    ui.progressMessage->setText("Exporting .obj file...");
//    ui.progressMessage->show();
//    ui.progressBar->show();
    double bar_ratio = 100.0 / double(num_node + num_face);

    //---------------------------------------------------------
    // Output vertices.
    //---------------------------------------------------------
    buffer.str("");
    int line_count = 0;
    for (int k=0; k < num_node; ++k)
    {
        Node* pnode = p_vox_mesh->volume_mesh.surface_node_list[k];
        buffer << "v "
               << pnode->ori_coordinate[0] << " "
               << pnode->ori_coordinate[1] << " "
               << pnode->ori_coordinate[2] << endl;
        line_count++;

        // Output to file every 10000 lines.
        if (line_count >= 10000)
        {
            //write the buffer into the file.
            ofs.write(buffer.str().c_str(), buffer.str().length());
            buffer.str("");
            line_count = 0;
        }

        // Update the progree bar and keep GUI resposive.
//        ui.progressBar->setValue(bar_ratio * double(k));
        QCoreApplication::processEvents();
    }

    // Write the last buffer to the file.
    ofs.write(buffer.str().c_str(), buffer.str().length());

    //----------------------------------------------------------
    // Output faces.
    //----------------------------------------------------------
    int face_count = 0;
    int num_mesh = p_vox_mesh->volume_mesh.element_group_lists.size();
    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
    {
        buffer.str("");
        line_count = 0;

        // If more than one sub mesh, put "g obj_mesh" before each sub mesh.
        if (num_mesh > 1)
        {
            buffer << "g obj_mesh" << mesh_idx << endl;
        }

        vector<CubicElement*> *p_elegroup = &p_vox_mesh->volume_mesh.element_group_lists[mesh_idx];
        for (vector<CubicElement*>::const_iterator
             pele = p_elegroup->begin(); pele != p_elegroup->end(); ++pele)
        {
            // output 6 faces of each element
            // face 1
            buffer << "f "
                   << (*pele)->node0->index + 1 << " "
                   << (*pele)->node1->index + 1 << " "
                   << (*pele)->node3->index + 1 << " "
                   << (*pele)->node2->index + 1 << endl;
            // face 2
            buffer << "f "
                   << (*pele)->node0->index + 1 << " "
                   << (*pele)->node2->index + 1 << " "
                   << (*pele)->node6->index + 1 << " "
                   << (*pele)->node4->index + 1 << endl;
            // face 3
            buffer << "f "
                   << (*pele)->node1->index + 1 << " "
                   << (*pele)->node5->index + 1 << " "
                   << (*pele)->node7->index + 1 << " "
                   << (*pele)->node3->index + 1 << endl;
            // face 4
            buffer << "f "
                   << (*pele)->node0->index + 1 << " "
                   << (*pele)->node4->index + 1 << " "
                   << (*pele)->node5->index + 1 << " "
                   << (*pele)->node1->index + 1 << endl;
            // face 5
            buffer << "f "
                   << (*pele)->node2->index + 1 << " "
                   << (*pele)->node3->index + 1 << " "
                   << (*pele)->node7->index + 1 << " "
                   << (*pele)->node6->index + 1 << endl;
            // face 6
            buffer << "f "
                   << (*pele)->node4->index + 1 << " "
                   << (*pele)->node6->index + 1 << " "
                   << (*pele)->node7->index + 1 << " "
                   << (*pele)->node5->index + 1 << endl;

            line_count += 6;

            // Output every 10000 lines.
            if (line_count >= 10000)
            {
                //write the buffer into the file.
                ofs.write(buffer.str().c_str(), buffer.str().length());
                buffer.str("");
                line_count = 0;
            }

            // Update the bar and keep GUI resposive.
//            ui.progressBar->setValue(bar_ratio * double(face_count++));
            QCoreApplication::processEvents();
        }

        //write the last buffer into the file.
        ofs.write(buffer.str().c_str(), buffer.str().length());
    }
    ofs.close();

    // Hide the bar.
//    ui.progressBar->hide();
//    ui.progressMessage->hide();
}


//==============================================================================
// Export the normalized input triangle mesh.
//==============================================================================
//void LoboVoxelizor::exportNormalizedOBJFiles(const char* filename)
//{
//    string objfilename;
//    objfilename.append(filename);
//    int pos = objfilename.find_first_of('.');
//    if (pos != string::npos)
//    {
//        objfilename.erase(pos, objfilename.length());
//    }
//    objfilename += ".obj";

//    ofstream ofs(objfilename, ios::binary);
//    int num_node = p_voxel_mesh->triangle_mesh->num_node;
//    int num_face = p_voxel_mesh->triangle_mesh->num_face;
//    ofs << "##Vertices : " << num_node << endl;
//    ofs << "##Triangle faces : " << num_face * 2 << endl;

//    // Show the progress bar.
//    ui.progressMessage->setText("Exporting .obj file...");
//    ui.progressMessage->show();
//    ui.progressBar->show();
//    double bar_ratio = 100.0 / double(num_node + num_face);

//    //----------------------------------------------------------------------
//    // Output vertices.
//    //----------------------------------------------------------------------
//    stringstream buffer;
//    buffer.str("");
//    int line_count = 0;
//    for (int k=0; k < num_node; ++k)
//    {
//        Node* pnode = p_voxel_mesh->triangle_mesh->node_list[k];
//        buffer << "v "
//               << pnode->norm_coordinate.data()[0] << " "
//               << pnode->norm_coordinate.data()[1] << " "
//               << pnode->norm_coordinate.data()[2] << endl;
//        line_count++;

//        // Output every 10000 lines.
//        if (line_count >= 10000)
//        {
//            //write the buffer into the file.
//            ofs.write(buffer.str().c_str(), buffer.str().length());
//            buffer.str("");
//            line_count = 0;
//        }

//        // To keep GUI resposive.
//        ui.progressBar->setValue(bar_ratio * double(k));
//        QCoreApplication::processEvents();
//    }

//    //----------------------------------------------------------------------------------
//    // Output faces.
//    //----------------------------------------------------------------------------------
//    buffer.str("");
//    line_count = 0;
//    int face_count = 0;
//    for (int mesh_idx=0; mesh_idx < p_voxel_mesh->triangle_mesh->num_mesh; ++mesh_idx)
//    {
//        for (const_iter_ptriangleface fi = p_voxel_mesh->triangle_mesh->mesh_list.at(mesh_idx)->begin();
//             fi != p_voxel_mesh->triangle_mesh->mesh_list.at(mesh_idx)->end(); ++fi)
//        {
//            buffer << "f "
//                   << (*fi)->idx0 + 1 << " "
//                   << (*fi)->idx1 + 1 << " "
//                   << (*fi)->idx2 + 1 << endl;
//            line_count++;
//        }

//        // output buffer every 10000 lines.
//        if (line_count >= 10000)
//        {
//            ofs.write(buffer.str().c_str(), buffer.str().length());
//            buffer.str("");
//            line_count = 0;
//        }

//        // Update the bar and keep GUI resposive.
//        ui.progressBar->setValue(bar_ratio * double(face_count++));
//        QCoreApplication::processEvents();
//    }

//    // Write the last buffer to the file.
//    ofs.write(buffer.str().c_str(), buffer.str().length());
//    ofs.close();

//    ui.progressBar->hide();
//    ui.progressMessage->hide();
//}

//====================================================================================
// For menu bar "EXPERIMENT"
//====================================================================================
// input multiple obj files, voxelize them and save the obj files one by one.
void LoboVoxelizor::on_actionMultiOBJ_AllElement_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Create and set output directory path
//    QString outputdir = workingdir + "/output/";
//    if (!QDir(outputdir).exists())
//    {
//        QDir().mkdir(outputdir);
//    }
//    QDir::setCurrent(outputdir);

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());

//        // Build a new voxel mesh.
//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);

//        // Set the name of the output file
//        string obj_output = outputdir.toStdString();
//        int found = obj_file.find_last_of("/");
//        if (found != string::npos)
//        {
//            found++;
//            int length = obj_file.size();
//            while (found < length)
//            {
//                obj_output += obj_file.at(found++);
//            }
//        }
//        cout << "obj output file = " << obj_output.c_str() << endl;

//        // output the obj file of all six faces of all the elements.
//        exportMulOBJFileQuadAllElement(obj_output.c_str());

//        QString msg;
//        msg.append("\n-----------------EXP-------------------\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//                   "\nObj file: " + QString(obj_output.c_str()) +
//                   "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us." +
//                   "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface));
//        displayMessage(msg);

//        delete p_mesh;
//        delete p_voxel_meshEXP;
//    }

//    QDir::setCurrent(workingdir);
}

// input multiple obj files, voxelize them and save the obj files one by one.
void LoboVoxelizor::on_actionMultiOBJ_File_Collisions_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Create and set output directory path
//    QString outputdir = workingdir + "/output/";
//    if (!QDir(outputdir).exists())
//    {
//        QDir().mkdir(outputdir);
//    }
//    QDir::setCurrent(outputdir);

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());

//        // Build a new voxel mesh.
//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);

//        // Set the name of the output file
//        string obj_output = outputdir.toStdString();
//        int found = obj_file.find_last_of("/");
//        if (found != string::npos)
//        {
//            found++;
//            int length = obj_file.size();
//            while (found < length)
//            {
//                obj_output += obj_file.at(found++);
//            }
//        }
//        cout << "obj output file = " << obj_output.c_str() << endl;

//        // Detect the collision.
//        detectCollision();

//        // Output the obj file of collision elements.
//        exportMulOBJFileQuadCollision(obj_output.c_str());

//        delete p_mesh;
//        delete p_voxel_meshEXP;
//    }

//    QDir::setCurrent(workingdir);
}

// Export obj file of six faces of all elements including the collision elements.
void LoboVoxelizor::on_actionMultiOBJ_AllElement_collision_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Create and set output directory path
//    QString outputdir = workingdir + "/output/";
//    if (!QDir(outputdir).exists())
//    {
//        QDir().mkdir(outputdir);
//    }
//    QDir::setCurrent(outputdir);

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());

//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);

//        // Set the name of the output file
//        string obj_output = outputdir.toStdString();
//        //cout << "obj output file = " << obj_output.c_str() << endl;
//        int found = obj_file.find_last_of("/");
//        if (found != string::npos)
//        {
//            found++;
//            int length = obj_file.size();
//            while (found < length)
//            {
//                obj_output += obj_file.at(found++);
//            }
//        }
//        cout << "obj output file = " << obj_output.c_str() << endl;

//        // Detect the collision.
//        detectCollision();

//        // Output the obj file of all six faces of all the elements, including collision elements.
//        exportMulOBJFileQuadAllElementWithCollision(obj_output.c_str());

//        delete p_mesh;
//        delete p_voxel_mesh;
//    }

//    QDir::setCurrent(workingdir);
}

// Add container to the input obj file.
void LoboVoxelizor::on_actionMultiOBJ_File_Add_Container_triggered()
{
    //Get the source OBJ file list.
    QFileDialog dialog(this);
    dialog.setDirectory(workingdir);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
    QStringList source_file_list;
    if (dialog.exec())
    {
        source_file_list = dialog.selectedFiles();
    }

    //Create and set output directory path
    QString outputdir = workingdir + "/output/";
    if (!QDir(outputdir).exists())
    {
        QDir().mkdir(outputdir);
    }
    QDir::setCurrent(outputdir);

    //Iterate all OBJ files.
    int num_files = source_file_list.size();
    for (int i=0; i<num_files; ++i)
    {
        // open the obj file one by one.
        string obj_file = source_file_list[i].toUtf8().constData();
        cout << "obj file = " << obj_file.c_str() << endl;

        // count the number of the vertices of the obj file.
        ifstream ifs(obj_file.c_str());
        char line[1024];
        char* token;
        char vertex[] = "v";
        int count = 0;

        while (!ifs.eof())
        {
            ifs.getline(line, 1024);
            token = strtok(line, " ");

            if (strlen(line) != 0 && strcmp(token, vertex) == 0)
            {
                count++;
            }
        }
        ifs.close();

        cout << "g container \n";

        // define the container geometry.
        double half_len = 5;
        double height = 2.0;
        zVec3 n0 = {-half_len,0,half_len};
        zVec3 n1 = {half_len,0,half_len};
        zVec3 n2 = {half_len,0,-half_len};
        zVec3 n3 = {-half_len,0,-half_len};
        zVec3 n4 = {-half_len,height,half_len};
        zVec3 n5 = {half_len,height,half_len};
        zVec3 n6 = {half_len,height,-half_len};
        zVec3 n7 = {-half_len,height,-half_len};

        // append the file by the container geometry.
        ofstream ofs(obj_file.c_str(), ofstream::app);
        ofs << "g container \n";

        // the vertices
        ofs << "v " << n0[0] << " " << n0[1] << " " << n0[2] << endl
            << "v " << n1[0] << " " << n1[1] << " " << n1[2] << endl
            << "v " << n2[0] << " " << n2[1] << " " << n2[2] << endl
            << "v " << n3[0] << " " << n3[1] << " " << n3[2] << endl
            << "v " << n4[0] << " " << n4[1] << " " << n4[2] << endl
            << "v " << n5[0] << " " << n5[1] << " " << n5[2] << endl
            << "v " << n6[0] << " " << n6[1] << " " << n6[2] << endl
            << "v " << n7[0] << " " << n7[1] << " " << n7[2] << endl;

        // the faces
        count++;
        ofs << "f " << count+0 << " " << count+4 << " " << count+5 << " " << count+1 << endl
            << "f " << count+1 << " " << count+5 << " " << count+6 << " " << count+2 << endl
            << "f " << count+2 << " " << count+6 << " " << count+7 << " " << count+3 << endl
            << "f " << count+0 << " " << count+3 << " " << count+7 << " " << count+4 << endl
            << "f " << count+0 << " " << count+1 << " " << count+2 << " " << count+3 << endl;

        ofs.close();
    }

    QDir::setCurrent(workingdir);
}

// Detect the collisions among all the sub meshes in the cubic mesh.
void LoboVoxelizor::detectCollision()
{
//    int grid_density = ui.spinBoxGridDensity->value();

//    //-------------------------------------------------------------------------------
//    // Initialize the detection hash table, a char array.
//    // 0: no object element is here.
//    // 1: an object element.
//    // 2: collision occured.
//    //-------------------------------------------------------------------------------
//    long sizearray = grid_density * grid_density *(long)grid_density;
//    cout << "Size of the collision table = " << sizearray << endl;

//    // The map is big, try memorry allocation.
//    unsigned char *collision_table;
//    try
//    {
//        collision_table = new unsigned char[sizearray];
//    }
//    catch(...)
//    {
//        cout << "Allocating memory for the collision table failed!" << endl;
//        return;
//    }
//    cout << "Allocating memory for the collision table succeeded." << endl;

//    // Set the map to all zeros.
//    memset(collision_table, 0, sizeof(unsigned char) * sizearray);

//    // Clear the collision element list.
//    collision_ele_list.clear();

//    // Detect collison.
//    int num_mesh = p_voxel_meshEXP->mesh_list.size();
//    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
//    {
//        for (vector<CubicElement*>::const_iterator
//             pelement = p_voxel_meshEXP->mesh_list.at(mesh_idx)->begin();
//             pelement != p_voxel_meshEXP->mesh_list.at(mesh_idx)->end(); ++pelement)
//        {
//            long vox_idx = (*pelement)->vox_idx;

//            if (collision_table[vox_idx] == 0)
//            {
//                // No element is here yet.
//                collision_table[vox_idx]++;

//                // Set the element not a collision.
//                (*pelement)->isCollision = false;
//            }
//            else if (collision_table[vox_idx] == 1)
//            {
//                // An element has already been here. Collision occured.
//                collision_table[vox_idx]++;
//                cout << "collision vox_idx = " << vox_idx << endl;
//                collision_ele_list.push_back(*pelement);

//                // Set the element a collision.
//                (*pelement)->isCollision = true;
//            }
//            else if (collision_table[vox_idx] == 2)
//            {
//                // Collision here more than once, the element has been recorded.
//                ;
//            }
//        }
//    }

//    delete collision_table;
}

// Output the obj file of all the elements.
// Six faces of each element.
void LoboVoxelizor::exportMulOBJFileQuadAllElement(const char* filename)
{
//    string objfilename;
//    objfilename.append(filename);
//    int pos = objfilename.find_first_of('.');
//    if (pos != string::npos)
//    {
//        objfilename.erase(pos, objfilename.length());
//    }
//    objfilename += ".obj";

//    ofstream ofs(objfilename, ios::binary);
//    int num_node = p_voxel_meshEXP->num_node;
//    int num_element = p_voxel_meshEXP->num_ele;
//    ofs << "#Vertices : " << num_node << endl;
//    ofs << "#Triangle faces : " << num_element * 6 << endl;

//    // Show the bar.
////    ui.progressMessage->setText("Exporting .obj file...");
////    ui.progressMessage->show();
////    ui.progressBar->show();
//    double bar_ratio = 100.0 / double(num_node + num_element);
//    int bar_count = 0;

//    //-----------------------------------------------------------------------------
//    // Output vertices.
//    //-----------------------------------------------------------------------------
//    stringstream buffer;
//    buffer.str("");
//    int line_count = 0;
//    for (int k=0; k < num_node; ++k)
//    {
//        Node* pnode = p_voxel_meshEXP->node_list[k];
//        buffer << "v "
//               << pnode->ori_coordinate[0] << " "
//               << pnode->ori_coordinate[1] << " "
//               << pnode->ori_coordinate[2] << endl;
//        line_count++;

//        if (line_count >= 10000)
//        {
//            //write the buffer into the file.
//            ofs.write(buffer.str().c_str(), buffer.str().length());
//            buffer.str("");
//            line_count = 0;
//        }

//        // Show the bar and keep GUI resposive.
////        ui.progressBar->setValue(bar_ratio * double(bar_count++));
//        QCoreApplication::processEvents();
//    }

//    //write the last buffer into the file.
//    ofs.write(buffer.str().c_str(), buffer.str().length());

//    //------------------------------------------------------------------------------
//    // Output faces.
//    //------------------------------------------------------------------------------
//    int num_mesh = p_voxel_meshEXP->mesh_list.size();
//    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
//    {
//        buffer.str("");
//        line_count = 0;

//        // If more than one sub mesh, put "g ele_mesh" before each sub mesh.
//        if (num_mesh > 1)
//        {
//            buffer << "g ele_mesh" << mesh_idx << endl;
//        }

//        for (vector<CubicElement*>::const_iterator
//             pelement = p_voxel_meshEXP->mesh_list.at(mesh_idx)->begin();
//             pelement != p_voxel_meshEXP->mesh_list.at(mesh_idx)->end(); ++pelement)
//        {
//            //Use Garcia's order in the element file.
//            int n0 = (*pelement)->node7->index;
//            int n1 = (*pelement)->node5->index;
//            int n2 = (*pelement)->node4->index;
//            int n3 = (*pelement)->node6->index;
//            int n4 = (*pelement)->node3->index;
//            int n5 = (*pelement)->node1->index;
//            int n6 = (*pelement)->node0->index;
//            int n7 = (*pelement)->node2->index;

//            buffer << "f "
//                   << n0+1 << " " << n1+1 << " " << n2+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n3+1 << " " << n7+1 << " " << n4+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n4+1 << " " << n5+1 << " " << n1+1 << endl;

//            buffer << "f "
//                   << n1+1 << " " << n5+1 << " " << n6+1 << " " << n2+1 << endl;

//            buffer << "f "
//                   << n2+1 << " " << n6+1 << " " << n7+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n4+1 << " " << n7+1 << " " << n6+1 << " " << n5+1 << endl;

//            line_count += 6;

//            if (line_count >= 10000)
//            {
//                //write the buffer into the file.
//                ofs.write(buffer.str().c_str(), buffer.str().length());
//                buffer.str("");
//                line_count = 0;
//            }

//            // Update the bar and keep GUI resposive.
////            ui.progressBar->setValue(bar_ratio * double(bar_count++));
//            QCoreApplication::processEvents();
//        }

//        //write the last buffer into the file.
//        ofs.write(buffer.str().c_str(), buffer.str().length());
//    }
//    ofs.close();

////    ui.progressBar->hide();
////    ui.progressMessage->hide();
}

void LoboVoxelizor::exportMulOBJFileQuadAllElementWithCollision(const char* filename)
{
//    string objfilename;
//    objfilename.append(filename);
//    int pos = objfilename.find_first_of('.');
//    if (pos != string::npos)
//    {
//        objfilename.erase(pos, objfilename.length());
//    }
//    objfilename += ".obj";

//    ofstream ofs(objfilename, ios::binary);
//    int num_node = p_voxel_meshEXP->num_node;
//    int num_element = p_voxel_meshEXP->num_ele;
//    ofs << "#Vertices : " << num_node << endl;
//    ofs << "#Triangle faces : " << num_element * 6 << endl;

//    // Show the bar.
////    ui.progressMessage->setText("Exporting .obj file...");
////    ui.progressMessage->show();
////    ui.progressBar->show();
//    double bar_ratio = 100.0 / double(num_node + num_element);
//    int bar_count = 0;

//    //-----------------------------------------------------------------------------
//    // Output vertices.
//    //-----------------------------------------------------------------------------
//    stringstream buffer;
//    buffer.str("");
//    int line_count = 0;
//    for (int k=0; k < num_node; ++k)
//    {
//        Node* pnode = p_voxel_meshEXP->node_list[k];
//        buffer << "v "
//               << pnode->ori_coordinate[0] << " "
//               << pnode->ori_coordinate[1] << " "
//               << pnode->ori_coordinate[2] << endl;
//        line_count++;

//        if (line_count >= 10000)
//        {
//            //write the buffer into the file.
//            ofs.write(buffer.str().c_str(), buffer.str().length());
//            buffer.str("");
//            line_count = 0;
//        }

//        // Show the bar and keep GUI resposive.
////        ui.progressBar->setValue(bar_ratio * double(bar_count++));
//        QCoreApplication::processEvents();
//    }

//    //write the last buffer into the file.
//    ofs.write(buffer.str().c_str(), buffer.str().length());

//    //------------------------------------------------------------------------------
//    // Output faces.
//    //------------------------------------------------------------------------------
//    int num_mesh = p_voxel_meshEXP->mesh_list.size();
//    for (int mesh_idx=0; mesh_idx < num_mesh; ++mesh_idx)
//    {
//        buffer.str("");
//        line_count = 0;

//        // If more than one sub mesh, put "g ele_mesh" before each sub mesh.
//        if (num_mesh > 1 || collision_ele_list.size() > 0)
//        {
//            buffer << "g ele_mesh" << mesh_idx << endl;
//        }

//        for (vector<CubicElement*>::const_iterator
//             pelement = p_voxel_meshEXP->mesh_list.at(mesh_idx)->begin();
//             pelement != p_voxel_meshEXP->mesh_list.at(mesh_idx)->end(); ++pelement)
//        {
//            //Use Garcia's order in the element file.
//            int n0 = (*pelement)->node7->index;
//            int n1 = (*pelement)->node5->index;
//            int n2 = (*pelement)->node4->index;
//            int n3 = (*pelement)->node6->index;
//            int n4 = (*pelement)->node3->index;
//            int n5 = (*pelement)->node1->index;
//            int n6 = (*pelement)->node0->index;
//            int n7 = (*pelement)->node2->index;

//            buffer << "f "
//                   << n0+1 << " " << n1+1 << " " << n2+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n3+1 << " " << n7+1 << " " << n4+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n4+1 << " " << n5+1 << " " << n1+1 << endl;

//            buffer << "f "
//                   << n1+1 << " " << n5+1 << " " << n6+1 << " " << n2+1 << endl;

//            buffer << "f "
//                   << n2+1 << " " << n6+1 << " " << n7+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n4+1 << " " << n7+1 << " " << n6+1 << " " << n5+1 << endl;

//            line_count += 6;

//            if (line_count >= 10000)
//            {
//                //write the buffer into the file.
//                ofs.write(buffer.str().c_str(), buffer.str().length());
//                buffer.str("");
//                line_count = 0;
//            }

//            // Update the bar and keep GUI resposive.
////            ui.progressBar->setValue(bar_ratio * double(bar_count++));
//            QCoreApplication::processEvents();
//        }

//        //write the last buffer into the file.
//        ofs.write(buffer.str().c_str(), buffer.str().length());
//    }

//    //------------------------------------------------------------------------------
//    // Output the faces of the collision elements.
//    //------------------------------------------------------------------------------
//    if (collision_ele_list.size() > 0)
//    {
//        buffer.str("");
//        line_count = 0;
//        buffer << "g ele_mesh_collision" << endl;
//        for (vector<CubicElement*>::const_iterator
//             pelement = collision_ele_list.begin();
//             pelement != collision_ele_list.end(); ++pelement)
//        {
//            //Use Garcia's order in the element file.
//            int n0 = (*pelement)->node7->index;
//            int n1 = (*pelement)->node5->index;
//            int n2 = (*pelement)->node4->index;
//            int n3 = (*pelement)->node6->index;
//            int n4 = (*pelement)->node3->index;
//            int n5 = (*pelement)->node1->index;
//            int n6 = (*pelement)->node0->index;
//            int n7 = (*pelement)->node2->index;

//            buffer << "f "
//                   << n0+1 << " " << n1+1 << " " << n2+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n3+1 << " " << n7+1 << " " << n4+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n4+1 << " " << n5+1 << " " << n1+1 << endl;

//            buffer << "f "
//                   << n1+1 << " " << n5+1 << " " << n6+1 << " " << n2+1 << endl;

//            buffer << "f "
//                   << n2+1 << " " << n6+1 << " " << n7+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n4+1 << " " << n7+1 << " " << n6+1 << " " << n5+1 << endl;

//            line_count += 6;

//            if (line_count >= 10000)
//            {
//                //write the buffer into the file.
//                ofs.write(buffer.str().c_str(), buffer.str().length());
//                buffer.str("");
//                line_count = 0;
//            }

//        }
//        //write the buffer into the file.
//        ofs.write(buffer.str().c_str(), buffer.str().length());
//    }

//    ofs.close();

////    ui.progressBar->hide();
////    ui.progressMessage->hide();
}

void LoboVoxelizor::exportMulOBJFileQuadCollision(const char* filename)
{
//    string objfilename;
//    objfilename.append(filename);
//    int pos = objfilename.find_first_of('.');
//    if (pos != string::npos)
//    {
//        objfilename.erase(pos, objfilename.length());
//    }
//    objfilename += ".obj";

//    ofstream ofs(objfilename, ios::binary);
//    int num_node = p_voxel_meshEXP->num_node;
//    int num_element = collision_ele_list.size();
//    ofs << "#Vertices : " << num_node << endl;
//    ofs << "#Triangle faces : " << num_element * 6 << endl;

//    // Show the bar.
////    ui.progressMessage->setText("Exporting .obj file...");
////    ui.progressMessage->show();
////    ui.progressBar->show();

//    // Output collisions if available.
//    if (num_element > 0)
//    {
//        stringstream buffer;
//        buffer.str("");
//        int line_count = 0;

//        //-----------------------------------------------------------------------------
//        // Output vertices.
//        //-----------------------------------------------------------------------------
//        for (int k=0; k < num_node; ++k)
//        {
//            Node* pnode = p_voxel_meshEXP->node_list[k];
//            buffer << "v "
//                   << pnode->ori_coordinate[0] << " "
//                   << pnode->ori_coordinate[1] << " "
//                   << pnode->ori_coordinate[2] << endl;
//            line_count++;

//            if (line_count >= 10000)
//            {
//                //write the buffer into the file.
//                ofs.write(buffer.str().c_str(), buffer.str().length());
//                buffer.str("");
//                line_count = 0;
//            }

//        }

//        //write the last buffer into the file.
//        ofs.write(buffer.str().c_str(), buffer.str().length());

//        //------------------------------------------------------------------------------
//        // Output the faces of the collision elements.
//        //------------------------------------------------------------------------------
//        buffer.str("");
//        line_count = 0;
//        buffer << "g ele_mesh_collision" << endl;
//        for (vector<CubicElement*>::const_iterator
//             pelement = collision_ele_list.begin();
//             pelement != collision_ele_list.end(); ++pelement)
//        {
//            //Use Garcia's order in the element file.
//            int n0 = (*pelement)->node7->index;
//            int n1 = (*pelement)->node5->index;
//            int n2 = (*pelement)->node4->index;
//            int n3 = (*pelement)->node6->index;
//            int n4 = (*pelement)->node3->index;
//            int n5 = (*pelement)->node1->index;
//            int n6 = (*pelement)->node0->index;
//            int n7 = (*pelement)->node2->index;

//            buffer << "f "
//                   << n0+1 << " " << n1+1 << " " << n2+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n3+1 << " " << n7+1 << " " << n4+1 << endl;

//            buffer << "f "
//                   << n0+1 << " " << n4+1 << " " << n5+1 << " " << n1+1 << endl;

//            buffer << "f "
//                   << n1+1 << " " << n5+1 << " " << n6+1 << " " << n2+1 << endl;

//            buffer << "f "
//                   << n2+1 << " " << n6+1 << " " << n7+1 << " " << n3+1 << endl;

//            buffer << "f "
//                   << n4+1 << " " << n7+1 << " " << n6+1 << " " << n5+1 << endl;

//            line_count += 6;

//            if (line_count >= 10000)
//            {
//                //write the buffer into the file.
//                ofs.write(buffer.str().c_str(), buffer.str().length());
//                buffer.str("");
//                line_count = 0;
//            }

//        }
//        //write the buffer into the file.
//        ofs.write(buffer.str().c_str(), buffer.str().length());
//    }

//    ofs.close();

////    ui.progressBar->hide();
////    ui.progressMessage->hide();
}

// Generate obj files of detailed triangle animation, voxel by voxel.
void LoboVoxelizor::on_actionTriangle_triggered()
{
//    // open the triangle file
//    string obj_filename("triangle.obj");

//    TriangleMesh* p_tri_mesh = new TriangleMesh;
//    p_tri_mesh->loadOBJFile(obj_filename.c_str());
//    p_tri_mesh->setBoundingBox();

//    if (p_voxel_meshEXP)
//    {
//        delete p_voxel_meshEXP;
//    }
//    p_voxel_meshEXP = new VoxelMeshEXP(p_tri_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//    p_voxel_meshEXP->computeVoxelMesh();

//    //output information of the obj file.
//    QString msg;
//    msg.append("OBJ file: " + QString(obj_filename.c_str()) +
//               "\n#Vertices: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_node) +
//               "\n#Faces: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_face) +
//               "\n#SubMeshes: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_mesh) + "\n");
//    msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//               "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//               "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " ms." +
//               "\n#interior voxels : " + QString::number(p_voxel_meshEXP->num_ele_interior) +
//               "\nVoxelizing interior: " + QString::number(p_voxel_meshEXP->time_interior,'g') + " ms.");
//    displayMessage(msg);

//    //Create and set output directory path
//    QString outputdir = workingdir + "/triangle_animation/";
//    if (!QDir(outputdir).exists())
//    {
//        QDir().mkdir(outputdir);
//    }
//    QDir::setCurrent(outputdir);

//    vector<CubicElement*>* p_submesh = p_voxel_meshEXP->mesh_list[0];
//    int num_ele = p_voxel_meshEXP->num_ele;

//    //=======================================================================
//    // OUTPUT VERTICES
//    //=======================================================================
//    cout << "p_voxel_meshEXP->vertex_list.size " << p_voxel_meshEXP->vertex_list.size() << endl;
//    outputdir = workingdir + "/triangle_animation/vetices/";
//    if (!QDir(outputdir).exists())
//    {
//        QDir().mkdir(outputdir);
//    }
//    QDir::setCurrent(outputdir);
//    for (int i=0; i<3; ++i)
//    {
//        // set the output file
//        char objfilename[512];
//        sprintf(objfilename, "tri_vertex_%02i.obj", i);
//        ofstream ofs(objfilename, ios::binary);

//        long long vox_idx = p_voxel_meshEXP->vertex_list[i];

//        // search the element list by the index.
//        CubicElement* p_ele = NULL;
//        for (int k=0; k<num_ele; ++k)
//        {
//            if (vox_idx == p_submesh->at(k)->vox_idx)
//            {
//                p_ele = p_submesh->at(k);
//            }
//        }

//        // output the six faces to obj faces
//        if (p_ele)
//        {
//            zVec3 v0 = p_ele->node0->ori_coordinate;
//            zVec3 v1 = p_ele->node1->ori_coordinate;
//            zVec3 v2 = p_ele->node2->ori_coordinate;
//            zVec3 v3 = p_ele->node3->ori_coordinate;
//            zVec3 v4 = p_ele->node4->ori_coordinate;
//            zVec3 v5 = p_ele->node5->ori_coordinate;
//            zVec3 v6 = p_ele->node6->ori_coordinate;
//            zVec3 v7 = p_ele->node7->ori_coordinate;

//            int n0 = 0;
//            int n1 = 1;
//            int n2 = 2;
//            int n3 = 3;
//            int n4 = 4;
//            int n5 = 5;
//            int n6 = 6;
//            int n7 = 7;

//            ofs << "v " << v0[0] << " " << v0[1] << " " << v0[2] << endl
//                << "v " << v1[0] << " " << v1[1] << " " << v1[2] << endl
//                << "v " << v2[0] << " " << v2[1] << " " << v2[2] << endl
//                << "v " << v3[0] << " " << v3[1] << " " << v3[2] << endl
//                << "v " << v4[0] << " " << v4[1] << " " << v4[2] << endl
//                << "v " << v5[0] << " " << v5[1] << " " << v5[2] << endl
//                << "v " << v6[0] << " " << v6[1] << " " << v6[2] << endl
//                << "v " << v7[0] << " " << v7[1] << " " << v7[2] << endl;

//            ofs << "f " << n0+1 << " " << n2+1 << " " << n3+1 << " " << n1+1 << endl
//                << "f " << n0+1 << " " << n1+1 << " " << n5+1 << " " << n4+1 << endl
//                << "f " << n1+1 << " " << n3+1 << " " << n7+1 << " " << n5+1 << endl
//                << "f " << n3+1 << " " << n2+1 << " " << n6+1 << " " << n7+1 << endl
//                << "f " << n0+1 << " " << n4+1 << " " << n6+1 << " " << n2+1 << endl
//                << "f " << n4+1 << " " << n5+1 << " " << n7+1 << " " << n6+1 << endl;
//        }
//        else
//        {
//            cout << "error, voxel element not found." << endl;
//        }

//        ofs.close();
//    }

//    //=======================================================================
//    // OUTPUT EDGES
//    //=======================================================================
//    cout << "p_voxel_meshEXP->edge_list.size " << p_voxel_meshEXP->edge_list.size() << endl;
//    outputdir = workingdir + "/triangle_animation/edges/";
//    if (!QDir(outputdir).exists())
//    {
//        QDir().mkdir(outputdir);
//    }
//    QDir::setCurrent(outputdir);
//    for (int i=0; i<3; ++i)
//    {
//        vector<long long>* p_edge = p_voxel_meshEXP->edge_list[i];

//        int num_vox = p_edge->size();
//        for (int j=0; j<num_vox; ++j)
//        {
//            // set the output file
//            char objfilename[512];
//            sprintf(objfilename, "tri_edge_%02i_%03i.obj", i, j);
//            ofstream ofs(objfilename, ios::binary);

//            long long vox_idx = p_edge->at(j);

//            // search the element list by the index.
//            CubicElement* p_ele = NULL;
//            for (int k=0; k<num_ele; ++k)
//            {
//                if (vox_idx == p_submesh->at(k)->vox_idx)
//                {
//                    p_ele = p_submesh->at(k);
//                }
//            }

//            // output the six faces to obj faces
//            if (p_ele)
//            {
//                zVec3 v0 = p_ele->node0->ori_coordinate;
//                zVec3 v1 = p_ele->node1->ori_coordinate;
//                zVec3 v2 = p_ele->node2->ori_coordinate;
//                zVec3 v3 = p_ele->node3->ori_coordinate;
//                zVec3 v4 = p_ele->node4->ori_coordinate;
//                zVec3 v5 = p_ele->node5->ori_coordinate;
//                zVec3 v6 = p_ele->node6->ori_coordinate;
//                zVec3 v7 = p_ele->node7->ori_coordinate;

//                int n0 = 0;
//                int n1 = 1;
//                int n2 = 2;
//                int n3 = 3;
//                int n4 = 4;
//                int n5 = 5;
//                int n6 = 6;
//                int n7 = 7;

//                ofs << "v " << v0[0] << " " << v0[1] << " " << v0[2] << endl
//                    << "v " << v1[0] << " " << v1[1] << " " << v1[2] << endl
//                    << "v " << v2[0] << " " << v2[1] << " " << v2[2] << endl
//                    << "v " << v3[0] << " " << v3[1] << " " << v3[2] << endl
//                    << "v " << v4[0] << " " << v4[1] << " " << v4[2] << endl
//                    << "v " << v5[0] << " " << v5[1] << " " << v5[2] << endl
//                    << "v " << v6[0] << " " << v6[1] << " " << v6[2] << endl
//                    << "v " << v7[0] << " " << v7[1] << " " << v7[2] << endl;

//                ofs << "f " << n0+1 << " " << n2+1 << " " << n3+1 << " " << n1+1 << endl
//                    << "f " << n0+1 << " " << n1+1 << " " << n5+1 << " " << n4+1 << endl
//                    << "f " << n1+1 << " " << n3+1 << " " << n7+1 << " " << n5+1 << endl
//                    << "f " << n3+1 << " " << n2+1 << " " << n6+1 << " " << n7+1 << endl
//                    << "f " << n0+1 << " " << n4+1 << " " << n6+1 << " " << n2+1 << endl
//                    << "f " << n4+1 << " " << n5+1 << " " << n7+1 << " " << n6+1 << endl;
//            }
//            else
//            {
//                cout << "error, voxel element not found." << endl;
//            }

//            ofs.close();
//        }
//    }

//    //=======================================================================
//    // OUTPUT SCANLINES AND SLICES
//    //=======================================================================
//    // Get the number of slices.
//    int num_scanline = p_voxel_meshEXP->scanline_list.size();
//    int num_slice = p_voxel_meshEXP->scanline_list[num_scanline-1]->at(0);
//    cout << "p_voxel_meshEXP->scanline_list.size " << num_slice << endl;

//    // The slice list is for recording each slice object.
//    vector<vector<CubicElement*>*> slice_list(num_slice);

//    // Output the obj files
//    int scanline_count = 0;
//    vector<long long>* p_scanline = p_voxel_meshEXP->scanline_list[0];
//    for (int i=0; i<=num_slice; ++i)
//    {
//        // set directories
//        char dir[512];
//        sprintf(dir, "/triangle_animation/slices%02d/", i);
//        QString outputdir1 = workingdir + dir;
//        if (!QDir(outputdir1).exists())
//        {
//            QDir().mkdir(outputdir1);
//        }

//        sprintf(dir, "/triangle_animation/slices%02d_ends/", i);
//        QString outputdir2 = workingdir + dir;
//        if (!QDir(outputdir2).exists())
//        {
//            QDir().mkdir(outputdir2);
//        }

//        // build a slice.
//        vector<CubicElement*>* p_slice = new vector<CubicElement*>;
//        slice_list.push_back(p_slice);

//        // iterate the scanlines in the slice.
//        while (p_scanline->at(0) == i)
//        {
//            int num_vox = p_scanline->size();

//            //----------------------------------------------------------------------
//            // output the voxels in the scanline.
//            //----------------------------------------------------------------------
//            QDir::setCurrent(outputdir1);
//            for (int j=1; j<num_vox; ++j)
//            {
//                long long vox_idx = p_scanline->at(j);

//                // set the output file
//                char objfilename[512];
//                sprintf(objfilename, "tri_slice_%02d_scanline_%03d_%03d.obj", i, scanline_count, j);
//                ofstream ofs(objfilename, ios::binary);

//                cout << vox_idx << " " << objfilename;

//                // search the element list by the index.
//                CubicElement* p_ele = NULL;
//                for (int k=0; k<num_ele; ++k)
//                {
//                    if (vox_idx == p_submesh->at(k)->vox_idx)
//                    {
//                        p_ele = p_submesh->at(k);
//                    }
//                }

//                // output the six faces to obj faces
//                if (p_ele)
//                {
//                    p_slice->push_back(p_ele);

//                    zVec3 v0 = p_ele->node0->ori_coordinate;
//                    zVec3 v1 = p_ele->node1->ori_coordinate;
//                    zVec3 v2 = p_ele->node2->ori_coordinate;
//                    zVec3 v3 = p_ele->node3->ori_coordinate;
//                    zVec3 v4 = p_ele->node4->ori_coordinate;
//                    zVec3 v5 = p_ele->node5->ori_coordinate;
//                    zVec3 v6 = p_ele->node6->ori_coordinate;
//                    zVec3 v7 = p_ele->node7->ori_coordinate;

//                    int n0 = 0;
//                    int n1 = 1;
//                    int n2 = 2;
//                    int n3 = 3;
//                    int n4 = 4;
//                    int n5 = 5;
//                    int n6 = 6;
//                    int n7 = 7;

//                    ofs << "v " << v0[0] << " " << v0[1] << " " << v0[2] << endl
//                        << "v " << v1[0] << " " << v1[1] << " " << v1[2] << endl
//                        << "v " << v2[0] << " " << v2[1] << " " << v2[2] << endl
//                        << "v " << v3[0] << " " << v3[1] << " " << v3[2] << endl
//                        << "v " << v4[0] << " " << v4[1] << " " << v4[2] << endl
//                        << "v " << v5[0] << " " << v5[1] << " " << v5[2] << endl
//                        << "v " << v6[0] << " " << v6[1] << " " << v6[2] << endl
//                        << "v " << v7[0] << " " << v7[1] << " " << v7[2] << endl;

//                    ofs << "f " << n0+1 << " " << n2+1 << " " << n3+1 << " " << n1+1 << endl
//                        << "f " << n0+1 << " " << n1+1 << " " << n5+1 << " " << n4+1 << endl
//                        << "f " << n1+1 << " " << n3+1 << " " << n7+1 << " " << n5+1 << endl
//                        << "f " << n3+1 << " " << n2+1 << " " << n6+1 << " " << n7+1 << endl
//                        << "f " << n0+1 << " " << n4+1 << " " << n6+1 << " " << n2+1 << endl
//                        << "f " << n4+1 << " " << n5+1 << " " << n7+1 << " " << n6+1 << endl;

//                    cout << " OK" << endl;
//                }
//                else
//                {
//                    cout << " Error, voxel element not found." << endl;
//                }

//                ofs.close();
//            }

//            //----------------------------------------------------------------------
//            // output the end voxels of the scanline.
//            //----------------------------------------------------------------------
//            QDir::setCurrent(outputdir2);

//            // the start point
//            long long vox_idx = p_scanline->at(1);

//            // set the output file
//            char objfilename[512];
//            sprintf(objfilename, "tri_slice_%02d_scanline_%03d_startpoint.obj", i, scanline_count);
//            ofstream ofs(objfilename, ios::binary);

//            // search the element list by the index.
//            CubicElement* p_ele = NULL;
//            for (int k=0; k<num_ele; ++k)
//            {
//                if (vox_idx == p_submesh->at(k)->vox_idx)
//                {
//                    p_ele = p_submesh->at(k);
//                }
//            }

//            // output the six faces to obj faces
//            if (p_ele)
//            {
//                p_slice->push_back(p_ele);

//                zVec3 v0 = p_ele->node0->ori_coordinate;
//                zVec3 v1 = p_ele->node1->ori_coordinate;
//                zVec3 v2 = p_ele->node2->ori_coordinate;
//                zVec3 v3 = p_ele->node3->ori_coordinate;
//                zVec3 v4 = p_ele->node4->ori_coordinate;
//                zVec3 v5 = p_ele->node5->ori_coordinate;
//                zVec3 v6 = p_ele->node6->ori_coordinate;
//                zVec3 v7 = p_ele->node7->ori_coordinate;

//                int n0 = 0;
//                int n1 = 1;
//                int n2 = 2;
//                int n3 = 3;
//                int n4 = 4;
//                int n5 = 5;
//                int n6 = 6;
//                int n7 = 7;

//                ofs << "v " << v0[0] << " " << v0[1] << " " << v0[2] << endl
//                    << "v " << v1[0] << " " << v1[1] << " " << v1[2] << endl
//                    << "v " << v2[0] << " " << v2[1] << " " << v2[2] << endl
//                    << "v " << v3[0] << " " << v3[1] << " " << v3[2] << endl
//                    << "v " << v4[0] << " " << v4[1] << " " << v4[2] << endl
//                    << "v " << v5[0] << " " << v5[1] << " " << v5[2] << endl
//                    << "v " << v6[0] << " " << v6[1] << " " << v6[2] << endl
//                    << "v " << v7[0] << " " << v7[1] << " " << v7[2] << endl;

//                ofs << "f " << n0+1 << " " << n2+1 << " " << n3+1 << " " << n1+1 << endl
//                    << "f " << n0+1 << " " << n1+1 << " " << n5+1 << " " << n4+1 << endl
//                    << "f " << n1+1 << " " << n3+1 << " " << n7+1 << " " << n5+1 << endl
//                    << "f " << n3+1 << " " << n2+1 << " " << n6+1 << " " << n7+1 << endl
//                    << "f " << n0+1 << " " << n4+1 << " " << n6+1 << " " << n2+1 << endl
//                    << "f " << n4+1 << " " << n5+1 << " " << n7+1 << " " << n6+1 << endl;
//            }
//            else
//            {
//                cout << " Error, voxel element not found." << endl;
//            }
//            ofs.close();

//            // the end point
//            vox_idx = p_scanline->at(num_vox-1);

//            // set the output file
//            objfilename[512];
//            sprintf(objfilename, "tri_slice_%02d_scanline_%03d_endpoint.obj", i, scanline_count);
//            ofs.open(objfilename, ios::binary);

//            // search the element list by the index.
//            p_ele = NULL;
//            for (int k=0; k<num_ele; ++k)
//            {
//                if (vox_idx == p_submesh->at(k)->vox_idx)
//                {
//                    p_ele = p_submesh->at(k);
//                }
//            }

//            // output the six faces to obj faces
//            if (p_ele)
//            {
//                p_slice->push_back(p_ele);

//                zVec3 v0 = p_ele->node0->ori_coordinate;
//                zVec3 v1 = p_ele->node1->ori_coordinate;
//                zVec3 v2 = p_ele->node2->ori_coordinate;
//                zVec3 v3 = p_ele->node3->ori_coordinate;
//                zVec3 v4 = p_ele->node4->ori_coordinate;
//                zVec3 v5 = p_ele->node5->ori_coordinate;
//                zVec3 v6 = p_ele->node6->ori_coordinate;
//                zVec3 v7 = p_ele->node7->ori_coordinate;

//                int n0 = 0;
//                int n1 = 1;
//                int n2 = 2;
//                int n3 = 3;
//                int n4 = 4;
//                int n5 = 5;
//                int n6 = 6;
//                int n7 = 7;

//                ofs << "v " << v0[0] << " " << v0[1] << " " << v0[2] << endl
//                    << "v " << v1[0] << " " << v1[1] << " " << v1[2] << endl
//                    << "v " << v2[0] << " " << v2[1] << " " << v2[2] << endl
//                    << "v " << v3[0] << " " << v3[1] << " " << v3[2] << endl
//                    << "v " << v4[0] << " " << v4[1] << " " << v4[2] << endl
//                    << "v " << v5[0] << " " << v5[1] << " " << v5[2] << endl
//                    << "v " << v6[0] << " " << v6[1] << " " << v6[2] << endl
//                    << "v " << v7[0] << " " << v7[1] << " " << v7[2] << endl;

//                ofs << "f " << n0+1 << " " << n2+1 << " " << n3+1 << " " << n1+1 << endl
//                    << "f " << n0+1 << " " << n1+1 << " " << n5+1 << " " << n4+1 << endl
//                    << "f " << n1+1 << " " << n3+1 << " " << n7+1 << " " << n5+1 << endl
//                    << "f " << n3+1 << " " << n2+1 << " " << n6+1 << " " << n7+1 << endl
//                    << "f " << n0+1 << " " << n4+1 << " " << n6+1 << " " << n2+1 << endl
//                    << "f " << n4+1 << " " << n5+1 << " " << n7+1 << " " << n6+1 << endl;
//            }
//            else
//            {
//                cout << " Error, voxel element not found." << endl;
//            }

//            ofs.close();

//            // move to next scanline
//            scanline_count++;
//            if (scanline_count >= num_scanline)
//            {
//                break;
//            }
//            else
//            {
//                p_scanline = p_voxel_meshEXP->scanline_list[scanline_count];
//            }
//        }

//        //----------------------------------------------------------------------
//        // Output the slice
//        //----------------------------------------------------------------------
////        char objfilename[512];
////        sprintf(objfilename, "tri_slice_%02i.obj", i);
////        ofstream ofs(objfilename, ios::binary);

////        int count_vert = 0;
////        int num_vox = p_slice->size();
////        for (int j=0; j<num_vox; ++j)
////        {
////            CubicElement* p_ele = p_slice->at(j);

////            zVec3 v0 = p_ele->node0->ori_coordinate;
////            zVec3 v1 = p_ele->node1->ori_coordinate;
////            zVec3 v2 = p_ele->node2->ori_coordinate;
////            zVec3 v3 = p_ele->node3->ori_coordinate;
////            zVec3 v4 = p_ele->node4->ori_coordinate;
////            zVec3 v5 = p_ele->node5->ori_coordinate;
////            zVec3 v6 = p_ele->node6->ori_coordinate;
////            zVec3 v7 = p_ele->node7->ori_coordinate;

////            int n0 = count_vert+0;
////            int n1 = count_vert+1;
////            int n2 = count_vert+2;
////            int n3 = count_vert+3;
////            int n4 = count_vert+4;
////            int n5 = count_vert+5;
////            int n6 = count_vert+6;
////            int n7 = count_vert+7;

////            ofs << "v " << v0[0] << " " << v0[1] << " " << v0[2] << endl
////                << "v " << v1[0] << " " << v1[1] << " " << v1[2] << endl
////                << "v " << v2[0] << " " << v2[1] << " " << v2[2] << endl
////                << "v " << v3[0] << " " << v3[1] << " " << v3[2] << endl
////                << "v " << v4[0] << " " << v4[1] << " " << v4[2] << endl
////                << "v " << v5[0] << " " << v5[1] << " " << v5[2] << endl
////                << "v " << v6[0] << " " << v6[1] << " " << v6[2] << endl
////                << "v " << v7[0] << " " << v7[1] << " " << v7[2] << endl;

////            ofs << "f " << n0+1 << " " << n2+1 << " " << n3+1 << " " << n1+1 << endl
////                << "f " << n0+1 << " " << n1+1 << " " << n5+1 << " " << n4+1 << endl
////                << "f " << n1+1 << " " << n3+1 << " " << n7+1 << " " << n5+1 << endl
////                << "f " << n3+1 << " " << n2+1 << " " << n6+1 << " " << n7+1 << endl
////                << "f " << n0+1 << " " << n4+1 << " " << n6+1 << " " << n2+1 << endl
////                << "f " << n4+1 << " " << n5+1 << " " << n7+1 << " " << n6+1 << endl;

////            count_vert += 8;
////        }
////        ofs.close();
//    }
}

void LoboVoxelizor::on_actionMultiOBJ_File_voxelization_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());
//        p_mesh->setBoundingBox();

//        // Build a new voxel mesh.
//        cleanAllVoxelMesh();
//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//        p_voxel_meshEXP->computeVoxelMesh();

//        QString msg;
//        msg.append("\n----------------EXP---------------\nOBJ file: " + QString(obj_file.c_str()) +
//                   "\n#Vertices: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_node) +
//                   "\n#Faces: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_face) +
//                   "\n#SubMeshes: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_mesh) + "\n");
//        msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//                   "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us." +
//                   "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//                   "\nVoxelizing interior: " + QString::number(p_voxel_meshEXP->time_interior,'g') + " us." +
//                   "\n#interior voxels : " + QString::number(p_voxel_meshEXP->num_ele_interior) + "\n");
//        displayMessage(msg);
//    }

//    // pass the last one to the viewer
//    if (num_files > 0)
//    {
//        ui.viewer->setVoxelMesh((VoxelMesh *)p_voxel_meshEXP);
//        ui.viewer->setGreenColor();
//        ui.viewer->updateGL();

//        //isShowingMeshEXP = true;
//    }
}

void LoboVoxelizor::on_actionSingleOBJ_File_voxelization_triggered()
{
//    QString obj_file = QFileDialog::getOpenFileName(this, tr("Open OBJ mesh"), ".", tr("*.obj"));
//    if (!obj_file.length())
//    {
//        return;
//    }

//    cout << "obj file = " << obj_file.toUtf8().constData() << endl;

//    // Build a triangle mesh.
//    TriangleMesh* p_mesh = new TriangleMesh;
//    p_mesh->loadOBJFile(obj_file.toLatin1());
//    //p_mesh->setBoundingBox(10.0, 10.0, 10.0, -10.0, -10.0, -10.0);
//    p_mesh->setBoundingBox();

//    // Build the voxel mesh.
//    cleanAllVoxelMesh();
//    p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//    p_voxel_meshEXP->computeVoxelMesh();

//    QString msg;
//    msg.append("\n----------------EXP---------------\nOBJ file: " + obj_file +
//               "\n#Vertices: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_node) +
//               "\n#Faces: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_face) +
//               "\n#SubMeshes: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_mesh) + "\n");
//    msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//               "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us." +
//               "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//               "\nVoxelizing interior: " + QString::number(p_voxel_meshEXP->time_interior,'g') + " us." +
//               "\n#interior voxels : " + QString::number(p_voxel_meshEXP->num_ele_interior) + "\n");
//    displayMessage(msg);

//    // pass to the viewer
//    ui.viewer->setVoxelMesh((VoxelMesh *)p_voxel_meshEXP);
//    ui.viewer->setGreenColor();
//    ui.viewer->updateGL();

//    //isShowingMeshEXP = true;
}

void LoboVoxelizor::on_actionSpeed_Test_Center_Scanlines_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());
//        p_mesh->setBoundingBox();

//        // Build a new voxel mesh.
//        cleanAllVoxelMesh();
//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//        p_voxel_meshEXP->computeVoxelMeshSpeed(2); //use center scanlines

//        QString msg;
//        msg.append("\n--------------------\nSpeed_Test_Center_Scan\nOBJ file: " + QString(obj_file.c_str()) +
//                   "\n#Vertices: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_node) +
//                   "\n#Faces: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_face) +
//                   "\n#SubMeshes: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_mesh));
//        msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//                   "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//                   "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us.\n");
//        displayMessage(msg);
//    }

//    // pass the last one to the viewer
//    if (num_files > 0)
//    {
//        ui.viewer->setVoxelMesh((VoxelMesh *)p_voxel_meshEXP);
//        ui.viewer->setGreenColor();
//        ui.viewer->updateGL();

//        //isShowingMeshEXP = true;
//    }
}

void LoboVoxelizor::on_actionSpeed_Test_Parallel_Scanlines_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());
//        p_mesh->setBoundingBox();

//        // Build a new voxel mesh.
//        cleanAllVoxelMesh();
//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//        p_voxel_meshEXP->computeVoxelMeshSpeed(1); // use parallel scanlines

//        QString msg;
//        msg.append("\n--------------------\nSpeed_Test_Parallel_Scan\nOBJ file: " + QString(obj_file.c_str()) +
//                   "\n#Vertices: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_node) +
//                   "\n#Faces: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_face) +
//                   "\n#SubMeshes: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_mesh));
//        msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//                   "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//                   "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us.\n");        displayMessage(msg);
//    }

//    // pass the last one to the viewer
//    if (num_files > 0)
//    {
//        ui.viewer->setVoxelMesh((VoxelMesh *)p_voxel_meshEXP);
//        ui.viewer->setGreenColor();
//        ui.viewer->updateGL();

//        //isShowingMeshEXP = true;
//    }
}

void LoboVoxelizor::on_actionSpeed_Test_SAT_based_triggered()
{
//    //Get the source OBJ file list.
//    QFileDialog dialog(this);
//    dialog.setDirectory(workingdir);
//    dialog.setFileMode(QFileDialog::ExistingFiles);
//    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
//    QStringList source_file_list;
//    if (dialog.exec())
//    {
//        source_file_list = dialog.selectedFiles();
//    }

//    //Iterate all OBJ files.
//    int num_files = source_file_list.size();
//    for (int i=0; i<num_files; ++i)
//    {
//        // open the obj file one by one.
//        string obj_file = source_file_list[i].toUtf8().constData();
//        cout << "obj file = " << obj_file.c_str() << endl;

//        // Build a triangle mesh.
//        TriangleMesh* p_mesh = new TriangleMesh;
//        p_mesh->loadOBJFile(obj_file.c_str());
//        p_mesh->setBoundingBox();

//        // Build a new voxel mesh.
//        cleanAllVoxelMesh();
//        p_voxel_meshEXP = new VoxelMeshEXP(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
//        p_voxel_meshEXP->computeVoxelMeshSpeed(0);  //use SAT-based method

//        QString msg;
//        msg.append("\n--------------------\nSpeed_Test_SAT_based\nOBJ file: " + QString(obj_file.c_str()) +
//                   "\n#Vertices: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_node) +
//                   "\n#Faces: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_face) +
//                   "\n#SubMeshes: " + QString::number(p_voxel_meshEXP->triangle_mesh->num_mesh));
//        msg.append("\nGrid Density: " + QString::number(p_voxel_meshEXP->grid_density) +
//                   "\n#surface voxels: " + QString::number(p_voxel_meshEXP->num_ele_surface) +
//                   "\nVoxelizing surface: " + QString::number(p_voxel_meshEXP->time_surface,'g') + " us.\n");
//        displayMessage(msg);
//    }

//    // pass the last one to the viewer
//    if (num_files > 0)
//    {
//        ui.viewer->setVoxelMesh((VoxelMesh *)p_voxel_meshEXP);
//        ui.viewer->setGreenColor();
//        ui.viewer->updateGL();

//        //isShowingMeshEXP = true;
//    }
}

void LoboVoxelizor::on_radioButton_SAT_clicked(bool checked)
{
    if (checked)
    {
        voxelization_method = 0;
    }
}

void LoboVoxelizor::on_radioButton_FLT_clicked(bool checked)
{
    if (checked)
    {
        voxelization_method = 1;
    }
}

void LoboVoxelizor::on_radioButton_INT_clicked(bool checked)
{
    if (checked)
    {
        voxelization_method = 2;
    }
}

void LoboVoxelizor::on_radioButton_Pan11_clicked(bool checked)
{
    if (checked)
    {
        voxelization_method = 3;
    }
}

void LoboVoxelizor::on_radioButton_ViewNormalized_clicked(bool checked)
{
    if (checked)
    {
        ui.action2D_Projections->setChecked(false);
        ui.viewer->setViewMode(0);
    }
}

void LoboVoxelizor::on_radioButton_ViewOriginal_clicked(bool checked)
{
    if (checked)
    {
        ui.action2D_Projections->setChecked(false);
        ui.viewer->setViewMode(1);
    }
}

void LoboVoxelizor::on_checkBox_setSpeedTest_clicked(bool checked)
{
    cout << "speed_test_mode " << checked << endl;
    speed_test_mode = checked;
    if (p_voxel_mesh)
    {
//        p_voxel_mesh->isSpeedTest = checked;
    }
}

void LoboVoxelizor::on_checkBox_setFillHoles_clicked(bool checked)
{
    if (p_voxel_mesh)
    {
        p_voxel_mesh->isFillingHole = checked;
    }
}

void LoboVoxelizor::on_checkBox_setMultipleThreads_clicked(bool checked)
{
    if (p_voxel_mesh)
    {
        p_voxel_mesh->isMultiThread = checked;
    }
    if (p_voxel_mesh_speed)
    {
        p_voxel_mesh_speed->isMultiThread = checked;
    }
}

// Load obj files and voxelize one by one, keep the last one only.
void LoboVoxelizor::on_pushButtonLoad_clicked()
{
    //Get the source OBJ file list.
    QFileDialog dialog(this);
    dialog.setDirectory(workingdir);
    dialog.setFileMode(QFileDialog::ExistingFiles);
    dialog.setNameFilter(trUtf8("Splits (*.obj)"));
    QStringList source_file_list;
    if (dialog.exec())
    {
        source_file_list = dialog.selectedFiles();
    }

    //Iterate all OBJ files.
    int num_files = source_file_list.size();
    for (int i=0; i<num_files; ++i)
    {
        // open the obj file one by one.
        string obj_file = source_file_list[i].toUtf8().constData();
        cout << "obj file = " << obj_file.c_str() << endl;

        // Build a triangle mesh.
        TriangleMesh* p_mesh = new TriangleMesh;
        p_mesh->loadOBJFile(obj_file.c_str());
        p_mesh->setBoundingBox();

        // Build the voxel mesh.
        if (p_voxel_mesh)
        {
            delete p_voxel_mesh;
        }
        p_voxel_mesh = new VoxelMesh(p_mesh, ui.spinBoxGridDensity->value(), voxelization_method);
        p_voxel_mesh->isFillingHole = ui.checkBox_setFillHoles->isChecked();
        p_voxel_mesh->isMultiThread = ui.checkBox_setMultipleThreads->isChecked();
        p_voxel_mesh->computeVoxelMesh();

        //output information of the obj file.
        QString msg;
        msg = "---------------------------------------\nOBJ file: " + QString(obj_file.c_str()) +
                "\n#Vertices: " + QString::number(p_voxel_mesh->triangle_mesh->num_node) +
                "\n#Faces: " + QString::number(p_voxel_mesh->triangle_mesh->num_face) +
                "\n#SubMeshes: " + QString::number(p_voxel_mesh->triangle_mesh->num_mesh) +
                "\n\nGrid Density: " + QString::number(p_voxel_mesh->grid_density) +
                "\nMethod: " + QString::number(p_voxel_mesh->voxelization_method) +
                "\n#surface voxels: " + QString::number(p_voxel_mesh->volume_mesh.num_element_surface) +
                "\nVoxelizing surface: " + QString::number(p_voxel_mesh->time_surface,'g') +
                " us. \n#interior voxels : " + QString::number(p_voxel_mesh->volume_mesh.num_element_interior) +
                "\nVoxelizing interior: " + QString::number(p_voxel_mesh->time_interior,'g') + " us.";

        // Speed test of the voxel mesh.
        if (ui.checkBox_setSpeedTest->isChecked())
        {
            msg += speedTest();
        }

        // Show the message
        displayMessage(msg);
    }

    // pass the last one to the viewer
    if (num_files > 0)
    {
        ui.viewer->setVoxelMesh(p_voxel_mesh);
//        ui.viewer->setGreenColor();
        ui.viewer->updateGL();
    }
}

QString LoboVoxelizor::speedTest()
{
    if (!p_voxel_mesh)
    {
        return QString("\n******************************************\n"
                       "No triangle mesh available."
                       "\n******************************************\n");
    }

    if (!p_voxel_mesh_speed || p_voxel_mesh_speed->triangle_mesh != p_voxel_mesh->triangle_mesh)
    {
        // get a new speed mesh if it is NULL or if it is not identical to the parent mesh
        p_voxel_mesh_speed = new VoxelMeshSpeed(p_voxel_mesh->triangle_mesh,
                                                ui.spinBoxGridDensity->value(), voxelization_method);
        p_voxel_mesh_speed->isFillingHole = ui.checkBox_setFillHoles->isChecked();
        p_voxel_mesh_speed->isMultiThread = ui.checkBox_setMultipleThreads->isChecked();
        p_voxel_mesh_speed->computeVoxelMesh();
    }
    else
    {
        p_voxel_mesh_speed->voxelization_method = voxelization_method;
        p_voxel_mesh_speed->grid_density = ui.spinBoxGridDensity->value();
        p_voxel_mesh_speed->isFillingHole = ui.checkBox_setFillHoles->isChecked();
        p_voxel_mesh_speed->isMultiThread = ui.checkBox_setMultipleThreads->isChecked();
        p_voxel_mesh_speed->computeVoxelMesh();
    }

    QString msg;
    msg.append("\n***************************************\n"
               "Speed Test Result \n#Surface voxels: " +
               QString::number(p_voxel_mesh_speed->num_element_surface_speedtest) +
               "\nVoxelizing surface: " +
               QString::number(p_voxel_mesh_speed->time_surface_speedtest,'g') + " us." +
               "\n***************************************\n");
    return msg;
}

void LoboVoxelizor::on_pushButton_Screenshot_clicked()
{
//    QDir::setCurrent(working_directory+"/screenshot");
    QString filename = QFileDialog::getSaveFileName
                       (this, tr("Save the snapshot"), ".", tr("*.png"));
    if (!filename.length() || !p_voxel_mesh)
    {
        return;
    }

//    glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
//    glViewport(0, 0, p_viewer->width(), p_viewer->height());
//    saveScreenShotPPM(filename.toLatin1());
    saveScreenShotPNG(filename.toLatin1());
}

void LoboVoxelizor::saveScreenShotPPM(const char *filename)
{
    ui.viewer->updateScreenShot();
    unsigned char *screenshot = ui.viewer->screenshot;
    int h = ui.viewer->height();
    int w = ui.viewer->width();

    FILE *f = fopen(filename, "w");
    fprintf(f, "P3\n%d %d\n%d\n", w, h, 255);
    for (int i = 0; i < h; i++)
    {
        for (int j = 0; j < w; j++)
        {
            int cur = 4 * ((h - i - 1) * w + j);
            fprintf(f, "%3d %3d %3d ", screenshot[cur], screenshot[cur + 1], screenshot[cur + 2]);
        }
        fprintf(f, "\n");
    }
    fclose(f);
}

void LoboVoxelizor::saveScreenShotPNG(const char *filename)
{
    ui.viewer->makeCurrent();
    ui.viewer->updateScreenShot();
    unsigned char *screenshot = ui.viewer->screenshot;
    int h = ui.viewer->height();
    int w = ui.viewer->width();

    QImage im(screenshot, w, h, QImage::Format_ARGB32);

    int len = im.bytesPerLine();
    unsigned char *im_line = new unsigned char[len];
    for (int y = 0; y < h/2; y++)
    {
        memcpy(im_line, im.scanLine(y), len);
        memcpy(im.scanLine(y), im.scanLine(h-y-1), len);
        memcpy(im.scanLine(h-y-1), im_line, len);
    }
    QImage im_rgba = im.rgbSwapped();
    im_rgba.save(filename, "PNG", 100);

    delete im_line;
}
