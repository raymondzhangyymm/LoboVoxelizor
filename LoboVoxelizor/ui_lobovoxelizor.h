/********************************************************************************
** Form generated from reading UI file 'lobovoxelizor.ui'
**
** Created by: Qt User Interface Compiler version 5.7.0
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOBOVOXELIZOR_H
#define UI_LOBOVOXELIZOR_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QMainWindow>
#include <QtWidgets/QMenu>
#include <QtWidgets/QMenuBar>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QRadioButton>
#include <QtWidgets/QSpinBox>
#include <QtWidgets/QTabWidget>
#include <QtWidgets/QTextEdit>
#include <QtWidgets/QWidget>
#include <mainviewer.h>

QT_BEGIN_NAMESPACE

class Ui_LoboVoxelizorClass
{
public:
    QAction *actionOpen_OBJ_file;
    QAction *actionSave_OBJ_file_Normalized;
    QAction *actionSave_OBJ_file_Quad;
    QAction *actionSave_OBJ_file_Triangle;
    QAction *actionMultiOBJ_AllElement;
    QAction *actionMultiOBJ_AllElement_collision;
    QAction *actionSave_ELE_Cubic_All_Elements;
    QAction *actionSave_ELE_Cubic_Surface_Only;
    QAction *actionSave_ELE_Tet_All_Elements;
    QAction *actionSave_ELE_Tet_Surface_Only;
    QAction *actionSave_OBJ_File_All_Quad_Face;
    QAction *actionMultiOBJ_File_Collisions;
    QAction *actionMultiOBJ_File_Add_Container;
    QAction *actionGenerate_triangle_slices;
    QAction *actionMultiOBJ_File_voxelization;
    QAction *actionSingleOBJ_File_voxelization;
    QAction *actionOriginal_Mesh;
    QAction *actionNormalized_Mesh;
    QAction *action2D_Projections;
    QAction *actionSpeed_Test_Parallel_Scanlines;
    QAction *actionSpeed_Test_Center_Scanlines;
    QAction *actionTriangle;
    QAction *action3D_Model;
    QAction *actionSpeed_Test_SAT_based;
    QWidget *centralWidget;
    QPushButton *pushButtonVoxelize;
    mainviewer *viewer;
    QTextEdit *MessageText;
    QTabWidget *tabWidget;
    QWidget *tab_Inform;
    QCheckBox *checkBox_setSpeedTest;
    QSpinBox *spinBoxGridDensity;
    QLabel *label;
    QWidget *gridLayoutWidget_3;
    QGridLayout *gridLayout_3;
    QRadioButton *radioButton_Pan11;
    QRadioButton *radioButton_SAT;
    QRadioButton *radioButton_FLT;
    QRadioButton *radioButton_INT;
    QCheckBox *checkBox_setFillHoles;
    QCheckBox *checkBox_setMultipleThreads;
    QWidget *tab_View;
    QWidget *gridLayoutWidget;
    QGridLayout *gridLayout;
    QCheckBox *checkBoxSetShowInteriorVoxels;
    QCheckBox *checkBoxSetShowBoundingBox;
    QCheckBox *checkBoxSetShowSurfaceVoxels;
    QCheckBox *checkBoxSetShowTriangelMesh;
    QCheckBox *checkBoxSetShowScanpixels;
    QCheckBox *checkBoxSetShowScanlines;
    QCheckBox *checkBoxSetShow2DProjections;
    QWidget *gridLayoutWidget_2;
    QGridLayout *gridLayout_2;
    QRadioButton *radioButton_ViewNormalized;
    QRadioButton *radioButton_ViewOriginal;
    QPushButton *pushButton_Screenshot;
    QPushButton *pushButtonLoad;
    QMenuBar *menuBar;
    QMenu *menuFiles;
    QMenu *menuExport;
    QMenu *menuELE_NODE_File_Cubic;
    QMenu *menuELE_NODE_File_Tet;
    QMenu *menuEXPERIMENTS;
    QMenu *menuView;

    void setupUi(QMainWindow *LoboVoxelizorClass)
    {
        if (LoboVoxelizorClass->objectName().isEmpty())
            LoboVoxelizorClass->setObjectName(QStringLiteral("LoboVoxelizorClass"));
        LoboVoxelizorClass->resize(1200, 800);
        QSizePolicy sizePolicy(QSizePolicy::Preferred, QSizePolicy::Preferred);
        sizePolicy.setHorizontalStretch(100);
        sizePolicy.setVerticalStretch(100);
        sizePolicy.setHeightForWidth(LoboVoxelizorClass->sizePolicy().hasHeightForWidth());
        LoboVoxelizorClass->setSizePolicy(sizePolicy);
        LoboVoxelizorClass->setMinimumSize(QSize(0, 0));
        LoboVoxelizorClass->setMaximumSize(QSize(2000, 1000));
        QIcon icon;
        icon.addFile(QStringLiteral(":/LoboVoxelizer/Resources/applications_development.png"), QSize(), QIcon::Normal, QIcon::Off);
        LoboVoxelizorClass->setWindowIcon(icon);
        actionOpen_OBJ_file = new QAction(LoboVoxelizorClass);
        actionOpen_OBJ_file->setObjectName(QStringLiteral("actionOpen_OBJ_file"));
        actionSave_OBJ_file_Normalized = new QAction(LoboVoxelizorClass);
        actionSave_OBJ_file_Normalized->setObjectName(QStringLiteral("actionSave_OBJ_file_Normalized"));
        actionSave_OBJ_file_Quad = new QAction(LoboVoxelizorClass);
        actionSave_OBJ_file_Quad->setObjectName(QStringLiteral("actionSave_OBJ_file_Quad"));
        actionSave_OBJ_file_Triangle = new QAction(LoboVoxelizorClass);
        actionSave_OBJ_file_Triangle->setObjectName(QStringLiteral("actionSave_OBJ_file_Triangle"));
        actionMultiOBJ_AllElement = new QAction(LoboVoxelizorClass);
        actionMultiOBJ_AllElement->setObjectName(QStringLiteral("actionMultiOBJ_AllElement"));
        actionMultiOBJ_AllElement_collision = new QAction(LoboVoxelizorClass);
        actionMultiOBJ_AllElement_collision->setObjectName(QStringLiteral("actionMultiOBJ_AllElement_collision"));
        actionSave_ELE_Cubic_All_Elements = new QAction(LoboVoxelizorClass);
        actionSave_ELE_Cubic_All_Elements->setObjectName(QStringLiteral("actionSave_ELE_Cubic_All_Elements"));
        actionSave_ELE_Cubic_Surface_Only = new QAction(LoboVoxelizorClass);
        actionSave_ELE_Cubic_Surface_Only->setObjectName(QStringLiteral("actionSave_ELE_Cubic_Surface_Only"));
        actionSave_ELE_Tet_All_Elements = new QAction(LoboVoxelizorClass);
        actionSave_ELE_Tet_All_Elements->setObjectName(QStringLiteral("actionSave_ELE_Tet_All_Elements"));
        actionSave_ELE_Tet_Surface_Only = new QAction(LoboVoxelizorClass);
        actionSave_ELE_Tet_Surface_Only->setObjectName(QStringLiteral("actionSave_ELE_Tet_Surface_Only"));
        actionSave_OBJ_File_All_Quad_Face = new QAction(LoboVoxelizorClass);
        actionSave_OBJ_File_All_Quad_Face->setObjectName(QStringLiteral("actionSave_OBJ_File_All_Quad_Face"));
        actionMultiOBJ_File_Collisions = new QAction(LoboVoxelizorClass);
        actionMultiOBJ_File_Collisions->setObjectName(QStringLiteral("actionMultiOBJ_File_Collisions"));
        actionMultiOBJ_File_Add_Container = new QAction(LoboVoxelizorClass);
        actionMultiOBJ_File_Add_Container->setObjectName(QStringLiteral("actionMultiOBJ_File_Add_Container"));
        actionGenerate_triangle_slices = new QAction(LoboVoxelizorClass);
        actionGenerate_triangle_slices->setObjectName(QStringLiteral("actionGenerate_triangle_slices"));
        actionMultiOBJ_File_voxelization = new QAction(LoboVoxelizorClass);
        actionMultiOBJ_File_voxelization->setObjectName(QStringLiteral("actionMultiOBJ_File_voxelization"));
        actionSingleOBJ_File_voxelization = new QAction(LoboVoxelizorClass);
        actionSingleOBJ_File_voxelization->setObjectName(QStringLiteral("actionSingleOBJ_File_voxelization"));
        actionOriginal_Mesh = new QAction(LoboVoxelizorClass);
        actionOriginal_Mesh->setObjectName(QStringLiteral("actionOriginal_Mesh"));
        actionOriginal_Mesh->setCheckable(true);
        actionNormalized_Mesh = new QAction(LoboVoxelizorClass);
        actionNormalized_Mesh->setObjectName(QStringLiteral("actionNormalized_Mesh"));
        actionNormalized_Mesh->setCheckable(true);
        actionNormalized_Mesh->setChecked(true);
        action2D_Projections = new QAction(LoboVoxelizorClass);
        action2D_Projections->setObjectName(QStringLiteral("action2D_Projections"));
        action2D_Projections->setCheckable(true);
        actionSpeed_Test_Parallel_Scanlines = new QAction(LoboVoxelizorClass);
        actionSpeed_Test_Parallel_Scanlines->setObjectName(QStringLiteral("actionSpeed_Test_Parallel_Scanlines"));
        actionSpeed_Test_Center_Scanlines = new QAction(LoboVoxelizorClass);
        actionSpeed_Test_Center_Scanlines->setObjectName(QStringLiteral("actionSpeed_Test_Center_Scanlines"));
        actionTriangle = new QAction(LoboVoxelizorClass);
        actionTriangle->setObjectName(QStringLiteral("actionTriangle"));
        action3D_Model = new QAction(LoboVoxelizorClass);
        action3D_Model->setObjectName(QStringLiteral("action3D_Model"));
        actionSpeed_Test_SAT_based = new QAction(LoboVoxelizorClass);
        actionSpeed_Test_SAT_based->setObjectName(QStringLiteral("actionSpeed_Test_SAT_based"));
        centralWidget = new QWidget(LoboVoxelizorClass);
        centralWidget->setObjectName(QStringLiteral("centralWidget"));
        pushButtonVoxelize = new QPushButton(centralWidget);
        pushButtonVoxelize->setObjectName(QStringLiteral("pushButtonVoxelize"));
        pushButtonVoxelize->setGeometry(QRect(1080, 730, 111, 41));
        QFont font;
        font.setPointSize(14);
        font.setBold(true);
        font.setWeight(75);
        pushButtonVoxelize->setFont(font);
        QIcon icon1;
        icon1.addFile(QStringLiteral(":/LoboVoxelizer/Resources/dialog_ok.png"), QSize(), QIcon::Normal, QIcon::Off);
        pushButtonVoxelize->setIcon(icon1);
        pushButtonVoxelize->setFlat(false);
        viewer = new mainviewer(centralWidget);
        viewer->setObjectName(QStringLiteral("viewer"));
        viewer->setGeometry(QRect(0, 0, 911, 771));
        MessageText = new QTextEdit(centralWidget);
        MessageText->setObjectName(QStringLiteral("MessageText"));
        MessageText->setGeometry(QRect(920, 0, 271, 411));
        tabWidget = new QTabWidget(centralWidget);
        tabWidget->setObjectName(QStringLiteral("tabWidget"));
        tabWidget->setGeometry(QRect(920, 420, 281, 301));
        QFont font1;
        font1.setPointSize(12);
        tabWidget->setFont(font1);
        tab_Inform = new QWidget();
        tab_Inform->setObjectName(QStringLiteral("tab_Inform"));
        checkBox_setSpeedTest = new QCheckBox(tab_Inform);
        checkBox_setSpeedTest->setObjectName(QStringLiteral("checkBox_setSpeedTest"));
        checkBox_setSpeedTest->setGeometry(QRect(30, 120, 181, 31));
        spinBoxGridDensity = new QSpinBox(tab_Inform);
        spinBoxGridDensity->setObjectName(QStringLiteral("spinBoxGridDensity"));
        spinBoxGridDensity->setGeometry(QRect(160, 230, 111, 31));
        spinBoxGridDensity->setFont(font1);
        spinBoxGridDensity->setMaximum(4097);
        label = new QLabel(tab_Inform);
        label->setObjectName(QStringLiteral("label"));
        label->setGeometry(QRect(20, 230, 101, 31));
        label->setFont(font1);
        label->setWordWrap(true);
        gridLayoutWidget_3 = new QWidget(tab_Inform);
        gridLayoutWidget_3->setObjectName(QStringLiteral("gridLayoutWidget_3"));
        gridLayoutWidget_3->setGeometry(QRect(30, 20, 211, 61));
        gridLayout_3 = new QGridLayout(gridLayoutWidget_3);
        gridLayout_3->setSpacing(6);
        gridLayout_3->setContentsMargins(11, 11, 11, 11);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        gridLayout_3->setContentsMargins(0, 0, 0, 0);
        radioButton_Pan11 = new QRadioButton(gridLayoutWidget_3);
        radioButton_Pan11->setObjectName(QStringLiteral("radioButton_Pan11"));

        gridLayout_3->addWidget(radioButton_Pan11, 1, 0, 1, 1);

        radioButton_SAT = new QRadioButton(gridLayoutWidget_3);
        radioButton_SAT->setObjectName(QStringLiteral("radioButton_SAT"));
        radioButton_SAT->setFont(font1);
        radioButton_SAT->setChecked(true);

        gridLayout_3->addWidget(radioButton_SAT, 0, 0, 1, 1);

        radioButton_FLT = new QRadioButton(gridLayoutWidget_3);
        radioButton_FLT->setObjectName(QStringLiteral("radioButton_FLT"));
        radioButton_FLT->setFont(font1);

        gridLayout_3->addWidget(radioButton_FLT, 0, 1, 1, 1);

        radioButton_INT = new QRadioButton(gridLayoutWidget_3);
        radioButton_INT->setObjectName(QStringLiteral("radioButton_INT"));
        radioButton_INT->setFont(font1);

        gridLayout_3->addWidget(radioButton_INT, 1, 1, 1, 1);

        checkBox_setFillHoles = new QCheckBox(tab_Inform);
        checkBox_setFillHoles->setObjectName(QStringLiteral("checkBox_setFillHoles"));
        checkBox_setFillHoles->setGeometry(QRect(30, 150, 181, 31));
        checkBox_setMultipleThreads = new QCheckBox(tab_Inform);
        checkBox_setMultipleThreads->setObjectName(QStringLiteral("checkBox_setMultipleThreads"));
        checkBox_setMultipleThreads->setGeometry(QRect(30, 180, 181, 31));
        tabWidget->addTab(tab_Inform, QString());
        tab_View = new QWidget();
        tab_View->setObjectName(QStringLiteral("tab_View"));
        gridLayoutWidget = new QWidget(tab_View);
        gridLayoutWidget->setObjectName(QStringLiteral("gridLayoutWidget"));
        gridLayoutWidget->setGeometry(QRect(10, 20, 261, 131));
        gridLayout = new QGridLayout(gridLayoutWidget);
        gridLayout->setSpacing(6);
        gridLayout->setContentsMargins(11, 11, 11, 11);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        gridLayout->setContentsMargins(0, 0, 0, 0);
        checkBoxSetShowInteriorVoxels = new QCheckBox(gridLayoutWidget);
        checkBoxSetShowInteriorVoxels->setObjectName(QStringLiteral("checkBoxSetShowInteriorVoxels"));
        checkBoxSetShowInteriorVoxels->setFont(font1);
        checkBoxSetShowInteriorVoxels->setChecked(true);

        gridLayout->addWidget(checkBoxSetShowInteriorVoxels, 1, 0, 1, 1);

        checkBoxSetShowBoundingBox = new QCheckBox(gridLayoutWidget);
        checkBoxSetShowBoundingBox->setObjectName(QStringLiteral("checkBoxSetShowBoundingBox"));
        checkBoxSetShowBoundingBox->setFont(font1);
        checkBoxSetShowBoundingBox->setChecked(false);

        gridLayout->addWidget(checkBoxSetShowBoundingBox, 3, 0, 1, 1);

        checkBoxSetShowSurfaceVoxels = new QCheckBox(gridLayoutWidget);
        checkBoxSetShowSurfaceVoxels->setObjectName(QStringLiteral("checkBoxSetShowSurfaceVoxels"));
        checkBoxSetShowSurfaceVoxels->setFont(font1);
        checkBoxSetShowSurfaceVoxels->setChecked(true);

        gridLayout->addWidget(checkBoxSetShowSurfaceVoxels, 2, 0, 1, 1);

        checkBoxSetShowTriangelMesh = new QCheckBox(gridLayoutWidget);
        checkBoxSetShowTriangelMesh->setObjectName(QStringLiteral("checkBoxSetShowTriangelMesh"));
        checkBoxSetShowTriangelMesh->setFont(font1);
        checkBoxSetShowTriangelMesh->setChecked(true);

        gridLayout->addWidget(checkBoxSetShowTriangelMesh, 0, 0, 1, 1);

        checkBoxSetShowScanpixels = new QCheckBox(gridLayoutWidget);
        checkBoxSetShowScanpixels->setObjectName(QStringLiteral("checkBoxSetShowScanpixels"));
        checkBoxSetShowScanpixels->setFont(font1);
        checkBoxSetShowScanpixels->setChecked(false);

        gridLayout->addWidget(checkBoxSetShowScanpixels, 2, 1, 1, 1);

        checkBoxSetShowScanlines = new QCheckBox(gridLayoutWidget);
        checkBoxSetShowScanlines->setObjectName(QStringLiteral("checkBoxSetShowScanlines"));
        checkBoxSetShowScanlines->setFont(font1);
        checkBoxSetShowScanlines->setChecked(false);

        gridLayout->addWidget(checkBoxSetShowScanlines, 1, 1, 1, 1);

        checkBoxSetShow2DProjections = new QCheckBox(gridLayoutWidget);
        checkBoxSetShow2DProjections->setObjectName(QStringLiteral("checkBoxSetShow2DProjections"));
        checkBoxSetShow2DProjections->setFont(font1);
        checkBoxSetShow2DProjections->setChecked(false);

        gridLayout->addWidget(checkBoxSetShow2DProjections, 0, 1, 1, 1);

        gridLayoutWidget_2 = new QWidget(tab_View);
        gridLayoutWidget_2->setObjectName(QStringLiteral("gridLayoutWidget_2"));
        gridLayoutWidget_2->setGeometry(QRect(10, 170, 261, 41));
        gridLayout_2 = new QGridLayout(gridLayoutWidget_2);
        gridLayout_2->setSpacing(6);
        gridLayout_2->setContentsMargins(11, 11, 11, 11);
        gridLayout_2->setObjectName(QStringLiteral("gridLayout_2"));
        gridLayout_2->setContentsMargins(0, 0, 0, 0);
        radioButton_ViewNormalized = new QRadioButton(gridLayoutWidget_2);
        radioButton_ViewNormalized->setObjectName(QStringLiteral("radioButton_ViewNormalized"));
        radioButton_ViewNormalized->setChecked(true);

        gridLayout_2->addWidget(radioButton_ViewNormalized, 0, 0, 1, 1);

        radioButton_ViewOriginal = new QRadioButton(gridLayoutWidget_2);
        radioButton_ViewOriginal->setObjectName(QStringLiteral("radioButton_ViewOriginal"));

        gridLayout_2->addWidget(radioButton_ViewOriginal, 0, 1, 1, 1);

        pushButton_Screenshot = new QPushButton(tab_View);
        pushButton_Screenshot->setObjectName(QStringLiteral("pushButton_Screenshot"));
        pushButton_Screenshot->setGeometry(QRect(60, 220, 171, 41));
        pushButton_Screenshot->setFont(font);
        pushButton_Screenshot->setIcon(icon1);
        pushButton_Screenshot->setFlat(false);
        tabWidget->addTab(tab_View, QString());
        pushButtonLoad = new QPushButton(centralWidget);
        pushButtonLoad->setObjectName(QStringLiteral("pushButtonLoad"));
        pushButtonLoad->setGeometry(QRect(930, 730, 111, 41));
        pushButtonLoad->setFont(font);
        pushButtonLoad->setIcon(icon1);
        pushButtonLoad->setFlat(false);
        LoboVoxelizorClass->setCentralWidget(centralWidget);
        menuBar = new QMenuBar(LoboVoxelizorClass);
        menuBar->setObjectName(QStringLiteral("menuBar"));
        menuBar->setGeometry(QRect(0, 0, 1200, 21));
        menuFiles = new QMenu(menuBar);
        menuFiles->setObjectName(QStringLiteral("menuFiles"));
        menuExport = new QMenu(menuBar);
        menuExport->setObjectName(QStringLiteral("menuExport"));
        menuELE_NODE_File_Cubic = new QMenu(menuExport);
        menuELE_NODE_File_Cubic->setObjectName(QStringLiteral("menuELE_NODE_File_Cubic"));
        menuELE_NODE_File_Tet = new QMenu(menuExport);
        menuELE_NODE_File_Tet->setObjectName(QStringLiteral("menuELE_NODE_File_Tet"));
        menuEXPERIMENTS = new QMenu(menuBar);
        menuEXPERIMENTS->setObjectName(QStringLiteral("menuEXPERIMENTS"));
        menuEXPERIMENTS->setGeometry(QRect(468, 84, 282, 310));
        menuView = new QMenu(menuBar);
        menuView->setObjectName(QStringLiteral("menuView"));
        LoboVoxelizorClass->setMenuBar(menuBar);

        menuBar->addAction(menuFiles->menuAction());
        menuBar->addAction(menuExport->menuAction());
        menuBar->addAction(menuView->menuAction());
        menuBar->addAction(menuEXPERIMENTS->menuAction());
        menuFiles->addAction(actionOpen_OBJ_file);
        menuExport->addAction(menuELE_NODE_File_Cubic->menuAction());
        menuExport->addAction(menuELE_NODE_File_Tet->menuAction());
        menuExport->addAction(actionSave_OBJ_file_Quad);
        menuExport->addAction(actionSave_OBJ_file_Triangle);
        menuExport->addAction(actionSave_OBJ_File_All_Quad_Face);
        menuELE_NODE_File_Cubic->addSeparator();
        menuELE_NODE_File_Cubic->addAction(actionSave_ELE_Cubic_All_Elements);
        menuELE_NODE_File_Cubic->addAction(actionSave_ELE_Cubic_Surface_Only);
        menuELE_NODE_File_Tet->addAction(actionSave_ELE_Tet_All_Elements);
        menuELE_NODE_File_Tet->addAction(actionSave_ELE_Tet_Surface_Only);
        menuEXPERIMENTS->addAction(actionSpeed_Test_Center_Scanlines);
        menuEXPERIMENTS->addAction(actionSpeed_Test_Parallel_Scanlines);
        menuEXPERIMENTS->addAction(actionSpeed_Test_SAT_based);
        menuEXPERIMENTS->addSeparator();
        menuEXPERIMENTS->addAction(actionTriangle);
        menuEXPERIMENTS->addAction(actionMultiOBJ_File_voxelization);
        menuEXPERIMENTS->addAction(actionSingleOBJ_File_voxelization);
        menuEXPERIMENTS->addSeparator();
        menuEXPERIMENTS->addAction(action3D_Model);
        menuEXPERIMENTS->addAction(actionMultiOBJ_AllElement);
        menuEXPERIMENTS->addAction(actionMultiOBJ_File_Collisions);
        menuEXPERIMENTS->addAction(actionMultiOBJ_AllElement_collision);
        menuEXPERIMENTS->addAction(actionMultiOBJ_File_Add_Container);
        menuView->addAction(actionNormalized_Mesh);
        menuView->addAction(actionOriginal_Mesh);

        retranslateUi(LoboVoxelizorClass);
        QObject::connect(checkBoxSetShowTriangelMesh, SIGNAL(toggled(bool)), viewer, SLOT(setShowTriangleMesh(bool)));
        QObject::connect(checkBoxSetShowSurfaceVoxels, SIGNAL(toggled(bool)), viewer, SLOT(setShowSurfaceVoxels(bool)));
        QObject::connect(checkBoxSetShowInteriorVoxels, SIGNAL(toggled(bool)), viewer, SLOT(setShowInteriorVoxels(bool)));
        QObject::connect(checkBoxSetShowBoundingBox, SIGNAL(toggled(bool)), viewer, SLOT(setShowBoundingBox(bool)));
        QObject::connect(checkBoxSetShowScanlines, SIGNAL(toggled(bool)), viewer, SLOT(setShowScanlines(bool)));
        QObject::connect(checkBoxSetShow2DProjections, SIGNAL(toggled(bool)), viewer, SLOT(setShow2DProjections(bool)));
        QObject::connect(checkBoxSetShowScanpixels, SIGNAL(toggled(bool)), viewer, SLOT(setShowScanpixels(bool)));

        pushButtonVoxelize->setDefault(true);
        tabWidget->setCurrentIndex(1);
        pushButton_Screenshot->setDefault(true);
        pushButtonLoad->setDefault(true);


        QMetaObject::connectSlotsByName(LoboVoxelizorClass);
    } // setupUi

    void retranslateUi(QMainWindow *LoboVoxelizorClass)
    {
        LoboVoxelizorClass->setWindowTitle(QApplication::translate("LoboVoxelizorClass", "LoboVoxelizor", 0));
        actionOpen_OBJ_file->setText(QApplication::translate("LoboVoxelizorClass", "open OBJ file", 0));
        actionSave_OBJ_file_Normalized->setText(QApplication::translate("LoboVoxelizorClass", "OBJ File (Normalized Triangle Mesh)", 0));
        actionSave_OBJ_file_Quad->setText(QApplication::translate("LoboVoxelizorClass", "OBJ File (Quad Mesh)", 0));
        actionSave_OBJ_file_Triangle->setText(QApplication::translate("LoboVoxelizorClass", "OBJ File (Triangle Mesh)", 0));
        actionMultiOBJ_AllElement->setText(QApplication::translate("LoboVoxelizorClass", "MultiOBJ File (All elements)", 0));
        actionMultiOBJ_AllElement_collision->setText(QApplication::translate("LoboVoxelizorClass", "MultiOBJ File (All Elements and Collisions)", 0));
        actionSave_ELE_Cubic_All_Elements->setText(QApplication::translate("LoboVoxelizorClass", "All Elements", 0));
        actionSave_ELE_Cubic_Surface_Only->setText(QApplication::translate("LoboVoxelizorClass", "Surface Only", 0));
        actionSave_ELE_Tet_All_Elements->setText(QApplication::translate("LoboVoxelizorClass", "All Elements", 0));
        actionSave_ELE_Tet_Surface_Only->setText(QApplication::translate("LoboVoxelizorClass", "Surface Only", 0));
        actionSave_OBJ_File_All_Quad_Face->setText(QApplication::translate("LoboVoxelizorClass", "OBJ File (All Quad Face)", 0));
        actionMultiOBJ_File_Collisions->setText(QApplication::translate("LoboVoxelizorClass", "MultiOBJ File (Collisions)", 0));
        actionMultiOBJ_File_Add_Container->setText(QApplication::translate("LoboVoxelizorClass", "MultiOBJ File(Add Container)", 0));
        actionGenerate_triangle_slices->setText(QApplication::translate("LoboVoxelizorClass", "Generate triangle slices", 0));
        actionMultiOBJ_File_voxelization->setText(QApplication::translate("LoboVoxelizorClass", "MultiOBJ File voxelization", 0));
        actionSingleOBJ_File_voxelization->setText(QApplication::translate("LoboVoxelizorClass", "SingleOBJ File voxelization", 0));
        actionOriginal_Mesh->setText(QApplication::translate("LoboVoxelizorClass", "Original Mesh", 0));
        actionNormalized_Mesh->setText(QApplication::translate("LoboVoxelizorClass", "Normalized Mesh", 0));
        action2D_Projections->setText(QApplication::translate("LoboVoxelizorClass", "2D Projections", 0));
        actionSpeed_Test_Parallel_Scanlines->setText(QApplication::translate("LoboVoxelizorClass", "Speed Test (Parallel Scanlines)", 0));
        actionSpeed_Test_Center_Scanlines->setText(QApplication::translate("LoboVoxelizorClass", "Speed Test (Center Scanlines)", 0));
        actionTriangle->setText(QApplication::translate("LoboVoxelizorClass", "Triangle Animation (triangle.obj)", 0));
        action3D_Model->setText(QApplication::translate("LoboVoxelizorClass", "3D Model Animation", 0));
        actionSpeed_Test_SAT_based->setText(QApplication::translate("LoboVoxelizorClass", "Speed Test (SAT-based)", 0));
        pushButtonVoxelize->setText(QApplication::translate("LoboVoxelizorClass", "Voxelize", 0));
        checkBox_setSpeedTest->setText(QApplication::translate("LoboVoxelizorClass", "Speed Test (Surface)", 0));
        label->setText(QApplication::translate("LoboVoxelizorClass", "Resolution", 0));
        radioButton_Pan11->setText(QApplication::translate("LoboVoxelizorClass", "Pan11", 0));
        radioButton_SAT->setText(QApplication::translate("LoboVoxelizorClass", "SS10", 0));
        radioButton_FLT->setText(QApplication::translate("LoboVoxelizorClass", "FLT", 0));
        radioButton_INT->setText(QApplication::translate("LoboVoxelizorClass", "INT", 0));
        checkBox_setFillHoles->setText(QApplication::translate("LoboVoxelizorClass", "Fill Holes", 0));
        checkBox_setMultipleThreads->setText(QApplication::translate("LoboVoxelizorClass", "Multiple Threads", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_Inform), QApplication::translate("LoboVoxelizorClass", "Method", 0));
        checkBoxSetShowInteriorVoxels->setText(QApplication::translate("LoboVoxelizorClass", "Interior Voxels", 0));
        checkBoxSetShowBoundingBox->setText(QApplication::translate("LoboVoxelizorClass", "Bounding Box", 0));
        checkBoxSetShowSurfaceVoxels->setText(QApplication::translate("LoboVoxelizorClass", "Surface Voxels", 0));
        checkBoxSetShowTriangelMesh->setText(QApplication::translate("LoboVoxelizorClass", "Triangle mesh", 0));
        checkBoxSetShowScanpixels->setText(QApplication::translate("LoboVoxelizorClass", "Scanpixels", 0));
        checkBoxSetShowScanlines->setText(QApplication::translate("LoboVoxelizorClass", "Scanlines", 0));
        checkBoxSetShow2DProjections->setText(QApplication::translate("LoboVoxelizorClass", "2D Projections", 0));
        radioButton_ViewNormalized->setText(QApplication::translate("LoboVoxelizorClass", "Normalized", 0));
        radioButton_ViewOriginal->setText(QApplication::translate("LoboVoxelizorClass", "Original", 0));
        pushButton_Screenshot->setText(QApplication::translate("LoboVoxelizorClass", "Screen Shot", 0));
        tabWidget->setTabText(tabWidget->indexOf(tab_View), QApplication::translate("LoboVoxelizorClass", "View", 0));
        pushButtonLoad->setText(QApplication::translate("LoboVoxelizorClass", "Load", 0));
        menuFiles->setTitle(QApplication::translate("LoboVoxelizorClass", "Files", 0));
        menuExport->setTitle(QApplication::translate("LoboVoxelizorClass", "Export", 0));
        menuELE_NODE_File_Cubic->setTitle(QApplication::translate("LoboVoxelizorClass", "ELE NODE File (Cubic)", 0));
        menuELE_NODE_File_Tet->setTitle(QApplication::translate("LoboVoxelizorClass", "ELE NODE File (Tet)", 0));
        menuEXPERIMENTS->setTitle(QApplication::translate("LoboVoxelizorClass", "EXPERIMENTS", 0));
        menuView->setTitle(QApplication::translate("LoboVoxelizorClass", "View", 0));
    } // retranslateUi

};

namespace Ui {
    class LoboVoxelizorClass: public Ui_LoboVoxelizorClass {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOBOVOXELIZOR_H
