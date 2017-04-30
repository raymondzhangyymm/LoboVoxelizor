/****************************************************************************
** Meta object code from reading C++ file 'mainviewer.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../mainviewer.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'mainviewer.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_mainviewer_t {
    QByteArrayData data[10];
    char stringdata0[152];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_mainviewer_t, stringdata0) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_mainviewer_t qt_meta_stringdata_mainviewer = {
    {
QT_MOC_LITERAL(0, 0, 10), // "mainviewer"
QT_MOC_LITERAL(1, 11, 19), // "setShowTriangleMesh"
QT_MOC_LITERAL(2, 31, 0), // ""
QT_MOC_LITERAL(3, 32, 1), // "b"
QT_MOC_LITERAL(4, 34, 20), // "setShowSurfaceVoxels"
QT_MOC_LITERAL(5, 55, 21), // "setShowInteriorVoxels"
QT_MOC_LITERAL(6, 77, 18), // "setShowBoundingBox"
QT_MOC_LITERAL(7, 96, 16), // "setShowScanlines"
QT_MOC_LITERAL(8, 113, 17), // "setShowScanpixels"
QT_MOC_LITERAL(9, 131, 20) // "setShow2DProjections"

    },
    "mainviewer\0setShowTriangleMesh\0\0b\0"
    "setShowSurfaceVoxels\0setShowInteriorVoxels\0"
    "setShowBoundingBox\0setShowScanlines\0"
    "setShowScanpixels\0setShow2DProjections"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_mainviewer[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
       7,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    1,   49,    2, 0x08 /* Private */,
       4,    1,   52,    2, 0x08 /* Private */,
       5,    1,   55,    2, 0x08 /* Private */,
       6,    1,   58,    2, 0x08 /* Private */,
       7,    1,   61,    2, 0x08 /* Private */,
       8,    1,   64,    2, 0x08 /* Private */,
       9,    1,   67,    2, 0x08 /* Private */,

 // slots: parameters
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,
    QMetaType::Void, QMetaType::Bool,    3,

       0        // eod
};

void mainviewer::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        mainviewer *_t = static_cast<mainviewer *>(_o);
        Q_UNUSED(_t)
        switch (_id) {
        case 0: _t->setShowTriangleMesh((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 1: _t->setShowSurfaceVoxels((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 2: _t->setShowInteriorVoxels((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 3: _t->setShowBoundingBox((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 4: _t->setShowScanlines((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 5: _t->setShowScanpixels((*reinterpret_cast< bool(*)>(_a[1]))); break;
        case 6: _t->setShow2DProjections((*reinterpret_cast< bool(*)>(_a[1]))); break;
        default: ;
        }
    }
}

const QMetaObject mainviewer::staticMetaObject = {
    { &QGLViewer::staticMetaObject, qt_meta_stringdata_mainviewer.data,
      qt_meta_data_mainviewer,  qt_static_metacall, Q_NULLPTR, Q_NULLPTR}
};


const QMetaObject *mainviewer::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *mainviewer::qt_metacast(const char *_clname)
{
    if (!_clname) return Q_NULLPTR;
    if (!strcmp(_clname, qt_meta_stringdata_mainviewer.stringdata0))
        return static_cast<void*>(const_cast< mainviewer*>(this));
    return QGLViewer::qt_metacast(_clname);
}

int mainviewer::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QGLViewer::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 7)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 7;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 7)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 7;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
