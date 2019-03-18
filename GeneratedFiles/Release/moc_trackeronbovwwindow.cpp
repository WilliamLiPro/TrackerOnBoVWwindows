/****************************************************************************
** Meta object code from reading C++ file 'trackeronbovwwindow.h'
**
** Created by: The Qt Meta Object Compiler version 67 (Qt 5.3.1)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "stdafx.h"
#include "../../trackeronbovwwindow.h"
#include <QtCore/qbytearray.h>
#include <QtCore/qmetatype.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'trackeronbovwwindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 67
#error "This file was generated using the moc from 5.3.1. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
struct qt_meta_stringdata_TrackerOnBoVWwindow_t {
    QByteArrayData data[14];
    char stringdata[185];
};
#define QT_MOC_LITERAL(idx, ofs, len) \
    Q_STATIC_BYTE_ARRAY_DATA_HEADER_INITIALIZER_WITH_OFFSET(len, \
    qptrdiff(offsetof(qt_meta_stringdata_TrackerOnBoVWwindow_t, stringdata) + ofs \
        - idx * sizeof(QByteArrayData)) \
    )
static const qt_meta_stringdata_TrackerOnBoVWwindow_t qt_meta_stringdata_TrackerOnBoVWwindow = {
    {
QT_MOC_LITERAL(0, 0, 19),
QT_MOC_LITERAL(1, 20, 14),
QT_MOC_LITERAL(2, 35, 0),
QT_MOC_LITERAL(3, 36, 12),
QT_MOC_LITERAL(4, 49, 9),
QT_MOC_LITERAL(5, 59, 14),
QT_MOC_LITERAL(6, 74, 11),
QT_MOC_LITERAL(7, 86, 14),
QT_MOC_LITERAL(8, 101, 17),
QT_MOC_LITERAL(9, 119, 11),
QT_MOC_LITERAL(10, 131, 14),
QT_MOC_LITERAL(11, 146, 11),
QT_MOC_LITERAL(12, 158, 13),
QT_MOC_LITERAL(13, 172, 12)
    },
    "TrackerOnBoVWwindow\0setDatasetPath\0\0"
    "updateCamera\0setCamera\0setFeatureType\0"
    "setBoVWpath\0changeBoVWpath\0setSampleInterval\0"
    "setSavePath\0changeSavePath\0runTracking\0"
    "pauseTracking\0stopTracking"
};
#undef QT_MOC_LITERAL

static const uint qt_meta_data_TrackerOnBoVWwindow[] = {

 // content:
       7,       // revision
       0,       // classname
       0,    0, // classinfo
      12,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       0,       // signalCount

 // slots: name, argc, parameters, tag, flags
       1,    0,   74,    2, 0x0a /* Public */,
       3,    0,   75,    2, 0x0a /* Public */,
       4,    0,   76,    2, 0x0a /* Public */,
       5,    0,   77,    2, 0x0a /* Public */,
       6,    0,   78,    2, 0x0a /* Public */,
       7,    0,   79,    2, 0x0a /* Public */,
       8,    0,   80,    2, 0x0a /* Public */,
       9,    0,   81,    2, 0x0a /* Public */,
      10,    0,   82,    2, 0x0a /* Public */,
      11,    0,   83,    2, 0x0a /* Public */,
      12,    0,   84,    2, 0x0a /* Public */,
      13,    0,   85,    2, 0x0a /* Public */,

 // slots: parameters
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,
    QMetaType::Void,

       0        // eod
};

void TrackerOnBoVWwindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    if (_c == QMetaObject::InvokeMetaMethod) {
        TrackerOnBoVWwindow *_t = static_cast<TrackerOnBoVWwindow *>(_o);
        switch (_id) {
        case 0: _t->setDatasetPath(); break;
        case 1: _t->updateCamera(); break;
        case 2: _t->setCamera(); break;
        case 3: _t->setFeatureType(); break;
        case 4: _t->setBoVWpath(); break;
        case 5: _t->changeBoVWpath(); break;
        case 6: _t->setSampleInterval(); break;
        case 7: _t->setSavePath(); break;
        case 8: _t->changeSavePath(); break;
        case 9: _t->runTracking(); break;
        case 10: _t->pauseTracking(); break;
        case 11: _t->stopTracking(); break;
        default: ;
        }
    }
    Q_UNUSED(_a);
}

const QMetaObject TrackerOnBoVWwindow::staticMetaObject = {
    { &QMainWindow::staticMetaObject, qt_meta_stringdata_TrackerOnBoVWwindow.data,
      qt_meta_data_TrackerOnBoVWwindow,  qt_static_metacall, 0, 0}
};


const QMetaObject *TrackerOnBoVWwindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *TrackerOnBoVWwindow::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_TrackerOnBoVWwindow.stringdata))
        return static_cast<void*>(const_cast< TrackerOnBoVWwindow*>(this));
    return QMainWindow::qt_metacast(_clname);
}

int TrackerOnBoVWwindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 12)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 12;
    } else if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 12)
            *reinterpret_cast<int*>(_a[0]) = -1;
        _id -= 12;
    }
    return _id;
}
QT_END_MOC_NAMESPACE
