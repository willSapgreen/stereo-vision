/****************************************************************************
** Meta object code from reading C++ file 'quadmatcherthread.h'
**
** Created: Thu Dec 9 09:47:58 2010
**      by: The Qt Meta Object Compiler version 62 (Qt 4.7.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "quadmatcherthread.h"
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'quadmatcherthread.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 62
#error "This file was generated using the moc from 4.7.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

QT_BEGIN_MOC_NAMESPACE
static const uint qt_meta_data_QuadMatcherThread[] = {

 // content:
       5,       // revision
       0,       // classname
       0,    0, // classinfo
       1,   14, // methods
       0,    0, // properties
       0,    0, // enums/sets
       0,    0, // constructors
       0,       // flags
       1,       // signalCount

 // signals: signature, parameters, type, tag, flags
      19,   18,   18,   18, 0x05,

       0        // eod
};

static const char qt_meta_stringdata_QuadMatcherThread[] = {
    "QuadMatcherThread\0\0newQuadMatchArrived()\0"
};

const QMetaObject QuadMatcherThread::staticMetaObject = {
    { &QThread::staticMetaObject, qt_meta_stringdata_QuadMatcherThread,
      qt_meta_data_QuadMatcherThread, 0 }
};

#ifdef Q_NO_DATA_RELOCATION
const QMetaObject &QuadMatcherThread::getStaticMetaObject() { return staticMetaObject; }
#endif //Q_NO_DATA_RELOCATION

const QMetaObject *QuadMatcherThread::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->metaObject : &staticMetaObject;
}

void *QuadMatcherThread::qt_metacast(const char *_clname)
{
    if (!_clname) return 0;
    if (!strcmp(_clname, qt_meta_stringdata_QuadMatcherThread))
        return static_cast<void*>(const_cast< QuadMatcherThread*>(this));
    return QThread::qt_metacast(_clname);
}

int QuadMatcherThread::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QThread::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: newQuadMatchArrived(); break;
        default: ;
        }
        _id -= 1;
    }
    return _id;
}

// SIGNAL 0
void QuadMatcherThread::newQuadMatchArrived()
{
    QMetaObject::activate(this, &staticMetaObject, 0, 0);
}
QT_END_MOC_NAMESPACE
