/****************************************************************************
** Meta object code from reading C++ file 'MainWindow.h'
**
** Created by: The Qt Meta Object Compiler version 69 (Qt 6.10.0)
**
** WARNING! All changes made in this file will be lost!
*****************************************************************************/

#include "../../../QtController/src/MainWindow.h"
#include <QtCore/qmetatype.h>

#include <QtCore/qtmochelpers.h>

#include <memory>


#include <QtCore/qxptype_traits.h>
#if !defined(Q_MOC_OUTPUT_REVISION)
#error "The header file 'MainWindow.h' doesn't include <QObject>."
#elif Q_MOC_OUTPUT_REVISION != 69
#error "This file was generated using the moc from 6.10.0. It"
#error "cannot be used with the include files from this version of Qt."
#error "(The moc has changed too much.)"
#endif

#ifndef Q_CONSTINIT
#define Q_CONSTINIT
#endif

QT_WARNING_PUSH
QT_WARNING_DISABLE_DEPRECATED
QT_WARNING_DISABLE_GCC("-Wuseless-cast")
namespace {
struct qt_meta_tag_ZN10MainWindowE_t {};
} // unnamed namespace

template <> constexpr inline auto MainWindow::qt_create_metaobjectdata<qt_meta_tag_ZN10MainWindowE_t>()
{
    namespace QMC = QtMocConstants;
    QtMocHelpers::StringRefStorage qt_stringData {
        "MainWindow",
        "RefreshPorts",
        "",
        "DiscoverEthernetDevices",
        "ConnectOrDisconnect",
        "OnSerialReadyRead",
        "OnTcpReadyRead",
        "OnTcpDisconnected",
        "OnTelemetryReadyRead",
        "OnTelemetryDisconnected",
        "OnUdpReadyRead",
        "OnSerialError",
        "QSerialPort::SerialPortError",
        "error",
        "SendStatusPoll",
        "SendEnable",
        "SendDisable",
        "SendStop",
        "SendEmergencyStop",
        "SendSetAbs",
        "SendSetRel",
        "SendZero",
        "ValidateFeedInput",
        "ShowMotionParametersDialog",
        "SendMove",
        "SendJogXPos",
        "SendJogXNeg",
        "SendJogYPos",
        "SendJogYNeg",
        "SendJogZPos",
        "SendJogZNeg",
        "SendJogAPos",
        "SendJogANeg",
        "LoadGCodeProgram",
        "OpenGCodeProgramFromMenu",
        "StartGCodeProgram",
        "PauseGCodeProgram",
        "StopGCodeProgram",
        "UpdateTransportUi"
    };

    QtMocHelpers::UintData qt_methods {
        // Slot 'RefreshPorts'
        QtMocHelpers::SlotData<void()>(1, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'DiscoverEthernetDevices'
        QtMocHelpers::SlotData<void()>(3, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'ConnectOrDisconnect'
        QtMocHelpers::SlotData<void()>(4, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnSerialReadyRead'
        QtMocHelpers::SlotData<void()>(5, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnTcpReadyRead'
        QtMocHelpers::SlotData<void()>(6, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnTcpDisconnected'
        QtMocHelpers::SlotData<void()>(7, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnTelemetryReadyRead'
        QtMocHelpers::SlotData<void()>(8, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnTelemetryDisconnected'
        QtMocHelpers::SlotData<void()>(9, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnUdpReadyRead'
        QtMocHelpers::SlotData<void()>(10, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OnSerialError'
        QtMocHelpers::SlotData<void(QSerialPort::SerialPortError)>(11, 2, QMC::AccessPrivate, QMetaType::Void, {{
            { 0x80000000 | 12, 13 },
        }}),
        // Slot 'SendStatusPoll'
        QtMocHelpers::SlotData<void()>(14, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendEnable'
        QtMocHelpers::SlotData<void()>(15, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendDisable'
        QtMocHelpers::SlotData<void()>(16, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendStop'
        QtMocHelpers::SlotData<void()>(17, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendEmergencyStop'
        QtMocHelpers::SlotData<void()>(18, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendSetAbs'
        QtMocHelpers::SlotData<void()>(19, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendSetRel'
        QtMocHelpers::SlotData<void()>(20, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendZero'
        QtMocHelpers::SlotData<void()>(21, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'ValidateFeedInput'
        QtMocHelpers::SlotData<void()>(22, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'ShowMotionParametersDialog'
        QtMocHelpers::SlotData<void()>(23, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendMove'
        QtMocHelpers::SlotData<void()>(24, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogXPos'
        QtMocHelpers::SlotData<void()>(25, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogXNeg'
        QtMocHelpers::SlotData<void()>(26, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogYPos'
        QtMocHelpers::SlotData<void()>(27, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogYNeg'
        QtMocHelpers::SlotData<void()>(28, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogZPos'
        QtMocHelpers::SlotData<void()>(29, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogZNeg'
        QtMocHelpers::SlotData<void()>(30, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogAPos'
        QtMocHelpers::SlotData<void()>(31, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'SendJogANeg'
        QtMocHelpers::SlotData<void()>(32, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'LoadGCodeProgram'
        QtMocHelpers::SlotData<void()>(33, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'OpenGCodeProgramFromMenu'
        QtMocHelpers::SlotData<void()>(34, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'StartGCodeProgram'
        QtMocHelpers::SlotData<void()>(35, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'PauseGCodeProgram'
        QtMocHelpers::SlotData<void()>(36, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'StopGCodeProgram'
        QtMocHelpers::SlotData<void()>(37, 2, QMC::AccessPrivate, QMetaType::Void),
        // Slot 'UpdateTransportUi'
        QtMocHelpers::SlotData<void()>(38, 2, QMC::AccessPrivate, QMetaType::Void),
    };
    QtMocHelpers::UintData qt_properties {
    };
    QtMocHelpers::UintData qt_enums {
    };
    return QtMocHelpers::metaObjectData<MainWindow, qt_meta_tag_ZN10MainWindowE_t>(QMC::MetaObjectFlag{}, qt_stringData,
            qt_methods, qt_properties, qt_enums);
}
Q_CONSTINIT const QMetaObject MainWindow::staticMetaObject = { {
    QMetaObject::SuperData::link<QMainWindow::staticMetaObject>(),
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN10MainWindowE_t>.stringdata,
    qt_staticMetaObjectStaticContent<qt_meta_tag_ZN10MainWindowE_t>.data,
    qt_static_metacall,
    nullptr,
    qt_staticMetaObjectRelocatingContent<qt_meta_tag_ZN10MainWindowE_t>.metaTypes,
    nullptr
} };

void MainWindow::qt_static_metacall(QObject *_o, QMetaObject::Call _c, int _id, void **_a)
{
    auto *_t = static_cast<MainWindow *>(_o);
    if (_c == QMetaObject::InvokeMetaMethod) {
        switch (_id) {
        case 0: _t->RefreshPorts(); break;
        case 1: _t->DiscoverEthernetDevices(); break;
        case 2: _t->ConnectOrDisconnect(); break;
        case 3: _t->OnSerialReadyRead(); break;
        case 4: _t->OnTcpReadyRead(); break;
        case 5: _t->OnTcpDisconnected(); break;
        case 6: _t->OnTelemetryReadyRead(); break;
        case 7: _t->OnTelemetryDisconnected(); break;
        case 8: _t->OnUdpReadyRead(); break;
        case 9: _t->OnSerialError((*reinterpret_cast<std::add_pointer_t<QSerialPort::SerialPortError>>(_a[1]))); break;
        case 10: _t->SendStatusPoll(); break;
        case 11: _t->SendEnable(); break;
        case 12: _t->SendDisable(); break;
        case 13: _t->SendStop(); break;
        case 14: _t->SendEmergencyStop(); break;
        case 15: _t->SendSetAbs(); break;
        case 16: _t->SendSetRel(); break;
        case 17: _t->SendZero(); break;
        case 18: _t->ValidateFeedInput(); break;
        case 19: _t->ShowMotionParametersDialog(); break;
        case 20: _t->SendMove(); break;
        case 21: _t->SendJogXPos(); break;
        case 22: _t->SendJogXNeg(); break;
        case 23: _t->SendJogYPos(); break;
        case 24: _t->SendJogYNeg(); break;
        case 25: _t->SendJogZPos(); break;
        case 26: _t->SendJogZNeg(); break;
        case 27: _t->SendJogAPos(); break;
        case 28: _t->SendJogANeg(); break;
        case 29: _t->LoadGCodeProgram(); break;
        case 30: _t->OpenGCodeProgramFromMenu(); break;
        case 31: _t->StartGCodeProgram(); break;
        case 32: _t->PauseGCodeProgram(); break;
        case 33: _t->StopGCodeProgram(); break;
        case 34: _t->UpdateTransportUi(); break;
        default: ;
        }
    }
}

const QMetaObject *MainWindow::metaObject() const
{
    return QObject::d_ptr->metaObject ? QObject::d_ptr->dynamicMetaObject() : &staticMetaObject;
}

void *MainWindow::qt_metacast(const char *_clname)
{
    if (!_clname) return nullptr;
    if (!strcmp(_clname, qt_staticMetaObjectStaticContent<qt_meta_tag_ZN10MainWindowE_t>.strings))
        return static_cast<void*>(this);
    return QMainWindow::qt_metacast(_clname);
}

int MainWindow::qt_metacall(QMetaObject::Call _c, int _id, void **_a)
{
    _id = QMainWindow::qt_metacall(_c, _id, _a);
    if (_id < 0)
        return _id;
    if (_c == QMetaObject::InvokeMetaMethod) {
        if (_id < 35)
            qt_static_metacall(this, _c, _id, _a);
        _id -= 35;
    }
    if (_c == QMetaObject::RegisterMethodArgumentMetaType) {
        if (_id < 35)
            *reinterpret_cast<QMetaType *>(_a[0]) = QMetaType();
        _id -= 35;
    }
    return _id;
}
QT_WARNING_POP
