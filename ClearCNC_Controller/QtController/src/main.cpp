#include "MainWindow.h"

#include <QApplication>
#include <QIcon>
#include <QString>
#include <QSurfaceFormat>

#ifdef Q_OS_WIN
#    define WIN32_LEAN_AND_MEAN
#    include <windows.h>
#    include <shobjidl.h> // SetCurrentProcessExplicitAppUserModelID
#endif

int main(int argc, char *argv[]) {
#ifdef Q_OS_WIN
    // Pinned taskbar uses shell identity; must be set before QApplication in main().
    (void)SetCurrentProcessExplicitAppUserModelID(L"ClearCNC.ClearCNCController.1.0");
#endif
    {
        QSurfaceFormat fmt;
        fmt.setDepthBufferSize(24);
        fmt.setStencilBufferSize(8);
        fmt.setSamples(4);
        fmt.setVersion(2, 1);
        fmt.setProfile(QSurfaceFormat::CompatibilityProfile);
        QSurfaceFormat::setDefaultFormat(fmt);
    }
    QApplication app(argc, argv);
    // MainWindow sets its own sheet; QMessageBox / QDialog are top-level and need explicit contrast here.
    app.setStyleSheet(QString::fromUtf8(
        "QMessageBox { background-color: #2a2f36; }"
        "QMessageBox QLabel { color: #f0f4f8; background: transparent; }"
        "QMessageBox QTextEdit { background: #1f242b; color: #e8edf4; border: 1px solid #3d4650; }"
        "QMessageBox QPushButton { background: #3d4450; color: #f2f6fb; border: 1px solid #5a6575;"
        " border-radius: 2px; min-width: 92px; min-height: 26px; padding: 4px 14px; }"
        "QMessageBox QPushButton:hover { background: #4c5565; }"
        "QMessageBox QPushButton:pressed { background: #353b46; }"
        "QDialog { background-color: #2a2f36; }"
        "QDialog QWidget { color: #e6ebf3; }"
        "QDialog QLabel { color: #e6ebf3; }"
        "QDialog QComboBox, QDialog QLineEdit, QDialog QDoubleSpinBox { background: #282d34; color: #e5e9ef;"
        " border: 1px solid #3d4650; border-radius: 2px; min-height: 24px; padding: 2px 8px; }"
        "QDialog QGroupBox { color: #e6ebf3; }"
        "QDialog QCheckBox { color: #e6ebf3; }"
        "QDialog QTabWidget::pane { border: 1px solid #3d4650; background: #262b32; }"
        "QDialog QTabBar::tab { color: #c8d0da; background: #323840; padding: 8px 16px; }"
        "QDialog QTabBar::tab:selected { color: #f2f6fb; background: #3a4250; }"
        "QDialog QDialogButtonBox QPushButton { background: #3d4450; color: #f2f6fb; border: 1px solid #5a6575;"
        " border-radius: 2px; min-width: 88px; min-height: 26px; padding: 4px 14px; }"
        "QDialog QDialogButtonBox QPushButton:hover { background: #4c5565; }"));
    app.setApplicationName("ClearCNC Controller");
    app.setOrganizationName("ClearCNC");
    app.setWindowIcon(QIcon(":/icons/clearcnc_app_icon.svg"));
    MainWindow window;
    window.setWindowIcon(QIcon(":/icons/clearcnc_app_icon.svg"));
    window.show();
    return app.exec();
}

