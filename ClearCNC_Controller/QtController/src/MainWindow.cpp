#include "MainWindow.h"
#include "GCodeProgramKinematics.h"
#include "PositionView3D.h"

#include <QCheckBox>
#include <QAction>
#include <QCloseEvent>
#include <QShowEvent>
#include <QComboBox>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QDoubleSpinBox>
#include <QFile>
#include <QFileDialog>
#include <QFontDatabase>
#include <QFormLayout>
#include <QGridLayout>
#include <QGroupBox>
#include <QKeySequence>
#include <QHBoxLayout>
#include <QFileInfo>
#include <QFrame>
#include <QFont>
#include <QLabel>
#include <QLineEdit>
#include <QMenuBar>
#include <QMessageBox>
#include <QSignalBlocker>
#include <QNetworkInterface>
#include <QPushButton>
#include <QColor>
#include <QSet>
#include <QRegularExpression>
#include <QSerialPortInfo>
#include <QSettings>
#include <QSplitter>
#include <QSizePolicy>
#include <QStatusBar>
#include <QStringList>
#include <QTimer>
#include <QTabWidget>
#include <QTextEdit>
#include <QTextBlock>
#include <QTextCursor>
#include <QTextFormat>
#include <QTextStream>
#include <QtMath>
#include <QVBoxLayout>
#include <QWidget>

namespace {
constexpr int kDefaultPollMs = 500;
constexpr int kDefaultControlPort = 8888;
constexpr int kDefaultTelemetryPort = 8889;

QSettings AppSettings() {
    return QSettings("ClearCNC", "ClearCNC_Controller");
}

bool ParseTaggedDouble(const QString &line, const QString &key, double &value) {
    const QStringList tokens = line.split(QRegularExpression("\\s+"), Qt::SkipEmptyParts);
    for (const QString &token : tokens) {
        if (!token.startsWith(key, Qt::CaseInsensitive)) {
            continue;
        }
        bool ok = false;
        value = token.mid(key.length()).toDouble(&ok);
        return ok;
    }
    return false;
}

// Log only: queue-full replies are backpressure, not a failure; keep wording neutral.
QString FormatControlRxForUserLog(const QString &line) {
    if (line.contains("queue full", Qt::CaseInsensitive)) {
        return QStringLiteral("Buffer full");
    }
    return line;
}

double ParseDoubleLine(const QLineEdit *e, double defaultValue) {
    if (!e) {
        return defaultValue;
    }
    bool ok = false;
    const double v = e->text().trimmed().toDouble(&ok);
    return ok ? v : defaultValue;
}
}

MainWindow::MainWindow(QWidget *parent)
    : QMainWindow(parent),
      m_portCombo(nullptr),
      m_baudCombo(nullptr),
      m_refreshPortsButton(nullptr),
      m_discoverButton(nullptr),
      m_connectButton(nullptr),
      m_serialSettingsWidget(nullptr),
      m_ethernetSettingsWidget(nullptr),
      m_enableButton(nullptr),
      m_disableButton(nullptr),
      m_stopButton(nullptr),
      m_estopButton(nullptr),
      m_absButton(nullptr),
      m_relButton(nullptr),
      m_zeroButton(nullptr),
      m_targetXLabel(nullptr),
      m_targetYLabel(nullptr),
      m_targetZLabel(nullptr),
      m_targetALabel(nullptr),
      m_moveXEdit(nullptr),
      m_moveYEdit(nullptr),
      m_moveZEdit(nullptr),
      m_moveAEdit(nullptr),
      m_feedEdit(nullptr),
      m_moveButton(nullptr),
      m_jogStepEdit(nullptr),
      m_jogXPosButton(nullptr),
      m_jogXNegButton(nullptr),
      m_jogYPosButton(nullptr),
      m_jogYNegButton(nullptr),
      m_jogZPosButton(nullptr),
      m_jogZNegButton(nullptr),
      m_jogAPosButton(nullptr),
      m_jogANegButton(nullptr),
      m_programPreview(nullptr),
      m_programStatusLabel(nullptr),
      m_bufferStatusLabel(nullptr),
      m_loadProgramButton(nullptr),
      m_runProgramButton(nullptr),
      m_pauseProgramButton(nullptr),
      m_stopProgramButton(nullptr),
      m_clearLogButton(nullptr),
      m_minimizeLogButton(nullptr),
      m_logEdit(nullptr),
      m_logGroup(nullptr),
      m_toggleLogAction(nullptr),
      m_showLogButton(nullptr),
      m_workAreaSplitter(nullptr),
      m_mainVSplitter(nullptr),
      m_workTabs(nullptr),
      m_programWorkTab(nullptr),
      m_positionView3D(nullptr),
      m_viewportPathTraceCheck(nullptr),
      m_vizX(0.0),
      m_vizY(0.0),
      m_vizZ(0.0),
      m_vizA(0.0),
      m_stepsPerUnit(160.0),
      m_velocity(5000.0),
      m_accel(85000.0),
      m_decel(85000.0),
      m_junctionDVmax(0.0),
      m_guiUnitsInches(false),
      m_singleMotorBench(false),
      m_programIndex(0),
      m_programRunning(false),
      m_programPaused(false),
      m_programAwaitingAck(false),
      m_programWaitingForQueueDrain(false),
      m_queueDrainStalledPolls(0),
      m_programQueueFullRetries(0),
      m_activeProgramLine(-1),
      m_controllerEnabled(false),
      m_usingEthernet(false),
      m_didWorkAreaInitialSplit(false),
      m_savedLogPaneHeight(320) {
    LoadMotionSettings();

    BuildUi();
    RefreshPorts();
    SetConnectedUi(false);

    connect(&m_serial, &QSerialPort::readyRead, this, &MainWindow::OnSerialReadyRead);
    connect(&m_serial, &QSerialPort::errorOccurred, this, &MainWindow::OnSerialError);
    connect(&m_tcpSocket, &QTcpSocket::readyRead, this, &MainWindow::OnTcpReadyRead);
    connect(&m_tcpSocket, &QTcpSocket::disconnected, this, &MainWindow::OnTcpDisconnected);
    connect(&m_telemetrySocket, &QTcpSocket::readyRead, this, &MainWindow::OnTelemetryReadyRead);
    connect(&m_telemetrySocket, &QTcpSocket::disconnected, this, &MainWindow::OnTelemetryDisconnected);
    connect(&m_udpSocket, &QUdpSocket::readyRead, this, &MainWindow::OnUdpReadyRead);

    m_pollTimer.setInterval(kDefaultPollMs);
    connect(&m_pollTimer, &QTimer::timeout, this, &MainWindow::SendStatusPoll);
}

MainWindow::~MainWindow() {
    DisableControllerBeforeDisconnect();
    if (m_serial.isOpen()) {
        m_serial.close();
    }
}

void MainWindow::showEvent(QShowEvent *event) {
    QMainWindow::showEvent(event);
    if (m_didWorkAreaInitialSplit || !m_workAreaSplitter) {
        return;
    }
    // After the first layout pass, widths are real (sync showEvent is often 0 on first show).
    QTimer::singleShot(0, this, [this]() {
        if (m_didWorkAreaInitialSplit || !m_workAreaSplitter) {
            return;
        }
        int w = m_workAreaSplitter->width();
        if (w < 8) {
            w = qMax(8, (centralWidget() ? centralWidget()->width() : width()) - 24);
        }
        if (w < 8) {
            return;
        }
        // Default: 40% MDI+program (left) / 60% 3D viewport (right).
        const int leftW = (w * 40) / 100;
        m_workAreaSplitter->setSizes({leftW, w - leftW});
        m_didWorkAreaInitialSplit = true;
    });
}

void MainWindow::closeEvent(QCloseEvent *event) {
    SaveWindowUiSettings();
    DisableControllerBeforeDisconnect();
    m_pollTimer.stop();
    if (m_telemetrySocket.state() == QAbstractSocket::ConnectedState) {
        m_telemetrySocket.disconnectFromHost();
    }
    if (m_tcpSocket.state() == QAbstractSocket::ConnectedState) {
        m_tcpSocket.disconnectFromHost();
    }
    if (m_serial.isOpen()) {
        m_serial.close();
    }
    event->accept();
}

void MainWindow::BuildUi() {
    resize(1280, 820);
    setMinimumSize(1080, 760);
    setStyleSheet(
        "QMainWindow { background: #25292f; border: none; }"
        "QMenuBar { background: #32363c; color: #d6d9de; border: none; border-bottom: 1px solid #20242a; padding: 4px 8px; }"
        "QMenuBar::item { padding: 6px 14px; border-radius: 2px; }"
        "QMenuBar::item:selected { background: #444b54; color: #f0f2f5; }"
        "QMenu { background: #2f343a; color: #d8dce2; border: 1px solid #20242a; padding: 4px 0; }"
        "QMenu::item { padding: 8px 32px 8px 16px; }"
        "QMenu::item:selected { background: #49525e; color: #f1f4f8; }"
        "QWidget { color: #dde2e8; font-size: 12px; }"
        "QGroupBox { background: #2f343a; border: 1px solid #1a1e24; border-radius: 3px; margin-top: 20px; padding-top: 2px; font-weight: 600; color: #e6ebf3; }"
        "QGroupBox::title {"
        " subcontrol-origin: margin; subcontrol-position: top left; left: 14px; top: 2px; padding: 0 6px; margin-top: 0px;"
        " color: #c9d1dc; background: transparent; border: none; font-weight: 600; }"
        "QPushButton { background: #3a3f46; color: #dce1e8; border: 1px solid #23272d; border-radius: 2px; min-height: 28px; padding: 4px 12px; font-weight: 600; }"
        "QPushButton:hover { border-color: #586473; background: #424850; }"
        "QPushButton:pressed { background: #30353b; border-color: #6d7a8c; }"
        "QPushButton:disabled { background: #2d3238; color: #7b8490; border-color: #252a31; }"
        "QPushButton#ClearCncModeAbs:checked { background: #214f38; color: #ecfff2; border: 1px solid #4ca06f; font-weight: 800; }"
        "QPushButton#ClearCncModeRel:checked { background: #243f63; color: #eef5ff; border: 1px solid #77a7de; font-weight: 800; }"
        "QPushButton#ClearCncModeAbs:!checked, QPushButton#ClearCncModeRel:!checked { background: #343a42; color: #8e97a3; border: 1px solid #23272d; font-weight: 600; }"
        "QPushButton#ClearCncModeAbs:!checked:hover, QPushButton#ClearCncModeRel:!checked:hover { background: #3d4450; color: #c5ccd6; border-color: #4a5563; }"
        "QPushButton#ClearCncModeAbs:disabled, QPushButton#ClearCncModeRel:disabled { background: #2d3238; color: #6a7380; border-color: #252a31; }"
        "QLineEdit, QComboBox, QDoubleSpinBox { background: #282d34; color: #e5e9ef; border: 1px solid #20242a; border-radius: 2px; min-height: 26px; padding: 3px 8px; selection-background-color: #5a82c2; }"
        "QLineEdit:disabled, QComboBox:disabled, QDoubleSpinBox:disabled { background: #242930; color: #aeb7c4; border-color: #20242a; }"
        "QLineEdit:read-only { color: #dbe2ec; background: #242930; }"
        "QDoubleSpinBox::up-button, QDoubleSpinBox::down-button { background: #2d3239; border-left: 1px solid #20242a; width: 16px; }"
        "QDoubleSpinBox::up-button:hover, QDoubleSpinBox::down-button:hover { background: #39404a; }"
        "QDoubleSpinBox::up-arrow { image: url(:/icons/spin-up.svg); width: 12px; height: 8px; border: none; }"
        "QDoubleSpinBox::down-arrow { image: url(:/icons/combo-down.svg); width: 12px; height: 8px; border: none; }"
        "QComboBox::drop-down { subcontrol-origin: padding; subcontrol-position: right center; border: none; width: 28px; background: #2d3239; border-left: 1px solid #20242a; border-top-right-radius: 2px; border-bottom-right-radius: 2px; }"
        "QComboBox::drop-down:hover { background: #353b44; }"
        "QComboBox::down-arrow { image: url(:/icons/combo-down.svg); width: 12px; height: 8px; border: none; }"
        "QComboBox:hover { background: #2c3138; border-color: #3d4650; }"
        "QComboBox:focus { border-color: #5a82c2; }"
        "QComboBox QAbstractItemView { background: #282d34; color: #e5e9ef; border: 1px solid #1a1e24; selection-background-color: #3d5a8c; selection-color: #f0f4f8; outline: 0; padding: 2px; }"
        "QComboBox QAbstractItemView::item { min-height: 24px; padding: 4px 8px; }"
        "QComboBox QAbstractItemView::item:selected { background: #3d5a8c; }"
        "QPushButton[role=\"primary\"] { background: #274874; color: #eef5ff; border: 1px solid #5a8fd4; font-weight: 700; }"
        "QPushButton[role=\"primary\"]:hover { background: #2e5385; border-color: #6aa0e0; }"
        "QPushButton[role=\"primary\"]:pressed { background: #1f3d5f; border-color: #4a7ec4; }"
        "QPushButton[role=\"primary\"]:disabled { background: #2d3238; color: #7a8595; border-color: #2f3540; }"
        "QTextEdit { background: #1f242b; color: #e6ebf3; border: 1px solid #20242a; border-radius: 2px; selection-background-color: #5a82c2; font-family: Consolas, 'Courier New', monospace; }"
        "QTabWidget::pane { border: none; background: #25292f; margin-top: 8px; }"
        "QTabBar { background: #2f343a; border: 1px solid #20242a; border-radius: 2px; padding: 2px; }"
        "QTabBar::tab { background: transparent; color: #aeb6c2; border: none; padding: 8px 16px; min-width: 72px; font-weight: 700; border-radius: 2px; }"
        "QTabBar::tab:selected { color: #f0f4f8; background: #3a3f46; border: 1px solid #4b5563; }"
        "QTabBar::tab:hover:!selected { background: #363c44; color: #d1d7df; }"
        "QSplitter { border: none; }"
        "QSplitter::handle:vertical { background: #20242a; height: 3px; }"
        "QSplitter::handle:vertical:hover { background: #3f4853; }"
        "QSplitter::handle:horizontal { background: #20242a; width: 3px; }"
        "QSplitter::handle:horizontal:hover { background: #3f4853; }"
        "QFrame#ClearCncMachineControlSep { color: #4a5563; }"
        "QStatusBar { background: #2c3036; color: #9fa6af; border-top: 1px solid #20242a; padding: 6px 14px; }"
        "QStatusBar::item { border: none; }");

    auto *setupMenu = menuBar()->addMenu("&Setup");
    auto *motionParamsAction = setupMenu->addAction("Motion Parameters...");
    connect(motionParamsAction, &QAction::triggered,
            this, &MainWindow::ShowMotionParametersDialog);
    auto *programMenu = menuBar()->addMenu("&Program");
    auto *openProgramAction = programMenu->addAction("Open G-Code...");
    connect(openProgramAction, &QAction::triggered, this, &MainWindow::OpenGCodeProgramFromMenu);

    auto *viewMenu = menuBar()->addMenu("&View");
    m_toggleLogAction = new QAction("Command &log", this);
    m_toggleLogAction->setCheckable(true);
    m_toggleLogAction->setChecked(true);
    m_toggleLogAction->setShortcut(QKeySequence(Qt::CTRL | Qt::Key_L));
    m_toggleLogAction->setStatusTip("Show or hide the command activity log (tabs and 3D viewer use more space when hidden).");
    viewMenu->addAction(m_toggleLogAction);
    connect(m_toggleLogAction, &QAction::toggled, this, [this](bool on) { SetLogPanelVisible(on); });

    auto *central = new QWidget(this);
    auto *root = new QVBoxLayout(central);
    root->setContentsMargins(12, 12, 12, 12);
    root->setSpacing(10);

    auto *connGroup = new QGroupBox("Connection", central);
    connGroup->setMinimumHeight(88);
    auto *connLayout = new QHBoxLayout(connGroup);
    connLayout->setContentsMargins(14, 18, 14, 12);
    connLayout->setSpacing(10);
    m_transportCombo = new QComboBox(connGroup);
    m_transportCombo->addItems({"Serial", "Ethernet"});
    const QString savedTransport = AppSettings().value("connection/transport", "Serial").toString();
    const int savedTransportIdx = m_transportCombo->findText(savedTransport);
    if (savedTransportIdx >= 0) {
        m_transportCombo->setCurrentIndex(savedTransportIdx);
    }
    m_portCombo = new QComboBox(connGroup);
    m_deviceCombo = new QComboBox(connGroup);
    m_deviceCombo->setMinimumWidth(210);
    const QString savedEthernetHost = AppSettings().value("connection/ethernetHost").toString();
    const int savedControlPort = AppSettings().value("connection/controlPort", kDefaultControlPort).toInt();
    const int savedTelemetryPort = AppSettings().value("connection/telemetryPort", kDefaultTelemetryPort).toInt();
    if (!savedEthernetHost.isEmpty()) {
        m_deviceCombo->addItem(QString("%1 (saved, TCP=%2 TEL=%3)")
                                   .arg(savedEthernetHost)
                                   .arg(savedControlPort)
                                   .arg(savedTelemetryPort),
                               savedEthernetHost);
    }
    m_baudCombo = new QComboBox(connGroup);
    m_baudCombo->addItems({"115200", "230400", "460800", "921600"});
    const QString savedBaud = AppSettings().value("connection/baud", "115200").toString();
    const int savedBaudIdx = m_baudCombo->findText(savedBaud);
    if (savedBaudIdx >= 0) {
        m_baudCombo->setCurrentIndex(savedBaudIdx);
    }
    m_refreshPortsButton = new QPushButton("Refresh", connGroup);
    m_discoverButton = new QPushButton("Discover", connGroup);
    m_connectButton = new QPushButton("Connect", connGroup);
    m_connectButton->setProperty("role", "primary");
    m_serialSettingsWidget = new QWidget(connGroup);
    auto *serialSettingsLayout = new QHBoxLayout(m_serialSettingsWidget);
    serialSettingsLayout->setContentsMargins(0, 0, 0, 0);
    serialSettingsLayout->setSpacing(10);
    serialSettingsLayout->addWidget(new QLabel("Port:", m_serialSettingsWidget));
    serialSettingsLayout->addWidget(m_portCombo, 2);
    serialSettingsLayout->addWidget(new QLabel("Baud:", m_serialSettingsWidget));
    serialSettingsLayout->addWidget(m_baudCombo, 1);
    serialSettingsLayout->addWidget(m_refreshPortsButton);

    m_ethernetSettingsWidget = new QWidget(connGroup);
    auto *ethernetSettingsLayout = new QHBoxLayout(m_ethernetSettingsWidget);
    ethernetSettingsLayout->setContentsMargins(0, 0, 0, 0);
    ethernetSettingsLayout->setSpacing(10);
    ethernetSettingsLayout->addWidget(new QLabel("Device:", m_ethernetSettingsWidget));
    ethernetSettingsLayout->addWidget(m_deviceCombo, 2);
    ethernetSettingsLayout->addWidget(m_discoverButton);

    connLayout->addWidget(new QLabel("Transport:", connGroup));
    connLayout->addWidget(m_transportCombo);
    connLayout->addWidget(m_serialSettingsWidget, 3);
    connLayout->addWidget(m_ethernetSettingsWidget, 3);
    connLayout->addWidget(m_connectButton);
    root->addWidget(connGroup);

    auto *workH = new QSplitter(Qt::Horizontal, central);
    workH->setObjectName("ClearCncWorkHSplitter");
    workH->setHandleWidth(3);
    workH->setChildrenCollapsible(false);
    m_workAreaSplitter = workH;

    auto *mainSplitter = new QSplitter(Qt::Vertical, central);
    mainSplitter->setHandleWidth(3);
    mainSplitter->setObjectName("ClearCncMainSplitter");
    mainSplitter->setChildrenCollapsible(false);
    m_mainVSplitter = mainSplitter;

    auto *controlGroup = new QGroupBox("Machine Control", mainSplitter);
    controlGroup->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Maximum);
    auto *controlLayout = new QVBoxLayout(controlGroup);
    controlLayout->setContentsMargins(14, 16, 14, 10);
    controlLayout->setSpacing(0);
    m_enableButton = new QPushButton("Enable", controlGroup);
    m_disableButton = new QPushButton("Disable", controlGroup);
    m_stopButton = new QPushButton("Stop", controlGroup);
    m_estopButton = new QPushButton("E-Stop", controlGroup);
    m_absButton = new QPushButton("ABS", controlGroup);
    m_relButton = new QPushButton("REL", controlGroup);
    m_zeroButton = new QPushButton("Zero", controlGroup);
    m_zeroButton->setToolTip("Zero work position (G92 X0 Y0; extended zero can be added later)");
    m_estopButton->setProperty("role", "danger");
    m_absButton->setObjectName("ClearCncModeAbs");
    m_relButton->setObjectName("ClearCncModeRel");
    m_absButton->setCheckable(true);
    m_relButton->setCheckable(true);
    m_estopButton->setStyleSheet("QPushButton { background: #9d2f34; color: #fff2f2; border: 1px solid #d15c62; border-radius: 2px; font-weight: 800; }"
                                 "QPushButton:hover { background: #b53a40; border-color: #e4777c; }"
                                 "QPushButton:disabled { background: #463236; color: #9c777c; border-color: #61454a; }");
    const int kCtrlBtnH = 30;
    for (auto *b : {m_enableButton, m_disableButton, m_stopButton, m_estopButton, m_absButton, m_relButton,
                    m_zeroButton}) {
        b->setMinimumHeight(kCtrlBtnH);
        b->setSizePolicy(QSizePolicy::Minimum, QSizePolicy::Fixed);
    }
    m_absButton->setMinimumWidth(52);
    m_relButton->setMinimumWidth(52);
    m_zeroButton->setMinimumWidth(64);

    auto *controlRow = new QHBoxLayout();
    controlRow->setSpacing(6);
    controlRow->addWidget(m_enableButton);
    controlRow->addWidget(m_disableButton);
    controlRow->addWidget(m_stopButton);
    controlRow->addWidget(m_estopButton);
    auto *modeSep = new QFrame(controlGroup);
    modeSep->setObjectName("ClearCncMachineControlSep");
    modeSep->setFrameShape(QFrame::VLine);
    modeSep->setFrameShadow(QFrame::Plain);
    modeSep->setLineWidth(1);
    modeSep->setFixedWidth(1);
    modeSep->setSizePolicy(QSizePolicy::Fixed, QSizePolicy::MinimumExpanding);
    controlRow->addWidget(modeSep);
    controlRow->addWidget(m_absButton);
    controlRow->addWidget(m_relButton);
    controlRow->addWidget(m_zeroButton);
    controlRow->addStretch(1);
    controlLayout->addLayout(controlRow);

    auto *workTabs = new QTabWidget(mainSplitter);
    auto *mdiTab = new QWidget(workTabs);
    auto *mdiLayout = new QVBoxLayout(mdiTab);
    mdiLayout->setContentsMargins(10, 10, 10, 10);
    mdiLayout->setSpacing(10);

    auto *motionGroup = new QGroupBox("Motion", mdiTab);
    motionGroup->setMinimumHeight(145);
    auto *motionLayout = new QGridLayout(motionGroup);
    motionLayout->setContentsMargins(14, 18, 14, 12);
    motionLayout->setHorizontalSpacing(8);
    motionLayout->setVerticalSpacing(8);
    m_moveXEdit = new QLineEdit(motionGroup);
    m_moveYEdit = new QLineEdit(motionGroup);
    m_moveZEdit = new QLineEdit(motionGroup);
    m_moveAEdit = new QLineEdit(motionGroup);
    m_feedEdit = new QLineEdit(motionGroup);
    for (auto *edit : {m_moveXEdit, m_moveYEdit, m_moveZEdit, m_moveAEdit, m_feedEdit}) {
        edit->setMaxLength(32);
    }
    m_moveXEdit->setText("0.000");
    m_moveYEdit->setText("0.000");
    m_moveZEdit->setText("0.000");
    m_moveAEdit->setText("0.000");
    m_feedEdit->setText("500.000");
    m_moveButton = new QPushButton("Move", motionGroup);
    m_moveButton->setProperty("role", "primary");
    m_moveButton->setStyleSheet("QPushButton { background: #274874; color: #eef5ff; border: 1px solid #77a7de; border-radius: 2px; font-weight: 800; }"
                                "QPushButton:hover { background: #335a8e; border-color: #9ac0ee; }"
                                "QPushButton:disabled { background: #323a46; color: #8d98a8; border-color: #505967; }");
    m_targetXLabel = new QLabel("Target X:", motionGroup);
    m_targetYLabel = new QLabel("Target Y:", motionGroup);
    m_targetZLabel = new QLabel("Target Z:", motionGroup);
    m_targetALabel = new QLabel("Target 4th:", motionGroup);
    motionLayout->addWidget(m_targetXLabel, 0, 0);
    motionLayout->addWidget(m_moveXEdit, 0, 1);
    motionLayout->addWidget(m_targetYLabel, 0, 2);
    motionLayout->addWidget(m_moveYEdit, 0, 3);
    motionLayout->addWidget(m_targetZLabel, 1, 0);
    motionLayout->addWidget(m_moveZEdit, 1, 1);
    motionLayout->addWidget(m_targetALabel, 1, 2);
    motionLayout->addWidget(m_moveAEdit, 1, 3);
    motionLayout->addWidget(new QLabel("Feed:", motionGroup), 2, 0);
    motionLayout->addWidget(m_feedEdit, 2, 1);
    // Match Target X / Z field width; Move spans the Y+4th columns.
    motionLayout->addWidget(m_moveButton, 2, 2, 1, 2);
    motionLayout->setColumnStretch(1, 1);
    motionLayout->setColumnStretch(3, 1);
    mdiLayout->addWidget(motionGroup);

    auto *jogGroup = new QGroupBox("Jog", mdiTab);
    jogGroup->setMinimumHeight(125);
    auto *jogLayout = new QGridLayout(jogGroup);
    jogLayout->setContentsMargins(14, 18, 14, 12);
    jogLayout->setHorizontalSpacing(8);
    m_jogStepEdit = new QLineEdit(jogGroup);
    m_jogStepEdit->setMaxLength(32);
    m_jogStepEdit->setText("1.000");
    m_jogXPosButton = new QPushButton("X+", jogGroup);
    m_jogXNegButton = new QPushButton("X-", jogGroup);
    m_jogYPosButton = new QPushButton("Y+", jogGroup);
    m_jogYNegButton = new QPushButton("Y-", jogGroup);
    m_jogZPosButton = new QPushButton("Z+", jogGroup);
    m_jogZNegButton = new QPushButton("Z-", jogGroup);
    m_jogAPosButton = new QPushButton("4th+", jogGroup);
    m_jogANegButton = new QPushButton("4th-", jogGroup);
    // Full-width step row: partial grid spans and missing column stretch(0) made jog columns unequal.
    {
        auto *jogStepRow = new QHBoxLayout();
        jogStepRow->setSpacing(8);
        jogStepRow->addWidget(new QLabel("Step:", jogGroup), 0, Qt::AlignRight);
        jogStepRow->addWidget(m_jogStepEdit, 1);
        jogLayout->addLayout(jogStepRow, 0, 0, 1, 4);
    }
    jogLayout->addWidget(m_jogXNegButton, 1, 0);
    jogLayout->addWidget(m_jogXPosButton, 1, 1);
    jogLayout->addWidget(m_jogYNegButton, 1, 2);
    jogLayout->addWidget(m_jogYPosButton, 1, 3);
    jogLayout->addWidget(m_jogZNegButton, 2, 0);
    jogLayout->addWidget(m_jogZPosButton, 2, 1);
    jogLayout->addWidget(m_jogANegButton, 2, 2);
    jogLayout->addWidget(m_jogAPosButton, 2, 3);
    for (auto *b : {m_jogXNegButton, m_jogXPosButton, m_jogYNegButton, m_jogYPosButton, m_jogZNegButton,
                  m_jogZPosButton, m_jogANegButton, m_jogAPosButton}) {
        b->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Fixed);
        b->setMinimumHeight(32);
    }
    jogLayout->setColumnStretch(0, 1);
    jogLayout->setColumnStretch(1, 1);
    jogLayout->setColumnStretch(2, 1);
    jogLayout->setColumnStretch(3, 1);
    mdiLayout->addWidget(jogGroup);
    mdiLayout->addStretch();

    auto *programTab = new QWidget(workTabs);
    auto *programTabLayout = new QVBoxLayout(programTab);
    programTabLayout->setContentsMargins(10, 10, 10, 10);
    programTabLayout->setSpacing(10);

    auto *programGroup = new QGroupBox("G-Code Program", programTab);
    programGroup->setMinimumHeight(170);
    auto *programLayout = new QVBoxLayout(programGroup);
    programLayout->setContentsMargins(14, 18, 14, 12);
    auto *programControls = new QHBoxLayout();
    m_programStatusLabel = new QLabel("No program loaded", programGroup);
    m_programStatusLabel->setVisible(false);
    m_loadProgramButton = new QPushButton("Load", programGroup);
    m_runProgramButton = new QPushButton("Run", programGroup);
    m_pauseProgramButton = new QPushButton("Pause", programGroup);
    m_stopProgramButton = new QPushButton("Stop", programGroup);
    auto *bufferLabel = new QLabel("Buffer:", programGroup);
    m_bufferStatusLabel = new QLabel(programGroup);
    m_bufferStatusLabel->setAlignment(Qt::AlignCenter);
    m_bufferStatusLabel->setFixedSize(14, 14);
    m_bufferStatusLabel->setToolTip("Controller motion buffer");
    SetBufferIndicatorFull(false);
    programControls->addWidget(bufferLabel);
    programControls->addWidget(m_bufferStatusLabel);
    programControls->addSpacing(10);
    programControls->addStretch(1);
    programControls->addWidget(m_programStatusLabel);
    programControls->addWidget(m_loadProgramButton);
    programControls->addWidget(m_runProgramButton);
    programControls->addWidget(m_pauseProgramButton);
    programControls->addWidget(m_stopProgramButton);
    m_programPreview = new QTextEdit(programGroup);
    m_programPreview->setReadOnly(true);
    m_programPreview->setLineWrapMode(QTextEdit::NoWrap);
    m_programPreview->setPlaceholderText("Load a .nc, .tap, or .gcode file to preview cleaned commands...");
    programLayout->addLayout(programControls);
    programLayout->addWidget(m_programPreview, 1);
    programTabLayout->addWidget(programGroup, 1);

    m_workTabs = workTabs;
    m_programWorkTab = programTab;
    workTabs->addTab(mdiTab, "MDI");
    workTabs->addTab(programTab, "Program");
    workTabs->setCurrentWidget(programTab);

    auto *logGroup = new QGroupBox("Log", mainSplitter);
    m_logGroup = logGroup;
    logGroup->setMinimumHeight(220);
    auto *logLayout = new QVBoxLayout(logGroup);
    logLayout->setContentsMargins(14, 18, 14, 12);
    auto *logHeader = new QHBoxLayout();
    auto *logTitle = new QLabel("Command Activity", logGroup);
    m_minimizeLogButton = new QPushButton("Minimize", logGroup);
    m_minimizeLogButton->setToolTip("Hide the log so the MDI/Program tabs have more room. "
                                    "View → Command log, Ctrl+L, or “Show log” in the status bar to restore.");
    m_clearLogButton = new QPushButton("Clear", logGroup);
    logHeader->addWidget(logTitle);
    logHeader->addStretch();
    logHeader->addWidget(m_minimizeLogButton);
    logHeader->addWidget(m_clearLogButton);
    m_logEdit = new QTextEdit(logGroup);
    m_logEdit->setReadOnly(true);
    QFont logFont = QFontDatabase::systemFont(QFontDatabase::FixedFont);
    logFont.setPointSize(logFont.pointSize() - 1);
    m_logEdit->setFont(logFont);
    m_logEdit->setPlaceholderText("Connection and command events will appear here...");
    logLayout->addLayout(logHeader);
    logLayout->addWidget(m_logEdit);
    // Children order: Machine Control, MDI/Program tabs, Log (set by construction order with mainSplitter parent).
    mainSplitter->setCollapsible(0, false);
    mainSplitter->setCollapsible(1, false);
    mainSplitter->setCollapsible(2, true);
    mainSplitter->setStretchFactor(0, 0);
    mainSplitter->setStretchFactor(1, 3);
    mainSplitter->setStretchFactor(2, 2);
    mainSplitter->setSizes({84, 520, 300});

    workH->addWidget(mainSplitter);
    auto *pos3dGroup = new QGroupBox("Tool position (3D)", central);
    pos3dGroup->setMinimumWidth(300);
    pos3dGroup->setSizePolicy(QSizePolicy::MinimumExpanding, QSizePolicy::Expanding);
    auto *pos3dOuter = new QVBoxLayout(pos3dGroup);
    pos3dOuter->setContentsMargins(12, 18, 12, 12);
    pos3dOuter->setSpacing(6);
    auto *pos3dHeader = new QHBoxLayout();
    pos3dHeader->addWidget(new QLabel("Z up · drag orbit · wheel zoom", pos3dGroup));
    pos3dHeader->addStretch();
    m_viewportPathTraceCheck = new QCheckBox("Path", pos3dGroup);
    m_viewportPathTraceCheck->setToolTip(
        "Draw the loaded file as a yellow path in 3D (from parsed G-code). "
        "The tool arrow always follows live DRO / telemetry when connected.");
    m_viewportPathTraceCheck->setChecked(false);
    m_viewportPathTraceCheck->setEnabled(false);
    pos3dHeader->addWidget(m_viewportPathTraceCheck);
    auto *reset3d = new QPushButton("Reset view", pos3dGroup);
    pos3dHeader->addWidget(reset3d);
    pos3dOuter->addLayout(pos3dHeader);
    m_positionView3D = new PositionView3D(pos3dGroup);
    pos3dOuter->addWidget(m_positionView3D, 1);
    workH->addWidget(pos3dGroup);
    // Extra width splits 40:60 (left : 3D) when the user resizes the window.
    workH->setStretchFactor(0, 2);
    workH->setStretchFactor(1, 3);
    connect(reset3d, &QPushButton::clicked, m_positionView3D, &PositionView3D::resetView);
    connect(m_viewportPathTraceCheck, &QCheckBox::toggled, this, [this](bool on) {
        if (m_positionView3D) {
            m_positionView3D->setPathTraceEnabled(on);
        }
    });
    m_positionView3D->setPathTraceEnabled(m_viewportPathTraceCheck->isChecked());

    root->addWidget(workH, 1);

    UpdateConnectionWindowTitle(false);
    UpdateControllerEnabledStateUi();
    setCentralWidget(central);
    statusBar()->showMessage("Ready");
    m_showLogButton = new QPushButton("Show log", this);
    m_showLogButton->setVisible(false);
    m_showLogButton->setToolTip("Restore the command activity log (also: View → Command log or Ctrl+L).");
    connect(m_showLogButton, &QPushButton::clicked, this, [this]() {
        if (m_toggleLogAction) {
            m_toggleLogAction->setChecked(true);
        }
    });
    connect(m_minimizeLogButton, &QPushButton::clicked, this, [this]() {
        if (m_toggleLogAction) {
            m_toggleLogAction->setChecked(false);
        }
    });
    statusBar()->addPermanentWidget(m_showLogButton);

    connect(m_refreshPortsButton, &QPushButton::clicked, this, &MainWindow::RefreshPorts);
    connect(m_discoverButton, &QPushButton::clicked, this, &MainWindow::DiscoverEthernetDevices);
    connect(m_connectButton, &QPushButton::clicked, this, &MainWindow::ConnectOrDisconnect);
    connect(m_transportCombo, &QComboBox::currentTextChanged, this, [this](const QString &transport) {
        AppSettings().setValue("connection/transport", transport);
        UpdateTransportUi();
    });
    connect(m_enableButton, &QPushButton::clicked, this, &MainWindow::SendEnable);
    connect(m_disableButton, &QPushButton::clicked, this, &MainWindow::SendDisable);
    connect(m_stopButton, &QPushButton::clicked, this, &MainWindow::SendStop);
    connect(m_estopButton, &QPushButton::clicked, this, &MainWindow::SendEmergencyStop);
    connect(m_absButton, &QPushButton::clicked, this, &MainWindow::SendSetAbs);
    connect(m_relButton, &QPushButton::clicked, this, &MainWindow::SendSetRel);
    connect(m_zeroButton, &QPushButton::clicked, this, &MainWindow::SendZero);
    connect(m_feedEdit, &QLineEdit::textChanged, this, &MainWindow::ValidateFeedInput);
    connect(m_moveButton, &QPushButton::clicked, this, &MainWindow::SendMove);
    connect(m_jogXPosButton, &QPushButton::clicked, this, &MainWindow::SendJogXPos);
    connect(m_jogXNegButton, &QPushButton::clicked, this, &MainWindow::SendJogXNeg);
    connect(m_jogYPosButton, &QPushButton::clicked, this, &MainWindow::SendJogYPos);
    connect(m_jogYNegButton, &QPushButton::clicked, this, &MainWindow::SendJogYNeg);
    connect(m_jogZPosButton, &QPushButton::clicked, this, &MainWindow::SendJogZPos);
    connect(m_jogZNegButton, &QPushButton::clicked, this, &MainWindow::SendJogZNeg);
    connect(m_jogAPosButton, &QPushButton::clicked, this, &MainWindow::SendJogAPos);
    connect(m_jogANegButton, &QPushButton::clicked, this, &MainWindow::SendJogANeg);
    connect(m_loadProgramButton, &QPushButton::clicked, this, &MainWindow::LoadGCodeProgram);
    connect(m_runProgramButton, &QPushButton::clicked, this, &MainWindow::StartGCodeProgram);
    connect(m_pauseProgramButton, &QPushButton::clicked, this, &MainWindow::PauseGCodeProgram);
    connect(m_stopProgramButton, &QPushButton::clicked, this, &MainWindow::StopGCodeProgram);
    connect(m_clearLogButton, &QPushButton::clicked, m_logEdit, &QTextEdit::clear);
    SetMotionModeUi(true);
    ApplyGuiUnitsUi();
    UpdateTransportUi();
    LoadWindowUiSettings();
}

void MainWindow::RefreshPorts() {
    const QString selected = m_portCombo->currentText().isEmpty()
        ? AppSettings().value("connection/port").toString()
        : m_portCombo->currentText();
    m_portCombo->clear();
    for (const QSerialPortInfo &info : QSerialPortInfo::availablePorts()) {
        m_portCombo->addItem(info.portName());
    }
    int selectedIdx = m_portCombo->findText(selected);
    if (selectedIdx >= 0) {
        m_portCombo->setCurrentIndex(selectedIdx);
    }
}

void MainWindow::DiscoverEthernetDevices() {
    if (m_udpSocket.state() != QAbstractSocket::BoundState) {
        m_udpSocket.bind(QHostAddress::AnyIPv4, 0, QUdpSocket::ShareAddress | QUdpSocket::ReuseAddressHint);
    }
    const QString savedEthernetHost = AppSettings().value("connection/ethernetHost").toString();
    const int savedControlPort = AppSettings().value("connection/controlPort", kDefaultControlPort).toInt();
    const int savedTelemetryPort = AppSettings().value("connection/telemetryPort", kDefaultTelemetryPort).toInt();
    m_deviceCombo->clear();
    if (!savedEthernetHost.isEmpty()) {
        m_deviceCombo->addItem(QString("%1 (saved, TCP=%2 TEL=%3)")
                                   .arg(savedEthernetHost)
                                   .arg(savedControlPort)
                                   .arg(savedTelemetryPort),
                               savedEthernetHost);
    }
    const QByteArray payload("CLEARCNC_DISCOVER?");
    QSet<QString> subnetBroadcasts;
    const auto interfaces = QNetworkInterface::allInterfaces();
    for (const QNetworkInterface &iface : interfaces) {
        const auto flags = iface.flags();
        if (!flags.testFlag(QNetworkInterface::IsUp) ||
            !flags.testFlag(QNetworkInterface::IsRunning) ||
            flags.testFlag(QNetworkInterface::IsLoopBack)) {
            continue;
        }
        for (const QNetworkAddressEntry &entry : iface.addressEntries()) {
            const QHostAddress ip = entry.ip();
            const QHostAddress broadcast = entry.broadcast();
            if (ip.protocol() != QAbstractSocket::IPv4Protocol ||
                broadcast.isNull() ||
                broadcast == QHostAddress::Null) {
                continue;
            }
            subnetBroadcasts.insert(broadcast.toString());
        }
    }

    if (subnetBroadcasts.isEmpty()) {
        m_udpSocket.writeDatagram(payload, QHostAddress::Broadcast, 10040);
        AppendLog("TX UDP: CLEARCNC_DISCOVER? -> 255.255.255.255:10040 (fallback)");
        return;
    }

    for (const QString &broadcastIp : subnetBroadcasts) {
        m_udpSocket.writeDatagram(payload, QHostAddress(broadcastIp), 10040);
        AppendLog(QString("TX UDP: CLEARCNC_DISCOVER? -> %1:10040").arg(broadcastIp));
    }
}

void MainWindow::ConnectOrDisconnect() {
    if (m_serial.isOpen() || m_tcpSocket.state() == QAbstractSocket::ConnectedState) {
        DisableControllerBeforeDisconnect();
        m_pollTimer.stop();
        if (m_serial.isOpen()) {
            m_serial.close();
        }
        if (m_tcpSocket.state() == QAbstractSocket::ConnectedState) {
            m_tcpSocket.disconnectFromHost();
        }
        if (m_telemetrySocket.state() == QAbstractSocket::ConnectedState) {
            m_telemetrySocket.disconnectFromHost();
        }
        ResetProgramTrackingState();
        AppendLog("Disconnected");
        SetConnectedUi(false);
        return;
    }

    m_usingEthernet = m_transportCombo->currentText() == "Ethernet";
    if (m_usingEthernet) {
        if (m_deviceCombo->currentData().isNull()) {
            QMessageBox::warning(this, "ClearCNC Controller", "No Ethernet device selected. Click Discover first.");
            return;
        }
        const QString host = m_deviceCombo->currentData().toString();
        const int controlPort = AppSettings().value("connection/controlPort", kDefaultControlPort).toInt();
        const int telemetryPort = AppSettings().value("connection/telemetryPort", kDefaultTelemetryPort).toInt();
        m_tcpSocket.connectToHost(host, controlPort);
        if (!m_tcpSocket.waitForConnected(3000)) {
            QMessageBox::critical(this, "ClearCNC Controller",
                                  QString("Failed to connect to %1:%2: %3")
                                      .arg(host)
                                      .arg(controlPort)
                                      .arg(m_tcpSocket.errorString()));
            return;
        }
        QSettings settings = AppSettings();
        settings.setValue("connection/transport", "Ethernet");
        settings.setValue("connection/ethernetHost", host);
        settings.setValue("connection/controlPort", controlPort);
        settings.setValue("connection/telemetryPort", telemetryPort);
        AppendLog(QString("Connected to %1:%2").arg(host).arg(controlPort));
        m_telemetrySocket.connectToHost(host, telemetryPort);
        if (m_telemetrySocket.waitForConnected(1000)) {
            AppendLog(QString("Telemetry connected to %1:%2").arg(host).arg(telemetryPort));
        } else {
            AppendLog(QString("Telemetry unavailable on %1:%2: %3")
                          .arg(host)
                          .arg(telemetryPort)
                          .arg(m_telemetrySocket.errorString()));
            m_telemetrySocket.abort();
        }
        SetConnectedUi(true);
        m_pollTimer.start();
        QTimer::singleShot(250, this, [this]() {
            SendCommand("HELP");
            SendActiveUnitsCommand();
            SendMotionConfigCommand();
            SendStatusPoll();
        });
        return;
    }

    const QString portName = m_portCombo->currentText();
    if (portName.isEmpty()) {
        QMessageBox::warning(this, "ClearCNC Controller", "No serial port selected.");
        return;
    }

    m_serial.setPortName(portName);
    m_serial.setBaudRate(m_baudCombo->currentText().toInt());
    QSettings settings = AppSettings();
    settings.setValue("connection/transport", "Serial");
    settings.setValue("connection/port", portName);
    settings.setValue("connection/baud", m_baudCombo->currentText());
    m_serial.setDataBits(QSerialPort::Data8);
    m_serial.setParity(QSerialPort::NoParity);
    m_serial.setStopBits(QSerialPort::OneStop);
    m_serial.setFlowControl(QSerialPort::NoFlowControl);

    if (!m_serial.open(QIODevice::ReadWrite)) {
        QMessageBox::critical(this, "ClearCNC Controller",
                              QString("Failed to open %1: %2")
                                  .arg(portName, m_serial.errorString()));
        return;
    }
    m_serial.setDataTerminalReady(true);
    m_serial.setRequestToSend(true);

    AppendLog(QString("Connected to %1 @ %2")
                  .arg(portName, m_baudCombo->currentText()));
    SetConnectedUi(true);
    m_pollTimer.start();
    QTimer::singleShot(250, this, [this]() {
        SendCommand("HELP");
        SendActiveUnitsCommand();
        SendMotionConfigCommand();
        SendStatusPoll();
    });
}

void MainWindow::SetConnectedUi(bool connected) {
    m_connectButton->setText(connected ? "Disconnect" : "Connect");
    m_transportCombo->setEnabled(!connected);
    m_serialSettingsWidget->setEnabled(!connected);
    m_ethernetSettingsWidget->setEnabled(!connected);

    for (auto *button : {m_enableButton, m_disableButton, m_stopButton, m_estopButton,
                         m_absButton, m_relButton, m_zeroButton, m_runProgramButton,
                         m_pauseProgramButton, m_stopProgramButton}) {
        button->setEnabled(connected);
    }
    m_loadProgramButton->setEnabled(true);
    m_feedEdit->setEnabled(connected);
    UpdateAxisCapabilityUi();
    if (!connected) {
        m_controllerEnabled = false;
    }
    UpdateConnectionWindowTitle(connected);
    UpdateControllerEnabledStateUi();

    if (connected) {
        statusBar()->showMessage("Connected");
    } else {
        statusBar()->showMessage("Disconnected");
    }
}

void MainWindow::UpdateConnectionWindowTitle(bool connected) {
    setWindowTitle(connected ? QStringLiteral("ClearCNC Controller [Connected]")
                             : QStringLiteral("ClearCNC Controller [Disconnected]"));
}

void MainWindow::UpdateControllerEnabledStateUi() {
    if (!m_enableButton || !m_disableButton) {
        return;
    }
    // Neutral = default; green = enabled; orange = not enabled (current state on Disable only).
    const QString kIdle(
        "QPushButton { background: #3a3f46; color: #dce1e8; border: 1px solid #23272d; border-radius: 2px; font-weight: 600; }"
        "QPushButton:hover { border-color: #586473; background: #424850; }"
        "QPushButton:disabled { background: #2d3238; color: #7b8490; border-color: #252a31; }");
    const QString kGreen(
        "QPushButton { background: #1e4a28; color: #e8f8ec; border: 1px solid #3a9d55; border-radius: 2px; font-weight: 700; }"
        "QPushButton:hover { background: #255a32; border-color: #4fc06e; }"
        "QPushButton:disabled { background: #2a332e; color: #7a9080; border-color: #3a4a3f; }");
    const QString kOrange(
        "QPushButton { background: #5a3a18; color: #fff0e0; border: 1px solid #c97a2c; border-radius: 2px; font-weight: 700; }"
        "QPushButton:hover { background: #6d4720; border-color: #e89640; }"
        "QPushButton:disabled { background: #3a3028; color: #a89888; border-color: #5a4a3a; }");
    m_enableButton->setToolTip(m_controllerEnabled
                                   ? QStringLiteral("Motors are enabled (per controller STATUS/telemetry).")
                                   : QStringLiteral("Enable motor drivers (M202)."));
    m_disableButton->setToolTip(m_controllerEnabled
                                    ? QStringLiteral("Disable motor drivers (M203).")
                                    : QStringLiteral("Motors are not enabled."));

    if (!IsControllerConnected()) {
        m_enableButton->setStyleSheet(kIdle);
        m_disableButton->setStyleSheet(kIdle);
        return;
    }
    if (m_controllerEnabled) {
        m_enableButton->setStyleSheet(kGreen);
        m_disableButton->setStyleSheet(kIdle);
    } else {
        m_enableButton->setStyleSheet(kIdle);
        m_disableButton->setStyleSheet(kOrange);
    }
}

void MainWindow::UpdateTransportUi() {
    const bool ethernet = m_transportCombo->currentText() == "Ethernet";
    m_serialSettingsWidget->setVisible(!ethernet);
    m_ethernetSettingsWidget->setVisible(ethernet);
}

void MainWindow::SetMotionModeUi(bool absoluteMode) {
    m_absButton->setChecked(absoluteMode);
    m_relButton->setChecked(!absoluteMode);
}

void MainWindow::ApplyGuiUnitsUi() {
    const QString distanceUnit = m_guiUnitsInches ? "in" : "mm";
    const QString feedUnit = m_guiUnitsInches ? "in/min" : "mm/min";
    m_moveXEdit->setToolTip(QString("Target X (%1)").arg(distanceUnit));
    m_moveYEdit->setToolTip(QString("Target Y (%1)").arg(distanceUnit));
    m_moveZEdit->setToolTip(QString("Target Z (%1)").arg(distanceUnit));
    m_moveAEdit->setToolTip("Target 4th (deg)");
    m_jogStepEdit->setToolTip(QString("Jog step (%1)").arg(distanceUnit));
    m_feedEdit->setToolTip(QString("Feed rate (%1)").arg(feedUnit));
    m_moveXEdit->setPlaceholderText(QString("0.000 %1").arg(distanceUnit));
    m_moveYEdit->setPlaceholderText(QString("0.000 %1").arg(distanceUnit));
    m_moveZEdit->setPlaceholderText(QString("0.000 %1").arg(distanceUnit));
    m_moveAEdit->setPlaceholderText("0.000 deg");
    m_jogStepEdit->setPlaceholderText(QString("1.000 %1").arg(distanceUnit));
    m_feedEdit->setPlaceholderText(QString("500.000 %1").arg(feedUnit));
    ValidateFeedInput();
    // 3D grid: one line per machine inch (1.0 in G20, 25.4 in G21 so spacing is 1" on the part).
    if (m_positionView3D) {
        m_positionView3D->setGridCellStep(m_guiUnitsInches ? 1.0f : 25.4f);
        m_positionView3D->setDroInches(m_guiUnitsInches);
    }
    RebuildProgramVizKinematics();
    UpdateDroOverlayText();
}

void MainWindow::LoadMotionSettings() {
    QSettings settings = AppSettings();
    m_guiUnitsInches = settings.value("motion/guiUnitsInches", m_guiUnitsInches).toBool();

    for (int i = 0; i < 4; ++i) {
        auto &port = m_motorPorts[i];
        const QString prefix = QString("motorPorts/%1/").arg(i);
        port.enabled = settings.value(prefix + "enabled", i == 0).toBool();
        port.axis = settings.value(prefix + "axis", i == 0 ? 1 : 0).toInt();
        port.axisType = settings.value(prefix + "axisType", 0).toInt();
        port.stepsPerRev = settings.value(prefix + "stepsPerRev", port.stepsPerRev).toDouble();
        port.linearPitchMm = settings.value(prefix + "linearPitchMm", port.linearPitchMm).toDouble();
        port.gearRatio = settings.value(prefix + "gearRatio", port.gearRatio).toDouble();
        port.maxRpm = settings.value(prefix + "maxRpm", port.maxRpm).toDouble();
        port.accel = settings.value(prefix + "accel", port.accel).toDouble();
        port.decel = settings.value(prefix + "decel", port.decel).toDouble();
    }

    // Backward compatibility with previous scalar settings.
    m_stepsPerUnit = settings.value("motion/stepsPerUnit", m_stepsPerUnit).toDouble();
    m_velocity = settings.value("motion/velocityStepsPerSec", m_velocity).toDouble();
    m_accel = settings.value("motion/accelStepsPerSec2", m_accel).toDouble();
    m_decel = settings.value("motion/decelStepsPerSec2", m_decel).toDouble();
    m_singleMotorBench = settings.value("motion/singleMotorBench", false).toBool();
    m_junctionDVmax = settings.value("motion/junctionDVmax", 0.0).toDouble();
    SyncActiveMotionSettingsFromPorts();
}

void MainWindow::SaveMotionSettings() {
    QSettings settings = AppSettings();
    settings.setValue("motion/guiUnitsInches", m_guiUnitsInches);
    settings.setValue("motion/stepsPerUnit", m_stepsPerUnit);
    settings.setValue("motion/velocityStepsPerSec", m_velocity);
    settings.setValue("motion/accelStepsPerSec2", m_accel);
    settings.setValue("motion/decelStepsPerSec2", m_decel);
    settings.setValue("motion/singleMotorBench", m_singleMotorBench);
    settings.setValue("motion/junctionDVmax", m_junctionDVmax);

    for (int i = 0; i < 4; ++i) {
        const auto &port = m_motorPorts[i];
        const QString prefix = QString("motorPorts/%1/").arg(i);
        settings.setValue(prefix + "enabled", port.enabled);
        settings.setValue(prefix + "axis", port.axis);
        settings.setValue(prefix + "axisType", port.axisType);
        settings.setValue(prefix + "stepsPerRev", port.stepsPerRev);
        settings.setValue(prefix + "linearPitchMm", port.linearPitchMm);
        settings.setValue(prefix + "gearRatio", port.gearRatio);
        settings.setValue(prefix + "maxRpm", port.maxRpm);
        settings.setValue(prefix + "accel", port.accel);
        settings.setValue(prefix + "decel", port.decel);
    }
}

void MainWindow::SyncActiveMotionSettingsFromPorts() {
    for (const auto &port : m_motorPorts) {
        if (!port.enabled || port.axis != 1) {
            continue;
        }

        if (port.axisType == 0) {
            m_stepsPerUnit = (port.stepsPerRev * port.gearRatio) / port.linearPitchMm;
        } else {
            m_stepsPerUnit = (port.stepsPerRev * port.gearRatio) / 360.0;
        }
        m_velocity = (port.stepsPerRev * port.gearRatio * port.maxRpm) / 60.0;
        m_accel = port.accel;
        m_decel = port.decel;
        return;
    }
}

double MainWindow::MaxFeedInGuiUnits() const {
    if (m_stepsPerUnit <= 0.0) {
        return 0.0;
    }
    const double maxFeedMmPerMin = (m_velocity * 60.0) / m_stepsPerUnit;
    return m_guiUnitsInches ? (maxFeedMmPerMin / 25.4) : maxFeedMmPerMin;
}

void MainWindow::ValidateFeedInput() {
    const double maxFeed = MaxFeedInGuiUnits();
    const QString feedText = m_feedEdit->text().trimmed();

    bool ok = false;
    const double typedFeed = feedText.toDouble(&ok);
    const bool exceeded = ok && maxFeed > 0.0 && typedFeed > maxFeed;

    const QString feedUnit = m_guiUnitsInches ? "in/min" : "mm/min";
    const QString baseTip = QString("Feed rate (%1)").arg(feedUnit);
    if (exceeded) {
        m_feedEdit->setStyleSheet("QLineEdit { background: #4a3a18; color: #fff4d2; border: 1px solid #d39e00; }"
                                    "QLineEdit:disabled { background: #342b18; color: #bca56c; border-color: #7a6428; }");
    } else {
        m_feedEdit->setStyleSheet("QLineEdit { background: #282d34; color: #e5e9ef; border: 1px solid #20242a; }"
                                  "QLineEdit:disabled { background: #242930; color: #aeb7c4; border-color: #20242a; }");
    }
    m_feedEdit->setToolTip(
        exceeded ? QString("Maximum feed is %1 %2. %3")
                       .arg(maxFeed, 0, 'f', 3)
                       .arg(feedUnit)
                       .arg(baseTip)
                 : baseTip);
}

bool MainWindow::EnsureFeedWithinLimit() {
    const double maxFeed = MaxFeedInGuiUnits();
    const double requestFeed = ParseDoubleLine(m_feedEdit, 500.0);
    if (maxFeed <= 0.0 || requestFeed <= maxFeed) {
        return true;
    }

    QMessageBox::warning(
        this,
        "Maximum Feed Exceeded",
        QString("The requested feed is %1 %2, but the configured motor speed allows a maximum of %3 %2.\n\n"
                "Reduce the feed rate or increase the motor RPM in Setup > Motion Parameters.")
            .arg(requestFeed, 0, 'f', 3)
            .arg(m_guiUnitsInches ? "in/min" : "mm/min")
            .arg(maxFeed, 0, 'f', 3));
    ValidateFeedInput();
    return false;
}

bool MainWindow::IsControllerConnected() const {
    return m_usingEthernet
        ? (m_tcpSocket.state() == QAbstractSocket::ConnectedState)
        : m_serial.isOpen();
}

bool MainWindow::IsTelemetryConnected() const {
    return m_usingEthernet &&
           m_telemetrySocket.state() == QAbstractSocket::ConnectedState;
}

void MainWindow::DisableControllerBeforeDisconnect() {
    if (!IsControllerConnected()) {
        return;
    }

    const QByteArray payload("M203\n");
    if (m_usingEthernet) {
        m_tcpSocket.write(payload);
        m_tcpSocket.waitForBytesWritten(250);
    } else if (m_serial.isOpen()) {
        m_serial.write(payload);
        m_serial.waitForBytesWritten(250);
    }
    AppendLog("TX: M203 (disable before disconnect)");
}

void MainWindow::SendActiveUnitsCommand() {
    SendCommand(m_guiUnitsInches ? "G20" : "G21");
}

void MainWindow::SendMotionConfigCommand() {
    SyncActiveMotionSettingsFromPorts();
    const int single = m_singleMotorBench ? 1 : 0;
    int axMask = 0;
    if (IsLogicalAxisEnabled(1)) {
        axMask |= 1;
    }
    if (IsLogicalAxisEnabled(2)) {
        axMask |= 2;
    }
    if (IsLogicalAxisEnabled(3)) {
        axMask |= 4;
    }
    if (IsLogicalAxisEnabled(4)) {
        axMask |= 8;
    }
    // Firmware converts G-code to pulses using SPMMX / SPMMY separately. Previously only
    // SPMMX was sent while SyncActiveMotionSettingsFromPorts() used the X port alone, so Y
    // always kept the firmware default scale — wrong pulse counts whenever Y ≠ X mechanics.
    auto portStepsPerMm = [](const MotorPortSettings &port) -> double {
        if (port.axisType == 0) {
            return (port.stepsPerRev * port.gearRatio) / port.linearPitchMm;
        }
        return (port.stepsPerRev * port.gearRatio) / 360.0;
    };
    double spmmX = m_stepsPerUnit;
    double spmmY = m_stepsPerUnit;
    for (const auto &port : m_motorPorts) {
        if (!port.enabled) {
            continue;
        }
        if (port.axis == 1) {
            spmmX = portStepsPerMm(port);
        } else if (port.axis == 2) {
            spmmY = portStepsPerMm(port);
        }
    }
    const QString cmd =
        QString("CONFIG SPMMX=%1 SPMMY=%2 VEL=%3 ACCEL=%4 DECEL=%5 DVMAX=%6 SINGLE=%7 AX=%8")
            .arg(spmmX, 0, 'f', 6)
            .arg(spmmY, 0, 'f', 6)
            .arg(m_velocity, 0, 'f', 0)
            .arg(m_accel, 0, 'f', 0)
            .arg(m_decel, 0, 'f', 0)
            .arg(m_junctionDVmax, 0, 'f', 0)
            .arg(single)
            .arg(axMask);
    SendCommand(cmd);
}

void MainWindow::AppendLog(const QString &line) {
    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    m_logEdit->append(QString("[%1] %2").arg(timestamp, line));
}

void MainWindow::SendCommand(const QString &command) {
    const QByteArray payload = command.toUtf8() + "\n";
    if (m_usingEthernet) {
        if (m_tcpSocket.state() != QAbstractSocket::ConnectedState) {
            return;
        }
        m_tcpSocket.write(payload);
    } else {
        if (!m_serial.isOpen()) {
            return;
        }
        m_serial.write(payload);
    }
    AppendLog(QString("TX: %1").arg(command));
    statusBar()->showMessage(QString("Sent: %1").arg(command), 1500);
}

void MainWindow::OnSerialReadyRead() {
    while (m_serial.canReadLine()) {
        const QByteArray raw = m_serial.readLine().trimmed();
        if (raw.isEmpty()) {
            continue;
        }
        HandleReceivedLine(QString::fromUtf8(raw));
    }
}

void MainWindow::OnTcpReadyRead() {
    while (m_tcpSocket.canReadLine()) {
        const QByteArray raw = m_tcpSocket.readLine().trimmed();
        if (raw.isEmpty()) {
            continue;
        }
        HandleReceivedLine(QString::fromUtf8(raw));
    }
}

void MainWindow::OnTcpDisconnected() {
    if (m_usingEthernet) {
        m_pollTimer.stop();
        if (m_telemetrySocket.state() == QAbstractSocket::ConnectedState) {
            m_telemetrySocket.disconnectFromHost();
        }
        ResetProgramTrackingState();
        SetConnectedUi(false);
        AppendLog("TCP disconnected");
    }
}

void MainWindow::OnTelemetryReadyRead() {
    while (m_telemetrySocket.canReadLine()) {
        const QByteArray raw = m_telemetrySocket.readLine().trimmed();
        if (raw.isEmpty()) {
            continue;
        }
        HandleTelemetryLine(QString::fromUtf8(raw));
    }
}

void MainWindow::OnTelemetryDisconnected() {
    if (m_usingEthernet && IsControllerConnected()) {
        AppendLog("Telemetry disconnected; falling back to command-port polling");
        m_pollTimer.start();
        SetConnectedUi(true);
    }
}

void MainWindow::OnUdpReadyRead() {
    while (m_udpSocket.hasPendingDatagrams()) {
        QByteArray datagram;
        datagram.resize((int)m_udpSocket.pendingDatagramSize());
        QHostAddress sender;
        quint16 senderPort = 0;
        m_udpSocket.readDatagram(datagram.data(), datagram.size(), &sender, &senderPort);
        const QString response = QString::fromUtf8(datagram).trimmed();
        if (!response.startsWith("CLEARCNC_DISCOVER", Qt::CaseInsensitive)) {
            continue;
        }
        QString ip = sender.toString();
        const int ipIdx = response.indexOf("IP=", 0, Qt::CaseInsensitive);
        if (ipIdx >= 0) {
            const QString tail = response.mid(ipIdx + 3);
            ip = tail.section(' ', 0, 0);
        }
        if (m_deviceCombo->findData(ip) < 0) {
            m_deviceCombo->addItem(QString("%1 (%2)").arg(ip, response), ip);
        }
        if (ip == AppSettings().value("connection/ethernetHost").toString()) {
            const int idx = m_deviceCombo->findData(ip);
            if (idx >= 0) {
                m_deviceCombo->setCurrentIndex(idx);
            }
        }
        AppendLog(QString("RX UDP %1:%2 %3").arg(sender.toString()).arg(senderPort).arg(response));
    }
}

void MainWindow::HandleReceivedLine(const QString &line) {
    AppendLog(QString("RX: %1").arg(FormatControlRxForUserLog(line)));
    if (line.compare("DONE", Qt::CaseInsensitive) == 0) {
        if (m_programRunning &&
            (m_programIndex >= m_programLines.size() || m_programWaitingForQueueDrain)) {
            m_programIndex = 0;
            m_programRunning = false;
            m_programPaused = false;
            m_programAwaitingAck = false;
            m_programWaitingForQueueDrain = false;
            m_queueDrainStalledPolls = 0;
            m_activeProgramLine = -1;
            m_pendingMotionLineIndices.clear();
            UpdateProgramPreviewHighlight(-1, false);
            SetProgramUiRunning(false);
            m_programStatusLabel->setText("Program complete");
            m_pollTimer.start();
            SendStatusPoll();
        }
        return;
    }
    if (line.startsWith("ACTIVE ", Qt::CaseInsensitive)) {
        double activeLine = 0.0;
        if (ParseTaggedDouble(line, "Line=", activeLine)) {
            const int lineIndex = (int)activeLine - 1;
            if (lineIndex >= 0 && lineIndex < m_programLines.size() &&
                lineIndex != m_activeProgramLine) {
                m_activeProgramLine = lineIndex;
                UpdateProgramPreviewHighlight(m_activeProgramLine, true);
            }
        }
        return;
    }
    HandleProgramResponse(line);

    if (line.startsWith("POS ", Qt::CaseInsensitive) ||
        line.startsWith("X:", Qt::CaseInsensitive)) {
            UpdateDroFromTelemetry(line);
    } else if (line.startsWith("STATUS ", Qt::CaseInsensitive) ||
               line.startsWith("Status:", Qt::CaseInsensitive)) {
            UpdateDroFromTelemetry(line);
            double parsed = 0.0;
            if (ParseTaggedDouble(line, "StepsPerMmX=", parsed)) {
                m_stepsPerUnit = parsed;
            }
            if (ParseTaggedDouble(line, "Vel=", parsed)) {
                m_velocity = parsed;
            }
            if (ParseTaggedDouble(line, "Accel=", parsed)) {
                m_accel = parsed;
            }
            if (ParseTaggedDouble(line, "Decel=", parsed)) {
                m_decel = parsed;
            }
            if (ParseTaggedDouble(line, "Enabled=", parsed)) {
                m_controllerEnabled = parsed > 0.0;
                UpdateControllerEnabledStateUi();
            }
            double queueDepth = 0.0;
            double active = 0.0;
            double activeLine = 0.0;
            const bool queueKnown = ParseTaggedDouble(line, "Queue=", queueDepth);
            const bool activeKnown = ParseTaggedDouble(line, "Active=", active);
            ParseTaggedDouble(line, "ActiveLine=", activeLine);
            if (queueKnown) {
                SetBufferIndicatorFull(queueDepth >= 16.0);
            }
            if (queueKnown && activeKnown &&
                (m_programRunning || m_programWaitingForQueueDrain || !m_pendingMotionLineIndices.isEmpty())) {
                ReconcileProgramQueueFromStatus(queueDepth, active, (int)activeLine);
            }
            if (m_programWaitingForQueueDrain) {
                if (queueKnown && activeKnown && queueDepth <= 0.0 && active <= 0.0) {
                    m_queueDrainStalledPolls = 0;
                    m_programWaitingForQueueDrain = false;
                    if (m_programRunning &&
                        (m_programIndex >= m_programLines.size() ||
                         ProgramLineIsTerminalStop(m_programLines.at(m_programIndex)))) {
                        m_programRunning = false;
                        m_programPaused = false;
                        m_programAwaitingAck = false;
                        m_programIndex = 0;
                        m_activeProgramLine = -1;
                        m_pendingMotionLineIndices.clear();
                        UpdateProgramPreviewHighlight(-1, false);
                        SetProgramUiRunning(false);
                        m_programStatusLabel->setText("Program complete");
                        SendStatusPoll();
                    } else if (m_programRunning && m_programIndex < m_programLines.size()) {
                        m_programAwaitingAck = true;
                        SendCommand(m_programLines.at(m_programIndex));
                    } else {
                        SendNextProgramLine();
                    }
                } else if (queueKnown && activeKnown && queueDepth > 0.0 && active <= 0.0) {
                    m_queueDrainStalledPolls++;
                    if (m_queueDrainStalledPolls >= 10) {
                        m_programRunning = false;
                        m_programPaused = false;
                        m_programAwaitingAck = false;
                        m_programWaitingForQueueDrain = false;
                        m_activeProgramLine = -1;
                        m_pendingMotionLineIndices.clear();
                        UpdateProgramPreviewHighlight(-1, false);
                        SetProgramUiRunning(false);
                        m_programStatusLabel->setText("Queue stalled; sent emergency clear");
                        SendCommand("M200");
                        QMessageBox::warning(
                            this,
                            "Queue Stalled",
                            "The controller reported queued moves but no active motion while the GUI was waiting for the queue to drain.\n\n"
                            "An emergency queue clear (M200) was sent. Inspect the mechanism before running again.");
                    }
                } else {
                    m_queueDrainStalledPolls = 0;
                }
            }
            if (line.contains("MODE=ABS", Qt::CaseInsensitive) || line.contains(" MODE=ABS", Qt::CaseInsensitive)) {
                SetMotionModeUi(true);
            } else if (line.contains("MODE=REL", Qt::CaseInsensitive) || line.contains(" MODE=REL", Qt::CaseInsensitive)) {
                SetMotionModeUi(false);
            }
    }
}

void MainWindow::HandleTelemetryLine(const QString &line) {
    if (!line.startsWith("TEL ", Qt::CaseInsensitive)) {
        return;
    }

    UpdateDroFromTelemetry(line);

    double enabled = 0.0;
    if (ParseTaggedDouble(line, "Enabled=", enabled)) {
        m_controllerEnabled = enabled > 0.0;
        UpdateControllerEnabledStateUi();
    }

    double queueDepth = 0.0;
    double active = 0.0;
    double activeLine = 0.0;
    const bool queueKnown = ParseTaggedDouble(line, "Queue=", queueDepth);
    const bool activeKnown = ParseTaggedDouble(line, "Active=", active);
    ParseTaggedDouble(line, "ActiveLine=", activeLine);
    if (queueKnown) {
        SetBufferIndicatorFull(queueDepth >= 16.0);
    }

    if (queueKnown && activeKnown &&
        (m_programRunning || m_programWaitingForQueueDrain || activeLine > 0.0)) {
        ReconcileProgramQueueFromStatus(queueDepth, active, (int)activeLine);
    }

    if (!m_programWaitingForQueueDrain || !queueKnown || !activeKnown) {
        return;
    }

    if (queueDepth <= 0.0 && active <= 0.0) {
        m_queueDrainStalledPolls = 0;
        m_programWaitingForQueueDrain = false;
        if (m_programRunning &&
            (m_programIndex >= m_programLines.size() ||
             ProgramLineIsTerminalStop(m_programLines.at(m_programIndex)))) {
            m_programRunning = false;
            m_programPaused = false;
            m_programAwaitingAck = false;
            m_programIndex = 0;
            ResetProgramTrackingState();
            SetProgramUiRunning(false);
            m_programStatusLabel->setText("Program complete");
            m_pollTimer.start();
        } else if (m_programRunning && m_programIndex < m_programLines.size()) {
            m_programAwaitingAck = true;
            SendCommand(m_programLines.at(m_programIndex));
        } else {
            SendNextProgramLine();
        }
    } else if (queueDepth > 0.0 && active <= 0.0) {
        m_queueDrainStalledPolls++;
        if (m_queueDrainStalledPolls >= 10) {
            m_programRunning = false;
            m_programPaused = false;
            m_programAwaitingAck = false;
            m_programWaitingForQueueDrain = false;
            ResetProgramTrackingState();
            SetProgramUiRunning(false);
            m_programStatusLabel->setText("Queue stalled; sent emergency clear");
            SendCommand("M200");
            QMessageBox::warning(
                this,
                "Queue Stalled",
                "The controller reported queued moves but no active motion while the GUI was waiting for the queue to drain.\n\n"
                "An emergency queue clear (M200) was sent. Inspect the mechanism before running again.");
        }
    } else {
        m_queueDrainStalledPolls = 0;
    }
}

void MainWindow::OnSerialError(QSerialPort::SerialPortError error) {
    if (error == QSerialPort::NoError || error == QSerialPort::TimeoutError) {
        return;
    }
    AppendLog(QString("Serial error: %1").arg(m_serial.errorString()));
    statusBar()->showMessage("Serial communication error", 2500);
}

void MainWindow::SendStatusPoll() {
    if (IsTelemetryConnected()) {
        return;
    }
    SendCommand("M115");
    SendCommand("M114");
}

void MainWindow::SendEnable() {
    SendCommand("M202");
    QTimer::singleShot(150, this, &MainWindow::SendStatusPoll);
}
void MainWindow::SendDisable() {
    m_controllerEnabled = false;
    SendCommand("M203");
    UpdateControllerEnabledStateUi();
}
void MainWindow::SendStop() { SendCommand("M201"); }
void MainWindow::SendEmergencyStop() {
    m_controllerEnabled = false;
    SendCommand("M200");
    UpdateControllerEnabledStateUi();
}
void MainWindow::SendSetAbs() {
    SetMotionModeUi(true);
    SendCommand("G90");
}
void MainWindow::SendSetRel() {
    SetMotionModeUi(false);
    SendCommand("G91");
}
void MainWindow::SendZero() { SendCommand("G92 X0 Y0"); }

void MainWindow::ShowMotionParametersDialog() {
    QDialog dialog(this);
    dialog.setWindowTitle("Motion Parameters");
    dialog.setModal(true);
    dialog.resize(540, 420);

    auto *layout = new QVBoxLayout(&dialog);

    auto *units = new QComboBox(&dialog);
    units->addItems({"Millimeters", "Inches"});
    units->setCurrentIndex(m_guiUnitsInches ? 1 : 0);
    auto *topForm = new QFormLayout();
    topForm->addRow("GUI units:", units);

    auto *dvmaxSpin = new QDoubleSpinBox(&dialog);
    dvmaxSpin->setRange(0.0, 1000000.0);
    dvmaxSpin->setDecimals(0);
    dvmaxSpin->setSingleStep(100.0);
    dvmaxSpin->setSuffix(" steps/sec");
    dvmaxSpin->setSpecialValueText("0  (disabled — GRBL angle method)");
    dvmaxSpin->setValue(m_junctionDVmax);
    dvmaxSpin->setToolTip(
        "Corner junction speed cap (dVmax) — Centroid-style per-axis velocity-delta limit.\n"
        "At each junction the planner allows at most this many steps/sec of velocity change\n"
        "on the fastest-moving axis. Smaller = more deceleration at corners, smoother feel.\n"
        "Typical starting point: 20-30% of max velocity (e.g. 1000 when VEL=5000).\n"
        "0 = disabled; falls back to GRBL angle-based junction deviation formula.");
    topForm->addRow("Corner DVmax:", dvmaxSpin);

    auto *singleMotor = new QCheckBox(
        "Single-motor / bench (independent M0 and M1 — not coordinated XY in firmware)", &dialog);
    singleMotor->setChecked(m_singleMotorBench);
    singleMotor->setToolTip(
        "Sends CONFIG SINGLE=1: always independent M0/M1 step generators (bench / one-motor). "
        "Unchecked sends SINGLE=0: diagonals use coordinated vector feed; pure X or pure Y "
        "segments still use independent moves (same math as bench for cardinals).");
    topForm->addRow(singleMotor);
    layout->addLayout(topForm);

    struct PortWidgets {
        QCheckBox *enabled = nullptr;
        QComboBox *axis = nullptr;
        QComboBox *axisType = nullptr;
        QDoubleSpinBox *stepsPerRev = nullptr;
        QDoubleSpinBox *pitch = nullptr;
        QDoubleSpinBox *gearRatio = nullptr;
        QDoubleSpinBox *maxRpm = nullptr;
        QDoubleSpinBox *accel = nullptr;
        QDoubleSpinBox *decel = nullptr;
        QLineEdit *computedScale = nullptr;
    };

    std::array<PortWidgets, 4> widgets;
    auto *tabs = new QTabWidget(&dialog);
    layout->addWidget(tabs);

    for (int i = 0; i < 4; ++i) {
        auto *page = new QWidget(tabs);
        auto *form = new QFormLayout(page);
        auto &w = widgets[i];
        const auto &settings = m_motorPorts[i];

        w.enabled = new QCheckBox("Enabled", page);
        w.enabled->setChecked(settings.enabled);

        w.axis = new QComboBox(page);
        w.axis->addItems({"Unmapped", "X", "Y", "Z", "4th"});
        w.axis->setCurrentIndex(settings.axis);

        w.axisType = new QComboBox(page);
        w.axisType->addItems({"Linear axis", "Rotary axis"});
        w.axisType->setCurrentIndex(settings.axisType);

        w.stepsPerRev = new QDoubleSpinBox(page);
        w.stepsPerRev->setRange(1.0, 10000000.0);
        w.stepsPerRev->setDecimals(0);
        w.stepsPerRev->setValue(settings.stepsPerRev);
        w.stepsPerRev->setSuffix(" steps/rev");

        w.pitch = new QDoubleSpinBox(page);
        w.pitch->setRange(0.000001, 1000000.0);
        w.pitch->setDecimals(6);
        w.pitch->setValue(settings.linearPitchMm);
        w.pitch->setSuffix(" mm/rev");

        w.gearRatio = new QDoubleSpinBox(page);
        w.gearRatio->setRange(0.000001, 1000000.0);
        w.gearRatio->setDecimals(6);
        w.gearRatio->setValue(settings.gearRatio);
        w.gearRatio->setSuffix(":1");

        w.maxRpm = new QDoubleSpinBox(page);
        w.maxRpm->setRange(0.001, 100000.0);
        w.maxRpm->setDecimals(3);
        w.maxRpm->setValue(settings.maxRpm);
        w.maxRpm->setSuffix(" rpm");

        w.accel = new QDoubleSpinBox(page);
        w.accel->setRange(1.0, 10000000.0);
        w.accel->setDecimals(0);
        w.accel->setValue(settings.accel);
        w.accel->setSuffix(" steps/s^2");

        w.decel = new QDoubleSpinBox(page);
        w.decel->setRange(1.0, 10000000.0);
        w.decel->setDecimals(0);
        w.decel->setValue(settings.decel);
        w.decel->setSuffix(" steps/s^2");

        w.computedScale = new QLineEdit(page);
        w.computedScale->setReadOnly(true);

        auto updateScale = [w]() {
            if (w.axisType->currentIndex() == 0) {
                const double scale = (w.stepsPerRev->value() * w.gearRatio->value()) / w.pitch->value();
                w.computedScale->setText(QString("%1 steps/mm").arg(scale, 0, 'f', 6));
            } else {
                const double scale = (w.stepsPerRev->value() * w.gearRatio->value()) / 360.0;
                w.computedScale->setText(QString("%1 steps/degree").arg(scale, 0, 'f', 6));
            }
            w.pitch->setEnabled(w.axisType->currentIndex() == 0);
        };
        updateScale();

        connect(w.axisType, &QComboBox::currentIndexChanged, page, updateScale);
        connect(w.stepsPerRev, &QDoubleSpinBox::valueChanged, page, updateScale);
        connect(w.pitch, &QDoubleSpinBox::valueChanged, page, updateScale);
        connect(w.gearRatio, &QDoubleSpinBox::valueChanged, page, updateScale);

        form->addRow(w.enabled);
        form->addRow("Axis mapping:", w.axis);
        form->addRow("Axis type:", w.axisType);
        form->addRow("Motor steps/rev:", w.stepsPerRev);
        form->addRow("Ballscrew pitch:", w.pitch);
        form->addRow("Gear ratio:", w.gearRatio);
        form->addRow("Computed scale:", w.computedScale);
        form->addRow("Max motor speed:", w.maxRpm);
        form->addRow("Acceleration:", w.accel);
        form->addRow("Deceleration:", w.decel);

        tabs->addTab(page, QString("Port %1").arg(i));
    }

    auto *buttons = new QDialogButtonBox(QDialogButtonBox::Ok | QDialogButtonBox::Cancel,
                                         &dialog);
    buttons->button(QDialogButtonBox::Ok)->setText("Apply");
    layout->addWidget(buttons);
    connect(buttons, &QDialogButtonBox::accepted, &dialog, &QDialog::accept);
    connect(buttons, &QDialogButtonBox::rejected, &dialog, &QDialog::reject);

    if (dialog.exec() != QDialog::Accepted) {
        return;
    }

    m_guiUnitsInches = units->currentIndex() == 1;
    m_singleMotorBench = singleMotor->isChecked();
    m_junctionDVmax = dvmaxSpin->value();
    for (int i = 0; i < 4; ++i) {
        auto &port = m_motorPorts[i];
        const auto &w = widgets[i];
        port.enabled = w.enabled->isChecked();
        port.axis = w.axis->currentIndex();
        port.axisType = w.axisType->currentIndex();
        port.stepsPerRev = w.stepsPerRev->value();
        port.linearPitchMm = w.pitch->value();
        port.gearRatio = w.gearRatio->value();
        port.maxRpm = w.maxRpm->value();
        port.accel = w.accel->value();
        port.decel = w.decel->value();
    }
    SyncActiveMotionSettingsFromPorts();
    SaveMotionSettings();

    ApplyGuiUnitsUi();
    ValidateFeedInput();
    SendActiveUnitsCommand();
    SendMotionConfigCommand();
    UpdateAxisCapabilityUi();
}

void MainWindow::SendMove() {
    if (!EnsureFeedWithinLimit()) {
        return;
    }
    const bool ex = IsLogicalAxisEnabled(1);
    const bool ey = IsLogicalAxisEnabled(2);
    const bool ez = IsLogicalAxisEnabled(3);
    const bool ea = IsLogicalAxisEnabled(4);
    if (!ex && !ey && !ez && !ea) {
        return;
    }
    SendActiveUnitsCommand();
    QStringList parts;
    parts.append(QStringLiteral("G01"));
    if (ex) {
        parts.append(QStringLiteral("X%1")
                         .arg(ParseDoubleLine(m_moveXEdit, 0.0), 0, 'f', 3));
    }
    if (ey) {
        parts.append(QStringLiteral("Y%1")
                         .arg(ParseDoubleLine(m_moveYEdit, 0.0), 0, 'f', 3));
    }
    if (ez) {
        parts.append(QStringLiteral("Z%1")
                         .arg(ParseDoubleLine(m_moveZEdit, 0.0), 0, 'f', 3));
    }
    if (ea) {
        parts.append(QStringLiteral("A%1")
                         .arg(ParseDoubleLine(m_moveAEdit, 0.0), 0, 'f', 3));
    }
    parts.append(
        QStringLiteral("F%1").arg(ParseDoubleLine(m_feedEdit, 500.0), 0, 'f', 3));
    SendCommand(parts.join(QLatin1Char(' ')));
}

void MainWindow::SendJog(double dx, double dy) {
    if (!EnsureFeedWithinLimit()) {
        return;
    }
    SendActiveUnitsCommand();
    QStringList parts;
    parts.append(QStringLiteral("G91"));
    parts.append(QStringLiteral("G01"));
    if (dx != 0.0) {
        parts.append(QStringLiteral("X%1").arg(dx, 0, 'f', 3));
    }
    if (dy != 0.0) {
        parts.append(QStringLiteral("Y%1").arg(dy, 0, 'f', 3));
    }
    if (parts.size() <= 2) {
        return;
    }
    parts.append(
        QStringLiteral("F%1").arg(ParseDoubleLine(m_feedEdit, 500.0), 0, 'f', 3));
    SendCommand(parts.join(QLatin1Char(' ')));
}

void MainWindow::SendJogXPos() { SendJog(ParseDoubleLine(m_jogStepEdit, 1.0), 0.0); }
void MainWindow::SendJogXNeg() { SendJog(-ParseDoubleLine(m_jogStepEdit, 1.0), 0.0); }
void MainWindow::SendJogYPos() { SendJog(0.0, ParseDoubleLine(m_jogStepEdit, 1.0)); }
void MainWindow::SendJogYNeg() { SendJog(0.0, -ParseDoubleLine(m_jogStepEdit, 1.0)); }
void MainWindow::SendJogZPos() {
    const double step = ParseDoubleLine(m_jogStepEdit, 1.0);
    SendActiveUnitsCommand();
    SendCommand(QString("G91 G01 Z%1 F%2")
                    .arg(step, 0, 'f', 3)
                    .arg(ParseDoubleLine(m_feedEdit, 500.0), 0, 'f', 3));
}
void MainWindow::SendJogZNeg() {
    const double step = ParseDoubleLine(m_jogStepEdit, 1.0);
    SendActiveUnitsCommand();
    SendCommand(QString("G91 G01 Z%1 F%2")
                    .arg(-step, 0, 'f', 3)
                    .arg(ParseDoubleLine(m_feedEdit, 500.0), 0, 'f', 3));
}
void MainWindow::SendJogAPos() {
    const double step = ParseDoubleLine(m_jogStepEdit, 1.0);
    SendActiveUnitsCommand();
    SendCommand(QString("G91 G01 A%1 F%2")
                    .arg(step, 0, 'f', 3)
                    .arg(ParseDoubleLine(m_feedEdit, 500.0), 0, 'f', 3));
}
void MainWindow::SendJogANeg() {
    const double step = ParseDoubleLine(m_jogStepEdit, 1.0);
    SendActiveUnitsCommand();
    SendCommand(QString("G91 G01 A%1 F%2")
                    .arg(-step, 0, 'f', 3)
                    .arg(ParseDoubleLine(m_feedEdit, 500.0), 0, 'f', 3));
}

QString MainWindow::CleanGCodeLine(const QString &line) const {
    QString cleaned;
    int parenDepth = 0;  // Track nesting so ( center=(6,2) ) doesn't leak text
    for (QChar ch : line) {
        if (ch == ';') {
            break;  // Rest-of-line comment
        }
        if (ch == '(') {
            parenDepth++;
            continue;
        }
        if (ch == ')') {
            if (parenDepth > 0) {
                parenDepth--;
            }
            continue;
        }
        if (parenDepth == 0) {
            cleaned.append(ch);
        }
    }

    int checksum = cleaned.indexOf('*');
    if (checksum >= 0) {
        cleaned = cleaned.left(checksum);
    }
    cleaned = cleaned.trimmed();
    if (cleaned == "%") {
        return QString();
    }

    if (cleaned.startsWith('N', Qt::CaseInsensitive)) {
        int pos = 1;
        while (pos < cleaned.size() && cleaned.at(pos).isDigit()) {
            pos++;
        }
        if (pos > 1 && (pos == cleaned.size() || cleaned.at(pos).isSpace())) {
            cleaned = cleaned.mid(pos).trimmed();
        }
    }
    return cleaned;
}

bool MainWindow::ProgramLineRequiresQueueDrain(const QString &line) const {
    const QString upper = line.trimmed().toUpper();
    return upper == "M0" || upper == "M00" ||
           upper == "M1" || upper == "M01" ||
           upper == "M2" || upper == "M02" ||
           upper == "M30" ||
           upper == "M201" || upper == "M203" ||
           upper.startsWith("G92");
}

bool MainWindow::ProgramLineIsTerminalStop(const QString &line) const {
    const QString upper = line.trimmed().toUpper();
    return upper == "M2" || upper == "M02" ||
           upper == "M30" ||
           upper == "M201";
}

bool MainWindow::ProgramLineQueuesMotion(const QString &line) const {
    const QString upper = line.trimmed().toUpper();
    return upper.startsWith("G0 ") || upper == "G0" ||
           upper.startsWith("G00 ") || upper == "G00" ||
           upper.startsWith("G1 ") || upper == "G1" ||
           upper.startsWith("G01 ") || upper == "G01" ||
           upper.startsWith("G2 ") || upper == "G2" ||
           upper.startsWith("G02 ") || upper == "G02" ||
           upper.startsWith("G3 ") || upper == "G3" ||
           upper.startsWith("G03 ") || upper == "G03";
}

void MainWindow::UpdateProgramPreviewHighlight(int lineIndex, bool autoScroll) {
    QList<QTextEdit::ExtraSelection> selections;
    if (lineIndex >= 0 && lineIndex < m_programLines.size()) {
        QTextCursor cursor(m_programPreview->document()->findBlockByLineNumber(lineIndex));
        if (cursor.block().isValid()) {
            cursor.movePosition(QTextCursor::EndOfBlock, QTextCursor::KeepAnchor);
            QTextEdit::ExtraSelection selection;
            selection.cursor = cursor;
            selection.format.setBackground(QColor("#fff7d6"));
            selection.format.setProperty(QTextFormat::FullWidthSelection, true);
            selections.append(selection);
            if (autoScroll) {
                m_programPreview->setTextCursor(cursor);
                m_programPreview->ensureCursorVisible();
            }
        }
    }
    m_programPreview->setExtraSelections(selections);
}

void MainWindow::ResetProgramTrackingState() {
    m_activeProgramLine = -1;
    m_pendingMotionLineIndices.clear();
    UpdateProgramPreviewHighlight(-1, false);
    SetBufferIndicatorFull(false);
}

void MainWindow::ReconcileProgramQueueFromStatus(double queueDepth, double active, int activeLine) {
    const int outstandingMotion = qMax(0, (int)(queueDepth + active + 0.5));
    while (m_pendingMotionLineIndices.size() > outstandingMotion) {
        m_pendingMotionLineIndices.removeFirst();
    }
    if (outstandingMotion <= 0) {
        m_pendingMotionLineIndices.clear();
    }

    int highlightLine = -1;
    if (active > 0.0 && activeLine > 0) {
        highlightLine = activeLine - 1;
    } else if (active > 0.0 && !m_pendingMotionLineIndices.isEmpty()) {
        // Fall back to queue tracking when ActiveLine telemetry is unavailable.
        highlightLine = m_pendingMotionLineIndices.first();
    }

    if (highlightLine != m_activeProgramLine) {
        m_activeProgramLine = highlightLine;
        UpdateProgramPreviewHighlight(m_activeProgramLine, m_activeProgramLine >= 0);
    }
}

void MainWindow::LoadGCodeProgram() {
    LoadGCodeProgramImpl(false);
}

void MainWindow::OpenGCodeProgramFromMenu() {
    LoadGCodeProgramImpl(true);
}

void MainWindow::LoadGCodeProgramImpl(bool focusProgramTabOnSuccess) {
    const QString path = QFileDialog::getOpenFileName(
        this,
        "Open G-Code Program",
        AppSettings().value("program/lastDirectory").toString(),
        "G-Code Files (*.nc *.tap *.gcode *.ngc *.txt);;All Files (*.*)");
    if (path.isEmpty()) {
        return;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text)) {
        QMessageBox::critical(this, "Open G-Code Program",
                              QString("Could not open %1").arg(path));
        return;
    }

    m_programLines.clear();
    QTextStream in(&file);
    while (!in.atEnd()) {
        const QString cleaned = CleanGCodeLine(in.readLine());
        if (!cleaned.isEmpty()) {
            m_programLines.append(cleaned);
        }
    }

    m_programPath = path;
    m_programIndex = 0;
    m_programRunning = false;
    m_programPaused = false;
    m_programAwaitingAck = false;
    m_programWaitingForQueueDrain = false;
    m_queueDrainStalledPolls = 0;
    m_programQueueFullRetries = 0;
    ResetProgramTrackingState();
    AppSettings().setValue("program/lastDirectory", QFileInfo(path).absolutePath());

    m_programPreview->setPlainText(m_programLines.join("\n"));
    m_programStatusLabel->setText(QString("%1 lines loaded").arg(m_programLines.size()));
    AppendLog(QString("Loaded program: %1 (%2 lines)").arg(path).arg(m_programLines.size()));
    RebuildProgramVizKinematics();
    if (focusProgramTabOnSuccess && m_workTabs && m_programWorkTab) {
        m_workTabs->setCurrentWidget(m_programWorkTab);
    }
}

void MainWindow::StartGCodeProgram() {
    if (!IsControllerConnected()) {
        QMessageBox::warning(this, "Run G-Code Program", "Connect to the controller first.");
        return;
    }
    if (m_programLines.isEmpty()) {
        QMessageBox::warning(this, "Run G-Code Program", "Load a G-code file first.");
        return;
    }
    if (!m_controllerEnabled) {
        QMessageBox::warning(
            this,
            "Run G-Code Program",
            "The controller motors are disabled. Press Enable before running the program.");
        SendStatusPoll();
        return;
    }

    m_programIndex = 0;
    m_programRunning = true;
    m_programPaused = false;
    m_programAwaitingAck = false;
    m_programWaitingForQueueDrain = false;
    m_queueDrainStalledPolls = 0;
    m_programQueueFullRetries = 0;
    ResetProgramTrackingState();
    SetProgramUiRunning(true);
    if (IsTelemetryConnected()) {
        m_pollTimer.stop();
    } else {
        m_pollTimer.start();
    }
    SendNextProgramLine();
    UpdateToolVizForViewport();
}

void MainWindow::PauseGCodeProgram() {
    if (!m_programRunning) {
        return;
    }
    m_programPaused = !m_programPaused;
    m_pauseProgramButton->setText(m_programPaused ? "Resume" : "Pause");
    SendCommand(m_programPaused ? "HOLD" : "RESUME");
    if (m_programPaused) {
        UpdateProgramPreviewHighlight(m_activeProgramLine, false);
    }
    m_programStatusLabel->setText(m_programPaused
        ? QString("Paused at line %1 of %2").arg(m_programIndex).arg(m_programLines.size())
        : QString("Running line %1 of %2").arg(m_programIndex + 1).arg(m_programLines.size()));
    if (!m_programPaused && !m_programAwaitingAck) {
        SendNextProgramLine();
    }
}

void MainWindow::StopGCodeProgram() {
    m_programRunning = false;
    m_programPaused = false;
    m_programAwaitingAck = false;
    m_programWaitingForQueueDrain = false;
    m_queueDrainStalledPolls = 0;
    m_programQueueFullRetries = 0;
    ResetProgramTrackingState();
    SetProgramUiRunning(false);
    m_programStatusLabel->setText("Program stopped");
    m_pollTimer.start();
    SendCommand("M201");
    if (m_usingEthernet) {
        m_tcpSocket.readAll();
    } else if (m_serial.isOpen()) {
        m_serial.readAll();
    }
    SendStatusPoll();
    UpdateToolVizForViewport();
}

void MainWindow::SetProgramUiRunning(bool running) {
    m_loadProgramButton->setEnabled(!running);
    m_runProgramButton->setEnabled(!running && IsControllerConnected());
    m_pauseProgramButton->setEnabled(running);
    m_stopProgramButton->setEnabled(running);
    m_pauseProgramButton->setText("Pause");
}

void MainWindow::SendNextProgramLine() {
    if (!m_programRunning || m_programPaused || m_programAwaitingAck ||
        m_programWaitingForQueueDrain) {
        return;
    }
    if (m_programIndex >= m_programLines.size()) {
        m_programAwaitingAck = false;
        m_programWaitingForQueueDrain = true;
        m_queueDrainStalledPolls = 0;
        m_programStatusLabel->setText("Waiting for queued motion to finish");
        m_pollTimer.start();
        SendStatusPoll();
        return;
    }

    const QString line = m_programLines.at(m_programIndex);
    if (ProgramLineRequiresQueueDrain(line)) {
        m_programWaitingForQueueDrain = true;
        m_queueDrainStalledPolls = 0;
        m_programStatusLabel->setText(QString("Waiting for queue to drain before line %1")
                                          .arg(m_programIndex + 1));
        if (!IsTelemetryConnected()) {
            SendCommand("M115");
        }
        return;
    }

    m_programStatusLabel->setText(QString("Running line %1 of %2")
                                      .arg(m_programIndex + 1)
                                      .arg(m_programLines.size()));
    m_programAwaitingAck = true;
    SendCommand(line);
}

void MainWindow::HandleProgramResponse(const QString &line) {
    if (!m_programAwaitingAck) {
        return;
    }

    if (line.compare("ok", Qt::CaseInsensitive) == 0) {
        m_programQueueFullRetries = 0;
        m_programAwaitingAck = false;
        if (m_programIndex >= 0 && m_programIndex < m_programLines.size() &&
            ProgramLineQueuesMotion(m_programLines.at(m_programIndex))) {
            m_pendingMotionLineIndices.append(m_programIndex);
        }
        m_programIndex++;
        UpdateToolVizForViewport();
        SendNextProgramLine();
        return;
    }

    if (line.contains("queue full", Qt::CaseInsensitive)) {
        m_programAwaitingAck = false;
        m_programQueueFullRetries++;
        SetBufferIndicatorFull(true);
        m_programStatusLabel->setText(QString("Buffer full, line %1")
                                          .arg(m_programIndex + 1));
        QTimer::singleShot(100, this, [this]() {
            if (m_programRunning && !m_programPaused) {
                SendNextProgramLine();
            }
        });
        return;
    }

    if (line.startsWith("error", Qt::CaseInsensitive)) {
        const int failedProgramLine1Based = m_programIndex + 1;
        m_programRunning = false;
        m_programPaused = false;
        m_programAwaitingAck = false;
        m_programIndex = 0;
        ResetProgramTrackingState();
        SetProgramUiRunning(false);
        m_pollTimer.start();
        UpdateToolVizForViewport();
        QMessageBox::warning(this, "G-Code Program Error",
                             QString("Controller reported an error at line %1:\n%2")
                                 .arg(failedProgramLine1Based)
                                 .arg(line));
    }
}

void MainWindow::UpdateDroFromTelemetry(const QString &line) {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    const bool hasX = ParseTaggedDouble(line, "X=", x);
    const bool hasY = ParseTaggedDouble(line, "Y=", y);
    const bool hasZ = ParseTaggedDouble(line, "Z=", z);
    const bool hasA = ParseTaggedDouble(line, "A=", a);
    if (hasX) {
        m_vizX = x;
    }
    if (hasY) {
        m_vizY = y;
    }
    if (hasZ) {
        m_vizZ = z;
    }
    if (hasA) {
        m_vizA = a;
    }
    if (hasX || hasY || hasZ || hasA) {
        UpdateDroOverlayText();
    }
    UpdateToolVizForViewport();
}

bool MainWindow::IsLogicalAxisEnabled(int axis) const {
    if (axis < 1 || axis > 4) {
        return false;
    }
    for (const auto &port : m_motorPorts) {
        if (port.enabled && port.axis == axis) {
            return true;
        }
    }
    return false;
}

void MainWindow::UpdateAxisCapabilityUi() {
    const bool conn = IsControllerConnected();
    const bool x = IsLogicalAxisEnabled(1);
    const bool y = IsLogicalAxisEnabled(2);
    const bool z = IsLogicalAxisEnabled(3);
    const bool a = IsLogicalAxisEnabled(4);
    const bool anyMotion = x || y || z || a;

    const auto setTarget = [](QLabel *lbl, QLineEdit *edit, bool axisOn) {
        if (lbl) {
            lbl->setEnabled(axisOn);
        }
        if (edit) {
            edit->setEnabled(axisOn);
        }
    };
    setTarget(m_targetXLabel, m_moveXEdit, x);
    setTarget(m_targetYLabel, m_moveYEdit, y);
    setTarget(m_targetZLabel, m_moveZEdit, z);
    setTarget(m_targetALabel, m_moveAEdit, a);

    if (m_moveButton) {
        m_moveButton->setEnabled(conn && anyMotion);
    }
    if (m_jogStepEdit) {
        m_jogStepEdit->setEnabled(conn && anyMotion);
    }
    const auto setJog = [conn](QPushButton *b, bool axisOn) {
        if (b) {
            b->setEnabled(conn && axisOn);
        }
    };
    setJog(m_jogXPosButton, x);
    setJog(m_jogXNegButton, x);
    setJog(m_jogYPosButton, y);
    setJog(m_jogYNegButton, y);
    setJog(m_jogZPosButton, z);
    setJog(m_jogZNegButton, z);
    setJog(m_jogAPosButton, a);
    setJog(m_jogANegButton, a);
}

void MainWindow::UpdateDroOverlayText() {
    if (!m_positionView3D) {
        return;
    }
    const QString u = m_guiUnitsInches ? QStringLiteral(" in") : QStringLiteral(" mm");
    QStringList lines;
    if (IsLogicalAxisEnabled(1)) {
        lines << QStringLiteral("X  %1%2").arg(m_vizX, 0, 'f', 3).arg(u);
    }
    if (IsLogicalAxisEnabled(2)) {
        lines << QStringLiteral("Y  %1%2").arg(m_vizY, 0, 'f', 3).arg(u);
    }
    if (IsLogicalAxisEnabled(3)) {
        lines << QStringLiteral("Z  %1%2").arg(m_vizZ, 0, 'f', 3).arg(u);
    }
    if (IsLogicalAxisEnabled(4)) {
        lines << QStringLiteral("4th  %1 deg").arg(m_vizA, 0, 'f', 3);
    }
    m_positionView3D->setDroOverlayText(lines.isEmpty()
        ? QStringLiteral("No enabled axes in Motion settings")
        : lines.join(QLatin1Char('\n')));
}

void MainWindow::RebuildProgramVizKinematics() {
    if (!m_positionView3D) {
        return;
    }
    m_positionView3D->setProgramPath(QVector<QVector3D>());
    if (m_viewportPathTraceCheck) {
        m_viewportPathTraceCheck->setEnabled(!m_programLines.isEmpty());
    }
    if (m_programLines.isEmpty()) {
        UpdateToolVizForViewport();
        return;
    }
    ProgramKinematicsResult r;
    BuildProgramKinematics(m_programLines, m_guiUnitsInches, &r);
    m_positionView3D->setProgramPath(r.pathInGuiUnits);
    UpdateToolVizForViewport();
}

void MainWindow::UpdateToolVizForViewport() {
    if (!m_positionView3D) {
        return;
    }
    m_positionView3D->setToolPosition(
        static_cast<float>(m_vizX), static_cast<float>(m_vizY), static_cast<float>(m_vizZ));
}

void MainWindow::SetBufferIndicatorFull(bool full) {
    if (!m_bufferStatusLabel) {
        return;
    }

    m_bufferStatusLabel->setText(QString());
    m_bufferStatusLabel->setToolTip(full ? "Controller motion buffer full"
                                         : "Controller motion buffer has room");
    m_bufferStatusLabel->setStyleSheet(full
        ? "QLabel { background: #d39e00; border: 1px solid #f0c15d; border-radius: 2px; }"
        : "QLabel { background: #37b26c; border: 1px solid #75d99a; border-radius: 2px; }");
}

void MainWindow::SetLogPanelVisible(bool visible) {
    if (!m_mainVSplitter || !m_logGroup || !m_toggleLogAction) {
        return;
    }

    {
        QSignalBlocker blocker(m_toggleLogAction);
        m_toggleLogAction->setChecked(visible);
    }

    if (!visible) {
        if (m_logGroup->isVisible()) {
            const QList<int> sizes = m_mainVSplitter->sizes();
            if (sizes.size() >= 3 && sizes[2] > 48) {
                m_savedLogPaneHeight = sizes[2];
            }
        }
        m_logGroup->setMinimumHeight(0);
        m_logGroup->hide();
    } else {
        m_logGroup->setMinimumHeight(220);
        m_logGroup->show();
        QTimer::singleShot(0, this, [this]() {
            if (!m_mainVSplitter || !m_logGroup || !m_logGroup->isVisible()) {
                return;
            }
            const int total = qMax(400, m_mainVSplitter->height());
            const int logH = qBound(120, m_savedLogPaneHeight, total - 200);
            const QList<int> cur = m_mainVSplitter->sizes();
            int ctrlH = 84;
            if (cur.size() >= 3 && cur[0] > 0) {
                ctrlH = qBound(64, cur[0], 200);
            }
            const int workH = qMax(200, total - ctrlH - logH);
            m_mainVSplitter->setSizes({ctrlH, workH, logH});
        });
    }

    if (m_showLogButton) {
        m_showLogButton->setVisible(!visible);
    }
}

void MainWindow::LoadWindowUiSettings() {
    m_savedLogPaneHeight = qBound(120, AppSettings().value("ui/logPaneHeight", 320).toInt(), 2000);
    const bool show = AppSettings().value("ui/logPanelVisible", true).toBool();
    SetLogPanelVisible(show);
}

void MainWindow::SaveWindowUiSettings() {
    if (m_mainVSplitter && m_logGroup && m_logGroup->isVisible()) {
        const QList<int> s = m_mainVSplitter->sizes();
        if (s.size() >= 3 && s[2] > 48) {
            m_savedLogPaneHeight = s[2];
        }
    }
    if (m_logGroup) {
        AppSettings().setValue("ui/logPanelVisible", m_logGroup->isVisible());
    }
    AppSettings().setValue("ui/logPaneHeight", m_savedLogPaneHeight);
}

