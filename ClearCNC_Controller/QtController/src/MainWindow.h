#pragma once

#include <QMainWindow>
#include <QHostAddress>
#include <QSerialPort>
#include <QStringList>
#include <QTcpSocket>
#include <QTimer>
#include <QUdpSocket>
#include <QList>

#include <array>

class QCheckBox;
class QComboBox;
class QCloseEvent;
class QShowEvent;
class QLabel;
class QGroupBox;
class QLineEdit;
class QAction;
class QPushButton;
class QTextEdit;
class QWidget;
class QSplitter;
class QTabWidget;
class PositionView3D;

class MainWindow : public QMainWindow {
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow() override;

protected:
    void showEvent(QShowEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

private slots:
    void RefreshPorts();
    void DiscoverEthernetDevices();
    void ConnectOrDisconnect();
    void OnSerialReadyRead();
    void OnTcpReadyRead();
    void OnTcpDisconnected();
    void OnTelemetryReadyRead();
    void OnTelemetryDisconnected();
    void OnUdpReadyRead();
    void OnSerialError(QSerialPort::SerialPortError error);
    void SendStatusPoll();
    void SendEnable();
    void SendDisable();
    void SendStop();
    void SendEmergencyStop();
    void SendSetAbs();
    void SendSetRel();
    void SendZero();
    void ValidateFeedInput();
    void ShowMotionParametersDialog();
    void SendMove();
    void SendJogXPos();
    void SendJogXNeg();
    void SendJogYPos();
    void SendJogYNeg();
    void SendJogZPos();
    void SendJogZNeg();
    void SendJogAPos();
    void SendJogANeg();
    void LoadGCodeProgram();
    void OpenGCodeProgramFromMenu();
    void StartGCodeProgram();
    void PauseGCodeProgram();
    void StopGCodeProgram();
    void UpdateTransportUi();

private:
    void BuildUi();
    void SetConnectedUi(bool connected);
    void SetMotionModeUi(bool absoluteMode);
    void ApplyGuiUnitsUi();
    void LoadMotionSettings();
    void SaveMotionSettings();
    void SyncActiveMotionSettingsFromPorts();
    double MaxFeedInGuiUnits() const;
    bool EnsureFeedWithinLimit();
    bool IsControllerConnected() const;
    bool IsTelemetryConnected() const;
    void DisableControllerBeforeDisconnect();
    void SendActiveUnitsCommand();
    void SendMotionConfigCommand();
    void AppendLog(const QString &line);
    void SendCommand(const QString &command);
    void SendJog(double dx, double dy);
    QString CleanGCodeLine(const QString &line) const;
    bool ProgramLineRequiresQueueDrain(const QString &line) const;
    bool ProgramLineIsTerminalStop(const QString &line) const;
    bool ProgramLineQueuesMotion(const QString &line) const;
    void SendNextProgramLine();
    void HandleProgramResponse(const QString &line);
    void SetProgramUiRunning(bool running);
    void HandleReceivedLine(const QString &line);
    void HandleTelemetryLine(const QString &line);
    void UpdateProgramPreviewHighlight(int lineIndex, bool autoScroll);
    void ResetProgramTrackingState();
    void ReconcileProgramQueueFromStatus(double queueDepth, double active, int activeLine);
    void UpdateDroFromTelemetry(const QString &line);
    void UpdateDroOverlayText();
    bool IsLogicalAxisEnabled(int axis) const; // 1=X .. 4=4th; uses motor port Enabled + axis mapping
    void UpdateAxisCapabilityUi();
    void UpdateConnectionWindowTitle(bool connected);
    void UpdateControllerEnabledStateUi();
    void RebuildProgramVizKinematics();
    void UpdateToolVizForViewport();
    void SetBufferIndicatorFull(bool full);
    void SetLogPanelVisible(bool visible);
    void LoadWindowUiSettings();
    void SaveWindowUiSettings();
    void LoadGCodeProgramImpl(bool focusProgramTabOnSuccess);

    QSerialPort m_serial;
    QTcpSocket m_tcpSocket;
    QTcpSocket m_telemetrySocket;
    QUdpSocket m_udpSocket;
    QTimer m_pollTimer;

    QComboBox *m_transportCombo;
    QComboBox *m_portCombo;
    QComboBox *m_deviceCombo;
    QComboBox *m_baudCombo;
    QPushButton *m_refreshPortsButton;
    QPushButton *m_discoverButton;
    QPushButton *m_connectButton;
    QWidget *m_serialSettingsWidget;
    QWidget *m_ethernetSettingsWidget;
    QPushButton *m_enableButton;
    QPushButton *m_disableButton;
    QPushButton *m_stopButton;
    QPushButton *m_estopButton;
    QPushButton *m_absButton;
    QPushButton *m_relButton;
    QPushButton *m_zeroButton;

    QLabel *m_targetXLabel;
    QLabel *m_targetYLabel;
    QLabel *m_targetZLabel;
    QLabel *m_targetALabel;
    QLineEdit *m_moveXEdit;
    QLineEdit *m_moveYEdit;
    QLineEdit *m_moveZEdit;
    QLineEdit *m_moveAEdit;
    QLineEdit *m_feedEdit;
    QPushButton *m_moveButton;

    QLineEdit *m_jogStepEdit;
    QPushButton *m_jogXPosButton;
    QPushButton *m_jogXNegButton;
    QPushButton *m_jogYPosButton;
    QPushButton *m_jogYNegButton;
    QPushButton *m_jogZPosButton;
    QPushButton *m_jogZNegButton;
    QPushButton *m_jogAPosButton;
    QPushButton *m_jogANegButton;

    QTextEdit *m_programPreview;
    QLabel *m_programStatusLabel;
    QLabel *m_bufferStatusLabel;
    QPushButton *m_loadProgramButton;
    QPushButton *m_runProgramButton;
    QPushButton *m_pauseProgramButton;
    QPushButton *m_stopProgramButton;

    QPushButton *m_clearLogButton;
    QPushButton *m_minimizeLogButton;
    QTextEdit *m_logEdit;
    QGroupBox *m_logGroup;
    QAction *m_toggleLogAction;
    QPushButton *m_showLogButton;

    QSplitter *m_workAreaSplitter;
    QSplitter *m_mainVSplitter;
    QTabWidget *m_workTabs;
    QWidget *m_programWorkTab;
    PositionView3D *m_positionView3D;
    QCheckBox *m_viewportPathTraceCheck;
    double m_vizX;
    double m_vizY;
    double m_vizZ;
    double m_vizA;

    struct MotorPortSettings {
        bool enabled = false;
        int axis = 0; // 0=None, 1=X, 2=Y, 3=Z, 4=4th
        int axisType = 0; // 0=Linear, 1=Rotary
        double stepsPerRev = 800.0;
        double linearPitchMm = 5.0;
        double gearRatio = 1.0;
        double maxRpm = 300.0;
        double accel = 85000.0;
        double decel = 85000.0;
    };

    std::array<MotorPortSettings, 4> m_motorPorts;
    double m_stepsPerUnit;
    double m_velocity;
    double m_accel;
    double m_decel;
    bool m_guiUnitsInches;
    /// When true, CONFIG SINGLE=1 (independent M0/M1; no CoordinatedMotionController in firmware).
    bool m_singleMotorBench;

    QStringList m_programLines;
    QString m_programPath;
    int m_programIndex;
    bool m_programRunning;
    bool m_programPaused;
    bool m_programAwaitingAck;
    bool m_programWaitingForQueueDrain;
    int m_queueDrainStalledPolls;
    int m_programQueueFullRetries;
    int m_activeProgramLine;
    QList<int> m_pendingMotionLineIndices;
    bool m_controllerEnabled;

    bool m_usingEthernet;
    bool m_didWorkAreaInitialSplit; // one-time 40/60 work-area sizes on first show
    int m_savedLogPaneHeight;
};

