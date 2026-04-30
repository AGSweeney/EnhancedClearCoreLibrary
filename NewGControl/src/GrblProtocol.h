#pragma once

#include <QByteArray>
#include <QString>
#include <QStringList>

class GrblProtocol {
public:
    enum class Variant {
        Unknown,
        Grbl11,
        GrblHal,
        FluidNc
    };

    struct StatusSnapshot {
        bool valid = false;
        QString state;
        bool hasMPos = false;
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        bool hasWPos = false;
        double wx = 0.0;
        double wy = 0.0;
        double wz = 0.0;
        bool hasFeed = false;
        double feed = 0.0;
        bool hasSpindle = false;
        double spindle = 0.0;
    };

    void ObserveLine(const QString &line);

    Variant CurrentVariant() const;
    QString VariantName() const;
    QString VariantSummary() const;

    QString EnableCommand() const;
    QString DisableCommand() const;
    QString StopCommand() const;
    QString PauseCommand() const;
    QString ResumeCommand() const;
    QString SetAbsCommand() const;
    QString SetRelCommand() const;
    QString ZeroWorkCommand() const;
    QStringList StatusPollCommands(bool telemetryConnected) const;

    QByteArray EmergencyRealtimeBytes() const;

    bool IsTerminalStopLine(const QString &line) const;
    bool IsMotionLine(const QString &line) const;
    bool IsQueueDrainRequiredLine(const QString &line) const;

    static bool ParseGrblSettingLine(const QString &line, int &settingId, double &value);
    static bool ParseBracketPayload(const QString &line, QString &payloadType, QString &payload);
    static StatusSnapshot ParseStatusLine(const QString &line);

private:
    Variant m_variant = Variant::Unknown;
};
