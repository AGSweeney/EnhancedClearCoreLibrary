#include "GrblProtocol.h"

#include <QByteArray>
#include <QRegularExpression>
#include <QStringView>

namespace {
QString UpperTrimmed(const QString &line) {
    return line.trimmed().toUpper();
}
} // namespace

void GrblProtocol::ObserveLine(const QString &line) {
    const QString upper = UpperTrimmed(line);
    if (upper.startsWith("GRBLHAL")) {
        m_variant = Variant::GrblHal;
    } else if (upper.startsWith("FLUIDNC")) {
        m_variant = Variant::FluidNc;
    } else if (upper.startsWith("GRBL ")) {
        m_variant = Variant::Grbl11;
    }
}

GrblProtocol::Variant GrblProtocol::CurrentVariant() const {
    return m_variant;
}

QString GrblProtocol::VariantName() const {
    switch (m_variant) {
        case Variant::Grbl11:
            return "GRBL 1.1";
        case Variant::GrblHal:
            return "grblHAL";
        case Variant::FluidNc:
            return "FluidNC";
        default:
            return "Unknown";
    }
}

QString GrblProtocol::VariantSummary() const {
    if (m_variant == Variant::Unknown) {
        return "Unknown (probing for GRBL-family)";
    }
    return VariantName();
}

QString GrblProtocol::EnableCommand() const {
    return "$X";
}

QString GrblProtocol::DisableCommand() const {
    return "M5";
}

QString GrblProtocol::StopCommand() const {
    return "!";
}

QString GrblProtocol::PauseCommand() const {
    return "!";
}

QString GrblProtocol::ResumeCommand() const {
    return "~";
}

QString GrblProtocol::SetAbsCommand() const {
    return "G90";
}

QString GrblProtocol::SetRelCommand() const {
    return "G91";
}

QString GrblProtocol::ZeroWorkCommand() const {
    return "G92 X0 Y0 Z0";
}

QStringList GrblProtocol::StatusPollCommands(bool telemetryConnected) const {
    if (telemetryConnected) {
        return {};
    }
    return {"?"};
}

QByteArray GrblProtocol::EmergencyRealtimeBytes() const {
    return QByteArray(1, char(0x18));
}

bool GrblProtocol::IsTerminalStopLine(const QString &line) const {
    const QString upper = UpperTrimmed(line);
    return upper == "M2" || upper == "M30";
}

bool GrblProtocol::IsMotionLine(const QString &line) const {
    const QString upper = UpperTrimmed(line);
    return upper.startsWith("G0 ") || upper == "G0" ||
           upper.startsWith("G00 ") || upper == "G00" ||
           upper.startsWith("G1 ") || upper == "G1" ||
           upper.startsWith("G01 ") || upper == "G01" ||
           upper.startsWith("G2 ") || upper == "G2" ||
           upper.startsWith("G02 ") || upper == "G02" ||
           upper.startsWith("G3 ") || upper == "G3" ||
           upper.startsWith("G03 ") || upper == "G03";
}

bool GrblProtocol::IsQueueDrainRequiredLine(const QString &line) const {
    const QString upper = UpperTrimmed(line);
    if (IsTerminalStopLine(upper)) {
        return true;
    }
    return upper.startsWith("G4");
}

bool GrblProtocol::ParseGrblSettingLine(const QString &line, int &settingId, double &value) {
    static const QRegularExpression rx("^\\$(\\d+)\\s*=\\s*([-+]?\\d*\\.?\\d+)$");
    const QRegularExpressionMatch match = rx.match(line.trimmed());
    if (!match.hasMatch()) {
        return false;
    }
    bool idOk = false;
    bool valueOk = false;
    const int id = match.captured(1).toInt(&idOk);
    const double parsedValue = match.captured(2).toDouble(&valueOk);
    if (!idOk || !valueOk) {
        return false;
    }
    settingId = id;
    value = parsedValue;
    return true;
}

bool GrblProtocol::ParseBracketPayload(const QString &line, QString &payloadType, QString &payload) {
    const QString trimmed = line.trimmed();
    if (!trimmed.startsWith('[') || !trimmed.endsWith(']')) {
        return false;
    }
    const QString content = trimmed.mid(1, trimmed.size() - 2);
    const int sep = content.indexOf(':');
    if (sep < 0) {
        return false;
    }
    payloadType = content.left(sep).trimmed().toUpper();
    payload = content.mid(sep + 1).trimmed();
    return true;
}

GrblProtocol::StatusSnapshot GrblProtocol::ParseStatusLine(const QString &line) {
    StatusSnapshot snapshot;
    const QString trimmed = line.trimmed();
    if (!trimmed.startsWith('<') || !trimmed.endsWith('>')) {
        return snapshot;
    }
    const QString content = trimmed.mid(1, trimmed.size() - 2);
    const QStringList parts = content.split('|', Qt::SkipEmptyParts);
    if (parts.isEmpty()) {
        return snapshot;
    }

    snapshot.valid = true;
    snapshot.state = parts.first().trimmed();
    for (int i = 1; i < parts.size(); ++i) {
        const QString part = parts.at(i).trimmed();
        if (part.startsWith("MPos:", Qt::CaseInsensitive)) {
            const QStringList xyz = part.mid(5).split(',', Qt::SkipEmptyParts);
            if (xyz.size() >= 3) {
                bool okX = false;
                bool okY = false;
                bool okZ = false;
                const double x = xyz.at(0).toDouble(&okX);
                const double y = xyz.at(1).toDouble(&okY);
                const double z = xyz.at(2).toDouble(&okZ);
                if (okX && okY && okZ) {
                    snapshot.hasMPos = true;
                    snapshot.x = x;
                    snapshot.y = y;
                    snapshot.z = z;
                }
            }
        } else if (part.startsWith("WPos:", Qt::CaseInsensitive)) {
            const QStringList xyz = part.mid(5).split(',', Qt::SkipEmptyParts);
            if (xyz.size() >= 3) {
                bool okX = false;
                bool okY = false;
                bool okZ = false;
                const double x = xyz.at(0).toDouble(&okX);
                const double y = xyz.at(1).toDouble(&okY);
                const double z = xyz.at(2).toDouble(&okZ);
                if (okX && okY && okZ) {
                    snapshot.hasWPos = true;
                    snapshot.wx = x;
                    snapshot.wy = y;
                    snapshot.wz = z;
                }
            }
        } else if (part.startsWith("FS:", Qt::CaseInsensitive)) {
            const QStringList fs = part.mid(3).split(',', Qt::SkipEmptyParts);
            if (!fs.isEmpty()) {
                bool ok = false;
                snapshot.feed = fs.first().toDouble(&ok);
                snapshot.hasFeed = ok;
            }
            if (fs.size() > 1) {
                bool ok = false;
                snapshot.spindle = fs.at(1).toDouble(&ok);
                snapshot.hasSpindle = ok;
            }
        }
    }
    return snapshot;
}
