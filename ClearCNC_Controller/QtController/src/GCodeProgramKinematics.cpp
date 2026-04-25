#include "GCodeProgramKinematics.h"

#include <QRegularExpression>
#include <QtMath>
#include <cmath>

namespace {
constexpr double kMmPerIn = 25.4;

QVector3D toGui3(double xMm, double yMm, double zMm, bool guiInches) {
    if (guiInches) {
        return QVector3D(float(xMm / kMmPerIn), float(yMm / kMmPerIn), float(zMm / kMmPerIn));
    }
    return QVector3D(float(xMm), float(yMm), float(zMm));
}

double toMmValue(double v, bool fileInches) {
    return fileInches ? (v * kMmPerIn) : v;
}

struct GCodeWord {
    QChar letter;
    double value = 0.0;
};

QVector<GCodeWord> parseWords(const QString &line) {
    QVector<GCodeWord> out;
    static const QRegularExpression re(
        QStringLiteral("([A-Za-z])\\s*([+-]?(?:\\d+\\.?\\d*|\\d*\\.?\\d+))"));
    QRegularExpressionMatchIterator it = re.globalMatch(line);
    while (it.hasNext()) {
        const QRegularExpressionMatch m = it.next();
        if (m.captured(1).size() != 1) {
            continue;
        }
        bool ok = false;
        const double val = m.captured(2).toDouble(&ok);
        if (!ok) {
            continue;
        }
        out.push_back({m.captured(1).at(0), val});
    }
    return out;
}

// Map G## from token value to motion 0-3, or 20, 21, 90, 91, or -1.
int decodeGCode(double g) {
    if (g < 0) {
        return -1;
    }
    // Whole numbers and common 2-digit
    int whole = int(qRound(g));
    if (qAbs(g - double(whole)) < 1.0e-4) {
        if (whole >= 0 && whole <= 3) {
            return whole;
        }
        if (whole == 20 || whole == 21 || whole == 90 || whole == 91) {
            return whole;
        }
    }
    // e.g. 1.0 for G1
    if (g >= 0.0 && g <= 3.0 + 1.0e-3) {
        return int(qRound(g));
    }
    return -1;
}

struct Kine {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    bool fileInches = false; // G20
    bool abs = true; // G90
    int modalMotionG = 1; // last G0..3 seen (default G1 for bare axis lines)

    void appendArc(
        const QVector3D &startGui,
        const QVector3D &endGui,
        bool guiInches,
        double iMm, double jMm,
        bool ccw,
        QVector<QVector3D> *path) {
        const double sx = double(startGui.x());
        const double sy = double(startGui.y());
        const double sz = double(startGui.z());
        const double ex = double(endGui.x());
        const double ey = double(endGui.y());
        const double ez = double(endGui.z());
        // work in mm for trig
        const double sXm = guiInches ? sx * kMmPerIn : sx;
        const double sYm = guiInches ? sy * kMmPerIn : sy;
        const double sZm = guiInches ? sz * kMmPerIn : sz;
        const double eXm = guiInches ? ex * kMmPerIn : ex;
        const double eYm = guiInches ? ey * kMmPerIn : ey;
        const double eZm = guiInches ? ez * kMmPerIn : ez;

        const double cx = sXm + iMm;
        const double cy = sYm + jMm;
        const double r0 = std::hypot(sXm - cx, sYm - cy);
        if (r0 < 1.0e-6) {
            path->append(endGui);
            return;
        }
        double th0 = std::atan2(sYm - cy, sXm - cx);
        double th1 = std::atan2(eYm - cy, eXm - cx);
        double dth = th1 - th0;
        if (ccw) {
            if (dth < 0.0) {
                dth += 2.0 * M_PI;
            }
        } else {
            if (dth > 0.0) {
                dth -= 2.0 * M_PI;
            }
        }
        const int n = qBound(2, int(std::ceil(std::abs(dth) / (M_PI / 8.0))), 48);
        for (int k = 1; k <= n; ++k) {
            const double t = double(k) / double(n);
            const double th = th0 + t * dth;
            const double xMm = cx + r0 * std::cos(th);
            const double yMm = cy + r0 * std::sin(th);
            const double zMm = sZm + t * (eZm - sZm);
            path->append(toGui3(xMm, yMm, zMm, guiInches));
        }
    }
};

void applyLine(
    const QString &line, Kine *k, bool guiInches, ProgramKinematicsResult *out) {
    const QVector<GCodeWord> words = parseWords(line);
    if (words.isEmpty()) {
        return;
    }

    // Collect G codes and other words
    int motionG = -1; // 0,1,2,3 from this line
    double valX = 0.0;
    double valY = 0.0;
    double valZ = 0.0;
    double valI = 0.0;
    double valJ = 0.0;
    double valR = 0.0;
    bool haveX = false;
    bool haveY = false;
    bool haveZ = false;
    bool haveI = false;
    bool haveJ = false;
    bool haveR = false;

    for (const GCodeWord &w : words) {
        const QChar c = w.letter.toUpper();
        if (c == u'G') {
            const int d = decodeGCode(w.value);
            if (d >= 0 && d <= 3) {
                motionG = d;
            } else if (d == 20) {
                k->fileInches = true;
            } else if (d == 21) {
                k->fileInches = false;
            } else if (d == 90) {
                k->abs = true;
            } else if (d == 91) {
                k->abs = false;
            }
        } else if (c == u'X') {
            valX = w.value;
            haveX = true;
        } else if (c == u'Y') {
            valY = w.value;
            haveY = true;
        } else if (c == u'Z') {
            valZ = w.value;
            haveZ = true;
        } else if (c == u'I') {
            valI = w.value;
            haveI = true;
        } else if (c == u'J') {
            valJ = w.value;
            haveJ = true;
        } else if (c == u'R') {
            valR = w.value;
            haveR = true;
        }
    }

    // Modal motion: explicit G0..3 on line wins, else if axes present use previous modal
    if (motionG < 0) {
        if (haveX || haveY || haveZ) {
            motionG = k->modalMotionG;
        }
    } else {
        if (motionG <= 3) {
            k->modalMotionG = motionG;
        }
    }

    if (motionG < 0 || motionG > 3) {
        return;
    }

    const double fIn = k->fileInches;

    // Target in mm
    double tx = k->x;
    double ty = k->y;
    double tz = k->z;
    if (k->abs) {
        if (haveX) {
            tx = toMmValue(valX, fIn);
        }
        if (haveY) {
            ty = toMmValue(valY, fIn);
        }
        if (haveZ) {
            tz = toMmValue(valZ, fIn);
        }
    } else {
        if (haveX) {
            tx += toMmValue(valX, fIn);
        }
        if (haveY) {
            ty += toMmValue(valY, fIn);
        }
        if (haveZ) {
            tz += toMmValue(valZ, fIn);
        }
    }

    const QVector3D endGui = toGui3(tx, ty, tz, guiInches);

    if (motionG == 0 || motionG == 1) {
        if (out->pathInGuiUnits.isEmpty()) {
            out->pathInGuiUnits.append(toGui3(0.0, 0.0, 0.0, guiInches));
        }
        const QVector3D startGui = toGui3(k->x, k->y, k->z, guiInches);
        if (QVector3D::dotProduct(endGui - startGui, endGui - startGui) > 1.0e-20f) {
            out->pathInGuiUnits.append(endGui);
        }
        k->x = tx;
        k->y = ty;
        k->z = tz;
        return;
    }

    // Arc: G2 CW, G3 CCW
    if (out->pathInGuiUnits.isEmpty()) {
        out->pathInGuiUnits.append(toGui3(0.0, 0.0, 0.0, guiInches));
    }
    const QVector3D startGui = toGui3(k->x, k->y, k->z, guiInches);
    const bool hasIJ = haveI && haveJ;
    if (hasIJ) {
        const double iMm = toMmValue(valI, fIn);
        const double jMm = toMmValue(valJ, fIn);
        k->appendArc(startGui, endGui, guiInches, iMm, jMm, (motionG == 3), &out->pathInGuiUnits);
    } else {
        // R format or no center: connect with a straight line (chord) for a usable preview
        (void)valR;
        (void)haveR;
        if (QVector3D::dotProduct(endGui - startGui, endGui - startGui) > 1.0e-20f) {
            out->pathInGuiUnits.append(endGui);
        }
    }
    k->x = tx;
    k->y = ty;
    k->z = tz;
}
} // namespace

bool BuildProgramKinematics(const QStringList &lines, bool guiInches, ProgramKinematicsResult *out) {
    if (!out) {
        return false;
    }
    out->pathInGuiUnits.clear();
    if (lines.isEmpty()) {
        return false;
    }
    Kine k;
    for (int i = 0; i < lines.size(); ++i) {
        const QString u = lines.at(i).trimmed();
        if (u.isEmpty()) {
            continue;
        }
        applyLine(u, &k, guiInches, out);
    }
    return true;
}
