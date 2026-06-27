#include "DxfImport.h"

#include <QFile>
#include <QFileInfo>
#include <QTextStream>
#include <QtMath>
#include <cmath>

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kMmPerIn = 25.4;
constexpr double kHpglUnitsPerMm = 40.0; // common pen-plotter scaling

bool readPair(const QStringList &lines, int &idx, int &code, QString &value) {
    while (idx < lines.size()) {
        const QString codeLine = lines.at(idx++).trimmed();
        if (codeLine.isEmpty()) {
            continue;
        }
        bool ok = false;
        code = codeLine.toInt(&ok);
        if (!ok) {
            continue;
        }
        if (idx >= lines.size()) {
            return false;
        }
        value = lines.at(idx++).trimmed();
        return true;
    }
    return false;
}

void ungetPair(int &idx) {
    idx -= 2;
    if (idx < 0) {
        idx = 0;
    }
}

double insUnitsToMmScale(int insunits) {
    // $INSUNITS: 0=unitless, 1=in, 2=ft, 4=mm, 5=cm, 6=m
    switch (insunits) {
    case 0:
        return kMmPerIn; // unitless → treat drawing units as inches
    case 1:
        return kMmPerIn;
    case 2:
        return kMmPerIn * 12.0;
    case 4:
        return 1.0;
    case 5:
        return 10.0;
    case 6:
        return 1000.0;
    default:
        return 1.0; // unknown → mm
    }
}

int readInsunitsFromHeader(const QStringList &lines) {
    int idx = 0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 9 && val == QLatin1String("$INSUNITS")) {
            int ucode = 0;
            QString uval;
            if (readPair(lines, idx, ucode, uval) && ucode == 70) {
                return uval.toInt();
            }
        }
    }
    // No $INSUNITS in HEADER: many exporters omit it. Treat as unitless → inches (see insUnitsToMmScale case 0).
    return 0;
}

QStringList readAsciiLines(QFile &file) {
    QByteArray raw = file.readAll();
    if (raw.startsWith("AutoCAD Binary DXF")) {
        return {};
    }
    QString text = QString::fromUtf8(raw);
    const QChar cr('\r');
    if (text.contains(cr)) {
        text.replace(QStringLiteral("\r\n"), QStringLiteral("\n"));
        text.replace(cr, QChar());
    }
    return text.split(QLatin1Char('\n'));
}

void appendStripFromPoints(const QVector<QVector3D> &pts, QVector<QVector<QVector3D>> *polylines,
                           QVector<QVector3D> *rapids, QVector3D *lastEndMm) {
    if (pts.size() < 2) {
        return;
    }
    const QVector3D &p0 = pts.first();
    // Only draw rapids between real geometry strips (not from machine origin).
    if (!polylines->isEmpty() && (p0 - *lastEndMm).length() > 1.0e-3f) {
        rapids->append(*lastEndMm);
        rapids->append(p0);
    }
    polylines->append(pts);
    *lastEndMm = pts.last();
}

struct PolyVertex {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    /// Bulge on this vertex applies to the segment from this vertex to the next (DXF group 42).
    double bulge = 0.0;
};

// Tessellates one segment P0→P1; returns points strictly after P0 through P1 (inclusive on P1).
QVector<QVector3D> tessellateBulgeSegmentMm(double x0, double y0, double z0, double x1, double y1, double z1,
                                            double bulge) {
    QVector<QVector3D> out;
    const double dx = x1 - x0;
    const double dy = y1 - y0;
    const double L = std::hypot(dx, dy);
    if (L < 1.0e-9) {
        out.append(QVector3D(float(x1), float(y1), float(z1)));
        return out;
    }
    if (std::abs(bulge) < 1.0e-10) {
        out.append(QVector3D(float(x1), float(y1), float(z1)));
        return out;
    }

    const double theta = 4.0 * std::atan(bulge); // signed included angle (CCW when bulge > 0)
    const double absTheta = std::abs(theta);
    if (absTheta < 1.0e-10) {
        out.append(QVector3D(float(x1), float(y1), float(z1)));
        return out;
    }

    const double sinHalf = std::sin(absTheta * 0.5);
    if (std::abs(sinHalf) < 1.0e-10) {
        out.append(QVector3D(float(x1), float(y1), float(z1)));
        return out;
    }

    const double r = L / (2.0 * sinHalf);
    const double halfL = L * 0.5;
    const double rSq = r * r;
    const double hSq = rSq - halfL * halfL;
    if (hSq < 0.0) {
        out.append(QVector3D(float(x1), float(y1), float(z1)));
        return out;
    }
    const double h = std::sqrt(hSq);

    const double mx = (x0 + x1) * 0.5;
    const double my = (y0 + y1) * 0.5;
    const double ux = dx / L;
    const double uy = dy / L;
    // Left of chord direction P0→P1 in OCS (Y up): (-uy, ux)
    const double vx = -uy;
    const double vy = ux;
    const double sgn = (bulge > 0.0) ? 1.0 : -1.0;
    const double cx = mx + vx * h * sgn;
    const double cy = my + vy * h * sgn;

    const double a0 = std::atan2(y0 - cy, x0 - cx);
    const double sweep = theta; // signed central angle matching bulge

    const int n = qBound(2, int(std::ceil(std::abs(sweep) / (kPi / 16.0))), 96);
    for (int k = 1; k <= n; ++k) {
        const double t = double(k) / double(n);
        const double ang = a0 + sweep * t;
        const float px = float(cx + r * std::cos(ang));
        const float py = float(cy + r * std::sin(ang));
        const float pz = float(z0 + (z1 - z0) * t);
        out.append(QVector3D(px, py, pz));
    }
    if (!out.isEmpty()) {
        out[out.size() - 1] = QVector3D(float(x1), float(y1), float(z1));
    }
    return out;
}

void expandPolylineWithBulges(const QVector<PolyVertex> &verts, bool closed, QVector<QVector3D> *stripOut) {
    stripOut->clear();
    const int n = verts.size();
    if (n < 2) {
        return;
    }
    stripOut->reserve(n * 4);
    stripOut->append(QVector3D(float(verts[0].x), float(verts[0].y), float(verts[0].z)));

    auto appendSeg = [&](const PolyVertex &a, const PolyVertex &b) {
        const QVector<QVector3D> seg = tessellateBulgeSegmentMm(a.x, a.y, a.z, b.x, b.y, b.z, a.bulge);
        for (const QVector3D &p : seg) {
            if (!stripOut->isEmpty() && (p - stripOut->last()).length() < 1.0e-4f) {
                continue;
            }
            stripOut->append(p);
        }
    };

    for (int i = 0; i < n - 1; ++i) {
        appendSeg(verts[i], verts[i + 1]);
    }
    if (closed) {
        appendSeg(verts[n - 1], verts[0]);
    }
}

void tessellateArcMm(double cx, double cy, double cz, double r, double a0Deg, double a1Deg, QVector<QVector3D> *out) {
    if (r <= 1.0e-9) {
        return;
    }
    const double th0 = qDegreesToRadians(a0Deg);
    const double th1 = qDegreesToRadians(a1Deg);
    double dth = th1 - th0;
    // DXF ARC is CCW from start to end
    while (dth < 0.0) {
        dth += 2.0 * kPi;
    }
    while (dth > 2.0 * kPi) {
        dth -= 2.0 * kPi;
    }
    if (dth < 1.0e-9) {
        dth = 2.0 * kPi; // full circle edge case
    }
    const int n = qBound(4, int(std::ceil(dth / (kPi / 16.0))), 96);
    for (int i = 0; i <= n; ++i) {
        const double t = double(i) / double(n);
        const double th = th0 + t * dth;
        out->append(QVector3D(float(cx + r * std::cos(th)), float(cy + r * std::sin(th)), float(cz)));
    }
}

void tessellateCircleMm(double cx, double cy, double cz, double r, QVector<QVector3D> *out) {
    if (r <= 1.0e-9) {
        return;
    }
    constexpr int n = 64;
    for (int i = 0; i <= n; ++i) {
        const double th = (2.0 * kPi * double(i)) / double(n);
        out->append(QVector3D(float(cx + r * std::cos(th)), float(cy + r * std::sin(th)), float(cz)));
    }
}

int hpglCoord(double mm) {
    return int(qRound(mm * kHpglUnitsPerMm));
}

QString buildHpgl(const QVector<QVector<QVector3D>> &polylinesMm) {
    QStringList lines;
    lines << QStringLiteral("IN;SP1;");
    for (const QVector<QVector3D> &strip : polylinesMm) {
        if (strip.size() < 2) {
            continue;
        }
        const QVector3D &p0 = strip.first();
        lines << QStringLiteral("PU%1,%2;")
                   .arg(hpglCoord(double(p0.x())))
                   .arg(hpglCoord(double(p0.y())));
        QStringList pdNums;
        for (int i = 1; i < strip.size(); ++i) {
            const QVector3D &p = strip.at(i);
            pdNums << QStringLiteral("%1,%2").arg(hpglCoord(double(p.x()))).arg(hpglCoord(double(p.y())));
        }
        if (!pdNums.isEmpty()) {
            lines << QStringLiteral("PD%1;").arg(pdNums.join(QLatin1Char(',')));
        }
    }
    return lines.join(QLatin1Char('\n'));
}

void parseLineEntity(const QStringList &lines, int &idx, double s, QVector<QVector<QVector3D>> *polylines,
                     QVector<QVector3D> *rapids, QVector3D *lastEnd) {
    double x0 = 0, y0 = 0, z0 = 0, x1 = 0, y1 = 0, z1 = 0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
        switch (code) {
        case 10:
            x0 = val.toDouble() * s;
            break;
        case 20:
            y0 = val.toDouble() * s;
            break;
        case 30:
            z0 = val.toDouble() * s;
            break;
        case 11:
            x1 = val.toDouble() * s;
            break;
        case 21:
            y1 = val.toDouble() * s;
            break;
        case 31:
            z1 = val.toDouble() * s;
            break;
        default:
            break;
        }
    }
    QVector<QVector3D> pts;
    pts.append(QVector3D(float(x0), float(y0), float(z0)));
    pts.append(QVector3D(float(x1), float(y1), float(z1)));
    appendStripFromPoints(pts, polylines, rapids, lastEnd);
}

void parseLwPolylineEntity(const QStringList &lines, int &idx, double s, QVector<QVector<QVector3D>> *polylines,
                           QVector<QVector3D> *rapids, QVector3D *lastEnd) {
    int flags = 0;
    QVector<PolyVertex> verts;
    bool hasPendingX = false;
    double pendingX = 0.0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
        switch (code) {
        case 70:
            flags = val.toInt();
            break;
        case 90:
            break;
        case 10:
            pendingX = val.toDouble() * s;
            hasPendingX = true;
            break;
        case 20:
            if (hasPendingX) {
                PolyVertex v;
                v.x = pendingX;
                v.y = val.toDouble() * s;
                v.z = 0.0;
                v.bulge = 0.0;
                verts.append(v);
                hasPendingX = false;
            }
            break;
        case 42:
            if (!verts.isEmpty()) {
                verts.last().bulge = val.toDouble();
            }
            break;
        default:
            break;
        }
    }
    if (verts.size() < 2) {
        return;
    }
    const bool closed = (flags & 1) != 0;
    QVector<QVector3D> strip;
    expandPolylineWithBulges(verts, closed, &strip);
    if (strip.size() >= 2) {
        appendStripFromPoints(strip, polylines, rapids, lastEnd);
    }
}

void parsePolyVertexBody(const QStringList &lines, int &idx, double s, PolyVertex *out) {
    out->x = 0.0;
    out->y = 0.0;
    out->z = 0.0;
    out->bulge = 0.0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
        switch (code) {
        case 10:
            out->x = val.toDouble() * s;
            break;
        case 20:
            out->y = val.toDouble() * s;
            break;
        case 30:
            out->z = val.toDouble() * s;
            break;
        case 42:
            out->bulge = val.toDouble();
            break;
        default:
            break;
        }
    }
}

void parsePolylineEntity(const QStringList &lines, int &idx, double s, QVector<QVector<QVector3D>> *polylines,
                         QVector<QVector3D> *rapids, QVector3D *lastEnd) {
    int flags = 0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
        if (code == 70) {
            flags = val.toInt();
        }
    }
    QVector<PolyVertex> verts;
    while (idx < lines.size()) {
        int c0 = 0;
        QString v0;
        if (!readPair(lines, idx, c0, v0) || c0 != 0) {
            break;
        }
        if (v0 == QLatin1String("SEQEND")) {
            while (readPair(lines, idx, code, val)) {
                if (code == 0) {
                    ungetPair(idx);
                    break;
                }
            }
            break;
        }
        if (v0 == QLatin1String("VERTEX")) {
            PolyVertex pv;
            parsePolyVertexBody(lines, idx, s, &pv);
            verts.append(pv);
            continue;
        }
        ungetPair(idx);
        break;
    }
    if (verts.size() < 2) {
        return;
    }
    const bool closed = (flags & 1) != 0;
    QVector<QVector3D> strip;
    expandPolylineWithBulges(verts, closed, &strip);
    if (strip.size() >= 2) {
        appendStripFromPoints(strip, polylines, rapids, lastEnd);
    }
}

void parseArcEntity(const QStringList &lines, int &idx, double s, QVector<QVector<QVector3D>> *polylines,
                    QVector<QVector3D> *rapids, QVector3D *lastEnd) {
    double cx = 0, cy = 0, cz = 0, r = 0, a0 = 0, a1 = 0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
        switch (code) {
        case 10:
            cx = val.toDouble() * s;
            break;
        case 20:
            cy = val.toDouble() * s;
            break;
        case 30:
            cz = val.toDouble() * s;
            break;
        case 40:
            r = val.toDouble() * s;
            break;
        case 50:
            a0 = val.toDouble();
            break;
        case 51:
            a1 = val.toDouble();
            break;
        default:
            break;
        }
    }
    QVector<QVector3D> pts;
    tessellateArcMm(cx, cy, cz, r, a0, a1, &pts);
    if (pts.size() >= 2) {
        appendStripFromPoints(pts, polylines, rapids, lastEnd);
    }
}

void parseCircleEntity(const QStringList &lines, int &idx, double s, QVector<QVector<QVector3D>> *polylines,
                       QVector<QVector3D> *rapids, QVector3D *lastEnd) {
    double cx = 0, cy = 0, cz = 0, r = 0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
        switch (code) {
        case 10:
            cx = val.toDouble() * s;
            break;
        case 20:
            cy = val.toDouble() * s;
            break;
        case 30:
            cz = val.toDouble() * s;
            break;
        case 40:
            r = val.toDouble() * s;
            break;
        default:
            break;
        }
    }
    QVector<QVector3D> pts;
    tessellateCircleMm(cx, cy, cz, r, &pts);
    if (pts.size() >= 2) {
        appendStripFromPoints(pts, polylines, rapids, lastEnd);
    }
}

void skipEntity(const QStringList &lines, int &idx) {
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0) {
            ungetPair(idx);
            break;
        }
    }
}

void parseEntitiesSection(const QStringList &lines, int &idx, double s, DxfImportOutput *out) {
    QVector3D lastEnd(0, 0, 0);
    bool any = false;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0 && val == QLatin1String("ENDSEC")) {
            break;
        }
        if (code != 0) {
            continue;
        }
        if (val == QLatin1String("LINE")) {
            parseLineEntity(lines, idx, s, &out->polylinesMm, &out->rapidSegmentsMm, &lastEnd);
            any = true;
        } else if (val == QLatin1String("LWPOLYLINE")) {
            parseLwPolylineEntity(lines, idx, s, &out->polylinesMm, &out->rapidSegmentsMm, &lastEnd);
            any = true;
        } else if (val == QLatin1String("POLYLINE")) {
            parsePolylineEntity(lines, idx, s, &out->polylinesMm, &out->rapidSegmentsMm, &lastEnd);
            any = true;
        } else if (val == QLatin1String("ARC")) {
            parseArcEntity(lines, idx, s, &out->polylinesMm, &out->rapidSegmentsMm, &lastEnd);
            any = true;
        } else if (val == QLatin1String("CIRCLE")) {
            parseCircleEntity(lines, idx, s, &out->polylinesMm, &out->rapidSegmentsMm, &lastEnd);
            any = true;
        } else if (val == QLatin1String("VERTEX") || val == QLatin1String("SEQEND")) {
            skipEntity(lines, idx);
        } else {
            skipEntity(lines, idx);
        }
    }
    if (!any) {
        out->errorMessage = QStringLiteral("No supported entities (LINE, LWPOLYLINE, POLYLINE, ARC, CIRCLE) found.");
    }
}

bool findEntitiesSection(const QStringList &lines, int *startIdx) {
    int idx = 0;
    int code = 0;
    QString val;
    while (readPair(lines, idx, code, val)) {
        if (code == 0 && val == QLatin1String("SECTION")) {
            int c2 = 0;
            QString v2;
            if (!readPair(lines, idx, c2, v2)) {
                break;
            }
            if (c2 == 2 && v2 == QLatin1String("ENTITIES")) {
                *startIdx = idx;
                return true;
            }
        }
    }
    return false;
}

} // namespace

QString BuildHpglFromPolylinesMm(const QVector<QVector<QVector3D>> &polylinesMm) {
    constexpr double kHpglUnitsPerMm = 40.0;
    auto hpglCoord = [](double mm) -> int { return int(qRound(mm * kHpglUnitsPerMm)); };
    QStringList lines;
    lines << QStringLiteral("IN;SP1;");
    for (const QVector<QVector3D> &strip : polylinesMm) {
        if (strip.size() < 2) {
            continue;
        }
        const QVector3D &p0 = strip.first();
        lines << QStringLiteral("PU%1,%2;")
                   .arg(hpglCoord(double(p0.x())))
                   .arg(hpglCoord(double(p0.y())));
        QStringList pdNums;
        for (int i = 1; i < strip.size(); ++i) {
            const QVector3D &p = strip.at(i);
            pdNums << QStringLiteral("%1,%2").arg(hpglCoord(double(p.x()))).arg(hpglCoord(double(p.y())));
        }
        if (!pdNums.isEmpty()) {
            lines << QStringLiteral("PD%1;").arg(pdNums.join(QLatin1Char(',')));
        }
    }
    return lines.join(QLatin1Char('\n'));
}

bool ImportDxfFile(const QString &filePath, DxfImportOutput *out) {
    if (!out) {
        return false;
    }
    *out = DxfImportOutput();

    QFile f(filePath);
    if (!f.open(QIODevice::ReadOnly)) {
        out->errorMessage = QStringLiteral("Could not open file.");
        return false;
    }
    const QStringList lines = readAsciiLines(f);
    f.close();
    if (lines.isEmpty()) {
        if (QFileInfo(filePath).size() > 0) {
            out->errorMessage = QStringLiteral("Binary DXF is not supported. Save as ASCII DXF from your CAD software.");
        } else {
            out->errorMessage = QStringLiteral("File is empty.");
        }
        return false;
    }

    const int ins = readInsunitsFromHeader(lines);
    const double s = insUnitsToMmScale(ins);

    int entIdx = 0;
    if (!findEntitiesSection(lines, &entIdx)) {
        out->errorMessage = QStringLiteral("ENTITIES section not found (not a valid DXF).");
        return false;
    }

    parseEntitiesSection(lines, entIdx, s, out);
    if (!out->errorMessage.isEmpty()) {
        return false;
    }
    if (out->polylinesMm.isEmpty()) {
        out->errorMessage = QStringLiteral("No geometry was produced from the DXF.");
        return false;
    }

    out->insunits = ins;
    out->drawingToMmScale = s;
    out->hpgl = buildHpgl(out->polylinesMm);
    out->errorMessage.clear();
    return true;
}

bool ExportAsciiDxfMinimal(const QString &filePath,
                           const QVector<QVector<QVector3D>> &polylinesMm,
                           int insunits,
                           double drawingToMmScale,
                           QString *errorMessage) {
    if (!errorMessage) {
        return false;
    }
    if (drawingToMmScale <= 1.0e-12) {
        *errorMessage = QStringLiteral("Invalid drawing scale (drawing units per mm).");
        return false;
    }
    QFile f(filePath);
    if (!f.open(QIODevice::WriteOnly | QIODevice::Truncate | QIODevice::Text)) {
        *errorMessage = QStringLiteral("Could not open file for writing.");
        return false;
    }
    QTextStream ts(&f);

    auto mmToDrawing = [&](double mm) -> double { return mm / drawingToMmScale; };

    quint64 handle = 0x30;
    auto nextHandle = [&]() -> QString { return QString::number(handle++, 16).toUpper(); };

    ts << "0\nSECTION\n2\nHEADER\n";
    ts << "9\n$ACADVER\n1\nAC1018\n";
    ts << "9\n$INSUNITS\n70\n" << insunits << "\n";
    ts << "0\nENDSEC\n";

    ts << "0\nSECTION\n2\nTABLES\n";
    ts << "0\nTABLE\n2\nLAYER\n5\n" << nextHandle() << "\n100\nAcDbSymbolTable\n70\n1\n";
    ts << "0\nLAYER\n5\n" << nextHandle() << "\n100\nAcDbLayerTableRecord\n2\n0\n70\n0\n62\n7\n6\nCONTINUOUS\n";
    ts << "0\nENDTAB\n0\nENDSEC\n";

    ts << "0\nSECTION\n2\nENTITIES\n";

    constexpr double kCloseEpsMm = 0.08;
    for (const QVector<QVector3D> &stripMm : polylinesMm) {
        if (stripMm.size() < 2) {
            continue;
        }
        int nVert = stripMm.size();
        int closedFlag = 0;
        if (nVert >= 3) {
            const QVector3D df = stripMm.first() - stripMm.last();
            const double closeMm = std::hypot(double(df.x()), double(df.y()));
            if (closeMm <= kCloseEpsMm) {
                closedFlag = 1;
                if (nVert > 2) {
                    --nVert; // drop duplicate closing vertex
                }
            }
        }

        ts << "0\nLWPOLYLINE\n5\n" << nextHandle() << "\n100\nAcDbEntity\n8\n0\n100\nAcDbPolyline\n";
        ts << "90\n" << nVert << "\n70\n" << closedFlag << "\n";

        for (int i = 0; i < nVert; ++i) {
            const QVector3D &p = stripMm.at(i);
            const double xd = mmToDrawing(double(p.x()));
            const double yd = mmToDrawing(double(p.y()));
            const double zd = mmToDrawing(double(p.z()));
            ts << "10\n" << xd << "\n20\n" << yd << "\n";
            ts << "30\n" << zd << "\n";
        }
    }

    ts << "0\nENDSEC\n0\nEOF\n";
    f.close();
    errorMessage->clear();
    return true;
}
