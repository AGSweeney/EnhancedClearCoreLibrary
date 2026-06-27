#include "PathVizIssues.h"

#include <QVector2D>
#include <QtMath>
#include <cmath>
#include <algorithm>

namespace {

constexpr double kMmPerIn = 25.4;
constexpr double kClosedTolMm = 0.08;     // first/last not coincident (tessellation slack)
constexpr double kVertexEpsMm = 0.05;     // shared endpoint / degenerate guards
constexpr double kMarkerDedupeMm = 0.12;  // merge nearby hits (combine tooltips)
constexpr int kMaxOverlapSegPairs = 5000000;

double guiXYToMm(double v, bool guiInches) {
    return guiInches ? (v * kMmPerIn) : v;
}

QVector2D guiToMmXY(const QVector3D &p, bool guiInches) {
    return QVector2D(float(guiXYToMm(double(p.x()), guiInches)), float(guiXYToMm(double(p.y()), guiInches)));
}

QVector3D mmXYToGui(float xMm, float yMm, float zGui, bool guiInches) {
    if (guiInches) {
        return QVector3D(xMm / float(kMmPerIn), yMm / float(kMmPerIn), zGui);
    }
    return QVector3D(xMm, yMm, zGui);
}

double cross2(const QVector2D &a, const QVector2D &b) {
    return double(a.x()) * double(b.y()) - double(a.y()) * double(b.x());
}

double len2d(const QVector2D &v) {
    return std::hypot(double(v.x()), double(v.y()));
}

bool appendDedupedMarker(QVector<PathIssueMarker> *out, PathIssueMarker m, bool guiInches) {
    const QVector2D pMm = guiToMmXY(m.positionGui, guiInches);
    for (PathIssueMarker &e : *out) {
        if (len2d(guiToMmXY(e.positionGui, guiInches) - pMm) < kMarkerDedupeMm) {
            if (!e.toolTip.contains(m.toolTip)) {
                e.toolTip += QLatin1String("\n\n");
                e.toolTip += m.toolTip;
            }
            return false;
        }
    }
    out->append(std::move(m));
    return true;
}

enum class SegOverlapKind { None, Crossing, CollinearOverlap };

SegOverlapKind segmentHitMm(const QVector2D &p, const QVector2D &p2, const QVector2D &q, const QVector2D &q2,
                            QVector2D *hitMm) {
    QVector2D r = p2 - p;
    QVector2D s = q2 - q;
    const double lenR = len2d(r);
    const double lenS = len2d(s);
    if (lenR < 1.0e-9 || lenS < 1.0e-9) {
        return SegOverlapKind::None;
    }

    const double rxs = cross2(r, s);
    const double parallelTol = 1.0e-12 * lenR * lenS;

    if (std::abs(rxs) <= parallelTol) {
        if (std::abs(cross2(q - p, r)) > kVertexEpsMm * 0.25 * lenR) {
            return SegOverlapKind::None;
        }
        const QVector2D ru(float(r.x() / lenR), float(r.y() / lenR));
        const auto proj = [&](const QVector2D &pt) -> double {
            return double(QVector2D::dotProduct(pt - p, ru));
        };
        const double t0 = 0.0;
        const double t1 = lenR;
        double u0 = proj(q);
        double u1 = proj(q2);
        if (u0 > u1) {
            std::swap(u0, u1);
        }
        const double lo = std::max(t0, u0);
        const double hi = std::min(t1, u1);
        const double minSeg = qMin(lenR, lenS);
        // Require meaningful overlap vs segment size (reduces false hits on tessellated arcs / stitch).
        if (hi - lo > qMax(kVertexEpsMm * 3.0, 0.22 * minSeg)) {
            const double mid = (lo + hi) * 0.5;
            *hitMm = p + ru * float(mid);
            return SegOverlapKind::CollinearOverlap;
        }
        return SegOverlapKind::None;
    }

    const QVector2D qp = q - p;
    const double t = cross2(qp, s) / rxs;
    const double u = cross2(qp, r) / rxs;
    if (t <= 0.0 || t >= 1.0 || u <= 0.0 || u >= 1.0) {
        return SegOverlapKind::None;
    }
    if (t * lenR < kVertexEpsMm || (1.0 - t) * lenR < kVertexEpsMm) {
        return SegOverlapKind::None;
    }
    if (u * lenS < kVertexEpsMm || (1.0 - u) * lenS < kVertexEpsMm) {
        return SegOverlapKind::None;
    }
    *hitMm = p + r * float(t);
    return SegOverlapKind::Crossing;
}

void collectUnclosedDxf(const QVector<QVector<QVector3D>> &strips, bool guiInches, QVector<PathIssueMarker> *out) {
    for (int si = 0; si < strips.size(); ++si) {
        const QVector<QVector3D> &strip = strips.at(si);
        if (strip.size() < 3) {
            continue;
        }
        const QVector2D a = guiToMmXY(strip.first(), guiInches);
        const QVector2D b = guiToMmXY(strip.last(), guiInches);
        const double gapMm = len2d(a - b);
        if (gapMm <= kClosedTolMm) {
            continue;
        }
        const QVector3D mid = (strip.first() + strip.last()) * 0.5f;
        QString inchPart;
        if (guiInches) {
            inchPart = QStringLiteral(" (~%1 in)").arg(gapMm / kMmPerIn, 0, 'f', 4);
        }
        const QString tip = QStringLiteral(
            "Open profile (laser): this contour does not close back on itself — gap ≈ %1 mm%2.\n"
            "Many laser tools expect closed loops; open vectors can confuse nesting, lead-in/out, or "
            "cause incomplete cuts. Close the path in CAD or join the start and end.\n"
            "Quick fix: right-click this marker → \"Close contour\" bridges the gap with a tangent-smoothed "
            "curve (not a straight chord across fillets), then use Program → Save DXF geometry to export.")
            .arg(gapMm, 0, 'f', 3)
            .arg(inchPart);
        PathIssueMarker m;
        m.positionGui = mid;
        m.toolTip = tip;
        m.kind = PathIssueKind::OpenProfileEnds;
        m.stripIndex = si;
        appendDedupedMarker(out, m, guiInches);
    }
}

// Gaps *inside* a pen-down strip (short edge between long ones, or one edge much longer than the rest).
// "Open profile" (first≠last) is handled separately; DXF often breaks corners as micro-edges instead.
void collectShortPenUpGaps(const QVector<QVector<QVector3D>> &strips, bool guiInches,
                           const QVector<QVector3D> *rapidsGui, QVector<PathIssueMarker> *out) {
    if (!rapidsGui || rapidsGui->isEmpty() || strips.size() < 2) {
        return;
    }
    if ((rapidsGui->size() % 2) != 0) {
        return;
    }
    // Short travel between consecutive cut strips (tight window avoids flagging normal part-to-part rapids).
    constexpr double kMinMm = 0.12;
    constexpr double kMaxMm = 8.0;
    for (int r = 0; r + 1 < rapidsGui->size(); r += 2) {
        const QVector3D a = rapidsGui->at(r);
        const QVector3D b = rapidsGui->at(r + 1);
        const double gapMm = len2d(guiToMmXY(b, guiInches) - guiToMmXY(a, guiInches));
        if (gapMm <= kMinMm || gapMm >= kMaxMm) {
            continue;
        }
        const QVector3D mid = (a + b) * 0.5f;
        QString inchPart;
        if (guiInches) {
            inchPart = QStringLiteral(" (~%1 in)").arg(gapMm / kMmPerIn, 0, 'f', 4);
        }
        const QString tip = QStringLiteral(
            "Open across pen-up (laser): consecutive cut contours in the file are separated by a ~%1 mm%2 "
            "rapid — the yellow path does not form one continuous loop here. Many laser posts need closed "
            "geometry; join the entities in CAD or use a single polyline for the outline.\n"
            "Quick fix: right-click → \"Connect across pen-up\" merges the contours with the same tangent "
            "bridge where the rapid was, then use Program → Save DXF geometry to export.")
            .arg(gapMm, 0, 'f', 3)
            .arg(inchPart);
        PathIssueMarker mk;
        mk.positionGui = mid;
        mk.toolTip = tip;
        mk.kind = PathIssueKind::OpenAcrossPenUp;
        mk.rapidPairIndex = r / 2;
        appendDedupedMarker(out, mk, guiInches);
    }
}

void collectInteriorPenDownGaps(const QVector<QVector<QVector3D>> &strips, bool guiInches,
                                QVector<PathIssueMarker> *out) {
    constexpr double kShortMinMm = 0.09;
    constexpr double kShortMaxMm = 120.0;
    constexpr double kShortVsNeighbor = 0.42; // L < this * min(neighbor lengths); looser catches wider gaps
    constexpr double kNeighborLongerMult = 4.0;

    constexpr double kJumpMinMm = 0.30;
    constexpr double kJumpVsMedian = 11.0;
    constexpr double kJumpSpanRatioMin = 2.6; // strip must have varied edge lengths

    for (int si = 0; si < strips.size(); ++si) {
        const QVector<QVector3D> &strip = strips.at(si);
        const int n = strip.size();
        if (n < 2) {
            continue;
        }
        QVector<double> lens;
        lens.reserve(n - 1);
        for (int i = 0; i < n - 1; ++i) {
            lens.append(len2d(guiToMmXY(strip.at(i + 1), guiInches) - guiToMmXY(strip.at(i), guiInches)));
        }

        if (n >= 4) {
            QVector<double> sorted = lens;
            std::sort(sorted.begin(), sorted.end());
            const double p50 = sorted[sorted.size() / 2];
            const double spanRatio = sorted.back() / qMax(sorted.front(), 1.0e-6);
            if (spanRatio >= kJumpSpanRatioMin && p50 > 1.0e-4) {
                const double jumpTh = qMax(kJumpVsMedian * p50, kJumpMinMm);
                for (int i = 0; i < lens.size(); ++i) {
                    if (lens[i] <= jumpTh) {
                        continue;
                    }
                    const QVector3D mid = (strip.at(i) + strip.at(i + 1)) * 0.5f;
                    const QString tip = QStringLiteral(
                        "Interior pen-down jump (laser): one edge (~%1 mm) is far longer than typical edges "
                        "in this contour — the path may stitch unrelated geometry. Check for missing vertices "
                        "or exploded outlines in CAD.")
                        .arg(lens[i], 0, 'f', 2);
                    PathIssueMarker m;
                    m.positionGui = mid;
                    m.toolTip = tip;
                    m.kind = PathIssueKind::InteriorJump;
                    m.stripIndex = si;
                    m.edgeStartVertex = i;
                    appendDedupedMarker(out, m, guiInches);
                }
            }
        }

        for (int i = 0; i < n - 1; ++i) {
            const double L = lens[i];
            if (L < kShortMinMm || L > kShortMaxMm) {
                continue;
            }
            const double Lprev = (i > 0) ? lens[i - 1] : -1.0;
            const double Lnext = (i + 1 < lens.size()) ? lens[i + 1] : -1.0;
            bool shortGap = false;
            if (Lprev > 0.0 && Lnext > 0.0) {
                const double m = qMin(Lprev, Lnext);
                if (L < kShortVsNeighbor * m && m >= kNeighborLongerMult * L) {
                    shortGap = true;
                }
            } else if (Lprev > 0.0) {
                if (L < kShortVsNeighbor * Lprev && Lprev >= kNeighborLongerMult * L) {
                    shortGap = true;
                }
            } else if (Lnext > 0.0) {
                if (L < kShortVsNeighbor * Lnext && Lnext >= kNeighborLongerMult * L) {
                    shortGap = true;
                }
            }
            if (!shortGap) {
                continue;
            }
            const QVector3D mid = (strip.at(i) + strip.at(i + 1)) * 0.5f;
            QString inchPart;
            if (guiInches) {
                inchPart = QStringLiteral(" (~%1 in)").arg(L / kMmPerIn, 0, 'f', 4);
            }
            const QString tip = QStringLiteral(
                "Open segment / gap (laser): a short pen-down edge (~%1 mm%2) between much longer cuts — "
                "typical of an intentional or accidental break in the outline. Many laser tools expect "
                "continuous loops; close the gap or reconnect vertices in CAD.")
                .arg(L, 0, 'f', 3)
                .arg(inchPart);
            PathIssueMarker m;
            m.positionGui = mid;
            m.toolTip = tip;
            m.kind = PathIssueKind::OpenInteriorShortEdge;
            m.stripIndex = si;
            m.edgeStartVertex = i;
            appendDedupedMarker(out, m, guiInches);
        }
    }
}

void collectJunctionOverrun(const QVector<QVector<QVector3D>> &strips, bool guiInches, double dvMax,
                            double spmmX, double spmmY, double feedMmPerMin, QVector<PathIssueMarker> *out) {
    if (dvMax <= 0.0 || feedMmPerMin <= 0.0 || spmmX <= 0.0 || spmmY <= 0.0) {
        return;
    }
    const double vMmPerSec = feedMmPerMin / 60.0;

    for (int si = 0; si < strips.size(); ++si) {
        const QVector<QVector3D> &strip = strips.at(si);
        const int n = strip.size();
        if (n < 3) {
            continue;
        }
        for (int i = 1; i < n - 1; ++i) {
            const QVector2D p0 = guiToMmXY(strip.at(i - 1), guiInches);
            const QVector2D p1 = guiToMmXY(strip.at(i), guiInches);
            const QVector2D p2 = guiToMmXY(strip.at(i + 1), guiInches);
            QVector2D u = p1 - p0;
            QVector2D v = p2 - p1;
            const double lu = len2d(u);
            const double lv = len2d(v);
            if (lu < kVertexEpsMm || lv < kVertexEpsMm) {
                continue;
            }
            u /= float(lu);
            v /= float(lv);
            const QVector2D delta = (v - u) * float(vMmPerSec);
            const double dvx = std::abs(double(delta.x()) * spmmX);
            const double dvy = std::abs(double(delta.y()) * spmmY);
            const double dV = std::max(dvx, dvy);
            if (dV > dvMax * (1.0 + 1.0e-9)) {
                const QString tip = QStringLiteral(
                    "Tight corner / junction (motion + laser timing): estimated max axis velocity change "
                    "≈ %1 steps/s at this vertex exceeds your Corner DVmax setting (%2 steps/s), assuming "
                    "roughly the machine max feed (%3 mm/min).\n"
                    "The planner may have to slow sharply here; for lasers that can look like corner dwell "
                    "or overburn. Lower feed, raise DVmax (if safe), or add fillets in CAD.")
                    .arg(dV, 0, 'f', 0)
                    .arg(dvMax, 0, 'f', 0)
                    .arg(feedMmPerMin, 0, 'f', 0);
                PathIssueMarker m;
                m.positionGui = strip.at(i);
                m.toolTip = tip;
                m.kind = PathIssueKind::JunctionOverrun;
                m.stripIndex = si;
                m.edgeStartVertex = i;
                appendDedupedMarker(out, m, guiInches);
            }
        }
    }
}

struct SegMm {
    QVector2D a;
    QVector2D b;
    int strip = 0;
    int i0 = 0;
    float zGui = 0.f;
};

void collectOverlaps(const QVector<QVector<QVector3D>> &strips, bool guiInches, QVector<PathIssueMarker> *out) {
    QVector<SegMm> segs;
    for (int si = 0; si < strips.size(); ++si) {
        const QVector<QVector3D> &strip = strips.at(si);
        for (int i = 0; i + 1 < strip.size(); ++i) {
            SegMm s;
            s.a = guiToMmXY(strip.at(i), guiInches);
            s.b = guiToMmXY(strip.at(i + 1), guiInches);
            s.strip = si;
            s.i0 = i;
            s.zGui = (strip.at(i).z() + strip.at(i + 1).z()) * 0.5f;
            segs.append(s);
        }
    }
    const qint64 n = segs.size();
    if (n < 2) {
        return;
    }
    const qint64 maxPairs = qint64(kMaxOverlapSegPairs);
    const qint64 approxPairs = (n * (n - 1)) / 2;
    if (approxPairs > maxPairs) {
        return;
    }

    QVector2D hit;
    for (qint64 i = 0; i < n; ++i) {
        for (qint64 j = i + 1; j < n; ++j) {
            const SegMm &A = segs[int(i)];
            const SegMm &B = segs[int(j)];
            if (A.strip == B.strip) {
                const int d = std::abs(A.i0 - B.i0);
                if (d <= 1) {
                    continue;
                }
            }
            const SegOverlapKind kind = segmentHitMm(A.a, A.b, B.a, B.b, &hit);
            if (kind == SegOverlapKind::None) {
                continue;
            }
            const float z = (A.zGui + B.zGui) * 0.5f;
            const QVector3D g = mmXYToGui(float(hit.x()), float(hit.y()), z, guiInches);
            const QString tip = (kind == SegOverlapKind::CollinearOverlap)
                ? QStringLiteral(
                      "Overlapping cuts (laser): two pen-down segments run on top of each other here — "
                      "the same kerf gets a double pass (overburn, width change, or resin charring). "
                      "Remove duplicate lines or explode and clean the artwork.")
                : QStringLiteral(
                      "Crossing cuts (laser): two pen-down paths cross — the junction gets extra energy "
                      "and many postprocessors flag self-intersections. Edit the drawing so loops do not "
                      "cross if your workflow requires clean closed contours.");
            PathIssueMarker m;
            m.positionGui = g;
            m.toolTip = tip;
            m.kind = (kind == SegOverlapKind::CollinearOverlap) ? PathIssueKind::OverlapCollinear
                                                                 : PathIssueKind::OverlapCrossing;
            m.stripIndex = A.strip;
            m.edgeStartVertex = A.i0;
            m.secondaryStripIndex = B.strip;
            appendDedupedMarker(out, m, guiInches);
        }
    }
}

} // namespace

void BuildPathIssueMarkers(const QVector<QVector<QVector3D>> &cutStripsGui,
                           bool dxfSource,
                           bool guiInches,
                           double junctionDVmaxStepsPerSec,
                           double spmmX,
                           double spmmY,
                           double feedMmPerMin,
                           const QVector<QVector3D> *rapidSegmentsGui,
                           QVector<PathIssueMarker> *outMarkers) {
    if (!outMarkers) {
        return;
    }
    outMarkers->clear();
    if (cutStripsGui.isEmpty()) {
        return;
    }
    if (dxfSource) {
        collectUnclosedDxf(cutStripsGui, guiInches, outMarkers);
        collectInteriorPenDownGaps(cutStripsGui, guiInches, outMarkers);
        collectShortPenUpGaps(cutStripsGui, guiInches, rapidSegmentsGui, outMarkers);
    }
    collectJunctionOverrun(cutStripsGui, guiInches, junctionDVmaxStepsPerSec, spmmX, spmmY, feedMmPerMin,
                           outMarkers);
    collectOverlaps(cutStripsGui, guiInches, outMarkers);
}
