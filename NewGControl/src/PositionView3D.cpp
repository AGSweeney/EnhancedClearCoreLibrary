#include "PositionView3D.h"

#include <QColor>
#include <QFont>
#include <QFontMetrics>
#include <QMouseEvent>
#include <QMatrix4x4>
#include <QPaintEvent>
#include <QOpenGLContext>
#include <QPainter>
#include <QOpenGLFunctions_2_1>
#include <QOpenGLVersionFunctionsFactory>
#include <QWheelEvent>

#include <QtMath>
#include <cmath>

namespace {
QOpenGLFunctions_2_1 *gl21() {
    return QOpenGLVersionFunctionsFactory::get<QOpenGLFunctions_2_1>(QOpenGLContext::currentContext());
}

// Z is world up. Azimuth rotates in the XY plane (around +Z). Elevation is angle above XY plane.
QVector3D eyeFromOrbit(const QVector3D &center, float dist, float azDeg, float elDeg) {
    const float radAz = qDegreesToRadians(azDeg);
    const float radEl = qDegreesToRadians(elDeg);
    const float cEl = std::cos(radEl);
    const QVector3D v(
        cEl * std::cos(radAz),
        cEl * std::sin(radAz),
        std::sin(radEl));
    return center + v * dist;
}

static constexpr float kTwoPi = 6.28318530717958647692f;

void addTriangle(QOpenGLFunctions_2_1 *f, float x0, float y0, float z0, float x1, float y1, float z1, float x2,
    float y2, float z2) {
    f->glVertex3f(x0, y0, z0);
    f->glVertex3f(x1, y1, z1);
    f->glVertex3f(x2, y2, z2);
}
} // namespace

PositionView3D::PositionView3D(QWidget *parent) : QOpenGLWidget(parent) {
    setMouseTracking(true);
    setMinimumSize(220, 200);
    setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
}

void PositionView3D::setToolPosition(float x, float y, float z) {
    // Telemetry can arrive very fast; skip redundant repaints when nothing changed.
    constexpr float eps = 1.0e-4f;
    if (qAbs(m_x - x) < eps && qAbs(m_y - y) < eps && qAbs(m_z - z) < eps) {
        return;
    }
    m_x = x;
    m_y = y;
    m_z = z;
    update();
}

void PositionView3D::setGridCellStep(float cellStep) {
    const float next = qMax(1.0e-4f, cellStep);
    if (m_gridCellStep > 1.0e-4f) {
        m_distance *= (next / m_gridCellStep);
    } else {
        m_distance = 20.0f * next;
    }
    m_gridCellStep = next;
    m_distance = qBound(0.3f * m_gridCellStep, m_distance, 120.0f * m_gridCellStep);
    update();
}

void PositionView3D::setDroInches(bool inches) {
    if (m_droInches == inches) {
        return;
    }
    m_droInches = inches;
    update();
}

void PositionView3D::setDroOverlayText(const QString &text) {
    if (m_droOverlayText == text) {
        return;
    }
    m_droOverlayText = text;
    update();
}

void PositionView3D::setProgramPath(const QVector<QVector3D> &points) {
    if (m_programPath == points) {
        return;
    }
    m_programPath = points;
    update();
}

void PositionView3D::setProgramPathSplit(const QVector<QVector<QVector3D>> &cutStrips,
                                         const QVector<QVector3D> &rapidSegments) {
    m_cutStrips = cutStrips;
    m_rapidSegments = rapidSegments;
    update();
}

void PositionView3D::setPathTraceEnabled(bool on) {
    if (m_pathTraceEnabled == on) {
        return;
    }
    m_pathTraceEnabled = on;
    update();
}

void PositionView3D::resetView() {
    m_azDeg = 35.0f;
    m_elDeg = 30.0f;
    // ~20" or ~500 mm: fits a 24" floor without the camera being miles away (200 was 200" in G20).
    m_distance = 20.0f * m_gridCellStep;
    update();
}

void PositionView3D::initializeGL() {
    if (!context()->isValid()) {
        return;
    }
    QOpenGLFunctions_2_1 *f = gl21();
    if (!f) {
        return;
    }
    f->initializeOpenGLFunctions();
    f->glEnable(GL_DEPTH_TEST);
    f->glDepthFunc(GL_LEQUAL);
    f->glEnable(GL_LINE_SMOOTH);
    f->glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    f->glEnable(GL_MULTISAMPLE);
    f->glClearColor(0.15f, 0.16f, 0.19f, 1.0f);
}

void PositionView3D::resizeGL(int w, int h) {
    if (h <= 0) {
        h = 1;
    }
    QOpenGLFunctions_2_1 *f = gl21();
    if (f) {
        f->glViewport(0, 0, w, h);
    }
}

void PositionView3D::paintGL() {
    if (!QOpenGLContext::currentContext()) {
        return;
    }
    QOpenGLFunctions_2_1 *f = gl21();
    if (!f) {
        return;
    }
    f->glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    const int w = width();
    const int h = qMax(1, height());
    const float aspect = float(w) / float(h);
    m_elDeg = qBound(-89.0f, m_elDeg, 89.0f);
    m_distance = qBound(0.3f * m_gridCellStep, m_distance, 120.0f * m_gridCellStep);

    const float fov = 45.0f;
    const float zNear = 0.5f;
    const float zFar = 100000.0f;
    QMatrix4x4 proj;
    proj.perspective(fov, aspect, zNear, zFar);

    const QVector3D at(m_x, m_y, m_z);
    const QVector3D eye = eyeFromOrbit(at, m_distance, m_azDeg, m_elDeg);
    QMatrix4x4 view;
    view.lookAt(eye, at, QVector3D(0, 0, 1));

    f->glMatrixMode(GL_PROJECTION);
    f->glLoadMatrixf(proj.constData());
    f->glMatrixMode(GL_MODELVIEW);
    f->glLoadMatrixf(view.constData());

    // Floor: fixed 24" × 24" in machine space (±12" from world origin; grid lines every 1").
    const float s = m_gridCellStep;
    static constexpr float kHalfFloorInches = 12.0f; // 24" total span per axis
    const float halfFloor = kHalfFloorInches * s;
    f->glDisable(GL_LIGHTING);
    drawGrid(halfFloor);
    const float axLen = 2.5f * s; // short RGB axis ticks (~2.5" / ~63 mm)
    drawAxes(axLen);
    if (m_pathTraceEnabled && (!m_cutStrips.isEmpty() || m_programPath.size() >= 2)) {
        drawProgramPath();
    }
    drawTool();
}

void PositionView3D::paintEvent(QPaintEvent *event) {
    QOpenGLWidget::paintEvent(event);
    if (m_droOverlayText.isEmpty()) {
        return;
    }
    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);
    p.setRenderHint(QPainter::TextAntialiasing, true);
    QFont f;
    f.setPointSize(12);
    f.setWeight(QFont::DemiBold);
    f.setStyleStrategy(QFont::PreferAntialias);
    f.setStyleHint(QFont::TypeWriter, QFont::PreferDefault);
    f.setFixedPitch(true);
    p.setFont(f);
    const QFontMetrics fm(f);
    const int margin = 10;
    const int padH = 14;
    const int padV = 12;
    const int maxW = qMin(520, qMax(180, (width() * 48) / 100));
    const QRect available(0, 0, maxW, 2000);
    const QRect textBounds = fm.boundingRect(available, Qt::AlignRight | Qt::TextWordWrap, m_droOverlayText);
    const int boxW = textBounds.width() + 2 * padH;
    const int boxH = textBounds.height() + 2 * padV;
    const int boxLeft = qMax(0, width() - margin - boxW);
    const QRect box(boxLeft, margin, boxW, boxH);
    p.setPen(QPen(QColor(78, 88, 102), 1.0));
    p.setBrush(QColor(28, 32, 40, 234));
    p.drawRoundedRect(box, 8, 8);
    p.setPen(QColor(0xf0, 0xf3, 0xf7));
    p.drawText(box.adjusted(padH, padV, -padH, -padV), Qt::AlignRight | Qt::AlignTop, m_droOverlayText);
}

void PositionView3D::drawProgramPath() {
    QOpenGLFunctions_2_1 *f = gl21();
    if (!f) {
        return;
    }
    f->glDisable(GL_LIGHTING);

    if (!m_cutStrips.isEmpty()) {
        // --- New split-path rendering ---
        // Rapids: thin gray lines
        if (!m_rapidSegments.isEmpty()) {
            f->glColor3f(0.45f, 0.45f, 0.45f);
            f->glLineWidth(1.0f);
            f->glBegin(GL_LINES);
            for (const QVector3D &p : m_rapidSegments) {
                f->glVertex3f(p.x(), p.y(), p.z());
            }
            f->glEnd();
        }
        // Cuts: bright yellow, one LINE_STRIP per run
        f->glColor3f(1.0f, 0.92f, 0.2f);
        f->glLineWidth(1.8f);
        for (const QVector<QVector3D> &strip : m_cutStrips) {
            if (strip.size() < 2) {
                continue;
            }
            f->glBegin(GL_LINE_STRIP);
            for (const QVector3D &p : strip) {
                f->glVertex3f(p.x(), p.y(), p.z());
            }
            f->glEnd();
        }
    } else {
        // Legacy fallback: single yellow strip (rapids included)
        f->glColor3f(1.0f, 0.92f, 0.2f);
        f->glLineWidth(1.5f);
        f->glBegin(GL_LINE_STRIP);
        for (const QVector3D &p : m_programPath) {
            f->glVertex3f(p.x(), p.y(), p.z());
        }
        f->glEnd();
    }
}

void PositionView3D::drawGrid(float halfExtent) {
    QOpenGLFunctions_2_1 *f = gl21();
    if (!f) {
        return;
    }
    const float step = m_gridCellStep;
    if (step <= 0.0f) {
        return;
    }
    const int n = int(std::ceil(double(halfExtent) / double(step))) + 1;
    f->glColor3f(0.32f, 0.35f, 0.4f);
    f->glLineWidth(1.0f);
    f->glBegin(GL_LINES);
    for (int k = -n; k <= n; ++k) {
        const float t = float(k) * step;
        if (t < -halfExtent || t > halfExtent) {
            continue;
        }
        f->glVertex3f(t, -halfExtent, 0.0f);
        f->glVertex3f(t, halfExtent, 0.0f);
        f->glVertex3f(-halfExtent, t, 0.0f);
        f->glVertex3f(halfExtent, t, 0.0f);
    }
    f->glEnd();
}

void PositionView3D::drawAxes(float length) {
    QOpenGLFunctions_2_1 *f = gl21();
    if (!f) {
        return;
    }
    f->glLineWidth(2.0f);
    f->glBegin(GL_LINES);
    f->glColor3f(0.95f, 0.25f, 0.25f);
    f->glVertex3f(0, 0, 0);
    f->glVertex3f(length, 0, 0);
    f->glColor3f(0.25f, 0.9f, 0.4f);
    f->glVertex3f(0, 0, 0);
    f->glVertex3f(0, length, 0);
    f->glColor3f(0.35f, 0.65f, 0.95f);
    f->glVertex3f(0, 0, 0);
    f->glVertex3f(0, 0, length);
    f->glEnd();
}

void PositionView3D::drawTool() {
    // Pointer: sharp tip (apex) at DRO (x,y,z), body along +Z (tool / spindle up).
    QOpenGLFunctions_2_1 *f = gl21();
    if (!f) {
        return;
    }
    const float x = m_x;
    const float y = m_y;
    const float z = m_z;
    // Design sizes in mm; multiply by toDro in G20 so 1" DRO ≠ 1" geometry.
    const float toDro = m_droInches ? (1.0f / 25.4f) : 1.0f;
    const int coneSeg = 20;
    const float coneH = 10.0f * toDro;
    const float headR = 3.5f * toDro;
    const float shaftH = 14.0f * toDro;
    const float halfW = 0.65f * toDro;
    const float zConeTop = z + coneH;
    const float zShaftTop = zConeTop + shaftH;

    f->glEnable(GL_POLYGON_OFFSET_FILL);
    f->glPolygonOffset(1.0f, 0.5f);

    f->glColor3f(0.25f, 0.75f, 0.88f);
    f->glBegin(GL_TRIANGLES);
    for (int i = 0; i < coneSeg; ++i) {
        const float a0 = kTwoPi * (float(i) / float(coneSeg));
        const float a1 = kTwoPi * (float(i + 1) / float(coneSeg));
        const float x0 = x + headR * std::cos(a0);
        const float y0 = y + headR * std::sin(a0);
        const float x1 = x + headR * std::cos(a1);
        const float y1 = y + headR * std::sin(a1);
        addTriangle(f, x, y, z, x0, y0, zConeTop, x1, y1, zConeTop);
    }
    // Cap at cone top (facing +Z)
    f->glColor3f(0.18f, 0.5f, 0.6f);
    for (int i = 0; i < coneSeg; ++i) {
        const float a0 = kTwoPi * (float(i) / float(coneSeg));
        const float a1 = kTwoPi * (float(i + 1) / float(coneSeg));
        const float x0 = x + headR * std::cos(a0);
        const float y0 = y + headR * std::sin(a0);
        const float x1 = x + headR * std::cos(a1);
        const float y1 = y + headR * std::sin(a1);
        addTriangle(f, x, y, zConeTop, x0, y0, zConeTop, x1, y1, zConeTop);
    }
    f->glEnd();

    f->glColor3f(0.2f, 0.6f, 0.7f);
    f->glBegin(GL_TRIANGLES);
    const float xl = x - halfW;
    const float xr = x + halfW;
    const float yb = y - halfW;
    const float yt = y + halfW;
    // +X side
    addTriangle(f, xr, yb, zConeTop, xr, yt, zConeTop, xr, yt, zShaftTop);
    addTriangle(f, xr, yb, zConeTop, xr, yt, zShaftTop, xr, yb, zShaftTop);
    // -X
    addTriangle(f, xl, yb, zConeTop, xl, yb, zShaftTop, xl, yt, zShaftTop);
    addTriangle(f, xl, yb, zConeTop, xl, yt, zShaftTop, xl, yt, zConeTop);
    // +Y
    addTriangle(f, xl, yt, zConeTop, xl, yt, zShaftTop, xr, yt, zShaftTop);
    addTriangle(f, xl, yt, zConeTop, xr, yt, zShaftTop, xr, yt, zConeTop);
    // -Y
    addTriangle(f, xl, yb, zConeTop, xr, yb, zConeTop, xr, yb, zShaftTop);
    addTriangle(f, xl, yb, zConeTop, xr, yb, zShaftTop, xl, yb, zShaftTop);
    f->glEnd();

    f->glDisable(GL_POLYGON_OFFSET_FILL);
}

void PositionView3D::mousePressEvent(QMouseEvent *event) {
    m_lastMouse = event->pos();
}

void PositionView3D::mouseMoveEvent(QMouseEvent *event) {
    if ((event->buttons() & Qt::LeftButton) == 0) {
        return;
    }
    const QPoint d = event->pos() - m_lastMouse;
    m_lastMouse = event->pos();
    m_azDeg += float(d.x()) * 0.4f;
    m_elDeg -= float(d.y()) * 0.4f;
    update();
}

void PositionView3D::wheelEvent(QWheelEvent *event) {
    const int steps = event->angleDelta().y() / 120;
    m_distance *= std::pow(0.9f, float(-steps));
    const float s = m_gridCellStep;
    // Sane zoom in DRO units (was unbounded; far zoom + huge grid made the view useless)
    m_distance = qBound(0.3f * s, m_distance, 120.0f * s);
    update();
}
