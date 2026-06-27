#pragma once

#include <QOpenGLWidget>

#include "PathVizIssues.h"

#include <QMatrix4x4>
#include <QString>
#include <QVector>
#include <QVector3D>

class QContextMenuEvent;
class QEvent;
class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

// Simple 3D view: grid, axes, and tool at (X,Y,Z) in machine / DRO units.
class PositionView3D : public QOpenGLWidget {
    Q_OBJECT
public:
    explicit PositionView3D(QWidget *parent = nullptr);
    void setToolPosition(float x, float y, float z);
    // Line spacing in the same machine units as the DRO (1.0 = 1 in, 25.4 = 1 in when DRO is mm).
    void setGridCellStep(float cellStep);
    // Arrow sizes are derived from "mm" design sizes; DRO in inches must scale 1/25.4 to stay small on screen.
    void setDroInches(bool inches);
    // Cutting polylines and rapid segments from GCodeProgramKinematics.
    // cutStrips: consecutive G1/G2/G3 runs between rapids (drawn yellow).
    // rapidSegments: flat list of start/end pairs for G0 moves (drawn gray).
    void setProgramPathSplit(const QVector<QVector<QVector3D>> &cutStrips,
                             const QVector<QVector3D> &rapidSegments);
    /// Red crosshairs when path trace is on; hover for laser-oriented explanation per issue.
    void setPathIssues(const QVector<PathIssueMarker> &markers);
    // Legacy single-path overload kept for backward compatibility.
    void setProgramPath(const QVector<QVector3D> &points);
    void setPathTraceEnabled(bool on);
    // Multi-line DRO; drawn in 2D on top of the GL view (reliable on Windows vs stacked QLabel).
    void setDroOverlayText(const QString &text);
    void resetView();
    /// Lock camera to plan view (looking down +Z onto XY). Enables with azimuth 0 (+X right, +Y up); orbit spins in plane.
    void setTopDownLocked(bool locked);
    bool isTopDownLocked() const { return m_topDownLocked; }
    /// When false (default), left mouse alone does not orbit; use checkbox in main UI to enable.
    void setLeftDragOrbitEnabled(bool on);
    bool isLeftDragOrbitEnabled() const { return m_leftDragOrbitEnabled; }

signals:
    /// DXF preview only: MainWindow merges polylines / HPGL in memory.
    void pathIssueQuickFixCloseContour(int stripIndex);
    void pathIssueQuickFixMergeAcrossRapid(int rapidPairIndex);

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintEvent(QPaintEvent *event) override;
    void paintGL() override;
    void leaveEvent(QEvent *event) override;
    void contextMenuEvent(QContextMenuEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void computeClipFromWorld(QMatrix4x4 *clipFromWorld) const;
    int pathIssueIndexAt(const QPoint &widgetPos) const;
    void updatePathIssueHover(const QPoint &widgetPos);
    void drawGrid(float halfExtent);
    void drawAxes(float length);
    void drawProgramPath();
    void drawPathIssueMarkers();
    void drawTool();

    float m_x = 0.0f;
    float m_y = 0.0f;
    float m_z = 0.0f;

    float m_azDeg = 35.0f; // camera orbit
    float m_elDeg = 30.0f;
    float m_distance = 508.0f; // ~20" at 25.4 mm/DRO unit; rescaled in setGridCellStep
    float m_gridCellStep = 25.4f; // 1" in mm; MainWindow overwrites
    bool m_droInches = false; // if true, 1.0 in DRO = 1" (not 1 mm) — must scale tool glyph
    bool m_pathTraceEnabled = false;
    QVector<QVector3D> m_programPath;            // legacy flat path
    QVector<QVector<QVector3D>> m_cutStrips;     // one polyline per consecutive cut run
    QVector<QVector3D> m_rapidSegments;          // flat start/end pairs for G0 moves
    QVector<PathIssueMarker> m_pathIssues;
    int m_hoverPathIssueIndex = -1;
    QString m_droOverlayText;
    QPoint m_lastMouse;
    /// World-space offset of the orbit / look-at target from the DRO tool position (pan).
    QVector3D m_viewPan{0.f, 0.f, 0.f};
    bool m_topDownLocked = false;
    bool m_leftDragOrbitEnabled = false;
};
