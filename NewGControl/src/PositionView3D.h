#pragma once

#include <QOpenGLWidget>

#include <QString>
#include <QVector>
#include <QVector3D>

class QMouseEvent;
class QPaintEvent;
class QWheelEvent;

// Simple 3D view: grid, axes, and tool at (X,Y,Z) in machine / DRO units.
class PositionView3D : public QOpenGLWidget {
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
    // Legacy single-path overload kept for backward compatibility.
    void setProgramPath(const QVector<QVector3D> &points);
    void setPathTraceEnabled(bool on);
    // Multi-line DRO; drawn in 2D on top of the GL view (reliable on Windows vs stacked QLabel).
    void setDroOverlayText(const QString &text);
    void resetView();

protected:
    void initializeGL() override;
    void resizeGL(int w, int h) override;
    void paintEvent(QPaintEvent *event) override;
    void paintGL() override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent *event) override;

private:
    void drawGrid(float halfExtent);
    void drawAxes(float length);
    void drawProgramPath();
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
    QString m_droOverlayText;
    QPoint m_lastMouse;
};
