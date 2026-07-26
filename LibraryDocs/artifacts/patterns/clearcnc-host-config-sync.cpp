// EXCERPT — source: ClearCNC_Controller/QtController/src/MainWindow.cpp
// EVIDENCE: E1 | symbol: host CONFIG sync | lines: 1253-1281

        const int v = qRound(sps);
        return qMax(1, v);
    };
    const int rvelX = toRvelStepsPerSec(m_rapidMmMinX, spmmX);
    const int rvelY = toRvelStepsPerSec(m_rapidMmMinY, spmmY);
    const int rvelZ = toRvelStepsPerSec(m_rapidMmMinZ, spmmZ);
    const int rvelA = toRvelStepsPerSec(m_rapidRateFourth, spmmA);
    const QString cmd =
        QString("CONFIG SPMMX=%1 SPMMY=%2 VEL=%3 ACCEL=%4 DECEL=%5 DVMAX=%6 SINGLE=%7 AX=%8 LASER=%9 "
                "RVELX=%10 RVELY=%11 RVELZ=%12 RVELA=%13")
            .arg(spmmX, 0, 'f', 6)
            .arg(spmmY, 0, 'f', 6)
            .arg(m_velocity, 0, 'f', 0)
            .arg(m_accel, 0, 'f', 0)
            .arg(m_decel, 0, 'f', 0)
            .arg(m_junctionDVmax, 0, 'f', 0)
            .arg(single)
            .arg(axMask)
            .arg(0)  // LASER always 0 — laser mode removed
            .arg(rvelX)
            .arg(rvelY)
            .arg(rvelZ)
            .arg(rvelA);
    SendCommand(cmd);
}

void MainWindow::AppendLog(const QString &line) {
    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    m_logEdit->append(QString("[%1] %2").arg(timestamp, line));
