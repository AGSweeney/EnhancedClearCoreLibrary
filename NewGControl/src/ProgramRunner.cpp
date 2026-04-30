#include "ProgramRunner.h"

#include "GrblProtocol.h"

void ProgramRunner::SetLines(const QStringList &lines) {
    m_lines = lines;
    Reset();
}

const QStringList &ProgramRunner::Lines() const {
    return m_lines;
}

void ProgramRunner::Reset() {
    m_state = {};
}

void ProgramRunner::Start() {
    m_state = {};
    m_state.running = true;
}

void ProgramRunner::Stop() {
    m_state = {};
}

void ProgramRunner::SetPaused(bool paused) {
    m_state.paused = paused;
}

bool ProgramRunner::Running() const {
    return m_state.running;
}

bool ProgramRunner::Paused() const {
    return m_state.paused;
}

bool ProgramRunner::AwaitingAck() const {
    return m_state.awaitingAck;
}

bool ProgramRunner::WaitingForQueueDrain() const {
    return m_state.waitingForQueueDrain;
}

int ProgramRunner::CurrentIndex() const {
    return m_state.index;
}

int ProgramRunner::ActiveLine() const {
    return m_state.activeLine;
}

const QList<int> &ProgramRunner::PendingMotionLines() const {
    return m_state.pendingMotionLineIndices;
}

void ProgramRunner::NoteActiveLine(int lineIndex) {
    m_state.activeLine = lineIndex;
}

void ProgramRunner::MarkQueueDrainPending() {
    m_state.waitingForQueueDrain = true;
    m_state.queueDrainStalledPolls = 0;
}

void ProgramRunner::MarkQueueDrainResolved() {
    m_state.waitingForQueueDrain = false;
    m_state.queueDrainStalledPolls = 0;
}

void ProgramRunner::MarkQueueDrainStall() {
    ++m_state.queueDrainStalledPolls;
}

int ProgramRunner::QueueDrainStallCount() const {
    return m_state.queueDrainStalledPolls;
}

void ProgramRunner::ResetQueueDrainStallCount() {
    m_state.queueDrainStalledPolls = 0;
}

void ProgramRunner::MarkAwaitingAck(bool awaiting) {
    m_state.awaitingAck = awaiting;
}

void ProgramRunner::OnAck(const GrblProtocol &protocol) {
    m_state.queueFullRetries = 0;
    m_state.awaitingAck = false;
    if (m_state.index >= 0 && m_state.index < m_lines.size() &&
        protocol.IsMotionLine(m_lines.at(m_state.index))) {
        m_state.pendingMotionLineIndices.append(m_state.index);
    }
    ++m_state.index;
}

void ProgramRunner::OnQueueFull() {
    m_state.awaitingAck = false;
    ++m_state.queueFullRetries;
}

void ProgramRunner::OnError() {
    Stop();
}

bool ProgramRunner::CompletedOrDraining() const {
    return m_state.index >= m_lines.size() || m_state.waitingForQueueDrain;
}

bool ProgramRunner::HasNextLine() const {
    return m_state.index >= 0 && m_state.index < m_lines.size();
}

QString ProgramRunner::CurrentLine() const {
    if (!HasNextLine()) {
        return {};
    }
    return m_lines.at(m_state.index);
}

bool ProgramRunner::CurrentLineRequiresDrain(const GrblProtocol &protocol) const {
    if (!HasNextLine()) {
        return false;
    }
    return protocol.IsQueueDrainRequiredLine(m_lines.at(m_state.index));
}

bool ProgramRunner::CurrentLineIsTerminal(const GrblProtocol &protocol) const {
    if (!HasNextLine()) {
        return true;
    }
    return protocol.IsTerminalStopLine(m_lines.at(m_state.index));
}
