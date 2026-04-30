#pragma once

#include <QList>
#include <QString>
#include <QStringList>

class GrblProtocol;

class ProgramRunner {
public:
    struct Snapshot {
        int index = 0;
        bool running = false;
        bool paused = false;
        bool awaitingAck = false;
        bool waitingForQueueDrain = false;
        int queueDrainStalledPolls = 0;
        int queueFullRetries = 0;
        int activeLine = -1;
        QList<int> pendingMotionLineIndices;
    };

    void SetLines(const QStringList &lines);
    const QStringList &Lines() const;

    void Reset();
    void Start();
    void Stop();
    void SetPaused(bool paused);

    bool Running() const;
    bool Paused() const;
    bool AwaitingAck() const;
    bool WaitingForQueueDrain() const;
    int CurrentIndex() const;
    int ActiveLine() const;
    const QList<int> &PendingMotionLines() const;

    void NoteActiveLine(int lineIndex);
    void MarkQueueDrainPending();
    void MarkQueueDrainResolved();
    void MarkQueueDrainStall();
    int QueueDrainStallCount() const;
    void ResetQueueDrainStallCount();

    void MarkAwaitingAck(bool awaiting);
    void OnAck(const GrblProtocol &protocol);
    void OnQueueFull();
    void OnError();

    bool CompletedOrDraining() const;
    bool HasNextLine() const;
    QString CurrentLine() const;
    bool CurrentLineRequiresDrain(const GrblProtocol &protocol) const;
    bool CurrentLineIsTerminal(const GrblProtocol &protocol) const;

private:
    QStringList m_lines;
    Snapshot m_state;
};
