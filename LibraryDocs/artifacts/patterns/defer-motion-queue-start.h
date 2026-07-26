// EXCERPT — source: libClearCore/inc/CoordinatedMotionController.h
// EVIDENCE: E1 | symbol: SetDeferMotionQueueStart | lines: 134-147

    /**
        \brief When true, QueueArc/QueueLinear (planner path) only enqueue; call
        StartDeferredMotionIfPending() after batching so RecalculatePlanner sees
        the full chain (smoother arc quadrants, etc.). Default false.
    **/
    void SetDeferMotionQueueStart(bool defer) {
        m_deferQueueStart = defer;
    }

    /**
        \brief Start first queued motion if idle and queue non-empty (after batch enqueue).
        \return true if motion started
    **/
    bool StartDeferredMotionIfPending();
