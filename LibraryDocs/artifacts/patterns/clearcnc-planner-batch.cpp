// EXCERPT — source: ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp
// EVIDENCE: E1 | symbol: StartMotionBlock coordinated batch | lines: 1663-1711

    if (g_xyCoordinated) {
        uint8_t xyChain = CountConsecutiveXyPlannerChainAtHead();
        // Never submit more segments than the planner queue can hold in one batch.
        // Overflow would cause QueueArc/QueueLinear to fail → StartMotionBlock returns false
        // → ClearMotionQueue() wipes everything. The cap is safe; the leftover segments stay
        // in the firmware queue and form the next batch (which will restart from 0 speed —
        // so the real fix is keeping ARC_QUEUE_SIZE ≥ MOTION_QUEUE_SIZE).
        if (xyChain > motionController.PlannerQueueCapacity()) {
            xyChain = motionController.PlannerQueueCapacity();
        }
        if (xyChain >= 1) {
            motionController.SetPosition(currentSteps[AXIS_X], currentSteps[AXIS_Y]);
            motionController.ArcAccelMax(axisConfig[AXIS_X].accelMax);
            motionController.SetDeferMotionQueueStart(xyChain > 1);
            uint8_t idx = queueHead;
            for (uint8_t k = 0; k < xyChain; k++) {
                MotionBlock &seg = motionQueue[idx];
                uint32_t pathSpeed = seg.nominalSpeed;
                if (pathSpeed < 1) {
                    pathSpeed = 1;
                }
                if (pathSpeed > axisConfig[AXIS_X].velocityMax) {
                    pathSpeed = axisConfig[AXIS_X].velocityMax;
                }
                motionController.ArcVelMax(pathSpeed);
                if (seg.kind == MOTION_BLOCK_LINEAR) {
                    if (!motionController.QueueLinear(seg.target[AXIS_X], seg.target[AXIS_Y])) {
                        motionController.SetDeferMotionQueueStart(false);
                        motionController.Stop();
                        return false;
                    }
                } else {
                    if (!motionController.QueueArc(seg.arcCenterX, seg.arcCenterY, seg.arcRadius,
                                                   seg.arcStartAngle, seg.arcEndAngle,
                                                   seg.arcClockwise)) {
                        motionController.SetDeferMotionQueueStart(false);
                        motionController.Stop();
                        return false;
                    }
                }
                idx = (idx + 1) % MOTION_QUEUE_SIZE;
            }
            motionController.SetDeferMotionQueueStart(false);
            if (!motionController.IsActive()) {
                if (!motionController.StartDeferredMotionIfPending()) {
                    motionController.Stop();
                    return false;
                }
            }
