// EXCERPT — source: ClearCNC_Controller/ClearCoreFirmware/ClearCNC_Firmware.cpp
// EVIDENCE: E1 | symbol: main | lines: 3005-3043

int main() {
    Delay_ms(300);

    SerialPort.Mode(Connector::USB_CDC);
    SerialPort.Speed(SERIAL_BAUD_RATE);
    SerialPort.PortOpen();
    uint32_t startTime = Milliseconds();
    while (WAIT_FOR_USB_CONNECTION && !SerialPort && Milliseconds() - startTime < 5000) {
        continue;
    }
    Delay_ms(100);

    InitializeController();
    if (!motorsInitialized) {
        SendLine("error: motion controller init failed");
    } else {
        SendLine("ClearCNC firmware ready");
        SendLine("Use ENABLE before motion commands");
    }
    InitializeEthernet();

    while (true) {
        if (!ethernetReady) {
            InitializeEthernet();
        }
        PollDiscovery();
        PollTelemetryClient();
        DrainDeferredCommandsForMainLoop();
        if (ReadLine(lineBuffer, MAX_LINE_LENGTH)) {
            ProcessCommand(lineBuffer);
        }
        if (ReadTcpLine(tcpLineBuffer, MAX_LINE_LENGTH)) {
            ProcessCommand(tcpLineBuffer);
        }
        UpdateMotionExecutor();
        HardwareEstopPoll();
        PublishTelemetry();
        Delay_ms(1);
    }
