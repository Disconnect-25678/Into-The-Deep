package org.firstinspires.ftc.teamcode.common.util;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.RunCommand;

import org.firstinspires.ftc.teamcode.subsystem.BulkReader;

public abstract class CommandOpModeEx extends CommandOpMode {
    protected LoopTimer timer;
    protected BulkReader reader;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        this.timer = new LoopTimer(telemetry);
        reader = new BulkReader(hardwareMap);
        schedule(new RunCommand(telemetry::update));
        schedule(new RunCommand(timer::updateLoop));
    }
}
