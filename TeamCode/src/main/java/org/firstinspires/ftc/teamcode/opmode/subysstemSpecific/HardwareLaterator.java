package org.firstinspires.ftc.teamcode.opmode.subysstemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Laterator;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@TeleOp(group = "Sub")
public class HardwareLaterator extends OpMode {
    private Laterator laterator;

    public static int target = 0;

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        laterator = new Laterator(RobotHardware.getInstance().lateratorMotor, telemetry);
    }

    @Override
    public void loop() {
        laterator.setTarget(target);
        laterator.periodic();
    }
}
