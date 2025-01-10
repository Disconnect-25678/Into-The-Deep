package org.firstinspires.ftc.teamcode.opmode.subysstemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;

@Config
@TeleOp(group = "Sub")
public class HardwareArm extends OpMode {
    private EndEffector effector;

    public static double target = 0;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        effector = new EndEffector(RobotHardware.getInstance().armLeft,
                RobotHardware.getInstance().armRight,
                RobotHardware.getInstance().wristLeft,
                RobotHardware.getInstance().wristRight,
                RobotHardware.getInstance().clawServo,
                telemetry);
    }

    @Override
    public void loop() {
        effector.setArmPosition(target);
        effector.periodic();
    }
}
