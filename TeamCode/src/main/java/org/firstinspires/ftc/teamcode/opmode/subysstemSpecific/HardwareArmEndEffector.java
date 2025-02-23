package org.firstinspires.ftc.teamcode.opmode.subysstemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.robot.Robot;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;

@Config
@TeleOp(group = "Sub")
public class HardwareArmEndEffector extends OpMode {
    private EndEffector effector;

    public static EndEffector.WristTwistAngles wtAngles = new EndEffector.WristTwistAngles(0.4, 0.05);

    public static double armPosition = 0.8;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        RobotHardware.getInstance().initializeEndEffector(hardwareMap);
        effector = new EndEffector(RobotHardware.getInstance().armLeft,
                RobotHardware.getInstance().armRight,
                RobotHardware.getInstance().wristLeft,
                RobotHardware.getInstance().wristRight,
                RobotHardware.getInstance().clawServo,
                telemetry);
    }

    @Override
    public void loop() {
        effector.setAngles(wtAngles);
        effector.setArmPosition(armPosition);
        effector.periodic();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().kill();
    }
}
