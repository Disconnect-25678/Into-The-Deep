package org.firstinspires.ftc.teamcode.opmode.subysstemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;

@Config
@TeleOp(group = "Sub", name = "Hardware-Just-the-Diffy")
public class HardwareWrist extends OpMode {
    private EndEffector effector;

    public static double leftPosition = 0;
    public static double rightPosition = 0;

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
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
        effector.setServoPositions(leftPosition, rightPosition);
        effector.periodic();
    }

    @Override
    public void stop() {
        RobotHardware.getInstance().kill();
    }
}
