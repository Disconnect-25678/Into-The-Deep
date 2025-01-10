package org.firstinspires.ftc.teamcode.opmode.subysstemSpecific;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@TeleOp(group = "Sub")
public class HardwareLift extends OpMode {
    private Lift lift;

    public static int target = 0;

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        lift = new Lift(RobotHardware.getInstance().leftLiftMotor,
                RobotHardware.getInstance().rightLiftMotor,
                telemetry);
    }

    @Override
    public void loop() {
        lift.setTarget(target);
        lift.periodic();
    }
}
