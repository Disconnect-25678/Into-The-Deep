package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

@Config
@Autonomous
public class DriveOut extends OpMode {
    private Drivetrain drivetrain;

    private ElapsedTime elapsedTime;

    public static double secondsDrive = 5;
    @Override
    public void init() {
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        drivetrain = new Drivetrain(RobotHardware.getInstance().initializeDrivetrain(hardwareMap).driveMotors,
                                    RobotHardware.getInstance().imu,
                                    telemetry);

        elapsedTime = new ElapsedTime();
    }

    @Override
    public void start() {
        elapsedTime.reset();
    }

    @Override
    public void loop() {
        if (elapsedTime.time() < secondsDrive) {
            drivetrain.drive(Algorithms.mapJoystick(0, 0.3), 0);
        }
        else {
            drivetrain.drive(Algorithms.mapJoystick(0, 0), 0);
        }
    }
}
