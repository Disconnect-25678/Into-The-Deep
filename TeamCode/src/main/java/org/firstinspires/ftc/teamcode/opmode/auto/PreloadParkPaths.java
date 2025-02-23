package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.LoopTimer;
import org.firstinspires.ftc.teamcode.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

@Config
@Autonomous
public class PreloadParkPaths extends LinearOpMode {
    public static long timeDrivePreload = 2100;
    public static double drivePowerPreloadX = -0.1;
    public static double drivePowerPreloadY = 0.3;
    public static double rotatePowerPrelaod = 0.0057;

    public static long timeDrivePark = 3500;
    public static double drivePowerParkX = 0.37;
    public static double drivePowerParkY = -0.12;
    public static double rotatePowerPark = -0.011315;

    public static long timeWait = 300;

    private Drivetrain drivetrain;
    private BulkReader bulkReader;
    private LoopTimer loopTimer;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);

        bulkReader = new BulkReader(hardwareMap);
        loopTimer = new LoopTimer(telemetry);

        RobotHardware.getInstance().initializeDrivetrain(hardwareMap);
        drivetrain = new Drivetrain(RobotHardware.getInstance().driveMotors,
                RobotHardware.getInstance().imu,
                this.telemetry);

        while (opModeInInit()) {
            telemetry.addLine("ready");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(

                    new InstantCommand(
                            () -> drivetrain.drive(new Vector2d(drivePowerPreloadX, drivePowerPreloadY), rotatePowerPrelaod), drivetrain
                    ),
                    new WaitCommand(timeDrivePreload),
                    new InstantCommand(() -> drivetrain.drive(new Vector2d(0, 0), 0), drivetrain),
                    new WaitCommand(timeWait),
                    new InstantCommand(
                            () -> drivetrain.drive(new Vector2d(drivePowerParkX, drivePowerParkY), rotatePowerPark), drivetrain
                    ),
                    new WaitCommand(timeDrivePark),
                    new InstantCommand(() -> drivetrain.drive(new Vector2d(0, 0), 0), drivetrain)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            loopTimer.updateLoop();
            telemetry.update();
        }
    }
}
