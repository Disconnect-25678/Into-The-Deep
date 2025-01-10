package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;

@TeleOp(name = "DT only")
public class DrivetrainOp extends CommandOpModeEx {
    private Drivetrain drivetrain;
    private GamepadEx gamepad;

    private boolean usingHeadingPID;

    @Override
    public void initialize(){
        super.initialize();
        usingHeadingPID = false;
        gamepad = new GamepadEx(gamepad1);

        drivetrain = new Drivetrain(RobotHardware.getInstance().initializeDrivetrain(hardwareMap).driveMotors,
                                    RobotHardware.getInstance().imu,
                                    this.telemetry);

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(drivetrain::resetYaw, drivetrain)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> usingHeadingPID = !usingHeadingPID)
        );

        schedule(new RunCommand(telemetry::update));
    }

    @Override
    public void run() {
        super.run();
        if (usingHeadingPID)
            drivetrain.driveFacingAngle(Algorithms.mapJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);
        else
            drivetrain.drive(Algorithms.mapJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);

        telemetry.addData("Using HeadingPID: ", usingHeadingPID);
        this.timer.updateLoop();
    }
}
