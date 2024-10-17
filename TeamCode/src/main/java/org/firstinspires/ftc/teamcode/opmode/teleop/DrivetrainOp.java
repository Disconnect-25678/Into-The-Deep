package org.firstinspires.ftc.teamcode.opmode.teleop;

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

    @Override
    public void initialize(){
        super.initialize();
        gamepad = new GamepadEx(gamepad1);

        drivetrain = new Drivetrain(RobotHardware.getInstance().initializeDrivetrain(hardwareMap).driveMotors,
                                    RobotHardware.getInstance().imu,
                                    this.telemetry);

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new RunCommand(drivetrain::resetYaw, drivetrain)
        );
    }

    @Override
    public void run() {
        super.run();

        drivetrain.driveFacingAngle(Algorithms.mapJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);

        this.timer.updateLoop();
    }
}
