package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;
import org.firstinspires.ftc.teamcode.subsystem.Laterator;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends CommandOpModeEx {
    private Drivetrain dt;
    private Lift lift;
    private Laterator laterator;
    private EndEffector effector;

    private boolean usingHeadingPID = true;

    GamepadEx gamepad;

    @Override
    public void initialize() {
        super.initialize();

        RobotHardware.getInstance().initialize(hardwareMap);

        gamepad = new GamepadEx(gamepad1);

        dt = new Drivetrain(RobotHardware.getInstance().driveMotors,
                            RobotHardware.getInstance().imu,
                            this.telemetry);

        lift = new Lift(RobotHardware.getInstance().leftLiftMotor,
                        RobotHardware.getInstance().rightLiftMotor,
                        telemetry);

        laterator = new Laterator(RobotHardware.getInstance().lateratorMotor, telemetry);

        effector = new EndEffector(RobotHardware.getInstance().armLeft,
                                    RobotHardware.getInstance().armRight,
                                    RobotHardware.getInstance().wristLeft,
                                    RobotHardware.getInstance().wristRight,
                                    RobotHardware.getInstance().clawServo,
                                    telemetry);

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new RunCommand(dt::resetYaw, dt)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> usingHeadingPID = !usingHeadingPID)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.START).whileHeld(
                new RunCommand(lift::doResetMovement)
        ).whenReleased(
                new InstantCommand(lift::reset)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.BACK).whileHeld(
                new RunCommand(laterator::doResetMovement)
        ).whenReleased(
                new InstantCommand(laterator::reset)
        );
    }

    @Override
    public void run() {
        super.run();
        if (usingHeadingPID)
            dt.driveFacingAngle(Algorithms.mapJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);
        else
            dt.drive(Algorithms.mapJoystick(gamepad1.left_stick_x, -gamepad1.left_stick_y), gamepad1.right_stick_x);

        telemetry.addData("Using HeadingPID: ", usingHeadingPID);
    }
}
