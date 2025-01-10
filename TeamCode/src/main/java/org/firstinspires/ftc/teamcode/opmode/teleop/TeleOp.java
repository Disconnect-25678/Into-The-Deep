package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;
import org.firstinspires.ftc.teamcode.subsystem.Laterator;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class TeleOp extends CommandOpModeEx {
    public static double DEAD_BAND = 0.2;
    private Drivetrain dt;
    private Lift lift;
    private Laterator laterator;
    private EndEffector effector;

    private boolean usingHeadingPID = true;

    private boolean cancel;

    private GamepadEx gamepad;

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

        setCancel(false);

        gamepad.getGamepadButton(GamepadKeys.Button.Y).whenPressed(
                new InstantCommand(dt::resetYaw, dt)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> usingHeadingPID = !usingHeadingPID)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.START).whileHeld(
                new RunCommand(lift::doResetMovement, lift)
        ).whenReleased(
                new InstantCommand(lift::reset, lift)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.BACK).whileHeld(
                new RunCommand(laterator::doResetMovement, laterator)
        ).whenReleased(
                new InstantCommand(laterator::reset, laterator)
        );

//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    lift.setRearIntakePosition();
//                    laterator.setRearIntakePosition();
//                    effector.setRearIntakePosition();
//                    effector.release();
//                }, lift, laterator, effector)
//        ).whenReleased(
//                new SequentialCommandGroup(
//                        new InstantCommand(effector::grab, effector),
//                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
//                        new InstantCommand(() -> {
//                            lift.setStow();
//                            laterator.stow();
//                            effector.setStow();
//                        })
//                )
//        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    setCancel(false);
                    lift.setRearIntakePosition();
                    laterator.setRearIntakePosition();
                    effector.setRearIntakePosition();
                    effector.release();
                }, lift, laterator, effector)
        );

        new Trigger(
                () -> !gamepad1.right_bumper && !cancel
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(() -> {
                            lift.setStow();
                            laterator.stow();
                            effector.setStow();
                        })
                )
        );

//        new Trigger(
//                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2
//        ).whenActive(
//                new InstantCommand(() -> {
//                    lift.setGroundIntakePosition();
//                    effector.setGroundIntakePosition();
//                    effector.release();
//                }, lift, effector)
//        ).whileActiveContinuous(
//                new RunCommand(() -> laterator.setScaleTarget(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
//        ).whenInactive(
//                new SequentialCommandGroup(
//                        new InstantCommand(effector::grab, effector),
//                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
//                        new InstantCommand(() -> {
//                            effector.setStow();
//                            lift.setStow();
//                            laterator.stow();
//                        })
//                )
//        );

        new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > DEAD_BAND
        ).whenActive(
                new InstantCommand(() -> {
                    setCancel(false);
                    lift.setGroundIntakePosition();
                    effector.setGroundIntakePosition();
                    effector.release();
                }, lift, effector)
        );

        new Trigger(
                () -> !cancel && gamepad1.right_trigger > DEAD_BAND
        ).whileActiveContinuous(
                new RunCommand(() -> laterator.setScaleTarget(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
        );

        new Trigger(
                () -> !cancel && !(gamepad1.right_trigger > DEAD_BAND)
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(() -> {
                            effector.setStow();
                            lift.setStow();
                            laterator.stow();
                        })
                )
        );

//        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    lift.setHighBucketLevel();
//                    laterator.setBucketScorePosition();
//                    effector.setBucketPosition();
//                    effector.release();
//                }, lift, laterator, effector)
//        ).whenReleased(
//                new SequentialCommandGroup(
//                        new InstantCommand(effector::grab, effector),
//                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
//                        new InstantCommand(() -> {
//                            lift.setStow();
//                            laterator.stow();
//                            effector.setStow();
//                        })
//                )
//        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    setCancel(false);
                    lift.setHighBucketLevel();
                    laterator.setBucketScorePosition();
                    effector.setBucketPosition();
                    effector.release();
                }, lift, laterator, effector)
        );

        new Trigger(
                () -> !cancel && !gamepad1.left_bumper
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(() -> {
                            lift.setStow();
                            laterator.stow();
                            effector.setStow();
                        })
                )
        );

//        new Trigger(
//                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2
//        ).whenActive(
//                new InstantCommand(() -> {
//                    effector.grab();
//                    lift.setHighSubLevel();
//                    laterator.setSubScorePosition();
//                    effector.setSubScorePosition();
//                }, lift, effector)
//        ).whenInactive(
//                new SequentialCommandGroup(
//                        new FunctionalCommand(
//                                () -> lift.setPostScorePosition(),
//                                () -> {},
//                                (interrupted) -> {},
//                                () -> lift.isAtTarget(),
//                                lift
//                        ),
//                        new InstantCommand(effector::release),
//                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
//                        new InstantCommand(() -> {
//                            effector.setStow();
//                            lift.setStow();
//                            laterator.stow();
//                        })
//                )
//        );

        new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > 0.2
        ).whenActive(
                new InstantCommand(() -> {
                    setCancel(false);
                    effector.grab();
                    lift.setHighSubLevel();
                    laterator.setSubScorePosition();
                    effector.setSubScorePosition();
                }, lift, effector)
        );

        new Trigger(
                () -> !cancel && !(gamepad1.left_trigger > DEAD_BAND)
        ).whenActive(
                new SequentialCommandGroup(
                        new FunctionalCommand(
                                () -> lift.setPostScorePosition(),
                                () -> {},
                                (interrupted) -> {},
                                () -> lift.isAtTarget(),
                                lift
                        ),
                        new InstantCommand(effector::release),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(() -> {
                            effector.setStow();
                            lift.setStow();
                            laterator.stow();
                        })
                )
        );

        new Trigger(
                () -> !cancel && gamepad1.a && gamepad1.right_trigger > DEAD_BAND
        ).whenActive(
                new InstantCommand(effector::toggleTwistRotation, effector)
        );

        new Trigger(
                () -> !cancel && gamepad1.a && gamepad1.left_trigger > DEAD_BAND
        ).whenActive(
                new InstantCommand(lift::setLowSubLevel, lift)
        );

        new Trigger(
                () -> !cancel && gamepad1.a && gamepad1.left_bumper
        ).whenActive(
                new InstantCommand(lift::setLowBucketLevel, lift)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    setCancel(true);
                    lift.setStow();
                    laterator.stow();
                    effector.setStow();
                })
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
        telemetry.addData("cancel: ", cancel);
    }

    private void setCancel(boolean b) {
        this.cancel = b;
    }
}
