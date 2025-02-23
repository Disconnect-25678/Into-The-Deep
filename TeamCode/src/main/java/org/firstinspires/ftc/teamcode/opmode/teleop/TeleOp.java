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

    private boolean usingHeadingPID = false;
    private boolean isIntakingGround = false;

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

        gamepad.getGamepadButton(GamepadKeys.Button.START).whenPressed(
                new InstantCommand(dt::resetYaw, dt)
        );

//        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
//                new InstantCommand(() -> usingHeadingPID = !usingHeadingPID)
//        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whileHeld(
                new RunCommand(lift::doResetMovement, lift)
        ).whenReleased(
                new InstantCommand(lift::reset, lift)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whileHeld(
                new RunCommand(laterator::doResetMovement, laterator)
        ).whenReleased(
                new InstantCommand(laterator::reset, laterator)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> setCancel(false)),
                        new FunctionalCommand(
                                () -> laterator.setRearIntakePosition(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setRearIntakePositionArm, effector),
                        new WaitCommand(EndEffector.TIME_ARM_PASSOVER),
                        new InstantCommand(effector::setRearIntakePositionWrist, effector)
                )
        );

        new Trigger(
                () -> !gamepad1.right_bumper && !cancel
        ).whenActive(
                new SequentialCommandGroup(
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setStowWrist),
                        new WaitCommand(EndEffector.TIME_WAIT_STWIST),
                        new InstantCommand(effector::setStowArm, effector),
                        new WaitCommand(EndEffector.TIME_ARM_PASSOVER)
                )
        );

        new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > DEAD_BAND
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> dt.setSlowDrive(true), dt),
                        new InstantCommand(() -> setCancel(false)),
                        new InstantCommand(lift::setGroundIntakePosition, lift),
                        new InstantCommand(effector::setGroundIntakeMid, effector)
                )
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
                        new InstantCommand(() -> dt.setSlowDrive(false), dt),
                        new InstantCommand(() -> setIntakingGround(false)),
                        new InstantCommand(effector::resetToggleAngle, effector),
                        new InstantCommand(effector::setGroundIntakeMid, effector),
                        new WaitCommand(EndEffector.TIME_LIFT_OFF_GROUND_SUB),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setStow, effector)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
//                new InstantCommand(() -> {
//                    setCancel(false);
//                    lift.setLowBucketLevel();
//                    laterator.setBucketScorePosition();
//                    effector.setBucketPosition();
//                    effector.release();
//                }, lift, laterator, effector)
                new SequentialCommandGroup(
                        new InstantCommand(() -> setCancel(false)),
                        new InstantCommand(lift::setLowBucketLevel, lift),
                        new FunctionalCommand(
                                () -> laterator.setBucketScorePosition(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setBucketPositionArm, effector),
                        new WaitCommand(EndEffector.TIME_ARM_PASSOVER),
                        new InstantCommand(effector::setBucketPositionWrist, effector)

                )
        );

        new Trigger(
                () -> !cancel && !gamepad1.left_bumper
        ).whenActive(
                new SequentialCommandGroup(

                        new InstantCommand(lift::setStow, lift),
                        new InstantCommand(effector::setStowWrist, effector),
                        new WaitCommand(EndEffector.TIME_WAIT_STWIST),
                        new InstantCommand(effector::setStowArm),
                        new WaitCommand(EndEffector.TIME_ARM_PASSOVER)
                )
        );

        new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER) > DEAD_BAND
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> dt.setSlowDrive(true), dt),
                        new FunctionalCommand(
                                () -> lift.setHighSubLevel(),
                                () -> {},
                                (interrupted) -> {},
                                () -> lift.isAtTarget(),
                                lift
                        ),
                    new InstantCommand(() -> {
                        setCancel(false);
                        laterator.setSubScorePosition();
                        effector.setSubScorePosition();
                    }, lift, effector)

                )
        );

        new Trigger(
                () -> !cancel && !(gamepad1.left_trigger > DEAD_BAND)
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> dt.setSlowDrive(false), dt),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(() -> {
                            effector.setStow();
                            lift.setStow();
                        }, effector, lift)
                )
        );

        new Trigger(
                () -> !cancel && gamepad1.x && gamepad1.right_bumper
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(() -> setCancel(true)),
                        new InstantCommand(lift::setStow, lift),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setStowArm, effector),
                        new WaitCommand(EndEffector.TIME_ARM_PASSOVER),
                        new InstantCommand(effector::setStowWrist, effector)
                )
        );

        new Trigger(
                () -> !cancel && gamepad1.y && gamepad1.right_trigger > DEAD_BAND
        ).whenActive(
                new InstantCommand(effector::toggleTwistRotation, effector)
        );

        new Trigger(
                () -> !cancel && gamepad1.x && gamepad1.right_trigger > DEAD_BAND
        ).toggleWhenActive(
                new InstantCommand(this::setGroundIntakePosition, effector),
                new InstantCommand(effector::setGroundIntakeMidArm, effector)
        );

        new Trigger(
                () -> !cancel && gamepad1.y && gamepad1.right_trigger > DEAD_BAND
        ).toggleWhenActive(
                new InstantCommand(effector::setGroundIntakeRotated, effector),
                new InstantCommand(effector::setGroundIntakeWrist, effector)
        );

        new Trigger(
                () -> !cancel && gamepad1.x && gamepad1.left_trigger > DEAD_BAND
        ).whenActive(
                new InstantCommand(lift::setPostScorePosition, lift)
        );

        new Trigger(
                () -> !cancel && gamepad1.dpad_up && gamepad1.left_trigger > DEAD_BAND
        ).whenActive(
                new InstantCommand(lift::setHighSubAlt, lift)
        );

        new Trigger(
                () -> !cancel && gamepad1.y && gamepad1.left_trigger > DEAD_BAND
        ).toggleWhenActive(
                new InstantCommand(effector::setPostScorePosition, effector),
                new InstantCommand(effector::setSubScorePosition, effector)
        );



        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(() -> setCancel(true)),
                        new InstantCommand(() -> setIntakingGround(false)),
                        new InstantCommand(() -> dt.setSlowDrive(false), dt),
                        new InstantCommand(lift::setStow, lift),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setStowArm, effector),
                        new WaitCommand(EndEffector.TIME_ARM_PASSOVER),
                        new InstantCommand(effector::setStowWrist, effector)
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new InstantCommand(effector::toggleClaw, effector)
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
        telemetry.addData("is intaking ground bool: ", isIntakingGround);
    }

    private void setCancel(boolean b) {
        this.cancel = b;
    }

    private void setGroundIntakePosition() {
        if (!isIntakingGround) {
            effector.setGroundIntakePosition();
            isIntakingGround = true;
        } else {
            effector.setGroundIntakeArm();
        }
    }

    private void setIntakingGround(boolean b) {
        this.isIntakingGround = b;
    }
}
