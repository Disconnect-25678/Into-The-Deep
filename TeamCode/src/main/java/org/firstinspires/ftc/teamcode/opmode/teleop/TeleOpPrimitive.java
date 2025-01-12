package org.firstinspires.ftc.teamcode.opmode.teleop;

import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;
import org.firstinspires.ftc.teamcode.subsystem.Laterator;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@TeleOp
public class TeleOpPrimitive extends CommandOpModeEx {
    public enum ScoreState{
        BUCKET,
        SUB,
        STOW
    }

    public static double DEAD_BAND = 0.2;
    private Drivetrain dt;
    private Lift lift;
    private Laterator laterator;
    private EndEffector effector;

    private boolean usingHeadingPID = true;

    private boolean cancel;

    private GamepadEx gamepad;

    private ScoreState scoreState = ScoreState.STOW;

    @Override
    public void initialize() {
        super.initialize();

        scoreState = ScoreState.STOW;
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

        gamepad.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new InstantCommand(() -> {
                    setCancel(false);
                    lift.setRearIntakePosition();
                    laterator.setRearIntakePosition();
                    effector.setRearIntakePosition();
                    effector.release();
                }, lift, laterator, effector)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.A).whenPressed(
                new SequentialCommandGroup(
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(() -> {
                            lift.setStow();
                            laterator.stow();
                            effector.setStow();
                            setScoreState(ScoreState.STOW);
                        })
                )
        );

        new Trigger(
                () -> gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER) > 0.2
        ).whenActive(
                new InstantCommand(() -> {
                    lift.setGroundIntakePosition();
                    effector.setGroundIntakePosition();
                    effector.release();
                }, lift, effector)
        ).whileActiveContinuous(
                new RunCommand(() -> laterator.setScaleTarget(gamepad.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER)))
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_LEFT).whenPressed(
                new InstantCommand(() -> {
                    setScoreState(ScoreState.BUCKET);
                    setCancel(false);
                    lift.setLowBucketLevel();
                    laterator.setBucketScorePosition();
                    effector.setBucketPosition();
                    effector.release();
                }, lift, laterator, effector)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                new InstantCommand(() -> {
                    setScoreState(ScoreState.SUB);
                    setCancel(false);
                    effector.grab();
                    lift.setHighSubLevel();
                    laterator.setSubScorePosition();
                    effector.setSubScorePosition();
                }, lift, effector)
        );

        gamepad.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                new InstantCommand(() -> {
                    setScoreState(ScoreState.SUB);
                    setCancel(false);
                    effector.grab();
                    lift.setLowSubLevel();
                    laterator.setSubScorePosition();
                    effector.setSubScorePosition();
                }, lift, effector)
        );

        new Trigger(
                () -> gamepad1.left_bumper && scoreState == ScoreState.BUCKET
        ).whenActive(
                new SequentialCommandGroup(
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(() -> {
                            lift.setStow();
                            laterator.stow();
                            effector.setStow();
                            setScoreState(ScoreState.STOW);
                        })
                )
        );

        new Trigger(
                () -> gamepad1.left_bumper && scoreState == ScoreState.SUB
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
                            setScoreState(ScoreState.STOW);
                        })
                )
        );

        gamepad.getGamepadButton(GamepadKeys.Button.B).whenPressed(
                new InstantCommand(() -> {
                    setCancel(true);
                    setScoreState(ScoreState.STOW);
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
        telemetry.addData("Score State: ", scoreState);
    }

    private void setCancel(boolean b) {
        this.cancel = b;
    }

    private void setScoreState(ScoreState s) {
        this.scoreState = s;
    }
}
