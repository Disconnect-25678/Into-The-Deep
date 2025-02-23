package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
import org.firstinspires.ftc.teamcode.subsystem.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;
import org.firstinspires.ftc.teamcode.subsystem.Laterator;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@Autonomous
public class Specimen1 extends CommandOpModeEx {
    public static Pose P_START_POSE = new Pose(0, 68, Math.toRadians(0));
    public static Pose P_SCORE_POSE = new Pose(32, 78, 0);
    public static Pose P_PARK_POSE = new Pose(3, 24, 0);

    private AutoDrivetrain follower;
    private Lift lift;
    private Laterator laterator;
    private EndEffector effector;

    private PathChain p_ScorePreload;
    private PathChain p_Park;

    @Override
    public void initialize() {
        super.initialize();
        RobotHardware.getInstance().initializeLift(hardwareMap);
        RobotHardware.getInstance().initializeEndEffector(hardwareMap);
        RobotHardware.getInstance().initializeLaterator(hardwareMap);

        follower = new AutoDrivetrain(hardwareMap);

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

        effector.grab();

        p_ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(P_START_POSE), new Point(P_SCORE_POSE)))
                .setLinearHeadingInterpolation(P_START_POSE.getHeading(), P_SCORE_POSE.getHeading())
                .build();

        p_Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(P_SCORE_POSE), new Point(P_PARK_POSE)))
                .setLinearHeadingInterpolation(P_SCORE_POSE.getHeading(), P_PARK_POSE.getHeading())
                .build();

        schedule(
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, p_ScorePreload),
                        new SequentialCommandGroup(
                                new FunctionalCommand(
                                        () -> lift.setHighSubLevel(),
                                        () -> {},
                                        (interrupted) -> {},
                                        () -> lift.isAtTarget(),
                                        lift
                                ),
                                new InstantCommand(effector::setSubScorePosition, effector),
                                new FunctionalCommand(
                                        () -> laterator.setSubScorePosition(),
                                        () -> {},
                                        (interrupted) -> {},
                                        () -> laterator.isAtTarget(),
                                        laterator
                                )
                        )
                ),
                new FunctionalCommand(
                        () -> lift.setPostScorePosition(),
                        () -> {},
                        (interrupted) -> {effector.release();},
                        () -> lift.isAtTarget(),
                        lift, effector
                ),
                new ParallelCommandGroup(
                        new FollowPathCommand(follower, p_Park),
                        new SequentialCommandGroup(
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
                )
        );
    }

    @Override
    public void run() {
        super.run();
        follower.telemetryDebug(telemetry);
    }
}
