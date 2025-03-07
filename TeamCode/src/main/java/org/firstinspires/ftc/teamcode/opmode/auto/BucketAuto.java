package org.firstinspires.ftc.teamcode.opmode.auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.FunctionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.pedropathing.pathgen.BezierCurve;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.LoopTimer;
import org.firstinspires.ftc.teamcode.common.util.PublicPose;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;
import org.firstinspires.ftc.teamcode.subsystem.AutoDrivetrain;
import org.firstinspires.ftc.teamcode.subsystem.BulkReader;
import org.firstinspires.ftc.teamcode.subsystem.EndEffector;
import org.firstinspires.ftc.teamcode.subsystem.Laterator;
import org.firstinspires.ftc.teamcode.subsystem.Lift;

@Config
@Autonomous
public class BucketAuto extends LinearOpMode {
    private AutoDrivetrain follower;
    private Lift lift;
    private EndEffector effector;
    private Laterator laterator;
    private BulkReader bulkReader;
    private LoopTimer loopTimer;

    private PathChain p_Preload, p_Grab1, p_Score1, p_Park;

    public static long TIME_ARM_PASSTHROUGH = 700;
    public static long TIME_ARM_CLEAR_BUCKET = 400;

    public static double SCALE_LATERATOR_EXTEND = 0.99;

    public static PublicPose P_START_POSE = new PublicPose(-72+88, 72-8, 270);
    public static PublicPose P_SCORE_POSE_1 = new PublicPose(59.1, 72-59, 270-45);
    public static PublicPose P_GRAB_POSE = new PublicPose(50.13, 41, 271);
    public static PublicPose P_SCORE_POSE_2 = new PublicPose(60, 58.5, 270-45);
    public static PublicPose P_PARK_POSE = new PublicPose(-72+133, 72-20, 270);

    public static PublicPose P_CONTROL = new PublicPose(-72+102.61828737300435, 72-19.22786647314949, 0);

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        Constants.setConstants(FConstants.class, LConstants.class);
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        loopTimer = new LoopTimer(telemetry);

        follower = new AutoDrivetrain(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);
        RobotHardware.getInstance().initializeLaterator(hardwareMap)
                .initializeLift(hardwareMap)
                .initializeEndEffector(hardwareMap);

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

        p_Preload = follower.pathBuilder()
                        .addPath(
                                new BezierCurve(
                                        P_START_POSE.toPedroPoint(),
                                        P_CONTROL.toPedroPoint(),
                                        P_SCORE_POSE_1.toPedroPoint()
                                )
                        ).setLinearHeadingInterpolation(P_START_POSE.toPedroPose().getHeading(), P_SCORE_POSE_1.toPedroPose().getHeading())
                        .build();

        p_Grab1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                P_SCORE_POSE_1.toPedroPoint(),
                                P_GRAB_POSE.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(P_SCORE_POSE_1.toPedroPose().getHeading(), P_GRAB_POSE.toPedroPose().getHeading())
                .build();

        p_Score1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                P_GRAB_POSE.toPedroPoint(),
                                P_SCORE_POSE_2.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(P_GRAB_POSE.toPedroPose().getHeading(), P_SCORE_POSE_2.toPedroPose().getHeading())
                .build();

        p_Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                P_SCORE_POSE_2.toPedroPoint(),
                                P_PARK_POSE.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(P_SCORE_POSE_2.toPedroPose().getHeading(), P_PARK_POSE.toPedroPose().getHeading())
                .build();

        follower.setStartingPose(P_START_POSE.toPedroPose());

        effector.grab();
        effector.setStow();

        while (opModeInInit() && !isStarted()) {
            telemetry.addLine("ready");
            telemetry.update();
        }

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(lift::setLowBucketLevel, lift),
                                        new WaitUntilCommand(lift::isAtTarget),
                                        new InstantCommand(effector::setBucketPosition, effector),
                                        new WaitCommand(TIME_ARM_PASSTHROUGH)
                                ),
                                new FollowPathCommand(follower, p_Preload)
                        ),
                        new InstantCommand(effector::release),
                        new WaitCommand(EndEffector.TIME_RELEASE_SPECIMEN),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(effector::setStow, effector),
                                        new WaitCommand(TIME_ARM_CLEAR_BUCKET),
                                        new InstantCommand(lift::setStow),
                                        new WaitUntilCommand(lift::isAtTarget)
                                ),
                                new FollowPathCommand(follower, p_Grab1)
                        ),
                        new InstantCommand(effector::setGroundIntakeMid, effector),
                        new InstantCommand(() -> laterator.setScaleTarget(SCALE_LATERATOR_EXTEND), laterator),
                        new WaitUntilCommand(laterator::isAtTarget),
                        new InstantCommand(effector::setGroundIntakePosition, effector),
                        new WaitCommand(EndEffector.TIME_LIFT_OFF_GROUND_SUB),
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new InstantCommand(effector::setGroundIntakeMid, effector),
                        new WaitCommand(EndEffector.TIME_LIFT_OFF_GROUND_SUB),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.isAtTarget(),
                                laterator
                        ),
                        new InstantCommand(effector::setStow, effector),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(lift::setLowBucketLevel, lift),
                                        new WaitUntilCommand(lift::isAtTarget),
                                        new InstantCommand(effector::setBucketPosition, effector),
                                        new WaitCommand(TIME_ARM_PASSTHROUGH)
                                ),
                                new FollowPathCommand(follower, p_Score1)
                        ),
                        new InstantCommand(effector::release),
                        new WaitCommand(EndEffector.TIME_RELEASE_SPECIMEN),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(effector::setStow, effector),
                                        new WaitCommand(TIME_ARM_CLEAR_BUCKET),
                                        new InstantCommand(lift::setStow),
                                        new WaitUntilCommand(lift::isAtTarget)
                                ),
                                new FollowPathCommand(follower, p_Park)
                        )
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            follower.update();
            follower.telemetryDebug(telemetry);
            loopTimer.updateLoop();
            telemetry.update();
        }
    }
}
