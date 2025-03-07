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
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.commands.LiftTargetCommand;
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
public class SpecimenMain extends LinearOpMode {
    public static PublicPose P_START_POSE = new PublicPose(-72+68, 72-0, (270));
    public static PublicPose P_SCORE_POSE = new PublicPose(-72+78, 43, (270));
    public static PublicPose P_PARK_POSE = new PublicPose(-72+24, 72-5, (270));
    public static PublicPose P_SAMPLE_1 = new PublicPose(-72+32.60377358490565, 72-10, 276);
    public static PublicPose P_SAMPLE_2 = new PublicPose(-72+22.780841799709727, 66, 276);
    public static PublicPose P_SCORE_POSE_2 = new PublicPose(-72+78, 72-29, 276);
    public static PublicPose P_SPEC_1 = new PublicPose(-72+22.780841799709727, 66, 276);
    public static PublicPose P_SCORE_POSE_3= new PublicPose(-72+79, 72-32, 276);


    public static double startTheta = 270;
    public static double endTheta = 274;

    public static double startTheta2 = 274;
    public static double endTheta2 = 274;


    public static int POS_LIFT_SCORE_READY_MOVE = 1100;

    public static PublicPose C_CONTROL_1 = new PublicPose(-72+25.497822931785198, 72 - 28.841799709724235, 0);
    public static PublicPose C_CONTROL_2 = new PublicPose(-72+46.8156748911466, 72-1.4629898403483308, 0);
    public static PublicPose C_CONTROL_3 = new PublicPose(-72+55.59361393323658, 72-75.23947750362846, 0);
    public static PublicPose C_CONTROL_4 = new PublicPose(-72+34.484760522496366, 72-96, 0);
    public static PublicPose C_CONTROL_5 = new PublicPose(-72+29.677793904208997, 72-56.847605224963715, 0);

    public static PublicPose C_CONTROL_B1 = new PublicPose(-72+34.275761973875184, 72 - 43.88969521044992, 0);
    public static PublicPose C_CONTROL_B2 = new PublicPose(-72+21.94484760522497, 72-39.50072568940493, 0);
    public static PublicPose C_CONTROL_B3 = new PublicPose(-72+48.69666182873731, 72-63.535558780841804, 0);
    public static PublicPose C_CONTROL_B4 = new PublicPose(-72+16.51088534107403, 72-91.75036284470247, 0);
    public static PublicPose C_CONTROL_B5 = new PublicPose(-72+23.61683599419448, 72-31.97677793904209, 0);


    private AutoDrivetrain follower;

    private PathChain p_ScorePreload;
    private PathChain p_Samp1, p_Samp2;
    private PathChain p_Score_1, p_Score_2;
    private PathChain p_Collect_1;
    private PathChain p_Park;

    private BulkReader bulkReader;
    private LoopTimer loopTimer;

    private Lift lift;
    private Laterator laterator;
    private EndEffector effector;

    private boolean hasStarted = false;

    @Override
    public void runOpMode() {
        CommandScheduler.getInstance().reset();
        Constants.setConstants(FConstants.class, LConstants.class);
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        loopTimer = new LoopTimer(telemetry);

        hasStarted = false;

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

        p_ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(P_START_POSE.toPedroPose()), new Point(P_SCORE_POSE.toPedroPose())))
                .setLinearHeadingInterpolation(P_START_POSE.toPedroPose().getHeading(), P_SCORE_POSE.toPedroPose().getHeading())
                .build();

        p_Samp1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                P_SCORE_POSE.toPedroPoint(),
                                C_CONTROL_1.toPedroPoint(),
                                C_CONTROL_2.toPedroPoint(),
                                C_CONTROL_3.toPedroPoint(),
                                C_CONTROL_4.toPedroPoint(),
                                C_CONTROL_5.toPedroPoint(),
                                P_SAMPLE_1.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(startTheta), Math.toRadians(endTheta))
                .build();
        p_Samp2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                P_SAMPLE_1.toPedroPoint(),
                                C_CONTROL_B1.toPedroPoint(),
                                C_CONTROL_B2.toPedroPoint(),
                                C_CONTROL_B3.toPedroPoint(),
                                C_CONTROL_B4.toPedroPoint(),
                                C_CONTROL_B5.toPedroPoint(),
                                P_SAMPLE_2.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(startTheta2), Math.toRadians(endTheta2))
                .build();

        p_Score_1 = follower.pathBuilder()
                .addPath(
                       new BezierLine(
                               P_SAMPLE_1.toPedroPoint(),
                               P_SCORE_POSE_2.toPedroPoint()
                       )
                ).setLinearHeadingInterpolation(P_SAMPLE_1.toPedroPose().getHeading(), P_SCORE_POSE_2.toPedroPose().getHeading())
                .build();

        p_Collect_1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                P_SCORE_POSE_2.toPedroPoint(),
                                P_SPEC_1.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(P_SAMPLE_2.toPedroPose().getHeading(), P_SCORE_POSE_2.toPedroPose().getHeading())
                .build();

        p_Score_2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                P_SPEC_1.toPedroPoint(),
                                P_SCORE_POSE_3.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(P_SPEC_1.toPedroPose().getHeading(), P_SCORE_POSE_3.toPedroPose().getHeading())
                .build();

        p_Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                P_SCORE_POSE_3.toPedroPoint(),
                                P_PARK_POSE.toPedroPoint()
                        )
                ).setLinearHeadingInterpolation(P_SCORE_POSE_3.toPedroPose().getHeading(), P_PARK_POSE.toPedroPose().getHeading())
                .build();




        follower.setStartingPose(P_START_POSE.toPedroPose());

        effector.setStow();
        effector.grab();


        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p_ScorePreload),
                                new InstantCommand(() -> {
                                    effector.setSubScorePosition();
                                    setHasStarted();
                                }, effector),
                                new SequentialCommandGroup(
                                        new LiftTargetCommand(lift, Lift.POS_SUB_HIGH, POS_LIFT_SCORE_READY_MOVE, true),
                                        new FunctionalCommand(
                                                () -> laterator.setSubScorePosition(),
                                                () -> {},
                                                (interrupted) -> {},
                                                () -> laterator.atSubScoreThreshold(),
                                                laterator
                                        )
                                )

                        ),
                        new WaitCommand(PreloadPark.timeWait),
                        new FunctionalCommand(
                                () -> lift.setPostScorePosition(),
                                () -> {},
                                (interrupted) -> {},
                                () -> lift.isAtTarget(),
                                lift
                        ),
                        new WaitCommand(PreloadPark.timeScoreSettle),
                        new InstantCommand(effector::release, effector),
                        new WaitCommand(EndEffector.TIME_RELEASE_SPECIMEN),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.atSubRetractThreshold(),
                                laterator
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(lift::setStow, lift),
                                        new InstantCommand(effector::setRearIntakePosition)
                                ),
                                new FollowPathCommand(follower, p_Samp1)
                        ),
//                        new FollowPathCommand(follower, p_Samp2),
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p_Score_1),
                                new SequentialCommandGroup(
                                        new InstantCommand(effector::setStowWrist),
                                        new WaitCommand(EndEffector.TIME_WAIT_STWIST),
                                        new InstantCommand(effector::setStowArm)
                                ),
                                new SequentialCommandGroup(
                                        new LiftTargetCommand(lift, Lift.POS_SUB_HIGH, POS_LIFT_SCORE_READY_MOVE, true),
                                        new InstantCommand(() -> {
                                            effector.setSubScorePosition();
                                        }, effector),
                                        new FunctionalCommand(
                                                () -> laterator.setSubScorePosition(),
                                                () -> {},
                                                (interrupted) -> {},
                                                () -> laterator.atSubScoreThreshold(),
                                                laterator
                                        )
                                )
                        ),
                        new WaitCommand(PreloadPark.timeWait),
                        new FunctionalCommand(
                                () -> lift.setPostScorePosition(),
                                () -> {},
                                (interrupted) -> {},
                                () -> lift.isAtTarget(),
                                lift
                        ),
                        new WaitCommand(PreloadPark.timeScoreSettle),
                        new InstantCommand(effector::release, effector),
                        new WaitCommand(EndEffector.TIME_RELEASE_SPECIMEN),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.atSubRetractThreshold(),
                                laterator
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(lift::setStow, lift),
                                        new InstantCommand(effector::setRearIntakePosition)
                                ),
                                new FollowPathCommand(follower, p_Collect_1)
                        ),
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(lift::setStow, lift),
                                        new InstantCommand(effector::setRearIntakePosition)
                                ),
                                new FollowPathCommand(follower, p_Samp1)
                        ),
                        new FollowPathCommand(follower, p_Samp2),
                        new InstantCommand(effector::grab, effector),
                        new WaitCommand(EndEffector.TIME_GRAB_SPECIMEN),
                        new ParallelCommandGroup(
                                new FollowPathCommand(follower, p_Score_2),
                                new SequentialCommandGroup(
                                        new InstantCommand(effector::setStowWrist),
                                        new WaitCommand(EndEffector.TIME_WAIT_STWIST),
                                        new InstantCommand(effector::setStowArm)
                                ),
                                new SequentialCommandGroup(
                                        new LiftTargetCommand(lift, Lift.POS_SUB_HIGH, POS_LIFT_SCORE_READY_MOVE, true),
                                        new InstantCommand(() -> {
                                            effector.setSubScorePosition();
                                        }, effector),
                                        new FunctionalCommand(
                                                () -> laterator.setSubScorePosition(),
                                                () -> {},
                                                (interrupted) -> {},
                                                () -> laterator.atSubScoreThreshold(),
                                                laterator
                                        )
                                )
                        ),
                        new WaitCommand(PreloadPark.timeWait),
                        new FunctionalCommand(
                                () -> lift.setPostScorePosition(),
                                () -> {},
                                (interrupted) -> {},
                                () -> lift.isAtTarget(),
                                lift
                        ),
                        new WaitCommand(PreloadPark.timeScoreSettle),
                        new InstantCommand(effector::release, effector),
                        new WaitCommand(EndEffector.TIME_RELEASE_SPECIMEN),
                        new FunctionalCommand(
                                () -> laterator.stow(),
                                () -> {},
                                (interrupted) -> {},
                                () -> laterator.atSubRetractThreshold(),
                                laterator
                        ),
                        new ParallelCommandGroup(
                                new SequentialCommandGroup(
                                        new InstantCommand(lift::setStow, lift),
                                        new InstantCommand(effector::setStow, effector)
                                ),
                                new FollowPathCommand(follower, p_Park, false)
                        )
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            loopTimer.updateLoop();
            follower.update();
            if (hasStarted) follower.telemetryDebug(this.telemetry);
            telemetry.update();
        }
    }

    private void setHasStarted() {
        hasStarted = true;
    }
}
