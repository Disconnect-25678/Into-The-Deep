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
import com.pedropathing.localization.Pose;
import com.pedropathing.pathgen.BezierLine;
import com.pedropathing.pathgen.PathChain;
import com.pedropathing.pathgen.Point;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.commands.FollowPathCommand;
import org.firstinspires.ftc.teamcode.common.RobotHardware;
import org.firstinspires.ftc.teamcode.common.util.CommandOpModeEx;
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
public class Specimen1Paths extends LinearOpMode {
    public static PublicPose P_START_POSE = new PublicPose(72, -4, (270));
    public static PublicPose P_SCORE_POSE = new PublicPose(40, 6, (270));
    public static PublicPose P_PARK_POSE = new PublicPose(-69, 48, (270));

    private AutoDrivetrain follower;

    private PathChain p_ScorePreload;
    private PathChain p_Park;

    private BulkReader bulkReader;
    private LoopTimer loopTimer;

    @Override
    public void runOpMode() {
        Constants.setConstants(FConstants.class, LConstants.class);
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
        loopTimer = new LoopTimer(telemetry);

        follower = new AutoDrivetrain(hardwareMap);
        bulkReader = new BulkReader(hardwareMap);

        p_ScorePreload = follower.pathBuilder()
                .addPath(new BezierLine(new Point(P_START_POSE.toPedroPose()), new Point(P_SCORE_POSE.toPedroPose())))
                .setLinearHeadingInterpolation(P_START_POSE.toPedroPose().getHeading(), P_SCORE_POSE.toPedroPose().getHeading())
                .build();

        p_Park = follower.pathBuilder()
                .addPath(new BezierLine(new Point(P_SCORE_POSE.toPedroPose()), new Point(P_PARK_POSE.toPedroPose())))
                .setLinearHeadingInterpolation(P_SCORE_POSE.toPedroPose().getHeading(), P_PARK_POSE.toPedroPose().getHeading())
                .build();

        follower.setStartingPose(P_START_POSE.toPedroPose());

        waitForStart();

        CommandScheduler.getInstance().schedule(
                new SequentialCommandGroup(
                        new FollowPathCommand(follower, p_ScorePreload),
                        new WaitCommand(1000),
                        new FollowPathCommand(follower, p_Park)
                )
        );

        while (opModeIsActive() && !isStopRequested()) {
            CommandScheduler.getInstance().run();
            loopTimer.updateLoop();
            follower.update();
            follower.telemetryDebug(this.telemetry);
            telemetry.update();
        }
    }
}
