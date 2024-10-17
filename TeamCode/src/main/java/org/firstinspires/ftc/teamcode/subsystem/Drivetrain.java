package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;

@Config
public class Drivetrain extends SubsystemBase {
    public static double DRIVETRAIN_MAX_ROTATION = 0.75;
    public static double DRIVETRAIN_MAX_DRIVE_POWER = 1;

    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double DRIVETRAIN_OMEGA_THRESHOLD = 1;

    public static double DRIVETRAIN_HEADING_TOLERANCE = 1.5;

    private final PIDController headingPIDController = new PIDController(kP, kI, kD);

    private final MotorWrapper[] motors;

    private final IMU imu;

    private final Telemetry telemetry;

    private double yaw;
    private double targetHeadingDegrees;
    private double rotationRate;

    private boolean inLeftOverRotation;

    public Drivetrain(DcMotorEx[] motors, IMU imu, Telemetry telemetry) {
        super();
        this.motors = new MotorWrapper[motors.length];

        for (int i = 0; i < this.motors.length; i++) {
            this.motors[i] = new MotorWrapper(motors[i]);
        }

        this.imu = imu;
        this.telemetry = telemetry;
    }

    public void drive(Vector2d driveStickVector, double rotation) {
        double[] vals = Algorithms.returnMecanumValues(driveStickVector.rotateBy(-this.getYaw()), rotation * DRIVETRAIN_MAX_ROTATION, DRIVETRAIN_MAX_DRIVE_POWER);
        for (int i = 0; i < vals.length; i++) {
            this.motors[i].setPower(vals[i]);
        }
    }

    public void driveFacingAngle(Vector2d driveStickVector, double rotation) {
        if (Double.compare(0, rotation) != 0) {
            drive(driveStickVector, rotation);
            this.inLeftOverRotation = true;
            return;
        } else if (inLeftOverRotation) {
            drive(driveStickVector, 0);
            this.rotationRate = imu.getRobotAngularVelocity(AngleUnit.DEGREES).yRotationRate;
            if (this.rotationRate < DRIVETRAIN_OMEGA_THRESHOLD) {
                this.inLeftOverRotation = false;
                this.targetHeadingDegrees = this.getYaw();
            }
            return;
        }
        double heading = this.getYaw();
        double error = this.targetHeadingDegrees - this.getYaw();
        if (error > 180) {
            heading -= 360;
            error -= 360;
        }
        else if (error < -180) {
            heading += 360;
            error += 360;
        }

        drive(driveStickVector,
                (Math.abs(error) < DRIVETRAIN_HEADING_TOLERANCE) ?
                        0 : this.headingPIDController.calculate(heading, this.targetHeadingDegrees));

    }

    public double getYaw() {
        return this.yaw;
    }

    public void resetYaw() {
        this.imu.resetYaw();
    }

    @Override
    public void periodic(){
        this.yaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

        telemetry.addLine("\nDT------------");
        telemetry.addData("Heading/Yaw: ", this.yaw);
        telemetry.addData("Target Heading/Yaw: ", this.targetHeadingDegrees);
        telemetry.addData("Rotation Rate: ", this.rotationRate);
        telemetry.addLine((this.inLeftOverRotation) ? "No HeadingPID" : "Heading PID");

        this.headingPIDController.setPID(kP, kI, kD);
    }
}
