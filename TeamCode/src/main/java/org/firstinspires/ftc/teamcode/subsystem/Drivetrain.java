package org.firstinspires.ftc.teamcode.subsystem;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.common.util.Algorithms;
import org.firstinspires.ftc.teamcode.common.util.MotorWrapper;

@Config
public class Drivetrain {
    public static double DRIVETRAIN_MAX_ROTATION = 0.75;
    public static double DRIVETRAIN_MAX_DRIVE_POWER = 1;

    private final MotorWrapper[] motors;

    private final IMU imu;

    private final Telemetry telemetry;

    public Drivetrain(DcMotorEx[] motors, IMU imu, Telemetry telemetry) {
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

    public double getYaw() {
        YawPitchRollAngles mecanumOrientation = imu.getRobotYawPitchRollAngles();
        return mecanumOrientation.getYaw(AngleUnit.DEGREES);
    }

    public void resetYaw() {
        this.imu.resetYaw();
    }
}
