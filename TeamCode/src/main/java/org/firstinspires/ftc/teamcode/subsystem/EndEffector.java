package org.firstinspires.ftc.teamcode.subsystem;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class EndEffector extends SubsystemBase {
    public static double maxServoPos = 1;
    public static double minServoPos = 0;

    public static double maxWristAngle = 90;
    public static double minWristAngle = -90;

    public static double maxWristServoDelta = 0.25;



    public static double maxTwistAngle = 90;
    public static double minTwistAngle = -90;

    public static double CLAW_OPEN = 0.8;
    public static double CLAW_CLOSE = 0.45;

    public static double ARM_MAX = 1;
    public static double ARM_MIN = 0;

    public static WristTwistAngles ANGLES_INTAKE_GROUND = new WristTwistAngles(0.45, 0.64);
    public static WristTwistAngles ANGLES_INTAKE_REAR = new WristTwistAngles(0.9, 0.27);
    public static WristTwistAngles ANGLES_BUCKET = new WristTwistAngles(0.81, 0.3);
    public static WristTwistAngles ANGLES_SUB = new WristTwistAngles(0.34, 0.18);
    public static WristTwistAngles ANGLES_STOW = new WristTwistAngles(0.4, 0.05);
    public static WristTwistAngles ANGLES_INTAKE_GROUND_MID = new WristTwistAngles(0.3, 0.15);


    public static double ARM_INTAKE_GROUND_MID = 0.23;
    public static double ARM_INTAKE_GROUND = 0.11;
    public static double ARM_INTAKE_REAR = 0.85;
    public static double ARM_BUCKET = 0.75;
    public static double ARM_SUB = 0.54;
    public static double ARM_STOW = 0.35;
    public static double ARM_POST_SUB_SCORE = 0.375;

    public static long TIME_GRAB_SPECIMEN = 500;
    public static long TIME_ARM_PASSOVER = 1500;
    public static long TIME_LIFT_OFF_GROUND_SUB = 900;

    public static long TIME_WAIT_STWIST = 1000;

    private boolean open = true;

    private Servo armLeftServo,
                armRightServo,
                wristLeftServo,
                wristRightServo;

    private Servo clawServo;

    private Telemetry telemetry;

    private int toggleCounter = 1;
    private double toggleAngle = 0;

    private WristTwistAngles wristTwistAnglesTarget = new WristTwistAngles(0, 0);

    public EndEffector(Servo alServo, Servo arServo, Servo wlServo, Servo wrServo, Servo clawServo, Telemetry telemetry) {
        this.armLeftServo = alServo;
        this.armRightServo = arServo;
        this.wristLeftServo = wlServo;
        this.wristRightServo = wrServo;

        this.clawServo = clawServo;
        this.telemetry = telemetry;
    }

    public void setAngles(WristTwistAngles angles) {
//        double avg = (angles.wristAngle - minWristAngle) / (maxWristAngle - minWristAngle) * (maxServoPos - minServoPos) + minServoPos;
//        double diff = angles.twistAngle / maxTwistAngle * (maxWristServoDelta);
//
//        this.wristTwistAnglesTarget = angles;

        toggleCounter = 1;

        wristLeftServo.setPosition(angles.wristAngle);
        wristRightServo.setPosition(angles.twistAngle);
    }

    public void setServoPositions(double left, double right) {
        this.wristLeftServo.setPosition(left);
        this.wristRightServo.setPosition(right);
    }

    public void setArmPosition(double pos) {
        this.armLeftServo.setPosition(pos);
        this.armRightServo.setPosition(pos);
    }

    public void setGroundIntakePosition() {
        this.setAngles(ANGLES_INTAKE_GROUND);
        this.setArmPosition(ARM_INTAKE_GROUND);
    }

    @Deprecated
    public void setRearIntakePosition() {
        this.setAngles(ANGLES_INTAKE_REAR);
        this.setArmPosition(ARM_INTAKE_REAR);
    }

    public void setRearIntakePositionWrist() {
        this.setAngles(ANGLES_INTAKE_REAR);
    }

    public void setRearIntakePositionArm() {
        this.setArmPosition(ARM_INTAKE_REAR);
    }

    public void setSubScorePosition() {
        this.setAngles(ANGLES_SUB);
        this.setArmPosition(ARM_SUB);
    }

    @Deprecated
    public void setBucketPosition() {
        this.setAngles(ANGLES_BUCKET);
        this.setArmPosition(ARM_BUCKET);
    }

    public void setBucketPositionWrist() {
        this.setAngles(ANGLES_BUCKET);
    }

    public void setBucketPositionArm() {
        this.setArmPosition(ARM_BUCKET);
    }

    public void setGroundIntakeMid() {
        this.setAngles(ANGLES_INTAKE_GROUND_MID);
        this.setArmPosition(ARM_INTAKE_GROUND_MID);
    }

    public void setGroundIntakeMidArm() {
        this.setArmPosition(ARM_INTAKE_GROUND_MID);
    }

    public void setPostScorePosition() {
        this.setArmPosition(ARM_POST_SUB_SCORE);
    }

    @Deprecated
    public void setStow() {
        this.setAngles(ANGLES_STOW);
        this.setArmPosition(ARM_STOW);
    }

    public void setStowArm() {
        this.setArmPosition(ARM_STOW);
    }

    public void setStowWrist() {
        this.setAngles(ANGLES_STOW);
    }

    public void toggleTwistRotation() {
        this.toggleAngle += 30;
        if (toggleAngle > maxTwistAngle) this.toggleAngle = minTwistAngle;

        this.wristTwistAnglesTarget = new WristTwistAngles(this.wristTwistAnglesTarget.wristAngle, this.toggleAngle);
        this.setAngles(this.wristTwistAnglesTarget);
    }

    public void resetToggleAngle() {
        this.toggleAngle = 0;
    }

    public void grab() {
        this.clawServo.setPosition(CLAW_CLOSE);
    }

    public void release() {
        this.clawServo.setPosition(CLAW_OPEN);
    }

    public void toggleClaw() {
        if (open) grab();
        else release();
        open = !open;
    }

    @Override
    public void periodic() {
        this.telemetry.addLine("---EndEffector-------------------------------------------");

        this.telemetry.addData("ee-Wrist Target Angle: ", this.wristTwistAnglesTarget.wristAngle);
        this.telemetry.addData("ee-Twist Target Angle: ", this.wristTwistAnglesTarget.twistAngle);

        this.telemetry.addData("ee-Wrist Left Pos: ", this.wristLeftServo.getPosition());

        this.telemetry.addData("ee-Toggle counter: ", this.toggleCounter);

    }

    public static class WristTwistAngles {
        public double wristAngle;
        public double twistAngle;

        public WristTwistAngles(double wristAngle, double twistAngle) {
//            this.wristAngle = MathUtils.clamp(wristAngle, -maxWristAngle, maxWristAngle);
//            this.twistAngle = clampAngles(this.wristAngle, twistAngle)[1];
            this.wristAngle = wristAngle;
            this.twistAngle = twistAngle;
        }

        private double[] clampAngles(double wristAngle, double twistAngle) {
            double slope = maxTwistAngle / maxWristAngle;

            double plusSlopeLarger = slope * wristAngle + maxTwistAngle;
            double plusSlopeSmaller = slope * wristAngle - maxTwistAngle;
            double minusSlopeLarger = -slope * wristAngle + maxTwistAngle;
            double minusSlopeSmaller = -slope * wristAngle - maxTwistAngle;

            return new double[] {wristAngle, MathUtils.clamp(
                    MathUtils.clamp(
                            twistAngle,
                            plusSlopeSmaller,
                            plusSlopeLarger
                    ),
                    minusSlopeSmaller,
                    minusSlopeLarger
            )};
        }


    }
}
