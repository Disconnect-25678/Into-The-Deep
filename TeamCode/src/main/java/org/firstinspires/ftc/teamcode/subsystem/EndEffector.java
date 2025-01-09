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

    public static double maxTwistAngle = 90;
    public static double minTwistAngle = -90;

    public static double CLAW_OPEN = 0.3;
    public static double CLAW_CLOSE = 0.6;

    public static WristTwistAngles ANGLES_INTAKE_GROUND = new WristTwistAngles(0, 0);
    public static WristTwistAngles ANGLES_INTAKE_REAR = new WristTwistAngles(0, 0);
    public static WristTwistAngles ANGLES_SUB = new WristTwistAngles(0, 0);
    public static WristTwistAngles ANGLES_STOW = new WristTwistAngles(0, 0);

    public static double ARM_INTAKE_GROUND = 0;
    public static double ARM_INTAKE_REAR = 0.2;
    public static double ARM_SUB = 0.3;
    public static double ARM_STOW = 0.1;


    private Servo armLeftServo,
                armRightServo,
                wristLeftServo,
                wristRightServo;

    private Servo clawServo;

    private Telemetry telemetry;

    private int toggleCounter = 1;

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
        double avg = angles.wristAngle / (maxWristAngle - minWristAngle) * (maxServoPos - minServoPos) + minServoPos;
        double diff = angles.twistAngle / maxTwistAngle * ((maxServoPos - minServoPos) / 2);

        this.wristTwistAnglesTarget = angles;

        toggleCounter = 1;

        wristLeftServo.setPosition(avg + diff);
        wristRightServo.setPosition(avg - diff);
    }

    public void setServoPositions(double left, double right) {
        this.wristRightServo.setPosition(left);
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

    public void setRearIntakePosition() {
        this.setAngles(ANGLES_INTAKE_REAR);
        this.setArmPosition(ARM_INTAKE_REAR);
    }

    public void setSubScorePosition() {
        this.setAngles(ANGLES_SUB);
        this.setArmPosition(ARM_SUB);
    }

    public void setSubScorePositionAlt() {

    }

    public void setStow() {
        this.setAngles(ANGLES_STOW);
        this.setArmPosition(ARM_STOW);
    }

    public void toggleTwistRotation() {
        double angle;
        switch (toggleCounter % 4) {
            case 0: angle = 0; break;
            case 1: angle = 90; break;
            case 2: angle = 180; break;
            default: angle = -90; break;
        }
        toggleCounter++;
        this.wristTwistAnglesTarget = new WristTwistAngles(this.wristTwistAnglesTarget.wristAngle, angle);
        this.setAngles(this.wristTwistAnglesTarget);
    }

    public void grab() {
        this.clawServo.setPosition(CLAW_CLOSE);
    }

    public void release() {
        this.clawServo.setPosition(CLAW_OPEN);
    }

    @Override
    public void periodic() {
        this.telemetry.addLine("---EndEffector-------------------------------------------");

        this.telemetry.addData("Wrist Target Angle: ", this.wristTwistAnglesTarget.wristAngle);
        this.telemetry.addData("Twist Target Angle: ", this.wristTwistAnglesTarget.twistAngle);

        this.telemetry.addData("Wrist Left Pos: ", this.wristLeftServo.getPosition());

        this.telemetry.addData("Toggle counter: ", this.toggleCounter);

    }

    public static class WristTwistAngles {
        public final double wristAngle;
        public final double twistAngle;

        public WristTwistAngles(double wristAngle, double twistAngle) {
            this.wristAngle = MathUtils.clamp(wristAngle, -maxWristAngle, maxWristAngle);
            this.twistAngle = clampAngles(this.wristAngle, twistAngle)[1];
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
