package org.firstinspires.ftc.teamcode.subsystem;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Lift extends SubsystemBase {
    public static int GROUND_POSITION = 0;
    public static int MAX = 3000;
    public static int TICK_PER_REQ = 250;
    public static double RESET_SPEED = -0.3;
    public static int TOLERANCE = 30;

    public static int POS_STOW = 0;
    public static int POS_INTAKE_GROUND = 200;
    public static int POS_INTAKE_REAR = 200;
    public static int POS_SUB_LOW = 400;
    public static int POS_SUB_HIGH = 500;
    public static int POS_BUCKET_LOW = 600;
    public static int POS_BUCKET_HIGH = 700;

    public static int SCORE_DELTA = 100;

    public static double
        kP = 0.025,
        kI = 0,
        kD = 0.0004;

    public static double kF = 0;

    private final DcMotorEx leftMotor;
    private final DcMotorEx rightMotor;

    private final PIDController controller;

    private int target, currentPosition;

    private boolean resetting = false;
    private boolean triggerMoving = false;

    private final Telemetry telemetry;

    public Lift(DcMotorEx leftMotor, DcMotorEx rightMotor, Telemetry telemetry){
        super();

        this.leftMotor = leftMotor;
        this.rightMotor = rightMotor;
        this.telemetry = telemetry;

        this.controller = new PIDController(kP, kI, kD);
    }

    public int getCurrentPosition(){
        return this.leftMotor.getCurrentPosition();
    }

    public int getTargetPosition(){
        return this.target;
    }

    public void doResetMovement(){
        this.resetting = true;
        this.setPower(RESET_SPEED);
    }

    public void reset(){
        this.setPower(0);
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.target = GROUND_POSITION;
        controller.reset();
        this.resetting = false;
    }

    public void setTarget(int target){
        this.target = MathUtils.clamp(target, GROUND_POSITION, MAX);
    }

    public void move(double du){
        if (Double.compare(0, du) != 0) {
            int add = (int) (Math.round(du * TICK_PER_REQ));
            if (this.target + add > MAX) this.target = MAX;
            else if (this.target + add < GROUND_POSITION)
                this.target = GROUND_POSITION;
            else this.target += add;
        }
    }

    public void moveManually(double speed){
        if (this.currentPosition < MAX && this.currentPosition > GROUND_POSITION){
            triggerMoving = true;
            this.setPower(speed);
        } else {
            triggerMoving = false;
        }
    }

    public void stopMovement(){
        triggerMoving = false;
        this.target = currentPosition;
    }

    public boolean isAtTarget() {
        return Math.abs(this.getCurrentPosition() - this.getTargetPosition()) <= TOLERANCE;
    }

    public void setGroundIntakePosition() {
        this.setTarget(POS_INTAKE_GROUND);
    }

    public void setRearIntakePosition() {
        this.setTarget(POS_INTAKE_REAR);
    }

    public void setHighSubLevel() {
        this.setTarget(POS_SUB_HIGH);
    }

    public void setLowSubLevel() {
        this.setTarget(POS_SUB_LOW);
    }

    public void setHighBucketLevel() {
        this.setTarget(POS_BUCKET_HIGH);
    }

    public void setLowBucketLevel() {
        this.setTarget(POS_BUCKET_LOW);
    }

    public void setStow() {
        this.setTarget(POS_STOW);
    }

    public void setPostScorePosition() {
        this.setTarget(this.getTargetPosition() + SCORE_DELTA);
    }

    private void setPower(double power){
        this.leftMotor.setPower(power);
        this.rightMotor.setPower(power);
        this.telemetry.addData("lift power: ", power);
    }

    @Override
    public void periodic(){
        telemetry.addLine("\n--Lift---------------------------------");
        this.currentPosition = this.leftMotor.getCurrentPosition();

        this.controller.setPID(kP, kI, kD);

        if (!resetting && !triggerMoving){
            double power = controller.calculate(this.currentPosition, this.target);
            this.setPower(power + kF);
        }

        telemetry.addData("current position: ", this.currentPosition);

        telemetry.addData("triggerMoving?: ", triggerMoving);
        this.telemetry.addData("target: ", this.target);
    }

}
