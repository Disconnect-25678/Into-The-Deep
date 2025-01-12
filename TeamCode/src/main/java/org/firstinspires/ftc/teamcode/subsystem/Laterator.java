package org.firstinspires.ftc.teamcode.subsystem;

import androidx.core.math.MathUtils;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class Laterator extends SubsystemBase {
    public static int MIN = 0;
    public static int MAX = 720;
    public static int TICK_PER_REQ = 250;
    public static double RESET_SPEED = -0.4;

    public static int POS_SUB = 570;
    public static int POS_REAR_INTAKE = 0;
    public static int POS_BUCKET = 0;

    public static double maxPower = 0.4;

    public static double TOLERANCE = 30;

    public static double
        kP = 0.03,
        kI = 0,
        kD = 0.0004;

    public static double kF = 0;

    public static int POS_STOW = 0;

    private final DcMotorEx leftMotor;

    private final PIDController controller;

    private int target, currentPosition;

    private boolean resetting = false;
    private boolean triggerMoving = false;

    private final Telemetry telemetry;

    public Laterator(DcMotorEx leftMotor, Telemetry telemetry){
        super();

        this.leftMotor = leftMotor;
        this.telemetry = telemetry;

        this.controller = new PIDController(kP, kI, kD);
    }

    public int getCurrentPosition(){
        return this.leftMotor.getCurrentPosition();
    }

    public int getTargetPosition(){
        return this.target;
    }

    public boolean isAtTarget() {
        return Math.abs(this.getCurrentPosition() - this.getTargetPosition()) <= TOLERANCE;
    }

    public void doResetMovement(){
        this.resetting = true;
        this.setPower(RESET_SPEED);
    }

    public void reset(){
        this.setPower(0);
        this.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.leftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.target = MIN;
        controller.reset();
        this.resetting = false;
    }

    public void setTarget(int target) {
        this.target = MathUtils.clamp(target, MIN, MAX);
    }

    public void move(double du){
        if (Double.compare(0, du) != 0) {
            int add = (int) (Math.round(du * TICK_PER_REQ));
            if (this.target + add > MAX) this.target = MAX;
            else if (this.target + add < MIN)
                this.target = MIN;
            else this.target += add;
        }
    }

    public void moveManually(double speed){
        if (this.currentPosition < MAX && this.currentPosition > MIN){
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

    public void setScaleTarget(double d) {
        double ds = MathUtils.clamp(d, 0d, 1d);
        this.setTarget((int)(ds * (MAX - MIN) + MIN));
    }

    public void setSubScorePosition() {
        this.setTarget(POS_SUB);
    }

    public void setBucketScorePosition() {
        this.setTarget(POS_BUCKET);
    }

    public void setRearIntakePosition() {
        this.setTarget(POS_REAR_INTAKE);
    }

    public void stow() {
        this.setTarget(POS_STOW);
    }

    private void setPower(double power){
        power = MathUtils.clamp(power, -maxPower, maxPower);
        this.leftMotor.setPower(power);
        this.telemetry.addData("Laterator power: ", power);
    }

    @Override
    public void periodic(){
        telemetry.addLine("\n--Laterator---------------------------------");
        this.currentPosition = this.leftMotor.getCurrentPosition();

        this.controller.setPID(kP, kI, kD);

        if (!resetting && !triggerMoving){
            double power = controller.calculate(this.currentPosition, this.target);
            this.setPower(power + kF);
        }

        telemetry.addData("laterator-current position: ", this.currentPosition);

        telemetry.addData("laterator-triggerMoving?: ", triggerMoving);
        this.telemetry.addData("laterator-target: ", this.target);
        telemetry.addData("laterator-at target: ", this.isAtTarget());
    }

}
