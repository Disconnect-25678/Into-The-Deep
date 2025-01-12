package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(group = "Hardware")
public class HardwareMotorMove extends OpMode {

    private DcMotorEx motor;

    public static DcMotorSimple.Direction DIRECTION = DcMotorSimple.Direction.FORWARD;

    public static double power = 0;

    public static String name = "Lift Left";

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Name: ", name);
    }

    @Override
    public void start(){
        this.motor = hardwareMap.get(DcMotorEx.class, name);
        this.motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.motor.setDirection(DIRECTION);
    }

    @Override
    public void loop() {

        this.motor.setPower(power);

        telemetry.addData("pos: ", motor.getCurrentPosition());
        telemetry.addData("dir: ", motor.getDirection());
    }
}
