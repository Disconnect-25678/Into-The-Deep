package org.firstinspires.ftc.teamcode.opmode.hardwareutil;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(group = "Hardware")
public class HardwareDualServo extends OpMode {
    private Servo servo;
    private Servo servo2;
    public static double pos = 0;
    public static double pos2 = 0;
    public static Servo.Direction DIRECTION = Servo.Direction.FORWARD;
    public static Servo.Direction DIRECTION2 = Servo.Direction.FORWARD;
    public static String str = "Wrist Left";
    public static String str2 = "Wrist Right";

    @Override
    public void init() {
        this.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), this.telemetry);
    }

    @Override
    public void init_loop(){
        telemetry.addData("Str: ", str);
        telemetry.addData("Str2: ", str2);
    }

    @Override
    public void start(){
        this.servo = this.hardwareMap.get(Servo.class, str);
        this.servo2 = this.hardwareMap.get(Servo.class, str2);

        this.servo.setDirection(DIRECTION);
        this.servo2.setDirection(DIRECTION2);
    }

    @Override
    public void loop() {


        this.servo.setPosition(pos);
        this.servo2.setPosition(pos2);

        telemetry.addData("Dir: ", this.servo.getDirection());
        telemetry.addData("Dir2: ", this.servo2.getDirection());
    }
}
