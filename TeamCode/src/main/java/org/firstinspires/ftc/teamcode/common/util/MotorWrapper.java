package org.firstinspires.ftc.teamcode.common.util;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class MotorWrapper {
    private double lastSetPower = 0;

    public final DcMotorEx motor;

    public MotorWrapper(DcMotorEx motor){
        this.motor = motor;
    }

    public void setPower(double power) {
        if (Double.compare(power, lastSetPower) == 0) return;
        this.lastSetPower = power;
        this.motor.setPower(power);
    }
}
