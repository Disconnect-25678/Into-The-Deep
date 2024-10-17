package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

public class RobotHardware {
    public DcMotorEx[] driveMotors;
    public IMU imu;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    private static RobotHardware instance = null;

    private RobotHardware(){}

    /**
     * Initializes all hardware on the robot (This includes EVERYTHING)
     * @param hardwareMap
     * @return this object with hardware initialized for convenience instead of calling the method after creating the object
     */
    public RobotHardware initialize(HardwareMap hardwareMap) {
        this.initializeDrivetrain(hardwareMap);
        this.initializeLift(hardwareMap);
        return this;
    }

    public RobotHardware initializeDrivetrain(HardwareMap hardwareMap) {
        this.driveMotors = new DcMotorEx[4];
        driveMotors[0] = hardwareMap.get(DcMotorEx.class, "FL");
        driveMotors[1] = hardwareMap.get(DcMotorEx.class, "FR");
        driveMotors[2] = hardwareMap.get(DcMotorEx.class, "BL");
        driveMotors[3] = hardwareMap.get(DcMotorEx.class, "BR");

        for (int i = 0; i < driveMotors.length; i++) {
            driveMotors[i].setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            driveMotors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            if (i < 2) {
                driveMotors[i].setDirection(DcMotorSimple.Direction.FORWARD);
            } else {
                driveMotors[i].setDirection(DcMotorSimple.Direction.REVERSE);
            }
        }

        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(
                new IMU.Parameters(
                        new RevHubOrientationOnRobot(
                                RevHubOrientationOnRobot.LogoFacingDirection.LEFT,
                                RevHubOrientationOnRobot.UsbFacingDirection.UP
                        )
                )
        );
        imu.resetYaw();
        return this;
    }

    public RobotHardware initializeLift(HardwareMap hardwareMap) {
        this.leftLiftMotor = hardwareMap.get(DcMotorEx.class, "Lift Left");
        this.rightLiftMotor = hardwareMap.get(DcMotorEx.class, "Lift Right");

        this.leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.leftLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.rightLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.leftLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.rightLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        return this;
    }

    public static RobotHardware getInstance() {
        if (instance != null) instance = new RobotHardware();
        return instance;
    }
}
