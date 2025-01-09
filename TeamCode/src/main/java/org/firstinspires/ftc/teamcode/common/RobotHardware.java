package org.firstinspires.ftc.teamcode.common;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

public class RobotHardware {
    public DcMotorEx[] driveMotors;
    public IMU imu;

    public DcMotorEx leftLiftMotor;
    public DcMotorEx rightLiftMotor;

    public DcMotorEx lateratorMotor;

    public Servo armLeft, armRight;
    public Servo wristLeft, wristRight;
    public Servo clawServo;

    private static RobotHardware instance = null;

    private RobotHardware(){}

    /**
     * Initializes all hardware on the robot (This includes EVERYTHING)
     * @param HardwareMap hardwareMap
     * @return this object with hardware initialized for convenience instead of calling the method after creating the object
     */
    public RobotHardware initialize(HardwareMap hardwareMap) {
        this.initializeDrivetrain(hardwareMap);
        this.initializeLift(hardwareMap);
        this.initializeLaterator(hardwareMap);
        this.initializeEndEffector(hardwareMap);
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

    public RobotHardware initializeLaterator(HardwareMap hardwareMap) {
        this.lateratorMotor = hardwareMap.get(DcMotorEx.class, "Laterator");
        this.lateratorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.lateratorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.lateratorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        return this;
    }

    public RobotHardware initializeEndEffector(HardwareMap hardwareMap) {
        this.armLeft = hardwareMap.get(Servo.class, "Arm left");
        this.armRight = hardwareMap.get(Servo.class, "Arm Right");

        this.armLeft.setDirection(Servo.Direction.FORWARD);
        this.armRight.setDirection(Servo.Direction.REVERSE);

        this.wristLeft = hardwareMap.get(Servo.class, "Wrist Left");
        this.wristRight = hardwareMap.get(Servo.class, "Wrist Right");

        this.wristLeft.setDirection(Servo.Direction.FORWARD);
        this.wristRight.setDirection(Servo.Direction.REVERSE);

        return this;
    }

    public void kill() {
        instance = null;
    }

    public static RobotHardware getInstance() {
        if (instance != null) instance = new RobotHardware();
        return instance;
    }
}
