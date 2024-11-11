package org.firstinspires.ftc.teamcode.components;




import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmstartpos;



public class RobotComponents {


    public static DcMotor front_left = null;
    public static DcMotor front_right = null;
    public static DcMotor back_left = null;
    public static DcMotor back_right = null;

    public static DcMotor pivot_motor = null;
    public static Servo pivot_Servo = null;
    public static Servo extendo_servo = null;
    public static CRServo intake_servo = null;

    public static IMU imu;


    public static void init(HardwareMap hardwareMap) {
        // Initialize motors from hardware map
        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        back_left = hardwareMap.get(DcMotor.class, "leftRear");
        back_right = hardwareMap.get(DcMotor.class, "rightRear");
        pivot_Servo = hardwareMap.get(Servo.class, "rodo");
        pivot_motor = hardwareMap.get(DcMotor.class, "pivot");
        extendo_servo = hardwareMap.get(Servo.class, "extendoarm");
        intake_servo = hardwareMap.get(CRServo.class, "eject");

        imu = hardwareMap.get(IMU.class, "imu");

        // flip one half of robot for mecanum drive
        // may swap to left half instead, see how it drives
        front_left.setDirection(DcMotorSimple.Direction.REVERSE);

        // this one would be reversed by both so we leave it running forward, adjust to back_left as needed
        //back_right.setDirection(DcMotorSimple.Direction.REVERSE);

        //flip back motors because their chains are mounted opposite to front
        //may swap to front half instead
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        pivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_motor.setTargetPosition(pivotmstartpos);
        pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_servo.setDirection(DcMotorSimple.Direction.FORWARD);
        pivot_motor.setPower(.8);
    }
}
