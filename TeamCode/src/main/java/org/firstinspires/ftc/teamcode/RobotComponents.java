package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import static org.firstinspires.ftc.teamcode.OpMode.Constants.tiltstartpos;



public class RobotComponents {

    public static Servo pincer_right = null;
    public static Servo pincer_left = null;
    public static Servo claw_tilt = null;

    public static DcMotor arm_tilt = null;
    public static DcMotor leftMotor;
    public static DcMotor rightMotor;
    public static ColorSensor colorSensor;


    public static void init(HardwareMap hardwareMap) {
        // Initialize motors from hardware map
        leftMotor = hardwareMap.dcMotor.get("left_motor");
        rightMotor = hardwareMap.dcMotor.get("right_motor");
        arm_tilt = hardwareMap.dcMotor.get("tilt");
        pincer_left = hardwareMap.servo.get("pincer left");
        pincer_right = hardwareMap.servo.get("pincer right");
        claw_tilt = hardwareMap.servo.get("claw tilt");

        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");



        // Reverse the direction of the right motor
        rightMotor.setDirection(DcMotor.Direction.REVERSE);

        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_tilt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        arm_tilt.setTargetPosition(tiltstartpos);

    }

}
