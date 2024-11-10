package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.excutil.Input;

@Autonomous(name="Nerdvana Auto", group="idk")
public class NerdvanaAuto extends OpMode {

    public static DcMotor
            front_left, front_right,
            back_left, back_right;

    public static IMU
            imu;

    Input input = new Input();

    DcMotor pivot_motor;
    Servo pivot_Servo;
    Servo extendo_servo;
    CRServo intake_servo;

    //pivot motor values
    int pivotmstartpos = 0;
    int pivotmdrivepos =60;
    int pivotmpickuppos = 360;
    int pivotmlowbucket = 1750;
    int pivotmhighbucket = 2187;
    int pivotmlowchamber = 870;
    int pivotmhighchamber = 1600;
    int pivotmclimbpos = 3300;

    //extendo positions
    int extendostartpos = 1;
    int extendoscorepos = 0;

    //Rodo-Intake positions
    int intakeleftpos = 1;
    double intakecenterpos = .5;
    int intakerightpos = 0;

    //Flags
    boolean initPositionsReached = false;
    boolean climbPositionReached = false;
    private double startTime;

    @Override
    public void init () {
        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        back_left = hardwareMap.get(DcMotor.class, "leftRear");
        back_right = hardwareMap.get(DcMotor.class, "rightRear");
        pivot_Servo = hardwareMap.get(Servo.class, "rodo");
        pivot_motor = hardwareMap.get(DcMotor.class, "pivot");
        extendo_servo = hardwareMap.get(Servo.class, "extendoarm");
        intake_servo = hardwareMap.get(CRServo.class,"eject");

        front_left.setDirection(DcMotorSimple.Direction.REVERSE);
        back_left.setDirection(DcMotorSimple.Direction.REVERSE);

        pivot_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pivot_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_motor.setTargetPosition(pivotmstartpos);
        pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_servo.setDirection(DcMotorSimple.Direction.FORWARD);




    }

    public void setStartTime() {
        this.startTime = getRuntime();
    }


    @Override
    public void start() {

        // Make initial movements automatically so the arm doesn't drag against the ground.
        pivot_motor.setPower(.8);
        pivot_motor.setTargetPosition(pivotmdrivepos);
        pivot_Servo.setPosition(intakerightpos);
        extendo_servo.setPosition(extendostartpos);

        setStartTime();

    }

    @Override
    public void loop() {

        pivot_motor.setPower(.5);

        if (pivot_motor.getCurrentPosition() >= pivotmdrivepos) {

            while (startTime - getRuntime() < 1) { // Move forward for 2.5 seconds.
                front_left.setPower(.5);
                front_right.setPower(.5);
                back_left.setPower(.5);
                back_right.setPower(.5);
            }

            if (startTime - getRuntime() >= 1) { // Stop moving after 2.5 seconds.
                front_left.setPower(0);
                front_right.setPower(0);
                back_left.setPower(0);
                back_right.setPower(0);

                pivot_motor.setTargetPosition(pivotmdrivepos);
                extendo_servo.setPosition(extendostartpos);
                pivot_Servo.setPosition(intakeleftpos);
            }

        }

    }

    @Override
    public void stop() {

        // ---------- Stops All Motors ----------
        front_left.setPower(0);
        front_right.setPower(0);
        back_left.setPower(0);
        back_right.setPower(0);
        pivot_motor.setPower(0);

        requestOpModeStop();
    }

}

