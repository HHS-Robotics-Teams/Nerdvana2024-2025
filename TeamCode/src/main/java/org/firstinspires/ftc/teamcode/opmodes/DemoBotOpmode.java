package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.excutil.Input;

/*
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 *
 *
 *  Lots of code from https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 *
 *
 * !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
 */
@TeleOp(name = "DemoBotOpmode", group = "Test BW")
public class DemoBotOpmode extends OpMode {
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
    int pivotmpickuppos = 50;
    int pivotmlowbucket = 200;
    int pivotmhighbucket = 500;
    int pivotmlowchamber = 150;
    int pivotmhighchamber = 300;

    //extendo positions
    int extendostartpos = 1;
    int extendoscorepos = 0;

    //Rodo-Intake positions
    int intakeleftpos = 0;
    double intakecenterpos = .5;
    int intakerightpos = 1;





    @Override
    public void init() {
        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        back_left = hardwareMap.get(DcMotor.class, "leftRear");
        back_right = hardwareMap.get(DcMotor.class, "rightRear");
        pivot_Servo = hardwareMap.get(Servo.class, "rodo");
        pivot_motor = hardwareMap.get(DcMotor.class, "pivot");
        extendo_servo = hardwareMap.get(Servo.class, "extendoarm");
        intake_servo = hardwareMap.get(CRServo.class,"");

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

        pivot_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_servo.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    @Override
    public void loop() {

        //init positions
        pivot_motor.setTargetPosition(pivotmstartpos);
        extendo_servo.setPosition(extendostartpos);
        pivot_Servo.setPosition(intakerightpos);

        pivot_motor.setPower(.5);


        input.pollGamepad(gamepad1);

        double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        // This button choice was made so that it is hard to hit on accident,
        // it can be freely changed based on preference.
        // The equivalent button is start on Xbox-style controllers.
        if (gamepad1.options) {
            imu.resetYaw();
        }

        double botHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

        // Rotate the movement direction counter to the bot's rotation
        double rotX = x /* Math.cos(botHeading) - y * Math.sin(botHeading)*/;
        double rotY = y /* Math.sin(botHeading) + y * Math.cos(botHeading)*/;

        //rotX = rotX * 1.1;  // Counteract imperfect strafing

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio,
        // but only if at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
        double frontLeftPower = (rotY + rotX + rx) / denominator;
        double backLeftPower = (rotY - rotX + rx) / denominator;
        double frontRightPower = (rotY - rotX - rx) / denominator;
        double backRightPower = (rotY + rotX - rx) / denominator;

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        // pickup
        if (input.right_bumper.held()){
            pivot_motor.setTargetPosition(pivotmpickuppos);
        }

        // rodo-intake
        if (input.dpad_left.held()) {
            pivot_Servo.setPosition(intakeleftpos);

        } else if (input.dpad_up.held()) {
            pivot_Servo.setPosition(intakecenterpos);

        } else if (input.dpad_right.held()) {
            pivot_Servo.setPosition(intakerightpos);
        }
        // intake
        if (input.right_trigger.down()) {
            intake_servo.setPower(1);
        }
        // outtake
        if (input.left_trigger.down()) {
            intake_servo.setPower(-1);
        }
        //high bucket scoring
        if (input.y.held()) {
            pivot_motor.setTargetPosition(pivotmhighbucket);
            pivot_Servo.setPosition(intakecenterpos);
            extendo_servo.setPosition(extendoscorepos);
        }
        //low bucket scoring
        if (input.b.held()) {
            pivot_motor.setTargetPosition(pivotmlowbucket);
            pivot_Servo.setPosition(intakecenterpos);
            extendo_servo.setPosition(extendostartpos);
        }
        //high chamber scoring
        if (input.x.held()) {
            pivot_motor.setTargetPosition(pivotmhighchamber);
            pivot_Servo.setPosition(intakeleftpos);
            extendo_servo.setPosition(extendoscorepos);
        }
        //low chamber scoring
        if (input.a.held()) {
            pivot_motor.setTargetPosition(pivotmlowchamber);
            pivot_Servo.setPosition(intakeleftpos);
            extendo_servo.setPosition(extendostartpos);
        }

        telemetry.addData("pivot motor target", pivot_motor.getTargetPosition());
        telemetry.addData("pivot motor position", pivot_motor.getCurrentPosition());
    }

}
