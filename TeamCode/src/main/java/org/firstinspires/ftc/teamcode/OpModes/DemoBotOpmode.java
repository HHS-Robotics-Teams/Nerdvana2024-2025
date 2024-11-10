package org.firstinspires.ftc.teamcode.OpModes;

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





    @Override
    public void init() {
        front_left = hardwareMap.get(DcMotor.class, "leftFront");
        front_right = hardwareMap.get(DcMotor.class, "rightFront");
        back_left = hardwareMap.get(DcMotor.class, "leftRear");
        back_right = hardwareMap.get(DcMotor.class, "rightRear");
        pivot_Servo = hardwareMap.get(Servo.class, "rodo");
        pivot_motor = hardwareMap.get(DcMotor.class, "pivot");
        extendo_servo = hardwareMap.get(Servo.class, "extendoarm");
        intake_servo = hardwareMap.get(CRServo.class,"eject");

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


    }

    @Override
    public void loop() {

        //init positions
        if(!initPositionsReached){
            pivot_motor.setTargetPosition(pivotmdrivepos);
            extendo_servo.setPosition(extendostartpos);
            pivot_Servo.setPosition(intakerightpos);
            initPositionsReached = true;
        }

        pivot_motor.setPower(.8);


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

        //home
        if (input.dpad_down.down()){
            pivot_motor.setTargetPosition(pivotmdrivepos);
            extendo_servo.setPosition(extendostartpos);
            pivot_Servo.setPosition(intakerightpos);
        }

        // pickup
        if (input.right_bumper.down()){
            pivot_motor.setTargetPosition(pivotmpickuppos);
            extendo_servo.setPosition(extendoscorepos);
            pivot_Servo.setPosition(intakecenterpos);

        }

        // rodo-intake
        if (input.dpad_left.down()) {
            pivot_Servo.setPosition(intakeleftpos);

        }
        if (input.dpad_up.down()) {
            pivot_Servo.setPosition(intakecenterpos);

        }
        if (input.dpad_right.down()) {
            pivot_Servo.setPosition(intakerightpos);
        }
        // intake
        if (input.right_trigger.held()) {
            intake_servo.setPower(1);

        }
        // outtake
        else if (input.left_trigger.held()) {
            intake_servo.setPower(-1);
        }
        // intake stop
        else intake_servo.setPower(0);

        //high bucket scoring
        if (input.y.down()) {
            pivot_motor.setTargetPosition(pivotmhighbucket);
            pivot_Servo.setPosition(intakecenterpos);
            extendo_servo.setPosition(extendoscorepos);
        }
        //low bucket scoring
        if (input.b.down()) {
            pivot_motor.setTargetPosition(pivotmlowbucket);
            pivot_Servo.setPosition(intakecenterpos);
            extendo_servo.setPosition(extendostartpos);
        }
        //high chamber scoring
        if (input.x.down()) {
            pivot_motor.setTargetPosition(pivotmhighchamber);
            pivot_Servo.setPosition(intakeleftpos);
            extendo_servo.setPosition(extendostartpos);
        }
        //low chamber scoring
        if (input.a.held()) {
            pivot_motor.setTargetPosition(pivotmlowchamber);
            pivot_Servo.setPosition(intakeleftpos);
            extendo_servo.setPosition(extendostartpos);
        }

        //climbing code
        if (input.back.down () ){
            pivot_motor.setDirection(DcMotorSimple.Direction.REVERSE);
            pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot_motor.setTargetPosition(pivotmclimbpos);
            pivot_Servo.setPosition(intakeleftpos);


            climbPositionReached = true;

            telemetry.speak("climb position reached");
        }
        if (input.start.up()&&climbPositionReached) {

            extendo_servo.setPosition(extendostartpos);
            pivot_motor.setDirection(DcMotorSimple.Direction.FORWARD);
            pivot_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            pivot_Servo.setPosition(intakeleftpos);

            climbPositionReached = false;


        }



        telemetry.addData("pivot motor target", pivot_motor.getTargetPosition());
        telemetry.addData("pivot motor position", pivot_motor.getCurrentPosition());
        telemetry.addData("climb active", climbPositionReached);
        telemetry.update();
    }

}
