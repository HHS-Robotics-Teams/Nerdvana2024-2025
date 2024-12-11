package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.components.RobotComponents;
import org.firstinspires.ftc.teamcode.excutil.Input;

import static org.firstinspires.ftc.teamcode.OpModes.Constants.EXTENDOINPOS;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.EXTENDOOUTPOS;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.elbowpickup;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.initPositionsReached;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakecenterpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakeleftpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakerightpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmMinPos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmdrivepos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmhighbucket;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmhighchamber;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmlowbucket;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmlowchamber;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmpickuppos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmrodopos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmstartpos;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.back_left;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.back_right;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.elbow_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.extendo_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.front_left;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.front_right;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.imu;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.intake_servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_Servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmclimbpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.rodoControlReached;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.climbPositionReached;

@TeleOp(name = "CompDrive25", group = "Test jr")
public class CompDrive25 extends OpMode {

    public Input input;

    @Override
    public void init() {
        input = new Input();

        RobotComponents.init(hardwareMap);
    }

    @Override
    public void loop() {

        input.pollGamepad(gamepad1);

        //init positions
        if(!initPositionsReached){
            pivot_motor.setTargetPosition(pivotmdrivepos);
            extendo_servo.setPosition(EXTENDOINPOS);
            pivot_Servo.setPosition(intakecenterpos);

            initPositionsReached = true;
        }



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

        if (pivot_motor.getCurrentPosition() > pivotmlowchamber){
            front_left.setPower(frontLeftPower / 4);
            back_left.setPower(backLeftPower / 4);
            front_right.setPower(frontRightPower / 4);
            back_right.setPower(backRightPower / 4);
        }

        front_left.setPower(frontLeftPower);
        back_left.setPower(backLeftPower);
        front_right.setPower(frontRightPower);
        back_right.setPower(backRightPower);

        //Arm logic


        if (pivot_motor.getCurrentPosition() < pivotmrodopos){
            rodoControlReached = false;
            pivot_Servo.setPosition(intakecenterpos);
        }

        //home
        if (gamepad1.left_stick_button) {
            pivot_motor.setTargetPosition(pivotmdrivepos);
            extendo_servo.setPosition(EXTENDOINPOS);
            pivot_Servo.setPosition(intakerightpos);
        }
        // pickup position
        if (gamepad1.right_stick_button) {
            pivot_motor.setTargetPosition(pivotmpickuppos);
            extendo_servo.setPosition(EXTENDOINPOS);
            pivot_Servo.setPosition(intakecenterpos);
        }

        // specimen pickup position
        if (input.start.down()) {
            extendo_servo.setPosition(EXTENDOINPOS);
            elbow_servo.setPosition(elbowpickup);
            pivot_motor.setTargetPosition(pivotmpickuppos);

        }

        // Manual Arm Tilt
            //arm up
        if (input.left_bumper.held() && (pivot_motor.getCurrentPosition() <= pivotmclimbpos)){
            pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getTargetPosition()+25);

        }
            //arm down
        if (input.right_bumper.held() && (pivot_motor.getCurrentPosition() >= pivotmMinPos)){
            pivot_motor.setTargetPosition(RobotComponents.pivot_motor.getTargetPosition()-25);
        }
        // Manual extension
            //arm out
        if (input.dpad_up.held() && (extendo_servo.getPosition() > EXTENDOOUTPOS )){
            extendo_servo.setPosition(extendo_servo.getPosition()-.05);

        }
            //arm in
        if (input.dpad_down.held()  && (extendo_servo.getPosition() < EXTENDOINPOS )){
            extendo_servo.setPosition(extendo_servo.getPosition()+.05);
        }

        // rodo-intake
        if (input.dpad_left.down() && rodoControlReached) {
            pivot_Servo.setPosition(intakeleftpos);

        }
        if (input.dpad_right.down() && rodoControlReached) {
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
            rodoControlReached = false;
            pivot_motor.setTargetPosition(pivotmhighbucket);
            pivot_Servo.setPosition(intakecenterpos);
            extendo_servo.setPosition(EXTENDOOUTPOS);
            elbow_servo.setPosition(0);
        }
        //low bucket scoring
        if (input.b.down()) {
            rodoControlReached = false;

            pivot_motor.setTargetPosition(pivotmlowbucket);
            pivot_Servo.setPosition(intakecenterpos);
            extendo_servo.setPosition(EXTENDOINPOS);
            elbow_servo.setPosition(0);
        }
        //high chamber scoring
        if (input.x.down()) {
            rodoControlReached = true;

            pivot_motor.setTargetPosition(pivotmhighchamber);
            pivot_Servo.setPosition(intakeleftpos);
            extendo_servo.setPosition(EXTENDOINPOS);

            elbow_servo.setPosition(elbowpickup);
            telemetry.speak("rodo control reached");
        }
        //low chamber scoring
        if (input.a.down()) {
            rodoControlReached = true;

            pivot_motor.setTargetPosition(pivotmlowchamber);
            pivot_Servo.setPosition(intakeleftpos);
            extendo_servo.setPosition(EXTENDOINPOS);
            elbow_servo.setPosition(elbowpickup);
            telemetry.speak("rodo control reached");
        }
        // Climbing
        if (input.back.down()){
            extendo_servo.setPosition(EXTENDOINPOS);
            pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            pivot_motor.setTargetPosition(pivotmclimbpos);
            pivot_Servo.setPosition(intakeleftpos);

            climbPositionReached = true;

            telemetry.speak("climb position reached");
        }
  /*      if (input.start.down() && climbPositionReached){
            extendo_servo.setPosition(EXTENDOINPOS);
            pivot_Servo.setPosition(intakeleftpos);
            pivot_motor.setDirection(DcMotorSimple.Direction.FORWARD);
            pivot_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            climbPositionReached = false;
        }*/

        telemetry.addData("pivot motor target", pivot_motor.getTargetPosition());
        telemetry.addData("pivot motor position", pivot_motor.getCurrentPosition());
        telemetry.addData("climb control status", climbPositionReached ? "True" : "False");
        telemetry.addData("rodo control status", rodoControlReached ? "True" : "False");
        telemetry.update();
    }

}
