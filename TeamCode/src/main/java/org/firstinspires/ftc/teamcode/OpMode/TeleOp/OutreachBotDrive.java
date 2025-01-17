package org.firstinspires.ftc.teamcode.OpMode.TeleOp;


import static org.firstinspires.ftc.teamcode.OpMode.Constants.clawtiltconepickup;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.clawtiltdroppos;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.clawtiltpickuppos;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.clawtiltstartpos;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.pincerleftclosed;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.pincerleftopen;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.pincerrightclosed;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.pincerrightopen;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.tiltconepickup;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.tiltdroppos;
import static org.firstinspires.ftc.teamcode.OpMode.Constants.tiltstartpos;
import static org.firstinspires.ftc.teamcode.RobotComponents.arm_tilt;
import static org.firstinspires.ftc.teamcode.RobotComponents.claw_tilt;
import static org.firstinspires.ftc.teamcode.RobotComponents.colorSensor;
import static org.firstinspires.ftc.teamcode.RobotComponents.leftMotor;
import static org.firstinspires.ftc.teamcode.RobotComponents.pincer_left;
import static org.firstinspires.ftc.teamcode.RobotComponents.pincer_right;
import static org.firstinspires.ftc.teamcode.RobotComponents.rightMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.excutil.Input;
import org.firstinspires.ftc.teamcode.excutil.coroutines.CoroutineManager;

import org.firstinspires.ftc.teamcode.RobotComponents;

@TeleOp ( name = "OutreachBotDrive")
public class OutreachBotDrive extends OpMode {
    private Input input;

    CoroutineManager coroutines = new CoroutineManager();
    boolean PincersClosed = false;



    @Override
    public void init() {

        RobotComponents.init(hardwareMap);


        pincer_left.setPosition(pincerleftopen);
        pincer_right.setPosition(pincerrightopen);
        claw_tilt.setPosition(clawtiltstartpos);
        arm_tilt.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        arm_tilt.setTargetPosition(tiltstartpos);
        arm_tilt.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        arm_tilt.setPower(1);

        input = new Input();

    }

    @Override
    public void loop() {
        DetectedColor.updateColor(colorSensor);
        input.pollGamepad(gamepad1);


        // Tank drive control
        double forwardPower = -gamepad1.left_stick_y; // Forward/backward movement
        double turnPower =-gamepad1.right_stick_x; // Left/right turning

        // Calculate motor powers for left and right motors
        double leftPower = forwardPower + turnPower;
        double rightPower = forwardPower - turnPower;

        // Set power to motors
        leftMotor.setPower(leftPower);
        rightMotor.setPower(rightPower);


        //pixel pickup
        if (input.right_bumper.down()){
            //arm_tilt.setPower(-.1);
            arm_tilt.setTargetPosition(tiltstartpos);
            pincer_left.setPosition(pincerleftopen);
            pincer_right.setPosition(pincerrightopen);
            claw_tilt.setPosition(clawtiltpickuppos);
            PincersClosed = false;
        }
        //cone pickup
        if (input.a.down()){
            arm_tilt.setTargetPosition(tiltconepickup);
            claw_tilt.setPosition(clawtiltconepickup);
            PincersClosed = false;
        }
        //pincer grab
        if (input.right_trigger.down()){
            pincer_left.setPosition(pincerleftclosed);
            pincer_right.setPosition(pincerrightclosed);
            PincersClosed = true;
        }
        //pincer drop
        if (input.left_trigger.down()){
            pincer_left.setPosition(pincerleftopen);
            pincer_right.setPosition(pincerrightopen);
            PincersClosed = false;
        }
        //Scoring
        if (input.left_bumper.down() ) {
            arm_tilt.setTargetPosition(tiltdroppos);
            claw_tilt.setPosition(clawtiltdroppos);
            PincersClosed = false;
        }
        // pincer up & closed
        if (input.dpad_up.down()){
            claw_tilt.setPosition(clawtiltstartpos);
            arm_tilt.setTargetPosition(tiltstartpos);
            pincer_left.setPosition(pincerleftclosed);
            pincer_right.setPosition(pincerrightclosed);
            PincersClosed = true;
        }
        // pincer up & open
        if (input.dpad_down.down()){
            claw_tilt.setPosition(clawtiltstartpos);
            arm_tilt.setTargetPosition(tiltstartpos);
            pincer_left.setPosition(pincerleftopen);
            pincer_right.setPosition(pincerrightopen);
            PincersClosed = false;
        }


        // Display the detected color on telemetry
        telemetry.addData("Detected Color", DetectedColor.getColor());
        telemetry.addData("arm tilt position", arm_tilt.getCurrentPosition());
        telemetry.addData("claw tilt position",claw_tilt.getPosition());
        // Display motor power on telemetry
        telemetry.addData("Left Motor Power", "%.2f", leftPower);
        telemetry.addData("Right Motor Power", "%.2f", rightPower);
        telemetry.update();
    }
}
