package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
@TeleOp(name = "DemoBotOpmode", group = "Test BW")
public class DemoBotOpmode extends OpMode {

    Servo elbow_servo = null;


    @Override
    public void init() {
        elbow_servo = hardwareMap.get(Servo.class, "elbow");
    }

    double currentPosition = 0;

    Input input = new Input();

    @Override
    public void loop() {

        input.pollGamepad(gamepad1);

        if (gamepad1.start)
            elbow_servo.setPosition(0);
        if (gamepad1.a)
            elbow_servo.setPosition(0.29);
       /* if (gamepad1.b)
            elbow_servo.setPosition(0.625);*/
        if (gamepad1.x)
            elbow_servo.setPosition(0.85);
/*        if (gamepad1.y)
            elbow_servo.setPosition(1);*/

/*        boolean faster = input.right_bumper.held();

        if (input.dpad_up.down())
            currentPosition += 0.005 * ((faster) ? 10 : 1);
        if (input.dpad_down.down())
            currentPosition -= 0.005 * ((faster) ? 10 : 1);

        telemetry.addData("Current elbow: ", currentPosition);
        telemetry.update();

        elbow_servo.setPosition(currentPosition);*/


    }

}
