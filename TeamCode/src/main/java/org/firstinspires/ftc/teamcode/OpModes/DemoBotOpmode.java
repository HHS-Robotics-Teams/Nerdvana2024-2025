package org.firstinspires.ftc.teamcode.OpModes;

import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakeleftpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakerightpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmdrivepos;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_Servo;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.pivot_motor;

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
import org.firstinspires.ftc.teamcode.components.RobotComponents;
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

    Servo elbow_servo = null;
    Servo pivot_servo = null;


    @Override
    public void init() {
        //RobotComponents.init(hardwareMap);

        elbow_servo = hardwareMap.get(Servo.class, "elbow");
        pivot_Servo = hardwareMap.get(Servo.class, "rodo");
    }

    double currentPosition = 0;

    Input input = new Input();

    @Override
    public void loop() {

        input.pollGamepad(gamepad1);

        if (gamepad1.start)
            elbow_servo.setPosition(0);
        if (gamepad1.a) {
            elbow_servo.setPosition(0.29);
            pivot_Servo.setPosition(intakeleftpos);
        }
       /* if (gamepad1.b)
            elbow_servo.setPosition(0.625);*/
        if (gamepad1.x)
            elbow_servo.setPosition(0.85);
/*        if (gamepad1.y)
            elbow_servo.setPosition(1);*/

//        boolean faster = input.right_bumper.held();

        if (input.dpad_up.down())
            currentPosition += 0.05 ;
        if (input.dpad_down.down())
            currentPosition -= 0.05 ;

        telemetry.addData("Current position: ", currentPosition);
        telemetry.update();

        // elbow_servo.setPosition(currentPosition);
        pivot_Servo.setPosition(currentPosition);


    }

}
