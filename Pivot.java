package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Pivot Tester")
public class Pivot extends OpMode {

    Servo pivotServo;
    @Override
    public void init() {

        pivotServo = hardwareMap.get(Servo.class, "pivot");
    }

    @Override
    public void loop() {
        // some button
        if (gamepad1.a) {
            pivotServo.setPosition(0);
        } else if (gamepad1.b) {
            pivotServo.setPosition(0.5);
        } else if (gamepad1.x) {
            pivotServo.setPosition(1);
        }
    }
}
