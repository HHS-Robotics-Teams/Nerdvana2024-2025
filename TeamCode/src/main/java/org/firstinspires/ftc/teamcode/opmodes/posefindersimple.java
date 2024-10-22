package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp (name = "posefindersimple",group ="testing" )
public class posefindersimple extends OpMode {

    DcMotor pivot_motor;
    Servo pivot_Servo;
    Servo  extendo_servo;
    CRServo intake_servo;


    @Override
    public void init() {
        pivot_Servo = hardwareMap.get(Servo.class, "rodo");
        pivot_motor =hardwareMap.get(DcMotor.class,"pivot");
        extendo_servo= hardwareMap.get(Servo.class,"extendoarm");
        intake_servo = hardwareMap.get(CRServo.class,"eject");

        pivot_motor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        pivot_motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        intake_servo.setDirection(DcMotorSimple.Direction.FORWARD);


    }

    @Override
    public void loop() {

        telemetry.addData("rodo position",pivot_Servo.getPosition());
        telemetry.addData("pivot position",pivot_motor.getCurrentPosition());
        telemetry.addData("extendo position",extendo_servo.getPosition());
        telemetry.addData("intake direction",intake_servo.getDirection());

        if (gamepad1.a){
            extendo_servo.setPosition(1);
        }
        if (gamepad1.b){
            extendo_servo.setPosition(0);
        }
    }
}
