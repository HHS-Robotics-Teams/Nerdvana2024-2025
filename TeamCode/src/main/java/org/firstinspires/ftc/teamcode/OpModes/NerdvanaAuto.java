package org.firstinspires.ftc.teamcode.OpModes;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.components.RobotComponents;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.EXTENDOINPOS;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.EXTENDOMINREACHED;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.EXTENDOOUTPOS;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.initPositionsReached;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakecenterpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakeleftpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.intakerightpos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmdrivepos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmhighbucket;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmhighchamber;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmlowbucket;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmlowchamber;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmpickuppos;
import static org.firstinspires.ftc.teamcode.OpModes.Constants.pivotmstartpos;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.back_left;
import static org.firstinspires.ftc.teamcode.components.RobotComponents.back_right;
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

@Autonomous(name="Nerdvana Auto", group="jr-test")
public class NerdvanaAuto extends OpMode {


    private double startTime;

    @Override
    public void init () {

        RobotComponents.init(hardwareMap);

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
        extendo_servo.setPosition(EXTENDOINPOS);

        setStartTime();

    }

    @Override
    public void loop() {

        pivot_motor.setPower(.5);

        if (pivot_motor.getCurrentPosition() >= pivotmdrivepos) {

            while (startTime - getRuntime() < .5) { // Move forward for 2.5 seconds.
                front_left.setPower(.5);
                front_right.setPower(.5);
                back_left.setPower(.5);
                back_right.setPower(.5);
            }

            if (startTime - getRuntime() >= .5) { // Stop moving after 2.5 seconds.
                front_left.setPower(0);
                front_right.setPower(0);
                back_left.setPower(0);
                back_right.setPower(0);

                pivot_motor.setTargetPosition(pivotmdrivepos);
                extendo_servo.setPosition(EXTENDOINPOS);
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

