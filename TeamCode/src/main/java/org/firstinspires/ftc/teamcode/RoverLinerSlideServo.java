package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name = "RoverTeleOp:RoverLinearSlideServo", group = "RoverTest")
@Disabled

public class RoverLinerSlideServo extends OpMode {
    private Servo linearServo;
    double position = 0.0;
    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  0.60;     // Maximum rotational position
    static final double MIN_POS     =  0.10;     // Minimum rotational position

    @Override
    public void init() {

        linearServo = hardwareMap.get(Servo.class, "armServo");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();


    }

    @Override
    public void loop() {

        if ((gamepad2.right_stick_y < -0.2) && (position<MAX_POS)) { //forward

            position+=INCREMENT;

        } else if ((gamepad2.right_stick_y  > 0.2) && (position>MIN_POS)) { //back

            position-=INCREMENT;

        }

        linearServo.setPosition(position);
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.update();


    }


}
