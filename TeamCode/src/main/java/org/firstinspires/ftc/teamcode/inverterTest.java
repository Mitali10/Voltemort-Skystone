package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="RelicTest:inverterTest",group = "RelicTest")
@Disabled
public class inverterTest extends LinearOpMode {

    static final double INCREMENT   =    0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle (??)
    static final double MAX_POS     =  0.50;     // Maximum rotational position
    static final double MIN_POS     =  0.65;     // Minimum rotational position

    // Define class members
    Servo inverter;
    //double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    double position = 0.49;
    boolean rampUp = true;


    @Override
    public void runOpMode() {

        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        inverter = hardwareMap.get(Servo.class, "inverter");

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        inverter.setPosition(0.50);
        sleep(1000);
        waitForStart();


        // Scan servo till stop pressed.
        while(opModeIsActive()){

            for (double k=0.49;k<=0.59; k=k+0.001){

                inverter.setPosition(k);
                sleep(75);
                telemetry.addData("Servo Position", "%5.2f", k);
                telemetry.addData(">", "Press Stop to end test." );
                telemetry.update();

            }

            sleep (3000);

            for (double k=0.59;k>=0.49; k=k-0.001){

                inverter.setPosition(k);
                sleep(75);
                telemetry.addData("Servo Position", "%5.2f", k);
                telemetry.addData(">", "Press Stop to end test." );
                telemetry.update();

            }

            sleep(3000);
            idle();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

}
