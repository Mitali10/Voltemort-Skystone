package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoRedFoundation", group = "Skystone")
//@Disabled
public class SkystoneAutoRedFoundation extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();

        foundationServo1.setPosition(0.6);  // horiz pos
        foundationServo2.setPosition(0.55);  // horiz pos

        nextTask = Task.DRIVE_TO_FOUNDATION;

        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();


        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {

                case DRIVE_TO_FOUNDATION:
                    encoderDrive(0.4, 27, 10);

                    encoderStrafeRight(0.4, 6, 5);

                    encoderDrive(0.1, 4, 10);

                    foundationServo1.setPosition(1.0);
                    foundationServo2.setPosition(1.0);

                    sleep(500);

                    nextTask = Task.MOVE_FOUNDATION;
                    break;

                case MOVE_FOUNDATION:

                    encoderDrive(0.4, -27, 10);
                    turnRight(87);

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    turnTo90Right();

                    sleep(500);

                    nextTask = Task.PARK;
                    break;


                case PARK:

                    encoderStrafeLeft(0.4, 15, 10);
                    encoderDrive(0.4, -36, 10);
                    nextTask = Task.HALT;
                    break;

                case HALT:

                    break;



            }


        }

    }



}



