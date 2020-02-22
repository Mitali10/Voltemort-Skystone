package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoMotionTest", group = "Skystone")
//@Disabled
public class SkystoneAutonomousMotionTest extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();
        //initOpenCV();

        nextTask = Task.MOVE;
        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();

        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {

                case MOVE:

                    encoderDriveForwardUntilDistance(0.2, 10, 20);

                    telemetry.addData("distanceTraveled:", distanceTraveled);
                    telemetry.update();


                    nextTask = Task.HALT;

                    break;


//                    if(checkRecognitions(0.8)) {
//                        telemetry.addData("Skystone at Pos A:", "");
//                        telemetry.update();
//                        nextTask = Task.HALT;
//                        break;
//                    }
//
//                    encoderStrafeRight(0.3, 8, 10);
//
//                    if(checkRecognitions(0.8)) {
//                        telemetry.addData("Skystone at Pos B:", "");
//                        telemetry.update();
//                        nextTask = Task.HALT;
//                        break;
//                    }
//
//                    telemetry.addData("Skystone must be at Pos C:", "");
//                    telemetry.update();
//                    encoderStrafeRight(0.3, 8, 10);
//


                //                    colorSensor.enableLed(true);
//                    encoderStrafeRightColor(0.2, 20, 20, 5000);
//                    colorSensor.enableLed(false);
//                    encoderStrafeRight(0.5, 8, 20);
//                    encoderStrafeRight(0.5, 15, 20);
//

                // encoderStrafeRightSearching(0.05, 35, 40, .7);
//                    telemetry.addData("forward", 24);
//                    telemetry.update();
//                    encoderDrive(0.2, 24, 30);
//
//                    telemetry.addData("back", 24);
//                    telemetry.update();
//                    encoderDrive(0.2, -24, 30);

//                    telemetry.addData("left", 24);
//                    telemetry.update();
//                    encoderStrafeLeft(0.3, 24, 30);
//
//                    sleep(3000);
//
//                    telemetry.addData("right", 24);
//                    telemetry.update();
//                    encoderStrafeRight(0.3, 24, 30);
//
//                    sleep(3000);

//                    telemetry.addData("left", 48);
//                    telemetry.update();
//                    encoderStrafeLeft(0.2, 48, 30);
//
//                    sleep(3000);
//
//                    telemetry.addData("right", 48);
//                    telemetry.update();
//                    encoderStrafeRight(0.2, 48, 30);
//
//                    sleep(3000);

//                    telemetry.addData("left", 120);
//                    telemetry.update();
//                    encoderStrafeLeft(0.3, 120, 30);
//
//                    sleep(4000);
//
//                    telemetry.addData("right", 120);
//                    telemetry.update();
//                    encoderStrafeRight(0.3, 120, 30);
//
//                    sleep(3000);

//                    telemetry.addData("Changing to encoder values", "");
//                    telemetry.update();
//
//                    double dist = wheelFrontLeft.getCurrentPosition();
//                    encoderDrive(0.2, 24, 30);
//                    telemetry.addData("forward", getEncoderPulse(wheelFrontLeft.getCurrentPosition() - dist));
//                    telemetry.update();
//
//                    dist = wheelFrontLeft.getCurrentPosition();
//                    encoderDrive(0.2, -24, 30);
//                    telemetry.addData("back", wheelFrontLeft.getCurrentPosition() - dist);
//                    telemetry.update();
//
//                    dist = wheelFrontLeft.getCurrentPosition();
//                    encoderStrafeLeft(0.2, 24, 30);
//                    telemetry.addData("left", wheelFrontLeft.getCurrentPosition() - dist);
//                    telemetry.update();
//
//                    dist = wheelFrontLeft.getCurrentPosition();
//                    encoderStrafeRight(0.2, 24, 30);
//                    telemetry.addData("right", wheelFrontLeft.getCurrentPosition() - dist);
//                    telemetry.update();

//                    while(colorSensor.blue() < 5000) {
//                        telemetry.addData("blue:", colorSensor.blue());
//                        telemetry.update();
//                        encoderDrive(.2, 2, 5);
//                    }
//
//                    while(colorSensor.red() < 5000) {
//                        telemetry.addData("red:", colorSensor.red());
//                        telemetry.update();
//                        encoderDrive(.2, 2, 5);
//                    }






                case HALT:
                    break;


            }


        }

    }



}



