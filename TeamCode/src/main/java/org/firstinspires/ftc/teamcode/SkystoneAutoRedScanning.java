package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoRedScanning", group = "Skystone")
@Disabled
public class SkystoneAutoRedScanning extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();

        int pos = -1;
        int d = 0; //36;

        foundationServo1.setPosition(0.0);  // horiz pos
        foundationServo2.setPosition(1.0);  // horiz pos
        turnServo.setPosition(0.5);
        gripperServo.setPosition(GRIPPER_CLOSE_POS);

        leftSlide.setPower(-0.2);
        rightSlide.setPower(0.2);

        sleep(1200);

        leftSlide.setPower(0.0);
        rightSlide.setPower(0.0);

        nextTask = Task.APPROACH_STONES;

        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();


        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {

                case APPROACH_STONES:
                    encoderDrive(0.4, 23, 10);
                    encoderStrafeLeft(0.4, 5, 6);

                    nextTask = Task.SEARCH;
                    break;


                case SEARCH:
//                    if(checkRecognitions(0.75)) {
//                        pos = 1;
//                        telemetry.addData("Skystone at Pos A:", "");
//                        telemetry.update();
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }
//
//                    encoderStrafeRight(0.3, 8, 10);
//
//                    if(checkRecognitions(0.75)) {
//                        pos = 2;
//                        telemetry.addData("Skystone at Pos B:", "");
//                        telemetry.update();
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }

                    pos = 3;
                    telemetry.addData("Skystone must be at Pos C:", "");
                    telemetry.update();
                    encoderStrafeRight(0.3, 8, 10);

                    nextTask = Task.LIFT_SKYSTONE;

                    break;


                case LIFT_SKYSTONE:
                    gripperServo.setPosition(GRIPPER_OPEN_POS);
                    encoderDrive(0.3, 7, 10);

                    leftSlide.setPower(0.28);        // down
                    rightSlide.setPower(-0.28);

                    sleep(800);

                    leftSlide.setPower(0.0);
                    rightSlide.setPower(0.0);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(650);

                    leftSlide.setPower(-0.25);   // lift block
                    rightSlide.setPower(0.25);

                    sleep(500);

                    leftSlide.setPower(0.0);
                    rightSlide.setPower(0.0);

                    nextTask = Task.DRIVE_TO_FOUNDATION;
                    break;

                case DRIVE_TO_FOUNDATION:
                    encoderDrive(0.3, -10, 10);

                    if(pos == 2) {
                        d = 8; //44;
                    } else if(pos == 1) {
                        d = 16; //52;
                    }

                    telemetry.addData("d:", d);
                    telemetry.update();

                    encoderStrafeRight(0.5, 41 + d, 20);

                    nextTask = Task.DROP_SKYSTONE;
                    break;

                case DROP_SKYSTONE:
                    encoderDrive(0.4, 3, 6);
                    gripperServo.setPosition(GRIPPER_OPEN_POS);
                    encoderDrive(0.4, -3, 6);

                    nextTask = Task.RETURN_TO_STONES;
                    break;

                case RETURN_TO_STONES:
                    encoderStrafeLeft(0.4, 41 + d + 24, 10);
                    encoderDrive(0.4, 12, 10);
                    nextTask = Task.LIFT_SKYSTONE2;
                    break;

                case LIFT_SKYSTONE2:
                    leftSlide.setPower(0.28);        // down
                    rightSlide.setPower(-0.28);

                    sleep(700);

                    leftSlide.setPower(0.0);
                    rightSlide.setPower(0.0);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(650);

                    leftSlide.setPower(-0.25);   // lift block
                    rightSlide.setPower(0.25);

                    sleep(500);

                    leftSlide.setPower(0.0);
                    rightSlide.setPower(0.0);

                    nextTask = Task.DROP_SKYSTONE2;
                    break;

                case DROP_SKYSTONE2:

                    encoderDrive(0.3, -12, 10);
                    encoderStrafeRight(0.5, 41 + d + 24, 20);

                    //encoderDrive(0.4, 3, 6);
                    gripperServo.setPosition(GRIPPER_OPEN_POS);
                    //encoderDrive(0.4, -3, 6);

                    nextTask = Task.HALT;
                    break;

                case PARK:
                    encoderStrafeLeft(0.4, 8, 10);
                    nextTask = Task.PARK;
                    break;

                case HALT:

                    break;

            }


        }

    }



}



