package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoBlueScanning3", group = "Skystone")
@Disabled
public class SkystoneAutoBlueScanning3 extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();

        int pos = -1;
        int d = 0; //36;

        foundationServo1.setPosition(1.0);  // horiz pos
        foundationServo2.setPosition(1.0);  // horiz pos
        turnServo.setPosition(ARM_UP);
        gripperServo.setPosition(GRIPPER_OPEN_POS);

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
                    encoderStrafeRight(0.4, 5, 6);

                    nextTask = Task.SEARCH;
                    break;


                case SEARCH:
//                    if(checkRecognitions(0.8)) {
//                        pos = 1;
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }
//
//                    encoderStrafeLeft(0.3, 9, 10);
//
//                    if(checkRecognitions(0.8)) {
//                        pos = 2;
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }

                    pos = 3;
                    encoderStrafeLeft(0.3, 8, 10);

                    nextTask = Task.LIFT_SKYSTONE;

                    break;


                case LIFT_SKYSTONE:
                    encoderDriveForwardUntilDistance(0.2, 10.5, 2.5);

                    turnServo.setPosition(ARM_DOWN);

                    sleep(500);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(500);

                    turnServo.setPosition(ARM_UP);

                    sleep(500);

                    nextTask = Task.DRIVE_TO_FOUNDATION;
                    break;

                case DRIVE_TO_FOUNDATION:
                    encoderDrive(0.3, -4, 10);

                    if(pos == 2) {
                        d = 8; //44;
                    } else if(pos == 1) {
                        d = 16; //52;
                    }

                    telemetry.addData("d:", d);
                    telemetry.update();

                    encoderStrafeLeft(0.5, 50 + d, 20);

                    nextTask = Task.DROP_SKYSTONE;
                    break;

                case DROP_SKYSTONE:
                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    sleep(500);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    if(pos == 1) {                          // if the other block is too close to the wall to pick up
                        nextTask = Task.PARK;
                    } else {
                        nextTask = Task.RETURN_TO_STONES;
                    }

                    break;

                case RETURN_TO_STONES:
                    encoderStrafeRight(0.4, 50 + d + 24, 10);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    turnToZero();

                    encoderDriveForwardUntilDistance(0.2, 10.5, 2.5);

                    nextTask = Task.LIFT_SKYSTONE2;
                    break;

                case LIFT_SKYSTONE2:

                    turnServo.setPosition(ARM_DOWN);

                    sleep(700);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(600);

                    turnServo.setPosition(ARM_UP);

                    sleep(600);

                    nextTask = Task.DROP_SKYSTONE2;
                    break;

                case DROP_SKYSTONE2:

                    encoderDrive(0.3, -5, 10);
                    encoderStrafeLeft(0.5, 44 + d + 24, 20);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    sleep(500);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    nextTask = Task.PARK;
                    break;

                case PARK:
                    encoderStrafeRight(0.4, 14, 10);
                    nextTask = Task.HALT;
                    break;

                case HALT:

                    break;



            }


        }

    }



}



