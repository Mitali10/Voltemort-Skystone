package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoBlueFull", group = "Skystone")
@Disabled
public class SkystoneAutoBlueFullOld extends SkystoneAutonomousBase {

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

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    encoderDrive(0.3, -4, 10);

                    if(pos == 2) {
                        d = 8;
                    } else if(pos == 1) {
                        d = 16 + 12; // + 12 for foundation
                    }

                    telemetry.addData("d:", d);
                    telemetry.update();

                    encoderStrafeLeft(0.5, 47 + d + 15, 20);

                    turnServo.setPosition(ARM_UP + 0.1);

                    encoderDrive(0.5, 10, 5);

                    nextTask = Task.DROP_SKYSTONE;
                    break;

                case DROP_SKYSTONE:

                    if(pos == 1) {                          // if the other block is too close to the wall to pick up
                        nextTask = Task.MOVE_FOUNDATION;
                    } else {
                        nextTask = Task.RETURN_TO_STONES;
                    }

                    break;

                case RETURN_TO_STONES:

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    sleep(600);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    encoderDrive(0.5, -10, 5);

                    foundationServo1.setPosition(1.0);  // horiz pos
                    foundationServo2.setPosition(1.0);  // horiz pos

                    encoderStrafeRight(0.4, 47 + d + 24 + 15, 10);

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

                    encoderDrive(0.4, -5, 10);

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    encoderStrafeLeft(0.5, 48 + d + 24 + 24, 20);

                    turnServo.setPosition(ARM_UP + 0.1);

                    encoderDrive(0.5, 12, 10);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    sleep(500);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    encoderDrive(0.5, -8, 5);

                    nextTask = Task.PARK;
                    break;

                case PARK:
                    turnServo.setPosition(0.0);
                    encoderStrafeRight(0.4, 44, 10);
                    nextTask = Task.HALT;
                    break;

                case MOVE_FOUNDATION:

                    encoderDriveForwardUntilDistance(0.2, 7.5, 10);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    foundationServo1.setPosition(1.0);
                    foundationServo2.setPosition(1.0);

                    sleep(500);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    encoderDrive(0.4, -27, 10);
                    turnLeft(90);

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos
                    sleep(500);

                    nextTask = Task.PARK2;
                    break;

                case PARK2:
                    turnServo.setPosition(0.0);
                    encoderStrafeRight(0.4, 10, 10);
                    encoderDrive(0.4, -35, 10);
                    nextTask = Task.HALT;
                    break;


                case HALT:

                    break;



            }


        }

    }



}



