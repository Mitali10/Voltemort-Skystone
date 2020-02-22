package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "SkystoneAutoBlueOldBackup", group = "Skystone")
@Disabled
public class SkystoneAutoBlueOldBackup extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();
        //initRecognitionTools();

        int pos = -1;
        int d = 0; //36;

        nextTask = Task.APPROACH_STONES;

        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            switch (nextTask) {

                case APPROACH_STONES:
                    gripperServo.setPosition(GRIPPER_OPEN_POS);
                    encoderDrive(0.4, 23, 10);
                    encoderStrafeRight(0.4, 5, 6);
                    sleep(3000);

                    nextTask = Task.SEARCH;
                    break;


                case SEARCH:
//                    if(checkRecognitions(0.8)) {
//                        pos = 1;    // farthest from skybridge
//                        d = 16;
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }
//
//                    encoderStrafeLeft(0.3, 9, 10);
//                    sleep(3000);
//
//                    if(checkRecognitions(0.8)) {
//                        pos = 2;
//                        d = 8;
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }

                    pos = 3;    // closest to skybridge
                    encoderStrafeLeft(0.3, 8, 10);
                    sleep(3000);

                    nextTask = Task.LIFT_SKYSTONE;

                    break;


                case LIFT_SKYSTONE:


                    turnServo.setPosition(ARM_DOWN);

                    sleep(400);

                    encoderDriveForwardUntilDistance(0.2, 10, 2.5);

                    sleep(3000);

                    encoderDrive(0.2, 2, 2);

                    sleep(3000);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(400);

                    turnServo.setPosition(ARM_UP);

                    sleep(400);

                    nextTask = Task.DRIVE_TO_FOUNDATION;
                    break;

                case DRIVE_TO_FOUNDATION:

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    encoderDrive(0.3, -7, 10);

                    sleep(3000);

                                        /*   Found out that the foundation is closer to the wall by full 4 inches
                         Adding 4 inches to the original strafing distance to account for that
                    encoderStrafeRight(0.6, 47 + d + 15, 20);
                    */

                    encoderStrafeLeft(0.6, 47 + d + 15 + 6, 20);

                    sleep(3000);

                    turnToZero();

                    turnServo.setPosition(ARM_UP + 0.13);

                    sleep(3000);

                    encoderDrive(0.5, 9, 5);

                    nextTask = Task.DROP_SKYSTONE;
                    break;

                case DROP_SKYSTONE:

                    encoderDriveForwardUntilDistance(0.2, 8, 2);

                    sleep(3000);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    sleep(300);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    nextTask = Task.RETURN_TO_STONES;

                    break;

                case RETURN_TO_STONES:

                    encoderDrive(0.5, -9, 5);

                    sleep(3000);

                    turnServo.setPosition(ARM_UP);

                    foundationServo1.setPosition(1.0);  // horiz pos
                    foundationServo2.setPosition(1.0);  // horiz pos

                    if(pos == 1) {
                        d = -24;
                    }

                    encoderStrafeRight(0.6, 48 + d + 24 + 15 + 8, 10);

                    sleep(3000);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    turnToZero();

                    nextTask = Task.LIFT_SKYSTONE2;
                    break;

                case LIFT_SKYSTONE2:

                    turnServo.setPosition(ARM_DOWN);

                    sleep(400);

                    encoderDriveForwardUntilDistance(0.2, 10, 2);

                    sleep(3000);

                    encoderDrive(0.2, 2, 2);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(400);

                    turnServo.setPosition(ARM_UP);

                    sleep(400);

                    nextTask = Task.DROP_SKYSTONE2;
                    break;

                case DROP_SKYSTONE2:

                    encoderDrive(0.4, -10, 10);

                    sleep(3000);

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    encoderStrafeLeft(0.6, 48 + d + 24 + 24 + 6, 20);

                    sleep(3000);

                    turnServo.setPosition(ARM_UP + 0.15);

                    sleep(3000);

                    turnToZero();

                    encoderDrive(0.5, 12, 10);

                    nextTask = Task.MOVE_FOUNDATION;
                    break;


                case MOVE_FOUNDATION:

                    encoderDriveForwardUntilDistance(0.2, 8, 2);

                    sleep(3000);

                    encoderDrive(0.2, 2, 2);

                    sleep(3000);


                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    foundationServo1.setPosition(1.0);
                    foundationServo2.setPosition(1.0);

                    sleep(500);

                    encoderDrive(0.5, -30, 10);

                    sleep(3000);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    turnLeft(85);

                    sleep(3000);

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    turnTo90Left();

                    sleep(300);

                    nextTask = Task.PARK;
                    break;

                case PARK:
                    turnServo.setPosition(0.0);
                    encoderStrafeRight(0.6, 15, 10);

                    sleep(3000);

                    encoderDrive(0.6, -35, 10);

                    sleep(3000);

                    nextTask = Task.HALT;
                    break;


                case HALT:

                    break;



            }


        }

    }



}



