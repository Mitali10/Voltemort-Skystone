package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;


@Autonomous(name = "SkystoneAutoBlueTF", group = "Skystone")
@Disabled
public class SkystoneAutoBlueTF extends SkystoneAutonomousBase {

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
//                    encoderStrafeLeft(0.3, 7, 10);
//
//                    if(checkRecognitions(0.8)) {
//                        pos = 2;
//                        d = 8;
//                        nextTask = Task.LIFT_SKYSTONE;
//                        break;
//                    }

                    pos = 3;    // closest to skybridge
                    encoderStrafeLeft(0.3, 8, 10);


                    nextTask = Task.LIFT_SKYSTONE;


                    break;


                case LIFT_SKYSTONE:

                    turnServo.setPosition(ARM_DOWN);

                    sleep(500);

                    encoderDriveForwardUntilDistance(0.2, 10, 2);

                    encoderDrive(0.4, 2, 2);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(400);

                    turnServo.setPosition(ARM_UP);

                    sleep(400);

                    nextTask = Task.DRIVE_TO_FOUNDATION;
                    break;

                case DRIVE_TO_FOUNDATION:

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    encoderDrive(0.3, -5, 10);

                    /*   Found out that the foundation is closer to the wall by full 4 inches
                         Adding 4 inches to the original strafing distance to account for that
                    encoderStrafeRight(0.6, 47 + d + 15, 20);
                    */

                    encoderStrafeLeft(0.6, 47 + d + 15 + 4, 20);

                    turnToZero();

                    turnServo.setPosition(ARM_UP + 0.13);

                    encoderDrive(0.3, 6, 5);

                    nextTask = Task.DROP_SKYSTONE;
                    break;

                case DROP_SKYSTONE:

                    encoderDriveForwardUntilDistance(0.2, 9, 2);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    sleep(500);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    nextTask = Task.RETURN_TO_STONES;

                    break;

                case RETURN_TO_STONES:

                    encoderDrive(0.5, -8, 5);

                    turnServo.setPosition(ARM_UP);

                    foundationServo1.setPosition(1.0);  // horiz pos
                    foundationServo2.setPosition(1.0);  // horiz pos

                    if(pos == 1) {
                        d = -26;
                    } else if(pos == 2) {
                        d = d - 3;
                    } else if(pos == 3) {
                        d--;
                    }

                    encoderStrafeRight(0.6, 48 + d + 24 + 15 + 5, 10);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    turnToZero();

                    turnServo.setPosition(ARM_DOWN);

                    encoderDriveForwardUntilDistance(0.2, 10, 2);


                    nextTask = Task.LIFT_SKYSTONE2;
                    break;

                case LIFT_SKYSTONE2:

                    sleep(400);

                    encoderDrive(0.4, 2, 2);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    sleep(400);

                    turnServo.setPosition(ARM_UP);

                    sleep(400);

                    nextTask = Task.DROP_SKYSTONE2;
                    break;

                case DROP_SKYSTONE2:

                    encoderDrive(0.4, -8, 10);

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos

                    encoderStrafeLeft(0.6, 48 + d + 24 + 24 + 6, 20);

                    turnServo.setPosition(ARM_UP + 0.15);

                    turnToZero();

                    encoderDrive(0.5, 4, 10);

                    nextTask = Task.MOVE_FOUNDATION;
                    break;


                case MOVE_FOUNDATION:

                    encoderDriveForwardUntilDistance(0.2, 10, 2);
                    encoderDrive(0.2, 3, 2);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    foundationServo1.setPosition(1.0);
                    foundationServo2.setPosition(1.0);

                    sleep(500);

                    encoderDrive(0.6, -28, 10);

                    gripperServo.setPosition(GRIPPER_CLOSE_POS);

                    turnLeft(85);

                    turnTo90Left();

                    foundationServo1.setPosition(0.6);  // horiz pos
                    foundationServo2.setPosition(0.55);  // horiz pos
                    sleep(300);

                    nextTask = Task.PARK;
                    break;

                case PARK:
                    turnServo.setPosition(0.0);

                    encoderStrafeRight(0.6, 13, 10);
                    encoderDrive(0.8, -42, 10);

                    nextTask = Task.HALT;
                    break;


                case HALT:
                    break;

            }


        }

    }



}



