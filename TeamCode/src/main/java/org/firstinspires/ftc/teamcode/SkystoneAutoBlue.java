package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoBlue2", group = "Skystone")
//@Disabled
public class SkystoneAutoBlue extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();
        initOpenCV("blue");

        int pos = -1;
        int d = 0;
        String loc = "";

        nextTask = Task.LOCATE_SKYSTONE;

        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {


            switch (nextTask) {

                case LOCATE_SKYSTONE:

                    loc = pipeline.location;

                    telemetry.addData("location:", loc);
                    telemetry.update();

                    nextTask = Task.APPROACH_STONES;
                    break;

                case APPROACH_STONES:
                    gripperServo.setPosition(GRIPPER_OPEN_POS);
                    encoderDrive(0.4, 23, 10);

                    if(loc == "right") {
                        pos = 1;
                        d = 16;
                        encoderStrafeRight(0.4, 8, 6);
                    } else if(loc == "center") {
                        pos = 2;
                        d = 8;
                        //encoderStrafeRight(0.4, 2, 5);
                    } else {
                        pos = 3;
                        encoderStrafeLeft(0.4, 8, 8);
                    }

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

                    encoderDrive(0.5, -9, 5);

                    turnServo.setPosition(ARM_UP);

                    foundationServo1.setPosition(1.0);  // horiz pos
                    foundationServo2.setPosition(1.0);  // horiz pos

                    if(pos == 1) {
                        d = -26;
                    } else if(pos == 2) {
                        d = d - 1;
                    } else if(pos == 3) {
                        d--;
                    }

                    encoderStrafeRight(0.6, 48 + d + 24 + 15 + 5, 10);

                    turnServo.setPosition(ARM_DOWN);

                    gripperServo.setPosition(GRIPPER_OPEN_POS);

                    turnToZero();

                    sleep(200);

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

                    encoderStrafeRight(0.6, 15, 10);
                    encoderDrive(0.7, -40, 10);

                    nextTask = Task.HALT;
                    break;


                case HALT:
                    break;

            }


        }

    }



}



