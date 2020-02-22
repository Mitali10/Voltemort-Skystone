package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "RoverAuto:RightRedAutonomous", group = "RoverTest")
@Disabled
public class RoverRightRedAutonomous extends RoverAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();

        nextTask = Task.LAND;

        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {

                case LAND:
                    climbDown();
                    encoderDrive(0.8, -5, 5);
                    encoderStrafeRight(0.6, 14, 10);
                    turnRight(270);
                    nextTask = Task.RUN_RIGHT;


                case RUN_RIGHT:
                    encoderStrafeRight(0.7, 10, 8);
                    nextTask = Task.SCAN_LEFT;
                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();
                    break;


                case SCAN_LEFT:

                    boolean result = encoderStrafeLeftSearching(0.15, 38, 10, .6);
                    if (result) {
                        nextTask = Task.PUSH_MINERAL;
                    } else {
                        nextTask = Task.CUBE_LEFT;
                        telemetry.addData("cube not found, nextTask:", nextTask);
                        telemetry.update();
                    }
                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();
                    break;


                case PUSH_MINERAL:

                    encoderDrive(0.7, 20, 6);

                    if(distanceTraveled > 30) {
                        nextTask = Task.CUBE_RIGHT;
                    } else if(30 > distanceTraveled && distanceTraveled > 19) {
                        nextTask = Task.CUBE_CENTER;
                    } else {
                        nextTask = Task.CUBE_LEFT;
                    }

                    if(nextTask != Task.CUBE_CENTER) {
                        encoderDrive(0.6, -10, 6);
                    }

                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();
                    break;

                case HALT:
                    break;

                case CUBE_LEFT:
                    //turnRight(315);

                    while (getHeading() >= 223) {      //angle is 270 for 90 deg turn (315 for 45 deg)

                        wheelFrontLeft.setPower(TURNING_SPEED);
                        wheelRearLeft.setPower(TURNING_SPEED);
                        wheelFrontRight.setPower(-TURNING_SPEED);
                        wheelRearRight.setPower(-TURNING_SPEED);

                    }

                    wheelFrontLeft.setPower(0);
                    wheelRearLeft.setPower(0);
                    wheelFrontRight.setPower(0);
                    wheelRearRight.setPower(0);


                    encoderStrafeLeft(0.6, 12, 8);
                    encoderDrive(0.8, 28, 10);
                    nextTask = Task.DROP_MARKER;
                    break;

                case CUBE_CENTER:
                    encoderDrive(0.8, 22, 10);  //up from 21 to avoid silver
                    //turnLeft(45);

                    while (getHeading() <= 313) {         //angle is 90 for 90 deg turns (45 for 45 deg)

                        wheelFrontLeft.setPower(-TURNING_SPEED);
                        wheelRearLeft.setPower(-TURNING_SPEED);
                        wheelFrontRight.setPower(TURNING_SPEED);
                        wheelRearRight.setPower(TURNING_SPEED);

                    }

                    wheelFrontLeft.setPower(0);
                    wheelRearLeft.setPower(0);
                    wheelFrontRight.setPower(0);
                    wheelRearRight.setPower(0);

                    nextTask = Task.DROP_MARKER;
                    break;

                case CUBE_RIGHT:
                    //turnLeft(40);

                    while (getHeading() <= 315) {         //angle is 90 for 90 deg turns (45 for 45 deg)

                        wheelFrontLeft.setPower(-TURNING_SPEED);
                        wheelRearLeft.setPower(-TURNING_SPEED);
                        wheelFrontRight.setPower(TURNING_SPEED);
                        wheelRearRight.setPower(TURNING_SPEED);

                    }

                    wheelFrontLeft.setPower(0);
                    wheelRearLeft.setPower(0);
                    wheelFrontRight.setPower(0);
                    wheelRearRight.setPower(0);

                    encoderStrafeRight(0.6, 10, 8);
                    encoderDrive(0.8, 28, 10);
                    nextTask = Task.DROP_MARKER;
                    break;

                case DROP_MARKER:
                    markerServo.setPosition(0.0);
                    sleep(1500);
                    nextTask = Task.RUN_BACK;
                    break;

                case RUN_BACK:
                    encoderDrive(0.8, -68, 10);
                    nextTask = Task.HALT;
                    break;

            }


        }

        // idle();

    }



}



