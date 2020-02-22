package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RoverAuto:LeftBlueAutonomous", group = "RoverTest")
@Disabled
public class RoverLeftBlueAutonomous extends RoverAutonomousBase {

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
                    encoderStrafeRight(0.6, 13, 10);
                    turnRight(273); //increased from 268 to turn LESS
                    nextTask = Task.RUN_RIGHT;


                case RUN_RIGHT:
                    encoderStrafeRight(0.7, 11, 8);
                    nextTask = Task.SCAN_LEFT;
                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();
                    break;

                case SCAN_LEFT:

                    boolean result = encoderStrafeLeftSearching(0.15, 38, 10, 0.45);
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

                    encoderDrive(0.7, 11, 6);
                    encoderDrive(0.6, -8.5, 6);

                    if(distanceTraveled > 30) {
                        nextTask = Task.CUBE_RIGHT;
                    } else if(30 > distanceTraveled && distanceTraveled > 19) {
                        nextTask = Task.CUBE_CENTER;
                    } else {
                        nextTask = Task.CUBE_LEFT;
                    }

                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();
                    break;

                case HALT:
                    break;

                case CUBE_LEFT:

                    encoderStrafeLeft(0.85, 18, 10);
                    nextTask = Task.RUN_STRAIGHT_2;
                    break;

                case CUBE_CENTER:

                    encoderStrafeLeft(0.8, 30, 10); //drop to 17?
                    nextTask = Task.RUN_STRAIGHT_2;
                    break;

                case CUBE_RIGHT:

                    encoderStrafeLeft(0.8, 44, 10);     //increased from 42 bc not close enough to wall
                    nextTask = Task.RUN_STRAIGHT_2;
                    break;

                case RUN_STRAIGHT_2:
                    //turnLeft(135);

                    telemetry.addData("heading:", getHeading());
                    telemetry.update();

                    while (getHeading() >= 260 || getHeading() <= 40) {         //angle is 90 for 90 deg turns (45 for 45 deg)

                        wheelFrontLeft.setPower(-TURNING_SPEED);
                        wheelRearLeft.setPower(-TURNING_SPEED);
                        wheelFrontRight.setPower(TURNING_SPEED);
                        wheelRearRight.setPower(TURNING_SPEED);

                    }

                    wheelFrontLeft.setPower(0);
                    wheelRearLeft.setPower(0);
                    wheelFrontRight.setPower(0);
                    wheelRearRight.setPower(0);

                    encoderDrive(0.8, 46, 12);
                    nextTask = Task.DROP_MARKER;
                    break;

                case DROP_MARKER:
                    markerServo.setPosition(0.0);
                    sleep(1500);
                    nextTask = Task.RUN_BACK;
                    break;

                case RUN_BACK:
                    encoderDrive(0.8, -32, 18);
                    nextTask = Task.HALT;
                    break;

            }

        }
        // idle();
    }
}
