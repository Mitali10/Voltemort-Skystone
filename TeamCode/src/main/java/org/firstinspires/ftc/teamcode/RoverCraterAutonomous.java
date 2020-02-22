package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RoverAuto:RoverCraterAutonomous", group = "RoverTest")
@Disabled
public class RoverCraterAutonomous extends RoverAutonomousBase {

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
                    nextTask = Task.BACK_UP;


                case BACK_UP:
                    encoderDrive(0.8, -13, 8);
                    nextTask = Task.SCAN;
                    telemetry.addData("nextTask:", nextTask);
                    telemetry.update();
                    break;


                case SCAN:
                    boolean result = encoderForwardSearching(0.12, 40, 20, 0.60);

                    if (result) {
                        //nextTask = Task.PUSH_MINERAL;
                        nextTask = Task.HALT;
                    } else {
                        nextTask = Task.NO_CUBE;
                        telemetry.addData("Cube not found. nextTask:", nextTask);
                        telemetry.update();
                    }

                    break;


                case PUSH_MINERAL:

                    encoderStrafeRight(0.8, 7, 6);
                    encoderStrafeLeft(0.8, 6.5, 6);


                    if(distanceTraveled < 8) {
                        nextTask = Task.CUBE_RIGHT;
                    } else if(8 < distanceTraveled && distanceTraveled < 20) {
                        nextTask = Task.CUBE_CENTER;
                    } else {
                        nextTask = Task.CUBE_LEFT;
                    }

                    //telemetry.addData("nextTask:", nextTask);
                    //telemetry.update();
                    break;

                case NO_CUBE:

                    encoderDrive(0.9, 28, 10);
                    nextTask = Task.RUN_TO_DEPOT;
                    break;


                case CUBE_LEFT:

                    encoderDrive(0.9, 31, 10);
                    nextTask = Task.RUN_TO_DEPOT;
                    break;


                case CUBE_CENTER:

                    encoderDrive(0.9, 46, 10);
                    nextTask = Task.RUN_TO_DEPOT;
                    break;


                case CUBE_RIGHT:

                    encoderDrive(0.9, 61, 10);
                    nextTask = Task.RUN_TO_DEPOT;
                    break;


                case RUN_TO_DEPOT:
                    turnLeft(41);

                    telemetry.addData("heading:", getHeading());
                    telemetry.update();

                    encoderDrive(0.9, 46, 12);
                    nextTask = Task.DROP_MARKER;
                    break;


                case DROP_MARKER:
                    markerServoSide.setPosition(1.0);
                    sleep(750);
                    nextTask = Task.RUN_TO_CRATER;
                    break;


                case RUN_TO_CRATER:
                    encoderDrive(0.9, -72, 18);

                    armMotor.setPower(0.4);
                    sleep(1000);
                    armMotor.setPower(0.0);

                    nextTask = Task.HALT;
                    break;


                case HALT:
                    break;
            }

        }
        // idle();
    }
}
