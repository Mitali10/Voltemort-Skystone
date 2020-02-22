package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;


@Autonomous(name = "SkystoneAutoParkRed", group = "Skystone")
//@Disabled
public class SkystoneAutoParkRed extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();

        nextTask = Task.PARK;
        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();

        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {

                case PARK:

                    sleep(500);
                    encoderStrafeLeft(0.4,12,5);

                    nextTask = Task.HALT;

                    break;

                case HALT:
                    break;


            }


        }

    }



}



