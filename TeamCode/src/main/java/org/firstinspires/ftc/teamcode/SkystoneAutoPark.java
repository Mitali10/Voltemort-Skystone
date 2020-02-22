package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

import java.util.List;

import static java.lang.Math.abs;


@Autonomous(name = "SkystoneAutoPark", group = "Skystone")
@Disabled
public class SkystoneAutoPark extends SkystoneAutonomousBase {

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

                    sleep(10000);

                    encoderDrive(0.2, 24, 10);



                    break;

                case HALT:
                    break;


            }


        }

    }



}



