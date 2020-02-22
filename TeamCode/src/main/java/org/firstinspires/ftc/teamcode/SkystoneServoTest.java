
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import java.util.List;
import static java.lang.Math.abs;


@Autonomous(name = "SkystoneServoTest", group = "Skystone")
@Disabled

public class SkystoneServoTest extends SkystoneAutonomousBase {

    @Override
    public void runOpMode() {

        setupAllHardware();



        nextTask = Task.MOVE;
        telemetry.addData("Hardware setup, ready to play", "");
        telemetry.update();

        waitForStart();
        runtime.reset();
        telemetry.addData("nextTask:", nextTask);
        telemetry.update();


        while (opModeIsActive()) {


            switch (nextTask) {


                case MOVE:
                    for(double pos = 0.0; pos < 1; pos += 0.025) {
                        gripperServo.setPosition(pos);
                        telemetry.addData("position:", pos);
                        telemetry.update();
                        sleep(250);
                    }
                case HALT:
                    break;

            }


        }

        // idle();

    }



}



