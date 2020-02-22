package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="RelicTest:LEDTest",group = "RelicTest")
@Disabled

public class RelicLEDTest extends LinearOpMode {

    static final double INCREMENT   = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int    CYCLE_MS    =  20;     // period of each cycle
    static final double MAX_FWD     =  0.60;     // Maximum FWD power applied to motor
    static final double MAX_REV     = 0.00;     // Maximum REV power applied to motor

    // Define class members
    DcMotor led;
	private ColorSensor color;
    double  ledPower   = 0;
    boolean ledRampUp  = true;


    @Override
    public void runOpMode() {

        led = hardwareMap.get(DcMotor.class, "led");
		color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(false);
        telemetry.addData(">", "Press Start to run LEDs" );
        telemetry.update();
        waitForStart();

        
        while(opModeIsActive()) {

        
            if (ledRampUp) {
                // Keep stepping up until we hit the max value.
                ledPower += INCREMENT ;
                if (ledPower >= MAX_FWD ) {
                    ledPower = MAX_FWD;
                    ledRampUp = !ledRampUp;   // Switch ramp direction
                }
            }
            else {
                // Keep stepping down until we hit the min value.
                ledPower -= INCREMENT ;
                if (ledPower <= MAX_REV ) {
                    ledPower = MAX_REV;
                    ledRampUp = !ledRampUp;  // Switch ramp direction
                }
            }

            // Display the current value
            telemetry.addData("LED Power", "%5.2f", ledPower);
            telemetry.update();

            // Set the LED to the new power and pause;
            led.setPower(ledPower);
            sleep(CYCLE_MS);
            idle();
        }

        // Turn off LED and signal done;
        led.setPower(0);
        telemetry.addData(">", "Done");
        telemetry.update();

    }
}
