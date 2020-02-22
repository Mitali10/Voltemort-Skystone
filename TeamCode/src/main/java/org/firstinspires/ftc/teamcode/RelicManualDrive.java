package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RelicTest:ManualDrive",group = "RelicTest")
@Disabled

public class RelicManualDrive extends OpMode{


    private DcMotor wheelRight , wheelLeft;


    @Override
    public void init() {

        wheelLeft = hardwareMap.get(DcMotor.class,"motorLeft");
        wheelRight = hardwareMap.get(DcMotor.class,"motorRight");

        wheelLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelRight.setDirection(DcMotor.Direction.FORWARD);
        wheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

    @Override
    public void loop() {

        /*
		 * Gamepad 1
		 *
		 * Gamepad 1 controls the motors via the left stick
		 */

        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right


        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;
       //float direction = gamepad1.right_stick_y;


        //calculate turning differential
        //Original

        //float right = throttle - direction;
        //float left = throttle + direction;

        float left = throttle + direction;
        float right = throttle - direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.

        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // Switching from linear to cubic scaling for speed

        /*
        BigDecimal bg1 = new BigDecimal(right);
        BigDecimal bg2 = new BigDecimal(left);
        right = bg1.pow(3).floatValue();
        left = bg2.pow(3).floatValue();
        */


        // set the values to the motors

            wheelRight.setPower(right);
            wheelLeft.setPower(left);



        telemetry.addData("Text", "*** Robot Data***");
        telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));




    }

    /*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.85, 1.00, 1.00 };


      //  double[] scaleArray = { 0.0, 0.02, 0.05, 0.08, 0.10, 0.11, 0.12, 0.13,
      //          0.15, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.30, 0.40 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 16.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 16) {
            index = 16;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray[index];
        } else {
            dScale = scaleArray[index];
        }

        // return scaled value.
        return dScale;
    }
}
