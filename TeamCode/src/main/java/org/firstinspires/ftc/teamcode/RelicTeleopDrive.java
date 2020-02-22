package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by volt-e-mort on 11/15/2018.
 */
@TeleOp(name="RelicTeleOp:TeleopDrive",group = "RelicTeleOp")
@Disabled
public class RelicTeleopDrive extends OpMode{


    private DcMotor wheelRight,wheelLeft,towerMotor;
    private Servo gripperLeftUpper, gripperRightUpper,gripperLeftBottom, gripperRightBottom, armServo, jewelServo;
    private double towerMotorPower = 0.3;
    private boolean holdGlyphBottom,holdGlyphUpper,holdGlyphAll ;
	private double[] scaleArray2 =  {0.00,0.00,0.00,0.00,0.00,0.00,0.00,0.05,0.05,0.05,0.05,0.05,0.05,0.05,0.06,0.07,0.08,0.09,0.10,0.11,0.12,0.13,0.15,0.16,0.17,0.18,0.20,0.21,0.23,0.24,0.26,0.27,0.29,0.31,0.32,0.34,0.36,0.38,0.39,0.41,0.43,0.45,0.47,0.49,0.51,0.53,0.55,0.58,0.60,0.62,0.64,0.67,0.69,0.71,0.74,0.76,0.79,0.81,0.84,0.87,0.89,0.92,0.95,0.97,1.00};
	private ColorSensor color;
	boolean firstTime = true;
    private DigitalChannel limitSwitch;


    @Override
    public void init() {

        wheelLeft = hardwareMap.get(DcMotor.class,"motorLeft");
        wheelRight = hardwareMap.get(DcMotor.class,"motorRight");

        wheelLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelRight.setDirection(DcMotor.Direction.FORWARD);
        wheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        towerMotor = hardwareMap.get(DcMotor.class,"towerMotor");

        towerMotor.setDirection(DcMotor.Direction.REVERSE);
        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "limit");

    }


    @Override
    public void loop() {

		// Gamepad 1 controls the motors via the left stick
        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

      if(firstTime){

          armServo = hardwareMap.get(Servo.class,"armServo");
          jewelServo = hardwareMap.get(Servo.class,"jewelServo");

          armServo.setPosition(0.92);
          jewelServo.setPosition(0.69);

          color = hardwareMap.get(ColorSensor.class,"color");
          color.enableLed(false);

          gripperLeftBottom = hardwareMap.get(Servo.class,"gripperLeftBottom");
          gripperLeftBottom.setPosition(0.05);

          gripperRightBottom = hardwareMap.get(Servo.class,"gripperRightBottom");
          gripperRightBottom.setPosition(0.90);

          gripperLeftUpper = hardwareMap.get(Servo.class,"gripperLeftUpper");
          gripperLeftUpper.setPosition(0.05);

          gripperRightUpper = hardwareMap.get(Servo.class,"gripperRightUpper");
          gripperRightUpper.setPosition(0.99);

          firstTime = false;
      }

        float throttle = -gamepad1.left_stick_y;
        float direction = gamepad1.left_stick_x;

        //calculate turning differential
        float right = throttle - direction;
        float left = throttle + direction;

        // clip the right/left values so that the values never exceed +/- 1
        right = Range.clip(right, -1, 1);
        left = Range.clip(left, -1, 1);

        // scale the joystick value to make it easier to control
        // the robot more precisely at slower speeds.

        right = (float)scaleInput(right);
        left =  (float)scaleInput(left);

        // set the values to the motors

        wheelRight.setPower(right);
        wheelLeft.setPower(left);


        if (gamepad2.dpad_up) {

            towerMotor.setPower(towerMotorPower);

        } else if (gamepad2.dpad_down && (!limitSwitch.getState())) {      //Added support for limit switch
            //pressed is false, not pressed it true
       // } else if (gamepad2.dpad_down) {

            towerMotor.setPower(-0.15);

        } else {

            towerMotor.setPower(0);
        }


        // New ARM mode to close the grippers for glyph grabbing


        if (gamepad2.left_bumper){   //ARM all grippers by subtracting a fixed decimal from the closed position.

            gripperLeftUpper.setPosition(0.50);
            gripperRightUpper.setPosition(0.55);
            gripperLeftBottom.setPosition(0.46);
            gripperRightBottom.setPosition(0.45);

        } else if (gamepad2.right_bumper){    //GRAB using all grippers top and bottom

            gripperLeftUpper.setPosition(0.69);
            gripperRightUpper.setPosition(0.35);
            gripperLeftBottom.setPosition(0.67);
            gripperRightBottom.setPosition(0.23);

        } else if (gamepad2.right_trigger>0.5){  //RELEASE all grippers

            gripperLeftUpper.setPosition(0.05);
            gripperRightUpper.setPosition(0.99);
            gripperLeftBottom.setPosition(0.05);
            gripperRightBottom.setPosition(0.9);

        } else if (gamepad2.b) {                //close top

            gripperLeftUpper.setPosition(0.69);
            gripperRightUpper.setPosition(0.35);

        } else if (gamepad2.y) {                //open top

            gripperLeftUpper.setPosition(0.05);
            gripperRightUpper.setPosition(0.99);

        } else if (gamepad2.a) {                //close bottom

            gripperLeftBottom.setPosition(0.67);
            gripperRightBottom.setPosition(0.23);

        } else if (gamepad2.x) {            //open bottom

            gripperLeftBottom.setPosition(0.05);
            gripperRightBottom.setPosition(0.9);

        }
    }


    /*
	 * This method scales the joystick input so for low joystick values, the
	 * scaled value is less than linear.  This is to make it easier to drive
	 * the robot more precisely at slower speeds.
	 */
    double scaleInput(double dVal)  {
        double[] scaleArray = { 0.0, 0.05, 0.07, 0.10, 0.12, 0.15, 0.18, 0.24, 0.30, 0.35, 0.40, 0.50, 0.60, 0.7, 0.80, 0.9, 1.00 };


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


// Improved scaling chart with 64 values using x^1.75 curve. Added 1/31/2018
	
    double scaleInput2(double dVal)  {

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * 64.0);

        // index should be positive.
        if (index < 0) {
            index = -index;
        }

        // index cannot exceed size of array minus 1.
        if (index > 64) {
            index = 64;
        }

        // get value from the array.
        double dScale = 0.0;
        if (dVal < 0) {
            dScale = -scaleArray2[index];
        } else {
            dScale = scaleArray2[index];
        }

        // return scaled value.
        return dScale;
    }
}