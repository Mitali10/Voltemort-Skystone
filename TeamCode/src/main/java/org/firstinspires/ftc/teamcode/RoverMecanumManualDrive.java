package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name = "RoverTeleOp:MecanumManualDrive", group = "RoverTest")
@Disabled

public class RoverMecanumManualDrive extends OpMode {
    private CRServo harvesterServo;
    private Servo markerServo, markerServoSide;
    private DigitalChannel limitSwitch, armLimitSwitch;
    private DcMotor wheelFrontRight, wheelFrontLeft, wheelRearRight, wheelRearLeft, linearMotor, linearMotor2, armMotor, linearSlideMotor;
    private boolean frontFacing = true, normalSpeed = true;

    double position = 0.1;
    static final double INCREMENT   = 0.001;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;      // period of each cycle
    static final double MAX_POS     =  0.60;     // Maximum rotational position
    static final double MIN_POS     =  0.10;     // Minimum rotational position


    @Override
    public void init() {

        wheelFrontRight = hardwareMap.get(DcMotor.class, "wheelFrontRight");
        wheelFrontLeft = hardwareMap.get(DcMotor.class, "wheelFrontLeft");
        wheelRearRight = hardwareMap.get(DcMotor.class, "wheelRearRight");
        wheelRearLeft = hardwareMap.get(DcMotor.class, "wheelRearLeft");

        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearRight.setDirection(DcMotor.Direction.REVERSE);
        wheelRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelFrontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelRearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);;
        wheelRearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearMotor = hardwareMap.dcMotor.get("linearMotor");
        linearMotor2 = hardwareMap.dcMotor.get("linearMotor2");

        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        linearMotor2.setDirection(DcMotor.Direction.FORWARD);
        linearMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        armMotor = hardwareMap.dcMotor.get("armMotor");
        armMotor.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch = hardwareMap.get(DigitalChannel.class, "mag1");   //top
        limitSwitch.setMode(DigitalChannel.Mode.INPUT);

        harvesterServo = hardwareMap.get(CRServo.class, "harvesterServo");
       // linearServo = hardwareMap.get(CRServo.class, "armServo");

        armLimitSwitch = hardwareMap.get(DigitalChannel.class, "magArm");
        armLimitSwitch.setMode(DigitalChannel.Mode.INPUT);


        markerServo = hardwareMap.get(Servo.class, "markerServo");
        markerServoSide = hardwareMap.get(Servo.class, "markerServoSide");
        markerServo.setPosition(1.0);
        markerServoSide.setPosition(1.0);

    }

    @Override
    public void loop() {


        /* gamepad1 controls */
        if (gamepad1.x) {
            frontFacing = false;
        } else if (gamepad1.y) {
            frontFacing = true;
        }


        if (gamepad1.right_bumper) {
            normalSpeed = false;
        } else if (gamepad1.left_bumper) {
            normalSpeed = true;
        }

        /* gamepad2 controls */

        if (gamepad2.dpad_down) {  //press the up button --> T goes down --> robot goes up
            linearMotor.setPower(1.0);
            linearMotor2.setPower(1.0); //motor is moving ccw, gear is moving cw
        } else if (gamepad2.dpad_up && limitSwitch.getState()) {
            linearMotor.setPower(-1.0);
            linearMotor2.setPower(-1.0);
        } else {
            linearMotor.setPower(0);
            linearMotor2.setPower(0);
        }


        if (gamepad2.left_stick_y < -0.2 && armLimitSwitch.getState()) {
            armMotor.setPower(0.3);
        } else if (gamepad2.left_stick_y  > 0.2) {
            armMotor.setPower(-0.3);
        } else {
            armMotor.setPower(0);
        }


        if(gamepad2.right_bumper) {
            harvesterServo.setPower(0.8);
        } else if(gamepad2.left_bumper) {
            harvesterServo.setPower(-0.8);
        } else {
            harvesterServo.setPower(0.0);
        }


        if (gamepad2.right_stick_y < -0.2) {
            linearSlideMotor.setPower(-0.8);
        } else if (gamepad2.right_stick_y  > 0.2) {
            linearSlideMotor.setPower(0.8);
        } else {
            linearSlideMotor.setPower(0.0);
        }




        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        float throttle = -gamepad1.left_stick_y;
        float straf = gamepad1.left_stick_x;
        float direction = gamepad1.right_stick_x;

        /*
        Original code

        float frontLeft = throttle + straf + direction ;
        float rearLeft = throttle - straf + direction ;
        float frontRight = throttle - straf - direction;
        float rearRight = throttle + straf - direction;

        */


        // New code for reversing the drive while still retaining the original turns

        float frontLeft, rearLeft, frontRight, rearRight;

        if (frontFacing) {

            frontLeft = throttle + straf + direction;
            rearLeft = throttle - straf + direction;
            frontRight = throttle - straf - direction;
            rearRight = throttle + straf - direction;

        } else {
            /*
            frontLeft = -throttle + straf + direction;
            rearLeft = -throttle - straf + direction;
            frontRight = -throttle - straf - direction;
            rearRight = -throttle + straf - direction;
            */
            frontLeft = -throttle - straf + direction;
            rearLeft = -throttle + straf + direction;
            frontRight = -throttle + straf - direction;
            rearRight = -throttle - straf - direction;


        }


        // clip the right/left values so that the values never exceed +/- 1
        frontLeft = Range.clip(frontLeft, -1, 1);
        rearLeft = Range.clip(rearLeft, -1, 1);
        frontRight = Range.clip(frontRight, -1, 1);
        rearRight = Range.clip(rearRight, -1, 1);


        if (normalSpeed) {
            frontLeft = (float) scaleInput(frontLeft);
            rearLeft = (float) scaleInput(rearLeft);
            frontRight = (float) scaleInput(frontRight);
            rearRight = (float) scaleInput(rearRight);
        } else {
            frontLeft = (float) scaleInputSlow(frontLeft);
            rearLeft = (float) scaleInputSlow(rearLeft);
            frontRight = (float) scaleInputSlow(frontRight);
            rearRight = (float) scaleInputSlow(rearRight);
        }


        // set the values to the motors
        wheelFrontRight.setPower(frontRight);
        wheelRearRight.setPower(rearRight);
        wheelFrontLeft.setPower(frontLeft);
        wheelRearLeft.setPower(rearLeft);
//
//        if(not frontfacting and joystick indicating strafe) {
//            -1 of corresponding wheels
//        } else {
//            do normal (lines 211-214)
//        }


        //telemetry.addData("Text", "*** Robot Data***");
        //telemetry.addData("left tgt pwr", "left  pwr: " + String.format("%.2f", left));
        //telemetry.addData("right tgt pwr", "right pwr: " + String.format("%.2f", right));
    }

    /*
     * This method scales the joystick input so for low joystick values, the
     * scaled value is less than linear.  This is to make it easier to drive
     * the robot more precisely at slower speeds.
     */
    double scaleInput(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.12, 0.15, 0.18, 0.24,
                0.30, 0.36, 0.43, 0.50, 0.60, 0.72, 0.80};

        //  double[] scaleArray = { 0.0, 0.02, 0.05, 0.08, 0.10, 0.11, 0.12, 0.13,
        //          0.15, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.30, 0.40 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * (scaleArray.length - 1));

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

    double scaleInputSlow(double dVal) {
        double[] scaleArray = {0.0, 0.05, 0.09, 0.10, 0.11, 0.12, 0.13, 0.14,
                0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.25};

        //  double[] scaleArray = { 0.0, 0.02, 0.05, 0.08, 0.10, 0.11, 0.12, 0.13,
        //          0.15, 0.15, 0.16, 0.17, 0.18, 0.19, 0.20, 0.30, 0.40 };

        // get the corresponding index for the scaleInput array.
        int index = (int) (dVal * (scaleArray.length - 1));

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
