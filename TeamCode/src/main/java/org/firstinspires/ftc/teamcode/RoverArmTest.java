package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import static java.lang.Math.abs;

@TeleOp(name = "RoverTest:RoverArmTest", group = "RoverTest")
@Disabled

public class RoverArmTest extends OpMode {

    DcMotor linearSlideMotor, armMotor;
    Servo scoopServo;

    final int ENCODER_CPR = 1120;  //1120 for Neverest 40s
    final double GEAR_RATIO = 1;   //changed 1/29
    final int WHEEL_DIA = 6;  //in inches

    ElapsedTime runtime = new ElapsedTime();

    public void init() {
        linearSlideMotor = hardwareMap.dcMotor.get("linearSlideMotor");
        armMotor = hardwareMap.dcMotor.get("armMotor");
        scoopServo = hardwareMap.get(Servo.class, "scoopServo");

        linearSlideMotor.setDirection(DcMotor.Direction.FORWARD);
        linearSlideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void loop() {

        if (gamepad2.right_trigger > 0.2) {      //open
            linearSlideMotor.setPower(1.0);
        } else if (gamepad2.left_trigger > 0.2) {     //close
            linearSlideMotor.setPower(-1.0);
        } else {
            linearSlideMotor.setPower(0.0);
        }

        if (gamepad2.y) { //up
            scoopServo.setPosition(0.5);
        }
        if (gamepad2.a) {  //down
            scoopServo.setPosition(0.0);
        }

        if (gamepad2.dpad_left) { //back
            armMotor.setPower(0.6);
            //encoderDrive(.02, 1, 1);
        } else if (gamepad2.dpad_right) { //forward
            armMotor.setPower(-0.6);
            //encoderDrive(.0.2, -1, 1);
        } else {
            armMotor.setPower(0);
        }
    }

    public void encoderDrive(double speed, double distance, double timeoutS) {
        int newTarget;

        // Ensure that the opmode is still active
        //removed if(opmode.isactive())

        // Determine new target position, and pass to motor controller
        newTarget = armMotor.getCurrentPosition() + getEncoderPulse(distance);

        armMotor.setTargetPosition(newTarget);

        // Turn On RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        runtime.reset();
        armMotor.setPower(abs(speed));


        // keep looping while we are still active, and there is time left, and both motors are running.
        // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
        // its target position, the motion will stop.  This is "safer" in the event that the robot will
        // always end the motion as soon as possible.
        // However, if you require that BOTH motors have finished their moves before the robot continues
        // onto the next step, use (isBusy() || isBusy()) in the loop test.
        while ((runtime.seconds() < timeoutS) &&
                (armMotor.isBusy())) {
                /*
                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        wheelLeft.getCurrentPosition(),
                        wheelRight.getCurrentPosition());
                telemetry.update();
				*/
        }

        // Stop all motion;
        armMotor.setPower(0);


        // Turn off RUN_TO_POSITION
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //sleep(250);   // optional pause after each move
    }


    public int getEncoderPulse(double dist) {

        return (int) ((dist / (Math.PI * WHEEL_DIA)) * ENCODER_CPR * GEAR_RATIO);
    }

}
