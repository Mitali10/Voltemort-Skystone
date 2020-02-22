package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


/**
 * Created by volt-e-mort on 11/15/2018.
 */
@TeleOp(name="RelicTeleOp:LinearSlideDrive",group = "RelicTeleOp")
@Disabled
public class RelicLinearSlideDrive extends OpMode {


    private DcMotor slideMotor;
    private Servo relicMoverServo, relicGripperServo;
    private double slideMotorPower = 0.3;
    private int SLIDE_ENCODER_MAX = 0;
    boolean firstTime = true;


    @Override
    public void init() {

        slideMotor = hardwareMap.get(DcMotor.class, "slideMotor");
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }


    @Override
    public void loop() {

        // Gamepad 1 controls the motors via the left stick
        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right

        if (firstTime) {

          /*
          relicMoverServo = hardwareMap.get(Servo.class,"relicMoverServo");
          relicGripperServo = hardwareMap.get(Servo.class,"relicGripperServo");

          relicMoverServo.setPosition();
          relicGripperServo.setPosition();

          */
            firstTime = false;
        }


        if (-gamepad1.right_stick_y > 0.5) {

            // if ((-gamepad1.right_stick_y > 0.5) && (slideMotor.getCurrentPosition() < SLIDE_ENCODER_MAX))

            slideMotor.setPower(slideMotorPower);
            telemetry.addData("Slide is at ", slideMotor.getCurrentPosition());

        } else if (-gamepad1.right_stick_y < -0.5) {

            // if ((-gamepad1.right_stick_y < 0.5) && (slideMotor.getCurrentPosition() > 10))

            slideMotor.setPower(-slideMotorPower);
            telemetry.addData("Slide is at ", slideMotor.getCurrentPosition());

        } else {

            slideMotor.setPower(0);
        }



/*
        if (gamepad1.left_trigger>0.5){   //Grab relic

            relicGripperServo.setPosition();

        } else if (gamepad2.left_bumper){    //Move the grabbed relic up
		
            relicMoverServo.setPosition();
            
        }  else if (gamepad2.right_bumper){    //Move the grabbed relic down
		
            relicMoverServo.setPosition();
			
		}  else if (gamepad2.right_trigger>0.5){  //RELEASE relic
		
		     relicGripperServo.setPosition();

	   }

 */
    }

}
    