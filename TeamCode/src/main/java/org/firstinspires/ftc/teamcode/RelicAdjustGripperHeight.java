package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by volt-e-mort on 11/15/2018.
 */
@TeleOp(name="RelicTeleOp:AdjustGripperHeight",group = "RelicTeleOp")
@Disabled
public class RelicAdjustGripperHeight extends OpMode{


    private DcMotor towerMotor;


    @Override
    public void init() {

        towerMotor = hardwareMap.get(DcMotor.class,"towerMotor");

        towerMotor.setDirection(DcMotor.Direction.REVERSE);
        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }


    @Override
    public void loop() {

		// Gamepad 1 controls the motors via the left stick
        // throttle: left_stick_y ranges from -1 to 1, where -1 is full up, and
        // 1 is full down
        // direction: left_stick_x ranges from -1 to 1, where -1 is full left
        // and 1 is full right


        if (gamepad1.dpad_up || gamepad2.dpad_up) {

            towerMotor.setPower(0.2);

        } else if (gamepad2.dpad_down || gamepad1.dpad_down) {      //Added support for limit switch
       // } else if (gamepad2.dpad_down) {

            towerMotor.setPower(-0.10);

        } else {

            towerMotor.setPower(0);
        }


    }

}