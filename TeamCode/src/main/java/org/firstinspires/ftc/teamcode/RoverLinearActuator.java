package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RoverTest:LinearActuatorTest",group = "RoverTest")
@Disabled

public class RoverLinearActuator extends OpMode{

    private DcMotor linearMotor,linearMotor2;
    private DigitalChannel limitSwitch1;

    @Override
    public void init() {

        linearMotor = hardwareMap.dcMotor.get("linearMotor");
        linearMotor2 = hardwareMap.dcMotor.get("linearMotor2");

        limitSwitch1 = hardwareMap.get(DigitalChannel.class, "mag1");   //top
        limitSwitch1.setMode(DigitalChannel.Mode.INPUT);

        linearMotor.setDirection(DcMotor.Direction.FORWARD);
        linearMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        linearMotor2.setDirection(DcMotor.Direction.FORWARD);
        linearMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    @Override
    public void loop() {

        if (gamepad2.dpad_down) {  //press the up button --> T goes down --> robot goes up
            linearMotor.setPower(1.0);
            linearMotor2.setPower(1.0);
            //motor is moving ccw, gear is moving cw
        } else if(gamepad2.dpad_up && limitSwitch1.getState()){
            linearMotor.setPower(-1.0);
            linearMotor2.setPower(-1.0);

        }

        linearMotor.setPower(0);
        linearMotor2.setPower(0);

        telemetry.addData("Switch State ", limitSwitch1.getState());

//        if(limitSwitch1.getState()){
//            linearMotor.setPower(-0.5);
//        }
    }

}
