package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RoverTest:LeftRear",group = "RoverTest")
@Disabled

public class RoverLeftRear extends OpMode{

    private DcMotor leftRear;

    @Override
    public void init() {

        leftRear = hardwareMap.dcMotor.get("wheelRearLeft");

        leftRear.setDirection(DcMotor.Direction.FORWARD);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void loop() {
        leftRear.setPower(0.5);
    }

}
