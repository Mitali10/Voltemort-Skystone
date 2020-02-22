package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RelicTest:MotorTest",group = "RelicTest")
@Disabled

public class RelicMotorTest extends OpMode{

    private DcMotor motorLeft, motorRight, towerMotor;

    @Override
    public void init() {

        motorLeft = hardwareMap.dcMotor.get("motorLeft");

        motorLeft.setDirection(DcMotor.Direction.FORWARD);
        motorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


    }

    @Override
    public void loop() {
        motorLeft.setPower(0.5);
    }

}
