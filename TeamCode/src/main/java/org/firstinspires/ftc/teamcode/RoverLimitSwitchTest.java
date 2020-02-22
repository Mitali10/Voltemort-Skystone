package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

/**
 * Created by shivesh on 11/15/2015.
 */
@TeleOp(name="RoverTest:MagneticLimitSwitchTest",group = "RoverTest")
@Disabled

public class RoverLimitSwitchTest extends OpMode{

    private DigitalChannel limitSwitch1, limitSwitch2;

    @Override
    public void init() {

	limitSwitch1 = hardwareMap.get(DigitalChannel.class, "mag1");
	limitSwitch2 = hardwareMap.get(DigitalChannel.class, "mag2");
	limitSwitch1.setMode(DigitalChannel.Mode.INPUT);
	limitSwitch2.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void loop() {

        telemetry.addData("Switch State ", limitSwitch1.getState());
        telemetry.addData("Switch State ", limitSwitch2.getState());
    }

}
