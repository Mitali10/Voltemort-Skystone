/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="RoverTest:RoverIMURightTurn",group = "RoverTest")
@Disabled
public class RoverIMURightTurn extends OpMode {

    private DcMotor wheelFrontRight , wheelFrontLeft, wheelRearRight , wheelRearLeft;

    private final double RUN_STRAIGHT_SPEED = 0.6;
    private final double TURNING_SPEED = 0.2;
    private final double BACKUP_SPEED =0.6;


    private final int ENCODER_CPR = 1680;  //1120 for Neverest 40s
    private  final double GEAR_RATIO = 1;   //changed 1/29
    private final int WHEEL_DIA = 6;  //in inches


    private BNO055IMU imu;
    private Orientation angles;
    private Acceleration gravity;

    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void init() {

        wheelFrontRight = hardwareMap.get(DcMotor.class,"wheelFrontRight");
        wheelFrontLeft = hardwareMap.get(DcMotor.class,"wheelFrontLeft");
        wheelRearRight = hardwareMap.get(DcMotor.class,"wheelRearRight");
        wheelRearLeft = hardwareMap.get(DcMotor.class,"wheelRearLeft");


        wheelFrontLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearLeft.setDirection(DcMotor.Direction.FORWARD);
        wheelRearLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelFrontRight.setDirection(DcMotor.Direction.REVERSE);
        wheelFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        wheelRearRight.setDirection(DcMotor.Direction.REVERSE);
        wheelRearRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit            = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit            = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled       = false;
        parameters.useExternalCrystal   = true;
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();


        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

    }


    @Override
    public void loop() {

        if ((1 >= getHeading()) || getHeading() >= 270){

            telemetry.addData("Right now at ",getHeading());

            wheelFrontLeft.setPower(TURNING_SPEED);
            wheelRearLeft.setPower(TURNING_SPEED);
            wheelFrontRight.setPower(-TURNING_SPEED);
            wheelRearRight.setPower(-TURNING_SPEED);

        } else {

            wheelFrontLeft.setPower(0);
            wheelRearLeft.setPower(0);
            wheelFrontRight.setPower(0);
            wheelRearRight.setPower(0);

            telemetry.addData("Turning Completed", getHeading());
            //requestOpModeStop();
        }

    }


    public double getHeading() {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return (angles.firstAngle+360)%360;
    }


    public double getError(double targetAngle){
        double angleError = 0;

        angleError = (targetAngle - getHeading());
        angleError -= (360*Math.floor(0.5+((angleError)/360.0)));

        return angleError;

    }

}
