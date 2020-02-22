package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Vuforia related
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.*;

/**
 * Created by Volt-e-mort on 11/15/2017.
 */

@Autonomous(name="Auto:RedRightLinear3",group = "Auto")
@Disabled
public class RelicRightREDAutonomousLinear3 extends LinearOpMode {

    enum Task{INIT_SERVO, INIT_PICTOGRAPH,INIT_BACKUP, DROP_ARM, MOVE_JEWEL, RAISE_ARM, GRAB_GLYPH, REVERSE, TURN_RIGHT, TURN_LEFT, MOVE_FORWARD, MOVE_FORWARD_AGAIN, TURN_LEFT_AGAIN, SEEK_BOX, STRADDLE, DROP_GLYPH, HALT, TEST, ADJUST_GLYPH,RUN_LED };

    Task nextTask;


    private DcMotor wheelRight,wheelLeft,towerMotor;
    private Servo gripperLeftUpper, gripperRightUpper,gripperLeftBottom, gripperRightBottom, armServo, jewelServo;

    private GyroSensor gyro;
	private ModernRoboticsI2cGyro MRgyro;
    private IntegratingGyroscope Intgyro;
    private ColorSensor color;


    private final double RUN_STRAIGHT_SPEED = 0.8;
    private final double TURNING_SPEED =0.2;
    private final double BACKUP_SPEED =0.4;
    private double towerMotorPower = 0.3;



    private final int ENCODER_CPR = 1120;
    private  final double GEAR_RATIO = 1;
    private final int WHEEL_DIA = 4;  //in inches


    private byte[] range1Cache; //The read will return an array of bytes. They are stored in this variable

    private I2cAddr RANGE1ADDRESS = new I2cAddr(0x14); //Default I2C address for MR Range (7-bit)
    private static final int RANGE1_REG_START = 0x04; //Register to start reading
    private static final int RANGE1_READ_LENGTH = 2; //Number of byte to read

    private I2cDevice RANGE1;
    private I2cDeviceSynch RANGE1Reader;
    private int ultraValue;
    private  int ultraODS;
    private ElapsedTime runtime = new ElapsedTime();

		// Vuforia related variables
	private VuforiaLocalizer vuforia;
	private int cameraMonitorViewId;
	private VuforiaLocalizer.Parameters parameters;
	private VuforiaTrackables relicTrackables;
	private VuforiaTrackable relicTemplate;
	private RelicRecoveryVuMark vuMark;
	private String PICTOGRAPH_BOX_LOCATION = "C" ; // Values L,C,R . C is default

    @Override
    public void runOpMode() {

        wheelLeft = hardwareMap.get(DcMotor.class, "motorLeft");
        wheelRight = hardwareMap.get(DcMotor.class, "motorRight");

        wheelLeft.setDirection(DcMotor.Direction.REVERSE);
        wheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        wheelRight.setDirection(DcMotor.Direction.FORWARD);
        wheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        towerMotor = hardwareMap.get(DcMotor.class, "towerMotor");

        towerMotor.setDirection(DcMotor.Direction.REVERSE);
        towerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        MRgyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        MRgyro.calibrate();
        while (MRgyro.isCalibrating()) {
            sleep(100);
        }
        MRgyro.resetZAxisIntegrator();

        color = hardwareMap.get(ColorSensor.class, "color");
        color.enableLed(false);

		/*

        RANGE1 = hardwareMap.i2cDevice.get("ultra");
        RANGE1Reader = new I2cDeviceSynchImpl(RANGE1, RANGE1ADDRESS, false);
        RANGE1Reader.engage();

	   */

        wheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        wheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        wheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
		
		  VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();
			
		  parameters.vuforiaLicenseKey = "AQ4Va/L/////AAAAmY8z7ZmR3kS9h4kjXYc/4zNVg3Zlto5QVTHJAKj9x7uf4yOcJ4OhNe4DfKQwrK20PsJyBMH8Di37xUsOzfuXh21jTiJxjzwpkgLO2EXMsirLNGqSsIGGLiIICDPmakq8qNcprbgCA1/YmbbtMGGlqeYtvR87tEs5z1gCtK0ljbMlqskDkqqIGi7JGbXa/RHDxxzaYoqOyTrV37vd45KkHs5KFVSX0zyH38pzmV3bjW81v6Za9A7Hndv8LqvE+qmp7IkogttT8+vefVTDSMLhjv5d5I9YyR/s860rByRv7IQqFsMSxlbkdFe3uIn4dI/FkVPpoZLAo/MoaaAJIvdVJ4Y0Vz8DgyGSAFuBJF2GMNY+";
		  parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
		  vuforia = ClassFactory.createVuforiaLocalizer(parameters);	
		  relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
		  relicTemplate = relicTrackables.get(0);	  
		  relicTemplate.setName("relicVuMarkTemplate"); 
		  
		  
        nextTask = Task.INIT_SERVO;
        waitForStart();
		
		//com.vuforia.CameraDevice.getInstance().setFlashTorchMode(true);
		relicTrackables.activate();
        MRgyro.resetZAxisIntegrator();
		runtime.reset();


        while (opModeIsActive()) {



            switch (nextTask) {

                case INIT_SERVO:
				
				    
                    armServo = hardwareMap.get(Servo.class, "armServo");
                    jewelServo = hardwareMap.get(Servo.class, "jewelServo");

                    armServo.setPosition(0.99);
                    jewelServo.setPosition(0.69);

                    gripperLeftBottom = hardwareMap.get(Servo.class,"gripperLeftBottom");
                    gripperLeftBottom.setPosition(0.05);

                    gripperRightBottom = hardwareMap.get(Servo.class,"gripperRightBottom");
                    gripperRightBottom.setPosition(0.90);

                    gripperLeftUpper = hardwareMap.get(Servo.class,"gripperLeftUpper");
                    gripperLeftUpper.setPosition(0.05);
            
                    gripperRightUpper = hardwareMap.get(Servo.class,"gripperRightUpper");                 
                    gripperRightUpper.setPosition(0.99);

					sleep(500);
				    nextTask = nextTask.INIT_PICTOGRAPH;
                    break;
					
					
  			   case INIT_PICTOGRAPH:
                   
				   
				   vuMark = RelicRecoveryVuMark.from(relicTemplate);
				   
				   if (vuMark == RelicRecoveryVuMark.LEFT){
                       PICTOGRAPH_BOX_LOCATION = "L";
				   } else if (vuMark == RelicRecoveryVuMark.RIGHT){
                       PICTOGRAPH_BOX_LOCATION = "R";
				   }
                   
				   telemetry.addData("Pictograph is ", vuMark);
				   telemetry.update();
				 //relicTrackables.deactivate();
                   nextTask = Task.DROP_ARM;
				   
			   break;
			

                case DROP_ARM:

                    color.enableLed(true);

                    while (armServo.getPosition() > 0.21) {
                        armServo.setPosition(armServo.getPosition() - 0.010);
                        //telemetry.addData("Arm BEING dropped", armServo.getPosition());

                    }
					/*
                    telemetry.addData("Arm COMPLETELY dropped", armServo.getPosition());
                    telemetry.update();
                    */
					
                    sleep(500);

                    nextTask = Task.MOVE_JEWEL;
                    break;

                case MOVE_JEWEL:

					
					if (color.red()>color.blue() && color.red()>color.green()){
						jewelServo.setPosition(0.01);
					} else if (color.blue()>color.red() && color.blue()>color.green()){
						jewelServo.setPosition(0.99);
					}
					
					
                    sleep(500);
                    //com.vuforia.CameraDevice.getInstance().setFlashTorchMode(false);
                    color.enableLed(false);

                    nextTask = Task.RAISE_ARM;

                    break;


                case RAISE_ARM:

                    while (armServo.getPosition() < 0.92) {
                        armServo.setPosition(armServo.getPosition() + 0.02);
                        //telemetry.addData("Arm Being Raised", armServo.getPosition());
                    }

                    jewelServo.setPosition(0.6);
                    //telemetry.addData("Arm COMPLETELY Raised", armServo.getPosition());


                   nextTask = Task.GRAB_GLYPH;

                   break;




                case GRAB_GLYPH:

                    gripperLeftBottom.setPosition(0.65);
                    gripperRightBottom.setPosition(0.25);

                    sleep(1000);

                    runtime.reset();

                    while (runtime.time() < 0.6) {
                        towerMotor.setPower(towerMotorPower);
                    }
                    towerMotor.setPower(0);

                    nextTask = Task.MOVE_FORWARD;
                    sleep(200);
                    break;


                case MOVE_FORWARD:

                    encoderDrive(0.4,24,7);
                    //changed from the normal 24
                    nextTask= Task.TURN_LEFT;
                    break;

                case TURN_LEFT:

					int turnAngle = 31;
				   
				   if (PICTOGRAPH_BOX_LOCATION.equalsIgnoreCase("L")) {
				       turnAngle = 45;
				   } else if (PICTOGRAPH_BOX_LOCATION.equalsIgnoreCase("R")) {
				       turnAngle = 11;
				   } 

                    runtime.reset();
                    while ((MRgyro.getIntegratedZValue() <= turnAngle) && runtime.seconds()<=5) {

                        //telemetry.addData("Right now at ", MRgyro.getIntegratedZValue());

                        wheelLeft.setPower(-TURNING_SPEED);
                        wheelRight.setPower(TURNING_SPEED);
                    }
                    wheelRight.setPower(0.0);
                    wheelLeft.setPower(0.0);
                    
					sleep(250);
                    nextTask= Task.MOVE_FORWARD_AGAIN;
                    //nextTask = nextTask.HALT;
                    break;


                case MOVE_FORWARD_AGAIN:

				   //double moveForward = 8.0;
                    double moveForward = 6;
				   
				   if (PICTOGRAPH_BOX_LOCATION.equalsIgnoreCase("L")) {
				       //moveForward = 13.5;
                       moveForward = 11;
				   } else if (PICTOGRAPH_BOX_LOCATION.equalsIgnoreCase("R")) {
				       //moveForward = 6.5;
                       moveForward = 3.4;
				   }
				   
                    encoderDrive(0.6, moveForward,3);

                    nextTask = Task.DROP_GLYPH;

                    break;

                case TURN_LEFT_AGAIN:


                    break;


                case DROP_GLYPH:

                    /*
                    runtime.reset();

                    while (runtime.time() < 0.5) {
                        towerMotor.setPower(-towerMotorPower);
                    }
                    towerMotor.setPower(0);
                    */

                    gripperLeftBottom.setPosition(0.35);
                    gripperRightBottom.setPosition(0.6);

                    sleep(1000);                
                    nextTask = Task.ADJUST_GLYPH;
					
                    break;


                case ADJUST_GLYPH:

                    encoderDrive(0.6, -2, 2);

                    nextTask = Task.HALT;
                    break;
					
				case RUN_LED:
				    
					DcMotor led = hardwareMap.get(DcMotor.class, "led");
					
					runtime.reset();
                    while (runtime.time()<6){
                        for (double k = 0.00;k<=0.60;k=k+0.01){
                            led.setPower(k);
                            sleep(20);
                        }

                        for (double k = 0.60;k>0.01;k=k-0.01){
                            led.setPower(k);
                            sleep(20);
                        }
                    }
					led.setPower(0);		
					nextTask = Task.HALT;
					break;

                case HALT:

                    break;

            }


        }

       // idle();

    }


    private int getEncoderPulse(double dist){

        return (int) ((dist / (Math.PI * WHEEL_DIA)) * ENCODER_CPR * GEAR_RATIO);
    }



    public void encoderDrive(double speed,
                             double distance,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = wheelLeft.getCurrentPosition() + getEncoderPulse(distance);
            newRightTarget = wheelRight.getCurrentPosition() + getEncoderPulse(distance);
            wheelLeft.setTargetPosition(newLeftTarget);
            wheelRight.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            wheelLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            wheelRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            wheelLeft.setPower(Math.abs(speed));
            wheelRight.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (wheelLeft.isBusy() && wheelRight.isBusy())) {
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
            wheelLeft.setPower(0);
            wheelRight.setPower(0);

            // Turn off RUN_TO_POSITION
            wheelLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            wheelRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);   // optional pause after each move
        }
    }
}
