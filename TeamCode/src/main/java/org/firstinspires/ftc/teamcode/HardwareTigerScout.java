package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.ModernRoboticsConstants;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is designed by group 1.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:   Front left drive motor:         "front_left"
 * Motor channel:   Front right drive motor:        "front_right"
 * Motor channel:   Back left front drive motor:    "back_left"
 * Motor channel:   Back right back drive motor:    "back_right"
 * Motor channel:   Ball pickup motor:              "pickup"
 * Motor channel:   Ball popper motor:              "popper"
 * Servo channel:   Front flipper for beacon:       "flipper"
 * Assumes 4 inch stealth wheels
 */

public class HardwareTigerScout
{
    //Robot parameters
    private static final double POPPER_GEAR_RATIO = 3.0;
    public static final double POPPER_CPR = Motors.ANDYMARK_60_CPR * POPPER_GEAR_RATIO;
    private static final double PICKUP_GEAR_RATIO = 1.0;
    public static final double PICKUP_CPR = Motors.TETRIX_CPR * PICKUP_GEAR_RATIO;

    private static final double     DRIVE_COUNTS_PER_MOTOR_REV    = Motors.ANDYMARK_40_CPR ;
    private static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     DRIVE_COUNTS_PER_INCH         = (DRIVE_COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);


    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor backLeftMotor = null;
    public DcMotor  backRightMotor  = null;
    public DcMotor popper = null;
    public DcMotor pickup = null;
    public Servo flipper = null;
    public ModernRoboticsI2cColorSensor colorSensor = null;
    public ModernRoboticsI2cGyro gyro = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public HardwareTigerScout(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors and Servos
        frontLeftMotor = hwMap.dcMotor.get("front_left");
        frontRightMotor  = hwMap.dcMotor.get("front_right");
        backLeftMotor  = hwMap.dcMotor.get("back_left");
        backRightMotor  = hwMap.dcMotor.get("back_right");
        pickup  = hwMap.dcMotor.get("pickup");
        popper  = hwMap.dcMotor.get("popper");
        flipper = hwMap.servo.get("flipper");
        colorSensor = (ModernRoboticsI2cColorSensor) hwMap.colorSensor.get("color");
        gyro = (ModernRoboticsI2cGyro) hwMap.gyroSensor.get("gyro");

        // Set motor direction (Inverted for AndyMark motors)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        popper.setDirection(DcMotor.Direction.FORWARD);

        // Set motor direction (Normal for Tetrix/Pitsco motors)
        pickup.setDirection(DcMotorSimple.Direction.FORWARD);

        //Set Brake/Coast Behavior
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        pickup.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        popper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set drive motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        pickup.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        popper.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Set all motors to zero power
        frontLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        pickup.setPower(0);
        popper.setPower(0);

        // Set max speed for encoded motors
        backLeftMotor.setMaxSpeed(Motors.ANDYMARK_40_CPR);
        frontLeftMotor.setMaxSpeed(Motors.ANDYMARK_40_CPR);
        backRightMotor.setMaxSpeed(Motors.ANDYMARK_40_CPR);
        frontRightMotor.setMaxSpeed(Motors.ANDYMARK_40_CPR);
        popper.setMaxSpeed(Motors.ANDYMARK_60_CPR);
        pickup.setMaxSpeed((int)(Motors.TETRIX_CPR * 1.5)); //make it go faster

        //Set Run_TO_POSITION Motors to go nowhere
        popper.setTargetPosition(popper.getCurrentPosition());
        flipper.setPosition(1);
    }

    /***
     *
     * waitForTick implements a periodic delay. However, this acts like a metronome with a regular
     * periodic tick.  This is used to compensate for varying processing times for each cycle.
     * The function looks at the elapsed cycle time, and sleeps for the remaining time interval.
     *
     * @param periodMs  Length of wait cycle in mSec.
     * @throws InterruptedException
     */
    public void waitForTick(long periodMs) throws InterruptedException {

        long  remaining = periodMs - (long)period.milliseconds();

        // sleep for the remaining portion of the regular cycle period.
        if (remaining > 0)
            Thread.sleep(remaining);

        // Reset the cycle clock for the next pass.
        period.reset();
    }
}

