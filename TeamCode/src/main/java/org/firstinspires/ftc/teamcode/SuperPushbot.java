package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
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
 * Motor channel:   Right ball shooter motor:       "bs_right"
 * Motor channel:   Left ball shooter motor:        "bs_left"
 */

public class SuperPushbot
{
    /* Public OpMode members. */
    public DcMotor frontLeftMotor = null;
    public DcMotor  frontRightMotor  = null;
    public DcMotor backLeftMotor = null;
    public DcMotor  backRightMotor  = null;
    public DcMotor pitchRight = null;
    public DcMotor pitchLeft = null;
    public CRServo one         = null;
    public CRServo two         = null;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public SuperPushbot(){

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
        pitchLeft = hwMap.dcMotor.get("bs_left");
        pitchLeft = hwMap.dcMotor.get("bs_right");
        one = hwMap.crservo.get("one");
        two = hwMap.crservo.get("two");


        // Set motor direction (Invert for AndyMark motors)
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        pitchLeft.setDirection(DcMotor.Direction.FORWARD);
        pitchLeft.setDirection(DcMotor.Direction.REVERSE);


        // Set all motors to zero power
        //frontLeftMotor.setPower(0);
        //frontRightMotor.setPower(0);
        //backLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        pitchLeft.setPower(0);
        pitchRight.setPower(0);

        //Set all servos to zero power
        one.setPower(0.0);
        two.setPower(0.0);

        // Set braking behavior
        pitchLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        pitchRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitchLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pitchRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

