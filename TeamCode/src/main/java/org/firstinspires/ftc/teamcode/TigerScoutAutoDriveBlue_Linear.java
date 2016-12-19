package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 *
 * The code REQUIRES encoders
 *
 *   The desired program is:
 *   - Fire 1 ball
 *   - Load a second ball
 *   - Fire the other ball
 *   - Move forward 67 inches (onto the base, knocking off the cap ball)
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement, based off of the AutoDriveByEncoder Example
 *  This methods assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 *  Separate methods control the popper (shooter) and the pickup mechanism
 */

@Autonomous(name="TigerScout: Auto Drive BLUE", group="TigerScout")
public class TigerScoutAutoDriveBlue_Linear extends TigerScoutAutoDriveBeacon_Linear {

    @Override
    int TURN_DIRECTION() {
        return -1;
    }

    @Override
    void activateBeacon(double timeoutS) {
        //TODO: Activate Blue Beacon
    }
}
