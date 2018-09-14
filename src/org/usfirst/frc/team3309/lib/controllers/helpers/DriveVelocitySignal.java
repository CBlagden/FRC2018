package org.usfirst.frc.team3309.lib.controllers.helpers;

public class DriveVelocitySignal {

    /*
     * Inches per second for speed
     */
    public double leftVelocity;
    public double rightVelocity;
    public double leftAcc;
    public double rightAcc;

    public DriveVelocitySignal(double left, double right) {
        this(left, 0, right, 0);
    }

    public DriveVelocitySignal(double left, double leftAcc, double right, double rightAcc) {
        leftVelocity = left;
        this.leftAcc = leftAcc;
        rightVelocity = right;
        this.rightAcc = rightAcc;
    }

}
