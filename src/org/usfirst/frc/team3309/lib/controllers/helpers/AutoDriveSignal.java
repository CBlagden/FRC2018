package org.usfirst.frc.team3309.lib.controllers.helpers;

public class AutoDriveSignal{

    public DriveVelocitySignal command;
    public boolean isDone;

    public AutoDriveSignal(DriveVelocitySignal command, boolean isDone) {
        this.command = command;
        this.isDone = isDone;
    }

}
