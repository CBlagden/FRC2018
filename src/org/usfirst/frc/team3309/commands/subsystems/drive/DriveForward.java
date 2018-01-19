package org.usfirst.frc.team3309.commands.subsystems.drive;

import edu.wpi.first.wpilibj.command.Command;
import lib.controllers.drive.DrivePositionController;
import lib.controllers.drive.DriveSignal;
import lib.controllers.drive.DriveState;
import lib.controllers.pid.PIDConstants;
import org.usfirst.frc.team3309.robot.Robot;
import org.usfirst.frc.team3309.robot.RobotMap;

public class DriveForward extends Command {

    private DrivePositionController  drivePositionController;

    // inches
    private final double goalPos;

    private final double goalAngle;

    public DriveForward(double goalPos, double goalAngle) {
        this.goalPos = goalPos;
        this.goalAngle = goalAngle;
        PIDConstants pidConstantsLinear = new PIDConstants(RobotMap.DRIVE_POSITION_CONTROLLER_P_SCALE,
                RobotMap.DRIVE_POSITION_CONTROLLER_I_SCALE,
                RobotMap.DRIVE_POSITION_CONTROLLER_D_SCALE);
        drivePositionController = new DrivePositionController(pidConstantsLinear, pidConstantsLinear, new PIDConstants());
        requires(Robot.drive);
    }

    public DriveForward(double goalPos) {
        this(goalPos, 0);
    }

    @Override
    protected void initialize() {
        Robot.drive.changeToPercentMode();
    }

    @Override
    protected void execute() {
        DriveState driveState = new DriveState(Robot.drive.getEncoderPos(), Robot.drive.getAngPos());
        DriveSignal driveSignal = drivePositionController.update(driveState,
                Robot.drive.encoderCountsToInches(goalPos),  goalAngle);
        Robot.drive.setLeftRight(driveSignal.getLeftMotor(), driveSignal.getRightMotor());
    }

    @Override
    protected boolean isFinished() {
        return drivePositionController.isFinished();
    }

}
