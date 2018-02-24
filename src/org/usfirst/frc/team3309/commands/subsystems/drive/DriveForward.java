package org.usfirst.frc.team3309.commands.subsystems.drive;

import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3309.lib.LibTimer;
import org.usfirst.frc.team3309.lib.controllers.pid.PIDConstants;
import org.usfirst.frc.team3309.lib.controllers.pid.PIDController;
import org.usfirst.frc.team3309.lib.math.Length;
import org.usfirst.frc.team3309.robot.Robot;

public class DriveForward extends Command {

    // inches
    private final double goalPos;
    private final double errorThreshold = 3;
    private double error;
    private final double CRUISE_VELOCITY = 15000; // 20000
    private PIDController turningController;
    private LibTimer timer = new LibTimer(.5);

    private boolean isInitialized = false;

    public DriveForward(Length goalPos) {
        this.goalPos = goalPos.toInches();
        requires(Robot.drive);
        turningController = new PIDController(new PIDConstants(0.0, 0, 0));
    }

    @Override
    public void initialize() {
        Robot.drive.reset();
        Robot.drive.setHighGear();
        Robot.drive.changeToBrakeMode();
        Robot.drive.setGoalPos(goalPos);
        Robot.drive.changeToMotionMagicMode();
       /* Robot.drive.configLeftRightCruiseVelocity(CRUISE_VELOCITY, CRUISE_VELOCITY);
        Robot.drive.setLeftRight(Robot.drive.inchesToEncoderCounts(Robot.drive.getGoalPos()),
                Robot.drive.inchesToEncoderCounts(Robot.drive.getGoalPos()));*/
    }
    
    @Override
    protected void execute() {
        if (!isInitialized) {
            this.initialize();
            isInitialized = true;
        }
        double adjustmentVelocity = turningController.update(Robot.drive.getAngPos(), 0.0);

        error = Robot.drive.getGoalPos() - Robot.drive.encoderCountsToInches(Robot.drive.getEncoderPos());

        Robot.drive.setLeftRight(Robot.drive.inchesToEncoderCounts(Robot.drive.getGoalPos()),
                Robot.drive.inchesToEncoderCounts(Robot.drive.getGoalPos()));
        double cv = CRUISE_VELOCITY;
        if (error < 0 ) {
            cv = -CRUISE_VELOCITY;
        }
        Robot.drive.configLeftRightCruiseVelocity(cv, cv);
        System.out.println("drive error " + error);
    }

    @Override
    protected boolean isFinished() {
        System.out.println("drive error " + error);
        return timer.isConditionMaintained(Math.abs(error) < errorThreshold);
    }

}
