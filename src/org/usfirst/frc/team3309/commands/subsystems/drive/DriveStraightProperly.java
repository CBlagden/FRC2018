package org.usfirst.frc.team3309.commands.subsystems.drive;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3309.robot.Robot;

public class DriveStraightProperly extends Command
{
    public enum DriveStrategy {
        VELOCITY,
        MOTION_MAGIC,
        POSITION,
    }
    private DriveStrategy strategy;
    private double distance;
    private int velocityTarget = 30000;

    public DriveStraightProperly(double distance, DriveStrategy strategy)
    {
        this.strategy = strategy;
        this.distance = Robot.drive.inchesToEncoderCounts(distance);
        requires(Robot.drive);
    }

    @Override
    protected void execute()
    {
        SmartDashboard.putNumber("Robot Distance Travelled: ",Robot.drive.getEncoderPos());
        switch(strategy)
        {
            case POSITION:
                Robot.drive.changeToPositionMode();
                Robot.drive.setLeftRight(distance,distance);
                break;
            case VELOCITY:
                Robot.drive.changeToVelocityMode();
                if(distance < Robot.drive.encoderCountsToInches(Robot.drive.getEncoderPos()))
                    Robot.drive.setLeftRight(-velocityTarget,-velocityTarget);
                else
                    Robot.drive.setLeftRight(velocityTarget,velocityTarget);
                break;
            case MOTION_MAGIC:
                Robot.drive.changeToMotionMagicMode();
                Robot.drive.configLeftRightCruiseVelocity(velocityTarget,velocityTarget);
                Robot.drive.setLeftRight(distance,distance);
        }
    }

    @Override
    protected boolean isFinished()
    {
        return (Math.abs(distance-Robot.drive.encoderCountsToInches(Robot.drive.getEncoderPos())) <= .750);
    }

    @Override
    protected void end()
    {
        super.end();
        Robot.drive.disableOutput();
    }
}