package org.usfirst.frc.team3309.commands.subsystems.drive;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3309.lib.CommandEx;
import org.usfirst.frc.team3309.lib.controllers.helpers.AutoDriveSignal;
import org.usfirst.frc.team3309.lib.controllers.motion.Path;
import org.usfirst.frc.team3309.lib.controllers.motion.PurePursuitController;
import org.usfirst.frc.team3309.lib.math.Translation2d;
import org.usfirst.frc.team3309.robot.Constants;
import org.usfirst.frc.team3309.robot.Robot;
import org.usfirst.frc.team3309.subsystems.RobotTracker;

public class DriveToPoints extends CommandEx {

    private PurePursuitController autonomousDriver;
    private AutoDriveSignal signal;

	public DriveToPoints(double speed, boolean isReversed, double accel, Translation2d... points) {

        Path drivePath = new Path(RobotTracker.getInstance().getOdometry().translationMat);
        SmartDashboard.putNumber("Initial x: ", RobotTracker.getInstance().getOdometry().translationMat.getX());
        SmartDashboard.putNumber("Initial y: ", RobotTracker.getInstance().getOdometry().translationMat.getY());

        for (Translation2d point : points) {
            drivePath.addPoint(point.getX(), point.getY(), speed);
        }
        autonomousDriver = new PurePursuitController(drivePath, isReversed, accel);
        autonomousDriver.resetTime();
        requires(Robot.drive);
	}

	@Override
	public void start() {
        Robot.drive.changeToVelocityMode();
        System.out.println("Drive To Points");
	}

	@Override
	protected void execute() {
        signal = autonomousDriver.calculate(RobotTracker.getInstance().getOdometry());
        Robot.drive.changeToVelocityMode();
        SmartDashboard.putBoolean("Path done: ", signal.isDone);
        if (signal.isDone) {
            Robot.drive.setLeftRight(0, 0);
        } else {
            double leftSetpoint = (signal.command.rightVelocity) * 4096 / (Constants.WheelDiameter * Math.PI * 10)
                    * (62d / 22d) * 3d;
            double rightSetpoint = (signal.command.leftVelocity) * 4096 / (Constants.WheelDiameter * Math.PI * 10) * (62 / 22d)
                    * 3d;
            Robot.drive.changeToVelocityMode();
            Robot.drive.setLeftRight(leftSetpoint, rightSetpoint);
            SmartDashboard.putNumber("Left Vel Setpoint: ", signal.command.leftVelocity);
            SmartDashboard.putNumber("Right Vel Setpoint: ", signal.command.rightVelocity);
        }


	}

	@Override
	public boolean isFinished() {
	    if (signal == null) {
	        return false;
        } else {
	        return signal.isDone;
        }
	}

}