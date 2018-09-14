package org.usfirst.frc.team3309.lib.controllers.motion;

import org.usfirst.frc.team3309.lib.controllers.RateLimiter;
import org.usfirst.frc.team3309.lib.controllers.helpers.AutoDriveSignal;
import org.usfirst.frc.team3309.lib.controllers.helpers.DriveVelocitySignal;
import org.usfirst.frc.team3309.lib.math.LibMath;
import org.usfirst.frc.team3309.lib.math.RigidTransform;
import org.usfirst.frc.team3309.lib.math.Translation2d;
import org.usfirst.frc.team3309.robot.Constants;

public class PurePursuitController {
	/*
	 * 1. Translation delta compared to robot 2. Find angle to path relative to
	 * robot 3. Drive towards point
	 */

	private Path robotPath;
	private boolean isReversed;
	private RateLimiter speedProfiler;

	public PurePursuitController(Path robotPath, boolean isReversed) {
		this.robotPath = robotPath;
		this.isReversed = isReversed;
		speedProfiler = new RateLimiter(100, 1000);
		if (robotPath.isEmpty()) {

		}

	}

	/**
	 * Calculates the look ahead and the desired speed for each side of the
	 * robot.
	 *
	 * @param robotPose
	 *            Robot position and gyro angle.
	 * @return Speed for each side of the robot.
	 *
	 */
	@SuppressWarnings("unchecked")
	public synchronized AutoDriveSignal calculate(RigidTransform robotPose) {
		if (isReversed) {
			robotPose = new RigidTransform(robotPose.translationMat, robotPose.rotationMat.flip());
		}
		double lookAheadDist = LibMath.coercedNormalize(speedProfiler.getLatestValue(), Constants.MinPathSpeed,
				Constants.MaxPathSpeed, Constants.MinLookAheadDistance, Constants.MaxLookAheadDistance);
		Path.DrivingData data = robotPath.getLookAheadPoint(robotPose.translationMat, lookAheadDist);
		if (data.remainingDist == 0.0) { // If robot passes point, remaining
											// distance is 0
			return new AutoDriveSignal(new DriveVelocitySignal(0, 0), true);
		}
		double robotSpeed = speedProfiler.update(data.maxSpeed, data.remainingDist);
		if (robotSpeed < 20) {
			robotSpeed = 20;
		}
		Translation2d robotToLookAhead = getRobotToLookAheadPoint(robotPose, data.lookAheadPoint);
		double radius;
		radius = getRadius(robotToLookAhead);
		double delta = (robotSpeed / radius);
		double deltaSpeed = Constants.TrackRadius * delta;


		if (isReversed) {
			robotSpeed *= -1;
		}
		double maxSpeed = Math.abs(robotSpeed) + Math.abs(deltaSpeed);
		if (maxSpeed > Constants.MaxPathSpeed) {
			robotSpeed -= Math.copySign(maxSpeed - Constants.MaxPathSpeed, robotSpeed);
		}
		return new AutoDriveSignal(new DriveVelocitySignal(robotSpeed + deltaSpeed, robotSpeed - deltaSpeed), false);
	}

	private double getRadius(Translation2d robotToLookAheadPoint) {
		// Hypotenuse^2 / (2 * X)
		double radius = Math.pow(Math.hypot(robotToLookAheadPoint.getX(), robotToLookAheadPoint.getY()), 2)
				/ (2 * robotToLookAheadPoint.getY());
		return radius;
	}

	private Translation2d getRobotToLookAheadPoint(RigidTransform robotPose, Translation2d lookAheadPoint) {
		Translation2d lookAheadPointToRobot = robotPose.translationMat.inverse().translateBy(lookAheadPoint);
		lookAheadPointToRobot = lookAheadPointToRobot.rotateBy(robotPose.rotationMat.inverse());
		return lookAheadPointToRobot;

	}

	/**
	 * Resets the time for the speed profiler.
	 */
	public void resetTime() {
		// TODO: Big Bang
		speedProfiler.reset();
	}

}
