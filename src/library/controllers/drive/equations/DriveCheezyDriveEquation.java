package library.controllers.drive.equations;

import library.controllers.Controller;
import library.controllers.statesandsignals.InputState;
import library.controllers.statesandsignals.OutputSignal;

import org.usfirst.frc.team3309.driverstation.Controls;
import org.usfirst.team3309.subsystems.Drive;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * <p>
 * Drive controller invented by Team 254,
 * https://github.com/Team254/FRC-2017-Public
 * /blob/master/src/com/team254/lib/util/CheesyDriveHelper.java
 * 
 * <p>
 * Helper class to implement "Cheesy Drive". "Cheesy Drive" simply means that
 * the "turning" stick controls the curvature of the robot's path rather than
 * its rate of heading change. This helps make the robot more controllable at
 * high speeds. Also handles the robot's quick turn functionality - "quick turn"
 * overrides constant-curvature turning for turn-in-place maneuvers.
 */
public class DriveCheezyDriveEquation extends Controller {

	public DriveCheezyDriveEquation() {
		Drive.getInstance().changeToPercentMode();
	}

	private double oldWheel, quickStopAccumulator;
	private double throttleDeadband = 0.02;
	private double wheelDeadband = 0.07;

	@Override
	public OutputSignal getOutputSignal(InputState input) {
		double throttle = Controls.driverRemote.getY(Hand.kLeft);
		double wheel = Controls.driverRemote.getX(Hand.kRight);
		boolean isQuickTurn = Controls.driverRemote.getBumper(Hand.kRight);
		boolean isHighGear = true;
		OutputSignal signal = new OutputSignal();
		double wheelNonLinearity;

		wheel = handleDeadband(wheel, wheelDeadband);
		throttle = handleDeadband(throttle, throttleDeadband);

		double negInertia = wheel - oldWheel;
		oldWheel = wheel;

		if (isHighGear) {
			wheelNonLinearity = 0.6;
			// Apply a sin function that's scaled to make it feel better.
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
					/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
					/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		} else {
			wheelNonLinearity = 0.5;
			// Apply a sin function that's scaled to make it feel better.
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
					/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
					/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
			wheel = Math.sin(Math.PI / 2.0 * wheelNonLinearity * wheel)
					/ Math.sin(Math.PI / 2.0 * wheelNonLinearity);
		}

		double leftPwm, rightPwm, overPower;
		double sensitivity = .3;

		double angularPower;
		double linearPower;

		// Negative inertia!
		double negInertiaAccumulator = 0.0;
		double negInertiaScalar = 5;
		if (isHighGear) {

			negInertiaScalar = 5.0;
		} else {
			/*
			 * if (wheel * negInertia > 0) { negInertiaScalar = 1.75; } else {
			 * if (Math.abs(wheel) > 0.65) { negInertiaScalar = 4.0; } else {
			 * negInertiaScalar = 3.0; } } sensitivity = .6;
			 */
		}
		double negInertiaPower = negInertia * negInertiaScalar;
		negInertiaAccumulator += negInertiaPower;

		wheel = wheel + negInertiaAccumulator;
		if (negInertiaAccumulator > 1) {
			negInertiaAccumulator -= 1;
		} else if (negInertiaAccumulator < -1) {
			negInertiaAccumulator += 1;
		} else {
			negInertiaAccumulator = 0;
		}
		linearPower = throttle;

		if (throttle > .4) {
			sensitivity = .15;
		} else {
			sensitivity = .3;
		}
		// Quickturn!
		if (isQuickTurn) {
			if (Math.abs(linearPower) < 0.2) {
				double alpha = 0.05;
				quickStopAccumulator = (1 - alpha) * quickStopAccumulator
						+ alpha * limit(wheel, 1.0) * 5;
			}
			overPower = 1.0;
			if (isHighGear) {
				sensitivity = .455;
			} else {
				sensitivity = .455;
			}
			angularPower = wheel * sensitivity;
			if (wheel == 1 && wheel == -1) {
				angularPower = wheel;
			}
		} else {
			overPower = 0.0;
			angularPower = Math.abs(throttle) * wheel * sensitivity
					- quickStopAccumulator;
			if (quickStopAccumulator > 1) {
				quickStopAccumulator -= 1;
			} else if (quickStopAccumulator < -1) {
				quickStopAccumulator += 1;
			} else {
				quickStopAccumulator = 0.0;
			}
		}

		rightPwm = leftPwm = linearPower;
		leftPwm -= angularPower;
		rightPwm += angularPower;

		if (leftPwm > 1.0) {
			rightPwm -= overPower * (leftPwm - 1.0);
			leftPwm = 1.0;
		} else if (rightPwm > 1.0) {
			leftPwm -= overPower * (rightPwm - 1.0);
			rightPwm = 1.0;
		} else if (leftPwm < -1.0) {
			rightPwm += overPower * (-1.0 - leftPwm);
			leftPwm = -1.0;
		} else if (rightPwm < -1.0) {
			leftPwm += overPower * (-1.0 - rightPwm);
			rightPwm = -1.0;
		}
		signal.setLeftMotor(leftPwm);
		signal.setRightMotor(rightPwm);
		SmartDashboard.putNumber("left", leftPwm);
		SmartDashboard.putNumber("right", rightPwm);
		return signal;
	}

	public double handleDeadband(double val, double deadband) {
		return (Math.abs(val) > Math.abs(deadband)) ? val : 0.0;
	}

	public static double limit(double v, double limit) {
		return (Math.abs(v) < limit) ? v : limit * (v < 0 ? -1 : 1);
	}

	@Override
	public void reset() {

	}

	@Override
	public boolean isCompleted() {
		return false;
	}

}