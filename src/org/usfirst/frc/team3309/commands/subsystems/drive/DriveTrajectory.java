package org.usfirst.frc.team3309.commands.subsystems.drive;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import org.usfirst.frc.team3309.lib.geometry.Pose2dWithCurvature;
import org.usfirst.frc.team3309.lib.planners.DriveMotionPlanner;
import org.usfirst.frc.team3309.lib.trajectory.TimedView;
import org.usfirst.frc.team3309.lib.trajectory.Trajectory;
import org.usfirst.frc.team3309.lib.trajectory.TrajectoryIterator;
import org.usfirst.frc.team3309.lib.trajectory.timing.TimedState;
import org.usfirst.frc.team3309.lib.util.DriveSignal;
import org.usfirst.frc.team3309.robot.Constants;
import org.usfirst.frc.team3309.robot.Robot;
import org.usfirst.frc.team3309.robot.RobotState;

public class DriveTrajectory extends Command {

    private static final RobotState mRobotState = RobotState.getInstance();

    private final TrajectoryIterator<TimedState<Pose2dWithCurvature>> mTrajectory;
    private final boolean mResetPose;
    private DriveMotionPlanner mMotionPlanner;

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory) {
        this(trajectory, false);
    }

    public DriveTrajectory(Trajectory<TimedState<Pose2dWithCurvature>> trajectory, boolean mResetPose) {
        mTrajectory = new TrajectoryIterator<>(new TimedView<>(trajectory));
        this.mResetPose = mResetPose;
        mMotionPlanner = new DriveMotionPlanner();
    }

    @Override
    public void initialize() {
        if (mResetPose) {
            mRobotState.reset(Timer.getFPGATimestamp(), mTrajectory.getState().state().getPose());
        }
        mMotionPlanner.reset();
        mMotionPlanner.setTrajectory(mTrajectory);
    }

    @Override
    protected void execute() {
        Robot.drive.changeToVelocityMode();
        super.execute();
        double now = Timer.getFPGATimestamp();
        DriveMotionPlanner.Output output = mMotionPlanner.update(now, RobotState.getInstance().getFieldToVehicle(now));

        double leftAccel = Robot.drive.radiansPerSecondToTicksPer100ms(output.left_accel) / 1000.0;
        double rightAccel = Robot.drive.radiansPerSecondToTicksPer100ms(output.right_accel) / 1000.0;
        DriveSignal powerSignal = new DriveSignal(Robot.drive.radiansPerSecondToTicksPer100ms(output.left_velocity),
                Robot.drive.radiansPerSecondToTicksPer100ms(output.right_velocity));
        DriveSignal feedforwardSignal = new DriveSignal(output.left_feedforward_voltage / 12.0,
                output.right_feedforward_voltage / 12.0);

        Robot.drive.getLeftMaster().set(ControlMode.Velocity, powerSignal.getLeft(), DemandType.ArbitraryFeedForward,
                feedforwardSignal.getLeft() + Constants.kDriveLowGearVelocityKd * leftAccel / 1023.0);
        Robot.drive.getRightMaster().set(ControlMode.Velocity, powerSignal.getRight(), DemandType.ArbitraryFeedForward,
                feedforwardSignal.getRight() + Constants.kDriveLowGearVelocityKd * rightAccel / 1023.0);
    }

    @Override
    protected boolean isFinished() {
        return mMotionPlanner.isDone();
    }


}
