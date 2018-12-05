package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveTeleop;
import org.usfirst.frc.team3309.lib.actuators.TalonSRXMC;
import org.usfirst.frc.team3309.lib.actuators.VictorSPXMC;
import org.usfirst.frc.team3309.lib.geometry.Pose2d;
import org.usfirst.frc.team3309.lib.geometry.Pose2dWithCurvature;
import org.usfirst.frc.team3309.lib.geometry.Rotation2d;
import org.usfirst.frc.team3309.lib.trajectory.timing.TimedState;
import org.usfirst.frc.team3309.robot.Constants;
import org.usfirst.frc.team3309.robot.Robot;

public class Drive extends Subsystem {

    private TalonSRXMC left0 = new TalonSRXMC(Constants.DRIVE_LEFT_0_ID);
    private VictorSPXMC left1 = new VictorSPXMC(Constants.DRIVE_LEFT_1_ID);
    private VictorSPXMC left2 = new VictorSPXMC(Constants.DRIVE_LEFT_2_ID);

    private TalonSRXMC right0 = new TalonSRXMC(Constants.DRIVE_RIGHT_0_ID);
    private VictorSPXMC right1 = new VictorSPXMC(Constants.DRIVE_RIGHT_1_ID);
    private VictorSPXMC right2 = new VictorSPXMC(Constants.DRIVE_RIGHT_2_ID);

    private PigeonIMU pigeonIMU = new PigeonIMU(Robot.lift.getLeft1());

    private Solenoid shifter = new Solenoid(Constants.DRIVE_SHIFTER);

    private AHRS navX = new AHRS(SPI.Port.kMXP);

    private PeriodicIO mPeriodicIO;
    private Rotation2d mGyroOffset = Rotation2d.identity();

    private double goalPos;

    public Drive() {
        mPeriodicIO = new PeriodicIO();

        left0.changeToPercentMode();
        left0.configPeakOutputForward(1.0, 10);
        left0.configPeakOutputReverse(-1.0, 10);
        left0.configNominalOutputForward(0.0, 10);
        left0.configNominalOutputReverse(-0.0, 10);
        left0.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        left1.follow(left0);
        left2.follow(left0);
        left0.configOpenloopRamp(0, 10);
        left0.configMotionAcceleration(30000, 10);
        left0.config_kP(0, 0.019, 10);
        left0.config_kD(0, 0.0006, 10);
        left0.config_kF(0, 0.002, 10);



        right0.changeToPercentMode();
        right0.configPeakOutputForward(1.0, 10);
        right0.configPeakOutputReverse(-1.0, 10);
        right0.configNominalOutputForward(0.0, 10);
        right0.configNominalOutputReverse(-0.0, 10);
        right0.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 10);
        right1.follow(right0);
        right2.follow(right0);
        right0.configOpenloopRamp(0, 10);
        right0.configMotionAcceleration(30000, 10);
        right0.config_kP(0, 0.019, 10);
        right0.config_kD(0, 0.0006, 10);
        right0.config_kF(0, 0.002, 10);


        setHighGear();
        changeToBrakeMode();
    }


    public void clearPigeon() {
        DriverStation.reportWarning("The pigeon has been reset~!!!!!!~~~", false);
        pigeonIMU.setYaw(0, 10);
        pigeonIMU.setFusedHeading(0, 10);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveTeleop());
    }

    public synchronized void setHeading(Rotation2d heading) {
        System.out.println("SET HEADING: " + heading.getDegrees());

        mGyroOffset = heading.rotateBy(Rotation2d.fromDegrees(pigeonIMU.getFusedHeading()).inverse());
        System.out.println("Gyro offset: " + mGyroOffset.getDegrees());

        mPeriodicIO.gyro_heading = heading;
    }

    public TalonSRXMC getLeftMaster() {
        return left0;
    }

    public TalonSRXMC getRightMaster() {
        return right0;
    }

    public double getDistance() {
        return (getLeftDistance() + getRightDistance()) / 2.0;
    }

    public double getLeftDistance() {
        return encoderCountsToInches(getLeftEncoder());
    }

    public double getRightDistance() {
        return encoderCountsToInches(getRightEncoder());
    }

    public double encoderCountsToInches(double counts) {
        return counts / Constants.DRIVE_ENCODER_COUNTS_PER_REV * (Math.PI * Constants.WHEEL_DIAMETER_INCHES);
    }

    public double inchesToEncoderCounts(double inches) {
        return inches * (Constants.DRIVE_ENCODER_COUNTS_PER_REV / (Math.PI * Constants.WHEEL_DIAMETER_INCHES));
    }

    public double radiansPerSecondToTicksPer100ms(double rad_s) {
        return rad_s / (Math.PI * 2.0) * 4096.0 / 10.0;
    }

    public void reset() {
        left0.clearMotionProfileTrajectories();
        right0.clearMotionProfileTrajectories();
        left0.changeToDisabledMode();
        right0.changeToDisabledMode();
        left0.setSelectedSensorPosition(0, 0,0);
        right0.setSelectedSensorPosition(0, 0,0);
        navX.zeroYaw();
    }

    public void zeroNavx() {
        navX.zeroYaw();
    }

    public double getEncoderPos() {
        return (getLeftEncoder() + getRightEncoder()) / 2.0;
    }

    public double getLeftEncoder() {
        return left0.getSelectedSensorPosition(0);
    }

    public double getRightEncoder() {
        return -right0.getSelectedSensorPosition(0);
    }

    public double getEncoderVelocity() {
        return (getLeftVelocity() + getRightVelocity()) / 2.0;
    }

    public int getLeftVelocity() {
        return left0.getSelectedSensorVelocity(0);
    }

    public double getRightVelocity() {
        return -right0.getSelectedSensorVelocity(0);
    }

    public double getAngPos() {
        return -navX.getAngle();
    }

    public void resetGyro() {
        navX.reset();
    }

    public double getAngVel() {
        return navX.getRate();
    }

    public double getPigeonPos() {
        double[] ypr = new double[3];
        pigeonIMU.getYawPitchRoll(ypr);
        pigeonIMU.getAbsoluteCompassHeading();
        double yaw = ypr[0];
        return yaw;
    }

    public void sendToDashboard() {
        SmartDashboard.putData(this);
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Drive");
        if (left0.getControlMode() == ControlMode.Velocity || left0.getControlMode() == ControlMode.MotionMagic
                || left0.getControlMode() == ControlMode.Position) {
            table.getEntry("Target velocity left: ").setNumber(left0.getClosedLoopTarget(0));
            table.getEntry("Velocity left error: ").setNumber(left0.getClosedLoopError(0));
        }
        if (right0.getControlMode() == ControlMode.Velocity || right0.getControlMode() == ControlMode.MotionMagic
                || right0.getControlMode() == ControlMode.Position) {
            table.getEntry("Target velocity right: ").setNumber(right0.getClosedLoopTarget(0));
            table.getEntry("Velocity right error: ").setNumber(right0.getClosedLoopError(0));
        }
        table.getEntry("Position (Inches): ").setNumber(encoderCountsToInches(getEncoderPos()));
        table.getEntry("Velocity (Inches): ").setNumber(encoderCountsToInches(getEncoderVelocity()));
        table.getEntry("Left Position (Inches): ").setNumber(encoderCountsToInches(getLeftEncoder()));
        table.getEntry("Right Position (Inches): ").setNumber(encoderCountsToInches(getRightEncoder()));
        table.getEntry("Position (Raw): ").setNumber(getEncoderPos());
        table.getEntry("Velocity (Raw): ").setNumber(getEncoderVelocity());
        table.getEntry("Left Position (Raw): ").setNumber(getLeftEncoder());
        table.getEntry("Right Position (Raw): ").setNumber(getRightEncoder());
        table.getEntry("Left Velocity (Raw): ").setNumber(getLeftVelocity());
        table.getEntry("Right Velocity (Raw): ").setNumber(getRightVelocity());
        table.getEntry("Angular Position: ").setNumber(getAngPos());
        table.getEntry("Angular Velocity: ").setNumber(getAngVel());
        table.getEntry("Left output: ").setNumber(left0.getMotorOutputPercent());
        table.getEntry("Right output: ").setNumber(right0.getMotorOutputPercent());
        table.getEntry("Left control: ").setString(left0.getControlMode().toString());
        table.getEntry("Right control: ").setString(right0.getControlMode().toString());

        table.getEntry("left0 current ").setNumber(left0.getOutputCurrent());
        table.getEntry("left1 current ").setNumber(left1.getOutputCurrent());
        table.getEntry("left2 current ").setNumber(left2.getOutputCurrent());

        table.getEntry("right0 current ").setNumber(right0.getOutputCurrent());
        table.getEntry("right1 current ").setNumber(right1.getOutputCurrent());
        table.getEntry("right2 current ").setNumber(right2.getOutputCurrent());

        table.getEntry("pigeon angle").setNumber(getPigeonPos());

        SmartDashboard.putNumber("pigeon angle", getPigeonPos());

    }

    @Override
    public void periodic()
    {
        super.periodic();
   /*     SmartDashboard.putNumber("Robot Distance Traversed (Left): ",getLeftEncoder());
        SmartDashboard.putNumber("Robot Distance Traversed (Right): ",getRightEncoder());

        SmartDashboard.putNumber("Robot Distance Traversed (Humanized) : ",encoderCountsToInches(getEncoderPos()));
        SmartDashboard.putNumber("Robot revolution ", (getEncoderPos()/Constants.DRIVE_ENCODER_COUNTS_PER_REV));
        SmartDashboard.putNumber("Robot enc calc: ", inchesToEncoderCounts(encoderCountsToInches(getEncoderPos())));*/
    }

    public void changeToBrakeMode() {
        left0.setNeutralMode(NeutralMode.Brake);
        right0.setNeutralMode(NeutralMode.Brake);
    }

    public void changeToCoastMode() {
        left0.setNeutralMode(NeutralMode.Coast);
        right0.setNeutralMode(NeutralMode.Coast);
    }

    public void setHighGear() {
        shifter.set(false);
    }

    public void setLowGear() {
        shifter.set(true);
    }

    public void setLeftRight(double left, double right) {
        setLeft(left);
        setRight(right);
    }

    public void setLeft(double value) {
        left0.set(value);
    }

    public void setRight(double value) {
        right0.set(-value);
    }

    public void changeToPercentMode() {
        left0.changeToPercentMode();
        right0.changeToPercentMode();
    }

    public void changeToPositionMode() {
        left0.changeToPositionMode();
        right0.changeToPositionMode();
    }

    public void changeToVelocityMode() {
        left0.changeToVelocityMode();
        right0.changeToVelocityMode();
    }

    public void changeToMotionMagicMode() {
        left0.changeToMotionMagic();
        right0.changeToMotionMagic();
    }

    public void configLeftRightCruiseVelocity(int leftSpeed, int rightSpeed) {
        configLeftCruiseVelocity(leftSpeed);
        configRightCruiseVelocity(rightSpeed);
    }

    public void configRightCruiseVelocity(int sensorUnitsPer100ms) {
        right0.configMotionCruiseVelocity(sensorUnitsPer100ms, 10);
    }

    public void configLeftCruiseVelocity(int sensorUnitsPer100ms) {
        left0.configMotionCruiseVelocity(sensorUnitsPer100ms, 10);
    }


    public void disableOutput()
    {
        left0.set(ControlMode.Disabled,0);
        right0.set(ControlMode.Disabled,0);
    }

    public double getGoalPos() {
        return goalPos;
    }

    public void setGoalPos(double goalPos) {
        this.goalPos = goalPos;
    }

    public static class PeriodicIO {
        // INPUTS
        public int left_position_ticks;
        public int right_position_ticks;
        public double left_distance;
        public double right_distance;
        public int left_velocity_ticks_per_100ms;
        public int right_velocity_ticks_per_100ms;
        public Rotation2d gyro_heading = Rotation2d.identity();
        public Pose2d error = Pose2d.identity();

        // OUTPUTS
        public double left_demand;
        public double right_demand;
        public double left_accel;
        public double right_accel;
        public double left_feedforward;
        public double right_feedforward;
        public TimedState<Pose2dWithCurvature> path_setpoint = new TimedState<Pose2dWithCurvature>(Pose2dWithCurvature.identity());
    }

}
