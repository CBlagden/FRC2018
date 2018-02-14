package org.usfirst.frc.team3309.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.command.Subsystem;
import org.usfirst.frc.team3309.commands.subsystems.rollers.RollersTeleop;
import org.usfirst.frc.team3309.lib.actuators.VictorSPXMC;
import org.usfirst.frc.team3309.robot.Constants;

public class Rollers extends Subsystem {

    private VictorSPXMC leftMotor = new VictorSPXMC(Constants.ROLLER_LEFT);
    private VictorSPXMC rightMotor = new VictorSPXMC(Constants.ROLLER_RIGHT);

    public Rollers() {
        leftMotor.set(0);
        rightMotor.set(0);
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new RollersTeleop());
    }

    public void setLeftRight(double left, double right) {
        setLeft(left);
        setRight(right);
    }

    public void setLeft(double power) {
        leftMotor.set(ControlMode.PercentOutput, power);
    }

    public void setRight(double power) {
        rightMotor.set(ControlMode.PercentOutput, power);
    }

}