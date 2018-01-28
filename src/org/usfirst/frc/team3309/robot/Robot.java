package org.usfirst.frc.team3309.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import org.usfirst.frc.team3309.commands.subsystems.AssemblyLocation;
import org.usfirst.frc.team3309.subsystems.*;

import java.util.logging.Logger;

/**
 * @author Chase Blagden
 */
public class Robot extends IterativeRobot {


    public static Drive drive;
    public static Intake intake;
    public static Lift lift;
    public static BeltBar beltBar;
    public static Shooter shooter;

    private Compressor c = new Compressor();

    private Command autoCommand = null;
    private OI oi;
    public static Logger logger = Logger.getLogger("Robot");

    @Override
    public void robotInit() {
        logger.info("robot init");
        drive = new Drive();
        intake = new Intake();
        lift = new Lift();
        beltBar = new BeltBar();
        shooter = new Shooter();
        oi = new OI();
        c.start();
        drive.sendToDashboard();
        AutoModeExecutor.displayAutos();
        drive.resetDrive();
    }

    @Override
    public void autonomousInit() {
        logger.info("autonomous init");
        drive.resetDrive();
        autoCommand = AutoModeExecutor.getAutoSelected();
        logger.info("Running " + autoCommand.getName());
        if (autoCommand != null) {
            autoCommand.start();
        }
    }

    @Override
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        Robot.drive.sendToDashboard();
    }

    @Override
    public void teleopInit() {
        logger.info("teleop init");
        if (autoCommand != null) {
            autoCommand.cancel();
        }
        drive.resetDrive();
        drive.setHighGear();
        drive.enableBrakeMode(false);
    }

    @Override
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        drive.sendToDashboard();
    }

    @Override
    public void testPeriodic() {
        LiveWindow.run();
    }

    @Override
    public void disabledPeriodic() {
        drive.resetDrive();
    }

}
