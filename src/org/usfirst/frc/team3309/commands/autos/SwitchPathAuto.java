package org.usfirst.frc.team3309.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.command.PrintCommand;
import org.usfirst.frc.team3309.commands.WaitAndMoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.AssemblyLocation;
import org.usfirst.frc.team3309.commands.subsystems.MoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.arms.ArmsClamp;
import org.usfirst.frc.team3309.commands.subsystems.arms.ArmsOpen;
import org.usfirst.frc.team3309.commands.subsystems.beltbar.BeltBarMoveToPos;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveEnd;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveStraight;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveToPoints;
import org.usfirst.frc.team3309.commands.subsystems.lift.LiftElevate;
import org.usfirst.frc.team3309.commands.subsystems.rollers.RollersActuate;
import org.usfirst.frc.team3309.commands.subsystems.rollers.RollersSetIn;
import org.usfirst.frc.team3309.lib.WaitCommand;
import org.usfirst.frc.team3309.lib.math.Translation2d;
import org.usfirst.frc.team3309.robot.Robot;

public class SwitchPathAuto extends CommandGroup {

    @Override
    public synchronized void start() {
        if (DriverStation.getInstance().getGameSpecificMessage().length() > 0) {

            addSequential(new LiftElevate(AssemblyLocation.SWITCH, 1.0));
            addParallel(new BeltBarMoveToPos(AssemblyLocation.SWITCH.getBeltBarPosition()));

            if (Robot.isLeftSwitch() ) {

                addSequential(new DriveToPoints(170, false, 100,
                        new Translation2d(8.6, 0),
                        new Translation2d(33.0, -18.5),
                        new Translation2d(42.8, -23.8),
                        new Translation2d(54.8, -30.2),
                        new Translation2d(74.6, -38.9),
                        new Translation2d(99.8, -38.2)));

                releaseCube();

                addSequential(new DriveToPoints(100, true, 52,
                        new Translation2d(90.25, -61.76),
                        new Translation2d(85.59, -65.53),
                        new Translation2d(70.55, -69.78)));

                addParallel(new RollersSetIn(true));
                addParallel(new MoveAssembly(5400, AssemblyLocation.SWITCH.getBeltBarPosition()));

                addSequential(new DriveToPoints(100, false, 52,
                        new Translation2d(69.46, -62.66),
                        new Translation2d(71.82, -56.95),
                        new Translation2d(74.27, -51.02),
                        new Translation2d(74.78, -40.69)));

                addSequential(new ArmsClamp());
                addSequential(new RollersSetIn(false));


//                addSequential(new DriveToPoints(170, true, 100,
//                        new Translation2d(81.85, -47.51),
//                        new Translation2d(63.25, -57.19),
//                        new Translation2d(98.27, -64.67)));
//                addSequential(new DriveStraight(-10, 17000, true, true));

//                addSequential(new WaitAndMoveAssembly(0.5, AssemblyLocation.INTAKE));

            } if (Robot.isRightSwitch()) {
                addSequential(new DriveToPoints(170, false, 100,
                        new Translation2d(8.6, 0),
                        new Translation2d(33.0, 18.5),
                        new Translation2d(46.8, 20.8),
                        new Translation2d(54.8, 24.2),
                        new Translation2d(74.6, 27.9),
                        new Translation2d(99.8, 27.2)));

                releaseCube();
            }
        }
        super.start();
    }

    void releaseCube() {
        addSequential(new WaitCommand(0.1));
        addParallel(new ArmsOpen());
        addSequential(new RollersActuate(0.5, 0.1));
    }

}
