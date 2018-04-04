package org.usfirst.frc.team3309.commands.autos;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3309.commands.WaitAndMoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.AssemblyLocation;
import org.usfirst.frc.team3309.commands.subsystems.MoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.arms.ArmsClamp;
import org.usfirst.frc.team3309.commands.subsystems.arms.ArmsOpen;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveArc;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveEnd;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveStraight;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveTurn;
import org.usfirst.frc.team3309.commands.subsystems.rollers.RollersActuate;
import org.usfirst.frc.team3309.commands.subsystems.rollers.RollersSetIn;
import org.usfirst.frc.team3309.lib.WaitCommand;
import org.usfirst.frc.team3309.lib.math.Length;
import org.usfirst.frc.team3309.robot.Robot;

public class ScaleOnlyAuto extends CommandGroup {

    private boolean onRight;
    private boolean shouldSwitchCube;

    private double start;

    public ScaleOnlyAuto(boolean onRight, boolean shouldSwitchCube) {
        this.onRight = onRight;
        this.shouldSwitchCube = shouldSwitchCube;
    }

    @Override
    public synchronized void start() {
        start = Timer.getFPGATimestamp();
        addParallel(new MoveAssembly(AssemblyLocation.BOTTOM));
        if (onRight) {
            if (Robot.isRightScale()) {
         //       addParallel(new WaitAndMoveAssembly(1, AssemblyLocation.SCALE_UP));
                addSequential(new DriveStraight(122, 40000, 0));
                addParallel(new WaitAndMoveAssembly(0.2, AssemblyLocation.SCALE_UP));
                addSequential(new DriveArc(Length.fromInches(5), -32, 25000, false, true));
                addSequential(new DriveArc(Length.fromInches(6), 24, 20000, false, true));
                addSequential(new DriveStraight(6, 12000, true, true));
                addSequential(new DriveTurn(15, 1.0, true));
                addParallel(new ArmsOpen());
                addSequential(new RollersActuate(0.7, 1.0));

                addParallel(new WaitAndMoveAssembly(0.2, AssemblyLocation.BOTTOM));
                addSequential(new DriveStraight(-12, 12000, 0));
                addSequential(new Command() {
                    @Override
                    protected boolean isFinished() {
                        return Math.abs(Robot.lift.getPosition()) < 500;
                    }
                });
                addParallel(new MoveAssembly(AssemblyLocation.INTAKE));
                addSequential(new DriveTurn(152, 1.0, true));
                addParallel(new RollersSetIn(true));
                addSequential(new DriveStraight(14, 17000, true, true));
                addSequential(new WaitCommand(0.25));
                addSequential(new ArmsClamp());
                //addSequential(new WaitCommand(0.17));
                addParallel(new MoveAssembly(AssemblyLocation.BOTTOM));
                addSequential(new DriveStraight(-5, 17000, true, true));
                addParallel(new RollersSetIn(false));
                addSequential(new DriveTurn(0, 1.0, true));
                addSequential(new MoveAssembly(AssemblyLocation.SCALE_UP));
                addSequential(new DriveStraight(8, 12000, 0));
                addParallel(new RollersActuate(0.7, 1.0));
                addSequential(new ArmsOpen());

                if (shouldSwitchCube && Robot.isRightSwitch()) {

        /*            addParallel(new MoveAssembly(AssemblyLocation.INTAKE));
                    addSequential(new DriveTurn(90, 0.5, true));
                                        addSequential(new DriveStraight(10, 25000, true, true));
                    addSequential(new DriveArc(Length.fromInches(10), -1.0, 23000, false, true));
                    addParallel(new RollersSetIn(true));
                    addSequential(new ArmsClamp());
                    addSequential(new WaitCommand(0.5));
                //    addParallel(new MoveAssembly(AssemblyLocation.SWITCH));
                    addSequential(new RollersSetIn(false));
                    addSequential(new DriveStraight(5, 22000, true, true));
                    addParallel(new RollersActuate(0.4, 1.0));
                    addSequential(new ArmsOpen() {
                        @Override
                        public void end() {
                            super.end();
                            System.out.println("I ended at " + (Timer.getFPGATimestamp() - start));
                        }
                    });
                    addSequential(new WaitCommand(0.2));
                    addSequential(new DriveStraight(-14, 15000, true, true));
                    addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));*/
                }

            } else if (Robot.isLeftScale()) {
                addSequential(new DriveStraight(133, 28000, true, 2.0));
                addSequential(new DriveArc(Length.fromInches(28), -80, 23000,false, true));
                addSequential(new DriveStraight(183, 24000, true,  2.0));
                addSequential(new DriveTurn(-120, 0.6));

                addSequential(new MoveAssembly(AssemblyLocation.SCALE_UP));
                addSequential(new WaitCommand(0.3));
                addSequential(new DriveStraight(25, 15000,  1.2));
                addSequential(new DriveEnd());
                addParallel(new ArmsOpen());
                addSequential(new RollersActuate(0.5, 1));

                addSequential(new DriveStraight(-25, 15000, 2.0));
                addSequential(new DriveEnd());
                addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));
            }

        } else if (!onRight) {
            if (Robot.isLeftScale()) {
                addParallel(new WaitAndMoveAssembly(1.5, AssemblyLocation.SCALE_UP));
                addSequential(new DriveStraight(185, 20000, true));
                addSequential(new DriveArc(Length.fromInches(40), 24, 26000, false, true));
                addSequential(new DriveEnd());

                addSequential(new WaitCommand(0.2));
                addParallel(new ArmsOpen());
                addSequential(new RollersActuate(0.4, 1.0));
                addSequential(new WaitCommand(0.5));
                addSequential(new DriveStraight(-20, 15000, 1.2));
                addSequential(new DriveEnd());

                addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));

                if (Robot.isLeftSwitch() && shouldSwitchCube) {
                    addParallel(new MoveAssembly(AssemblyLocation.INTAKE));
                    addSequential(new DriveTurn(-90, 0.8));

                    addParallel(new RollersSetIn(true));
                    addSequential(new DriveStraight(29, 15000, 1.5));
                    addSequential(new WaitCommand(0.5));
                    addSequential(new ArmsClamp());
                    addSequential(new WaitCommand(0.25));
                    addSequential(new DriveStraight(-4, 20000, 1.5));
                    addSequential(new RollersSetIn(false));

                    addSequential(new MoveAssembly(AssemblyLocation.SWITCH));
                    addSequential(new DriveArc(Length.fromInches(20), 13, 15000, false, true));
                    addParallel(new RollersActuate(0.4, 1.0));
                    addSequential(new ArmsOpen());
                    addSequential(new WaitCommand(0.2));
                    addSequential(new DriveStraight(-20, 15000, 1.2));
                    addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));
                }

            } else if (Robot.isRightScale()) {

                addSequential(new DriveStraight(133, 28000, true, 2.0));
                addSequential(new DriveArc(Length.fromInches(28), 80, 23000,false, true));
                addSequential(new DriveStraight(163, 24000, true,  2.0));
                addSequential(new DriveTurn(125, 0.6));

                addSequential(new MoveAssembly(AssemblyLocation.SCALE_UP));
                addSequential(new WaitCommand(0.3));
                addSequential(new DriveStraight(20, 15000,  1.2));
                addSequential(new DriveEnd());
                addParallel(new ArmsOpen());
                addSequential(new RollersActuate(0.5, 1));

                addSequential(new DriveStraight(-20, 15000, 1.2));
                addSequential(new DriveEnd());
                addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));
            }
        }
        super.start();
    }

}
