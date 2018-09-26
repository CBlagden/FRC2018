package org.usfirst.frc.team3309.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3309.commands.WaitAndMoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.AssemblyLocation;
import org.usfirst.frc.team3309.commands.subsystems.MoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveToPoints;
import org.usfirst.frc.team3309.lib.math.Translation2d;
import org.usfirst.frc.team3309.robot.Robot;

public class SideSwitchAutoPath extends CommandGroup {

    public SideSwitchAutoPath(boolean isRightSide) {
        this.isRightSide = isRightSide;
    }
    private boolean isRightSide;

    @Override
    public synchronized void start() {

        if(DriverStation.getInstance().getGameSpecificMessage().length() > 0) {

            addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));

            if(Robot.isRightSwitch() && isRightSide) {
                addParallel(new WaitAndMoveAssembly(0.0, AssemblyLocation.SWITCH));
                addSequential(new DriveToPoints(180, false, 150,

                        new Translation2d(20, 0),
                        new Translation2d(30, 35),
                    //    new Translation2d(65, 5),
                        new Translation2d(35, -14 ),
                    //    new Translation2d(80 , -20),
                        new Translation2d(40, -55)));



            } else if (Robot.isLeftSwitch() && !isRightSide) {
                addParallel(new WaitAndMoveAssembly(1.7, AssemblyLocation.SWITCH));


            }
        }

        super.start();
    }
}
