package org.usfirst.frc.team3309.commands.autos;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.CommandGroup;
import javafx.scene.transform.Scale;
import org.usfirst.frc.team3309.commands.WaitAndMoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.AssemblyLocation;
import org.usfirst.frc.team3309.commands.subsystems.MoveAssembly;
import org.usfirst.frc.team3309.commands.subsystems.arms.ArmsOpen;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveStraight;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveToPoints;
import org.usfirst.frc.team3309.commands.subsystems.rollers.RollersActuate;
import org.usfirst.frc.team3309.lib.math.Translation2d;
import org.usfirst.frc.team3309.robot.Robot;

import java.sql.Driver;

public class ScalePathAuto extends CommandGroup {

    private boolean isStartRight;

    public ScalePathAuto(boolean isStartRight) {
        this.isStartRight = isStartRight;
    }

    @Override
    public synchronized void start() {

        if (DriverStation.getInstance().getGameSpecificMessage().length() > 0) {

            addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));

            if (Robot.isRightScale() && isStartRight) {

                addParallel(new WaitAndMoveAssembly(1.7, AssemblyLocation.SCALE_UP));
                addSequential(new DriveToPoints(180, false, 150,
                                new Translation2d(115.8, 0),
                                new Translation2d(174.6, -1.65),
                                new Translation2d(200.55, -5.69),
                                new Translation2d(250.59, -22.27)
                        )
                );

                releaseCube();

                addSequential(new DriveStraight(-17, 12000, true, true));
                addSequential(new MoveAssembly(AssemblyLocation.BOTTOM));

 /*           addSequential(new DriveToPoints(100, false,
                    new Translation2d(245.52, -30.38),
                    new Translation2d(248.53, -36.42),
                    new Translation2d(245.12, -44.49),
                    new Translation2d(240.06, -47.52),
                    new Translation2d(234.19, -48.37),
                    new Translation2d(197.38, -50.49)));*/
            } else if (Robot.isLeftScale() && isStartRight) {
           //     addSequential(new MoveAssembly(AssemblyLocation.SWITCH));
                addSequential(new DriveToPoints(128, false, 150,
                                new Translation2d(110.71, 0),
                                new Translation2d(154.81, -10.05),
                                new Translation2d(195.56, -44.09),
                                new Translation2d(211.53, -74.50),
                                new Translation2d(211.53, -122.0),
                                new Translation2d(220.53, -133.78),
                                new Translation2d(258.04, -137.40),
                                new Translation2d(270.49, -145.16),
                                new Translation2d(274.17, -152.64)
                        )
                );
            }
        }
        super.start();
    }

    private void releaseCube() {
        addParallel(new ArmsOpen());
        addSequential(new RollersActuate(0.8, 1.0));
    }

}
