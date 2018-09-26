package org.usfirst.frc.team3309.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3309.commands.autos.*;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveToPoints;
import org.usfirst.frc.team3309.lib.math.Translation2d;

public class AutoModeExecutor {

    private static SendableChooser<Command> autos = new SendableChooser<>();

    public static void displayAutos() {

        autos.addDefault("No Action", new NoActionAuto());
        autos.addObject("AutoLineAuto", new AutoLineAuto());

        autos.addObject("MiddleSwitchPath", new SwitchPathAuto());

        autos.addObject("RightScaleAutoPath", new ScalePathAuto(true));

        autos.addObject("DriveBackTest",  new CommandGroup() {
            @Override
            public synchronized void start() {
                addSequential(new DriveToPoints(140, true, 100,
                        new Translation2d(0, -40)));
                super.start();
            }
        });

        SmartDashboard.putData("Autos: ", autos);
    }

    public static Command getAutoSelected() {
        return autos.getSelected();
    }


}
