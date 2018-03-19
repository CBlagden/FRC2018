package org.usfirst.frc.team3309.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.usfirst.frc.team3309.commands.autos.AutoLineAuto;
import org.usfirst.frc.team3309.commands.autos.CurvyToSwitchAuto;
import org.usfirst.frc.team3309.commands.autos.NoActionAuto;
import org.usfirst.frc.team3309.commands.autos.ScaleOnlyAuto;

public class AutoModeExecutor {

    private static SendableChooser<Command> autos = new SendableChooser<>();

    public static void displayAutos() {

        autos.addDefault("No Action", new NoActionAuto());
        autos.addObject("AutoLineAuto", new AutoLineAuto());
        autos.addObject("MiddleSwitchAuto", new CurvyToSwitchAuto());

        autos.addObject("RightScaleAuto", new ScaleOnlyAuto(true, false));
        autos.addObject("LeftScaleAuto", new ScaleOnlyAuto(false, false));

        autos.addObject("RightScaleAnd(Switch)Auto", new ScaleOnlyAuto(true, true));
        autos.addObject("LeftScaleAnd(Switch)Auto", new ScaleOnlyAuto(false, true));

        SmartDashboard.putData("Autos: ", autos);
    }

    public static Command getAutoSelected() {
        return autos.getSelected();
    }


}
