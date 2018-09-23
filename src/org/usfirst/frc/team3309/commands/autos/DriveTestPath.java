package org.usfirst.frc.team3309.commands.autos;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveToPoints;
import org.usfirst.frc.team3309.lib.math.Translation2d;

public class DriveTestPath extends CommandGroup {

    @Override
    public void start() {
        addSequential(new DriveToPoints(120, false, 100,
                new Translation2d(190, 0),
                new Translation2d(214, -24),
                new Translation2d(229, -70),
                new Translation2d(229, -130),
                new Translation2d(207, -151),
                new Translation2d(189, -163),
                new Translation2d(96, -163),
                new Translation2d(15, -163),
                new Translation2d(-12, -141),
                new Translation2d(-44, -101),
                new Translation2d(-44, -33),
                new Translation2d(-22, -12),
                new Translation2d(0, 0)));
        super.start();
    }

}
