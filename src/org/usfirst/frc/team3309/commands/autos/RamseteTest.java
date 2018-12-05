package org.usfirst.frc.team3309.commands.autos;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.usfirst.frc.team3309.commands.subsystems.drive.DriveTrajectory;
import org.usfirst.frc.team3309.paths.TrajectoryGenerator;

public class RamseteTest extends CommandGroup {

    private DriveTrajectory trajectory;

    public RamseteTest() {
        this.trajectory = new DriveTrajectory(TrajectoryGenerator.getInstance().getTrajectorySet()
                .sideStartToNearScale.get(true));
    }

    @Override
    public synchronized void start() {
        addSequential(trajectory);
        super.start();
    }
}
