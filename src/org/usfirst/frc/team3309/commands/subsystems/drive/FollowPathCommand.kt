package org.usfirst.frc.team3309.commands.subsystems.drive

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import org.usfirst.frc.team3309.robot.Robot
import org.usfirst.frc.team4322.math.Path
import org.usfirst.frc.team4322.motion.Lookahead
import org.usfirst.frc.team4322.motion.PathFollower
import org.usfirst.frc.team4322.motion.RobotPositionIntegrator

class FollowPathCommand(private val path : Path, private val reverse : Boolean = false) : Command() {
    val pathFollower = PathFollower(path,reverse, PathFollower.Parameters(Lookahead(6.0,18.0,6.0,60.0),
            0.000013309,1.8,0.015,0.02,1.0,0.06,60.0,90.0,0.75,8.0,3.0))

    init {
        requires(Robot.drive)
    }

    override fun initialize() {
        RobotPositionIntegrator.reset()
    }

    override fun execute() {
        Robot.drive.changeToVelocityMode()
        val out = pathFollower.execute(Timer.getFPGATimestamp())
        val outLeft = Robot.drive.inchesToEncoderCounts(out.second)/2
        val outRight = Robot.drive.inchesToEncoderCounts(out.first)/2
        SmartDashboard.putNumber("Left Target: ",outLeft)
        SmartDashboard.putNumber("Right Target: ",outRight)
        Robot.drive.setLeftRight(outLeft,outRight)
    }

    override fun isFinished(): Boolean {
        return pathFollower.isFinished
    }
}