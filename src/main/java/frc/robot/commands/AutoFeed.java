package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.ShooterSystems.BallElevator;

public class AutoFeed extends Command {
    private final Spindexer spindexer;
    private final BallElevator ballElevator;

    public AutoFeed(Spindexer spindexer, BallElevator ballElevator) {
        this.spindexer = spindexer;
        this.ballElevator = ballElevator;
        addRequirements(spindexer, ballElevator);
    }

    @Override
    public void execute() {
        spindexer.runFeed(1.0);
        ballElevator.dumbSpeed(1);
    }

    @Override
    public void end(boolean interrupted) {
        spindexer.stop();
        ballElevator.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
