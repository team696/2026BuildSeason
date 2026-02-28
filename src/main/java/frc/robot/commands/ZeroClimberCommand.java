package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Climber;

public class ZeroClimberCommand extends Command {

    private final Climber mClimber;
    private static final double STALL_CURRENT_THRESHOLD = 30.0; // amps — tune this!
    private static final double ZERO_VELOCITY = -0.5;           // rps — toward hard stop

    private boolean mHitHardStop = false;

    public ZeroClimberCommand(Climber climber) {
        this.mClimber = climber;
        addRequirements(climber);
    }

    @Override
    public void initialize() {
        mHitHardStop = false;
        System.out.println("[ZeroClimber] Starting zero sequence...");
    }

    @Override
    public void execute() {
        mClimber.setVelocity(ZERO_VELOCITY); // drive toward hard stop slowly

        double current = mClimber.getStatorCurrent();
        if (current > STALL_CURRENT_THRESHOLD) {
            mHitHardStop = true; // flag it — isFinished() will pick this up
        }
    }

    @Override
    public boolean isFinished() {
        return mHitHardStop;
    }

    @Override
    public void end(boolean interrupted) {
        mClimber.stop(); // always stop motor first

        if (!interrupted) {
            mClimber.zeroEncoder();         // reset encoder to 0
            mClimber.setIsZeroed(true);     // flag subsystem as zeroed
            System.out.println("[ZeroClimber] Zeroed successfully!");
        } else {
            System.out.println("[ZeroClimber] Command interrupted — encoder NOT zeroed.");
        }
    }
}