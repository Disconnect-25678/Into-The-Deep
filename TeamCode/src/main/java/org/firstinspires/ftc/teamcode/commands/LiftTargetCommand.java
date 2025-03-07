package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Lift;

public class LiftTargetCommand extends CommandBase {
    private final Lift lift;

    private int target, threshold;
    private boolean goingUp;

    public LiftTargetCommand(Lift lift, int target, int threshold, boolean goingUp) {
        this.lift = lift;
        this.target = target;
        this.threshold = threshold;
        this.goingUp = goingUp;

        addRequirements(this.lift);
    }

    @Override
    public void initialize() {
        lift.setTarget(target);
    }

    @Override
    public boolean isFinished() {
        if (goingUp) return lift.getCurrentPosition() > threshold;
        return lift.getCurrentPosition() < threshold;
    }
}
