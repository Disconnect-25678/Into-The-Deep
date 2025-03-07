package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystem.Laterator;

public class LateratorStowCommand extends CommandBase {
    private final Laterator laterator;

    public LateratorStowCommand(Laterator laterator) {
        this.laterator = laterator;

        addRequirements(this.laterator);
    }

    @Override
    public void initialize() {
        laterator.stow();
    }

    @Override
    public boolean isFinished() {
        return laterator.isAtTarget();
    }
}
