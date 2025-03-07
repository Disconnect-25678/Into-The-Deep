package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.pedropathing.pathgen.Path;
import com.pedropathing.pathgen.PathChain;

import org.firstinspires.ftc.teamcode.subsystem.AutoDrivetrain;

public class FollowPathCommand extends CommandBase {
    private final AutoDrivetrain follower;

    private Path path = null;
    private PathChain pathChain = null;

    private boolean holdEnd = true;

    public FollowPathCommand(AutoDrivetrain follower, Path path) {
        super();
        this.follower = follower;
        this.path = path;
        this.addRequirements(follower);
    }

    public FollowPathCommand(AutoDrivetrain follower, PathChain pathChain) {
        super();
        this.follower = follower;
        this.pathChain = pathChain;
        this.addRequirements(follower);
    }

    public FollowPathCommand(AutoDrivetrain follower, PathChain pathChain, boolean holdEnd) {
        super();
        this.follower = follower;
        this.pathChain = pathChain;
        this.holdEnd = holdEnd;
        this.addRequirements(follower);
    }

    @Override
    public void initialize() {
        if (this.path != null) this.follower.followPath(this.path);
        else this.follower.followPath(pathChain, holdEnd);
    }

    @Override
    public boolean isFinished() {
        return !follower.isBusy();
    }
}
