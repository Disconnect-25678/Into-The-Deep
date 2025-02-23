package org.firstinspires.ftc.teamcode.subsystem;

import com.arcrobotics.ftclib.command.Subsystem;
import com.pedropathing.follower.Follower;
import com.pedropathing.localization.Localizer;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AutoDrivetrain extends Follower implements Subsystem {
    public AutoDrivetrain(HardwareMap hardwareMap) {
        super(hardwareMap);
        this.register();
    }

    public AutoDrivetrain(HardwareMap hardwareMap, Localizer localizer) {
        super(hardwareMap, localizer);
        this.register();
    }
}
