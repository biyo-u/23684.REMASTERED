package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import org.firstinspires.ftc.teamcode.Subsystems.Drive;

// attempts to circumvent issues by creating seq cmmd grp separately, run in AutoTestNew.java

public class justmovealready extends SequentialCommandGroup {

    Drive drive;

    public justmovealready() {
        addCommands(
            drive.moveQuickly(0, 24).withTimeout(3000),
            drive.moveQuickly(0, 0).withTimeout(3000)
        );

        addRequirements(drive);// just work already
    }
}
