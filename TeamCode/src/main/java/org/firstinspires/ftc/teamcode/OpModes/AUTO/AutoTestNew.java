package org.firstinspires.ftc.teamcode.OpModes.AUTO;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="AutoTest New!")
public class AutoTestNew extends CommandOpMode {

    @Override
    public void initialize() {
        schedule(new justmovealready());
    }
}
