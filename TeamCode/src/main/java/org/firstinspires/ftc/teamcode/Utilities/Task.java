package org.firstinspires.ftc.teamcode.Utilities;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public abstract class Task {
	public void init(){}

	public void init(Pose2D target){}

	abstract public boolean run();
}
