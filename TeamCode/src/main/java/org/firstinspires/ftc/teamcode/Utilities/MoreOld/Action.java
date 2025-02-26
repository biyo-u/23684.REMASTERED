package org.firstinspires.ftc.teamcode.Utilities.MoreOld;

public class Action {
	private final Task task;
	private final String id;
	private final String requirement;

	public Action(Task task, String id, String requirement){
		this.task = task;
		this.id = id;
		this.requirement = requirement;
	}

	public Action(Task task, String id){
		this.task = task;
		this.id = id;
		this.requirement = null;
	}

	public String getRequirement(){
		return requirement;
	}

	public String getId(){
		return id;
	}

	public Task getTask(){
		return task;
	}
}
