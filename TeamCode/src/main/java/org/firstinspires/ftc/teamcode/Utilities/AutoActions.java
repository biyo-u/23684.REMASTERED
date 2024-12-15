package org.firstinspires.ftc.teamcode.Utilities;

import java.util.ArrayList;

public class AutoActions {
	ArrayList<Action> actions = new ArrayList<Action>();
	ArrayList<String> ids = new ArrayList<String>();

	public AutoActions(){}

	public AutoActions(ArrayList<Action> actions, ArrayList<String> ids){
		this.actions = actions;
		this.ids = ids;
	}

	public AutoActions add(Action action){
		actions.add(action);
		ids.add(action.getId());
		return this;
	}

	public ArrayList<Action> getActions(){
		return actions;
	}

	public ArrayList<String> getIds(){
		return ids;
	}
}
