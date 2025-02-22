package org.firstinspires.ftc.teamcode.OpModes.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;


import org.firstinspires.ftc.teamcode.EagleMatrix.NEW.botPID;
import org.firstinspires.ftc.teamcode.EagleMatrix.NEW.botPIDConstants;

@Autonomous
public class BucketAuto extends OpMode {

 botPID botPID;
 botPIDConstants constants;

 String action = "Case 1";

 Boolean isDone = false;


 public void init(){



 }

 public void loop(){

     switch (action){

         case "Case 1":
             //botPID.setLiftTarget(botPIDConstants.Lift_Constants.Lift_Score_Baskets);
             //botPID.setArmTarget(botPIDConstants.Arm_Constants.Arm_Score_High_Baskets);

             //botPID.runArm();
             //botPID.runLift();

             isDone = true;

             if (isDone = true){
                 action = "Case 2";
             }

         case "Case 2":
             //botPID.setLiftTarget(botPIDConstants.Lift_Constants.Lift_Home);
             //botPID.setArmTarget(botPIDConstants.Arm_Constants.Arm_PickUp_Sample);

             //botPID.runArm();
             //botPID.runLift();
     }
 }

}
