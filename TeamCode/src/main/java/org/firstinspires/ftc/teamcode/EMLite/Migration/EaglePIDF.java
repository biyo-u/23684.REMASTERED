package org.firstinspires.ftc.teamcode.EMLite.Migration;


import org.firstinspires.ftc.teamcode.EagleMatrix.NEW.botPID;
import org.firstinspires.ftc.teamcode.EagleMatrix.NEW.botPIDConstants;
import org.firstinspires.ftc.teamcode.EMLite.Migration.WingMove.MotorDirection;
import org.firstinspires.ftc.teamcode.Utilities.Robot;

public class EaglePIDF {

 botPID botPID;
 botPIDConstants constants;
 WingMove wingMove;

 Robot robot;

 String CurrentAction = "hes just standing there, MENACINGLY";
 String NextAction = "initialization";


 private boolean IsFinished = false;




 public EaglePIDF(WingMove wingmove,botPID botPID, botPIDConstants constants, Robot robot){
     this.wingMove = wingmove;
     this.botPID = botPID;
     this.constants = constants;
     this.robot = robot;
 }

 public void prepareToScore(){
     IsFinished = false;

     CurrentAction = "PREPARING TO SCORE";
     robot.intake.clawClose();
     IsFinished = true;

     if(IsFinished == true){
         NextAction = "MOVE TO THE BASKET";
     }
 }

 public void moveToBasket(){
     IsFinished = false;

     CurrentAction = "MOVING TO BASKET";
     botPID.setArmTarget(6000);
     botPID.setLiftTarget(botPIDConstants.Lift_Constants.Lift_Score_Baskets);
     botPID.setYTarget(30);

     botPID.runLift();
     botPID.runArm();
     robot.intake.wristDown();
     botPID.runDrive();
     IsFinished = true;

     if (IsFinished == true){
         NextAction = "SCORE IN BASKET";
     }
 }

 public void scorePreloadInBasket(){
     IsFinished = false;

     CurrentAction = "SCORING IN BASKET";
     robot.intake.clawOpen();

     IsFinished = true;

     if (IsFinished == true){
         NextAction = "PICK UP 1st SPIKE MARK SAMPLE";
     }
 }
 public void pickUp1stSample(){
     IsFinished = false;

     CurrentAction = "PICKING UP 1st SPIKE MARK SAMPLE";


    }
}
