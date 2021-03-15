package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class WobbleGoalSubSys extends Subsystem {

public Servo wobbleGoalLift;
public static double wobbleGoalUp;
public double wobbleGoalDown;
public static double wobbleLiftHigh; // for lifting over drop zone

public Servo wobbleGoalHold;
public double wobbleHold;
public double  wobbleRelease;

   public WobbleGoalSubSys(Robot robot){
super("WobbleGoalSubSys");

wobbleGoalLift = robot.getServo("wobbleGoalLift");
wobbleGoalHold = robot.getServo("wobbleGoalHold");

   wobbleGoalUp = 1;
   wobbleGoalDown = 0.12;
   wobbleHold = 0.325; // 0.28
   wobbleRelease= 0.8;
   wobbleLiftHigh = 0.5;

   }

    @Override
    public void update(Canvas fieldOverlay) {
        telemetryData.addData("armPos", wobbleGoalLift.getPosition());
        telemetryData.addData("handPos", wobbleGoalHold.getPosition());

    }

    public void wobbleGoalArm(boolean dpadup, boolean dpaddown){

       if(dpadup){

           wobbleGoalLift.setPosition(wobbleGoalUp);
       }
    if(dpaddown){
        wobbleGoalLift.setPosition(wobbleGoalDown);
    }
   }
public void wobbleGoalHand(boolean a){

   if(a){
       wobbleGoalHold.setPosition(wobbleHold);

   }
   else {
     wobbleGoalHold.setPosition(wobbleRelease);
   }
}

    public void wobbleGoalHandLoose(){
       wobbleGoalHold.setPosition(0.3); // loose hold on wobble so when it comes down during auto it wont get stuck and carried
    }

    public void setWobbleGoalRelease(){
       wobbleGoalHold.setPosition(wobbleRelease);
    }

    public void wobbleHigh(){
        wobbleGoalLift.setPosition(wobbleLiftHigh);
    }
}
