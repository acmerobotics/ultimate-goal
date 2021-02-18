package com.acmerobotics.robot;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WobbleGoalSubSys extends Subsystem {
private LinearOpMode opMode;

public Servo wobbleGoalLift;
public double wobbleGoalUp ;
public double wobbleGoalDown;
public Servo wobbleGoalHold;
public double wobbleHold;
public double  wobbleRelease;

   public WobbleGoalSubSys(Robot robot, LinearOpMode opMode){
super("WobbleGoalSubSys");

this.opMode = opMode;

wobbleGoalLift = robot.getServo("wobbleGoalLift");
wobbleGoalHold = robot.getServo("wobbleGoalHold");

// Not ready. Test this.
   wobbleGoalUp = 3;
   wobbleGoalDown = 2;
   wobbleHold = 3;
   wobbleRelease= 4;

   }

    @Override
    public void update(Canvas fieldOverlay) {

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
}
