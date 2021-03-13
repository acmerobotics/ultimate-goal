package com.acmerobotics.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.RobotLog;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "vuforiaTest")
public class vuforiaTest extends LinearOpMode {

    //A lot of help can be found on ConceptVuforiaNavigation and a lot of the code is pulled from there

    WebcamName webcam;
    OpenGLMatrix location;
    VuforiaLocalizer vuforia;
    VuforiaLocalizer.Parameters parameters;

    @Override
    public void runOpMode() {

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        VuforiaTrackables ultimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

        //Use these two lines below to create a VuMark(you can find the assets in FtcRobotController.assets)
        VuforiaTrackable BlueTowerGoal = ultimateGoal.get(0/*put the number of the image here*/);
        BlueTowerGoal.setName("BlueTowerGoal");

        VuforiaTrackable RedTowerGoal = ultimateGoal.get(1/*put the number of the image here*/);
        RedTowerGoal.setName("RedTowerGoal");

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ultimateGoal);

        float mmPerInch        = 25.4f;
        float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
        float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

        //Use the code below to create and set the location for the VumMark
        OpenGLMatrix blueGoalLocation = OpenGLMatrix
                //Location
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                //Rotation
                AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90, 90, 0));
        BlueTowerGoal.setLocation(blueGoalLocation);
        RobotLog.ii("Vuforia Test", "Blue Goal=%s", format(blueGoalLocation));

        OpenGLMatrix redGoalLocation = OpenGLMatrix
                //Location
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                //Rotation
                AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90, 90, 0));
        RedTowerGoal.setLocation(redGoalLocation);
        RobotLog.ii(RobotLog.TAG, "Red Goal=%s", format(redGoalLocation));

        //Use this to show where the camera is on the robot coordinate system
        OpenGLMatrix cameraLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii("Vuforia Test", "camera=%s", format(cameraLocationOnRobot));

        ((VuforiaTrackableDefaultListener)BlueTowerGoal.getListener()).setPhoneInformation(cameraLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)RedTowerGoal.getListener()).setPhoneInformation(cameraLocationOnRobot, parameters.cameraDirection);

        telemetry.addData(">", "Start");
        telemetry.update();

        waitForStart();

        ultimateGoal.activate();

        while(opModeIsActive()){

            for (VuforiaTrackable trackable : allTrackables) {

                telemetry.addData(trackable.getName(), ((VuforiaTrackableDefaultListener)trackable.getListener()).isVisible() ? "Visible" : "Not Visible");    //

                OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener)trackable.getListener()).getUpdatedRobotLocation();

                if (robotLocationTransform != null) {
                    location = robotLocationTransform;
                }

            }

            if (location != null) {
                //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
                telemetry.addData("Pos", format(location));
            } else {
                telemetry.addData("Pos", "Unknown");
            }
            telemetry.update();

        }

    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

}