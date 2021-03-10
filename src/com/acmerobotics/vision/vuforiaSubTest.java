package com.acmerobotics.vision;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.ArrayList;
import java.util.List;

public class vuforiaSubTest {

    float mmPerInch        = 25.4f;
    float mmBotWidth       = 18 * mmPerInch;            // ... or whatever is right for your robot
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    public OpenGLMatrix location;
    VuforiaLocalizer vuforia;

    int cameraMonitorViewId;

    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackable BlueTowerGoal;

    VuforiaTrackable RedTowerGoal;

    OpenGLMatrix blueGoalLocation;

    OpenGLMatrix redGoalLocation;

    OpenGLMatrix cameraLocationOnRobot;

    public VuforiaTrackables ultimateGoal;

    private LinearOpMode opMode;

    public vuforiaSubTest(LinearOpMode opMode){

        this.opMode = opMode;

        cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());

        //Setting Vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdzxQPD/////AAABmWQr+AytbUpprVx2VNTgNiJgawbK313otTyXa3Th2KAhi06wLMwml/nAjh58jdIbDitq5cji21735oTIvYjaoFNdeEZzhQW6aieofpzebPDxtAUTXKVDws1MCES3iCBk2z0z8YhwaRfREOj6VdiqY1zPyhc5vBrnc8ioV2B1Jyuz56SGeHq9tmQ5KiYwUPfGSKZ1+p3vWqymEmwOcN7Ym/oOf6ZVLJgrd+NEJM2TNg1xwepmiexQVVwiBWpUDx9/Q6DpQIPpapiyoCDzXZoOMBUxxqw3HhHI1ZWA//utIdWRElWbZ2+Y4umURnZg54HT4PVe6nMzR8t56YXZUB851ly0D6G3WQoySEOkVg46CrL7";
        //parameters.cameraName = opMode.hardwareMap.get(WebcamName.class, "Webcam 1");
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        ultimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

        BlueTowerGoal = ultimateGoal.get(0/*put the number of the image here*/);
        BlueTowerGoal.setName("BlueTowerGoal");

        RedTowerGoal = ultimateGoal.get(1/*put the number of the image here*/);
        RedTowerGoal.setName("RedTowerGoal");




        //Use to set the location of a vumark
        blueGoalLocation = OpenGLMatrix
                //Location
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        //Rotation
                        AxesReference.EXTRINSIC, AxesOrder.XZX, AngleUnit.DEGREES, 90, 90, 0));
        BlueTowerGoal.setLocation(blueGoalLocation);
        RobotLog.ii("Vuforia Test", "Blue Goal=%s", format(blueGoalLocation));

        redGoalLocation = OpenGLMatrix
                //Location
                .translation(-mmFTCFieldWidth/2, 0, 0)
                .multiplied(Orientation.getRotationMatrix(
                        //Rotation
                        AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90, 90, 0));
        RedTowerGoal.setLocation(redGoalLocation);
        RobotLog.ii(RobotLog.TAG, "Red Goal=%s", format(redGoalLocation));

        //Use to set the location of the phone
        cameraLocationOnRobot = OpenGLMatrix
                .translation(mmBotWidth/2,0,0)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZY,
                        AngleUnit.DEGREES, -90, 0, 0));
        RobotLog.ii("Vuforia Test", "camera=%s", format(cameraLocationOnRobot));




        ((VuforiaTrackableDefaultListener)BlueTowerGoal.getListener()).setPhoneInformation(cameraLocationOnRobot, parameters.cameraDirection);
        ((VuforiaTrackableDefaultListener)RedTowerGoal.getListener()).setPhoneInformation(cameraLocationOnRobot, parameters.cameraDirection);

    }


    public void update(Telemetry telemetryData) {

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ultimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {

            telemetryData.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

            if (robotLocationTransform != null) {
                location = robotLocationTransform;
            }

        }

        if (location != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetryData.addData("Pos", format(location));

            telemetryData.addData("x", returnX());
            telemetryData.addData("y", returnY());
            telemetryData.addData("z", returnZ());
        } else {
            telemetryData.addData("Pos", "Unknown");
        }
        telemetryData.update();
    }

    public float returnX(){
        return location.getTranslation().get(0);
    }

    public float returnY(){
        return location.getTranslation().get(1);
    }

    public float returnZ(){
        return location.getTranslation().get(2);
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();

    }

}
