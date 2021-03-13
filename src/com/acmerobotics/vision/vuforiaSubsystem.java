package com.acmerobotics.vision;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.robomatic.robot.Robot;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
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

@Config
public class vuforiaSubsystem extends Subsystem {

    float mmPerInch        = 25.4f;
    float mmFTCFieldWidth  = (12*12 - 2) * mmPerInch;   // the FTC field is ~11'10" center-to-center of the glass panels

    OpenGLMatrix location;
    VuforiaLocalizer vuforia;

    Orientation rotation;

    boolean isBlueGoalVisibleLocal;

    WebcamName webcam;
    int cameraMonitorViewId;

    VuforiaLocalizer.Parameters parameters;

    VuforiaTrackable BlueTowerGoal;

    //VuforiaTrackable RedTowerGoal;

    OpenGLMatrix blueGoalLocation;

    //OpenGLMatrix redGoalLocation;

    OpenGLMatrix cameraLocationOnRobot;

    public VuforiaTrackables ultimateGoal;

    VectorF translation;

    //for setting webcam position. negative means opposite of the direction in variable name
    final float webcamForwardDistance = 8.25f * mmPerInch;
    final float webcamUpwardDistance = 6f * mmPerInch;
    final float webcamLeftDistance = -7.625f * mmPerInch;

    private LinearOpMode opMode;

    public vuforiaSubsystem(Robot robot, LinearOpMode opMode){
        super("vuforiaSubsystem");

        this.opMode = opMode;

        webcam = robot.getWebcam();
        cameraMonitorViewId = robot.getCameraMonitorViewId();

        //Setting Vuforia parameters
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AdzxQPD/////AAABmWQr+AytbUpprVx2VNTgNiJgawbK313otTyXa3Th2KAhi06wLMwml/nAjh58jdIbDitq5cji21735oTIvYjaoFNdeEZzhQW6aieofpzebPDxtAUTXKVDws1MCES3iCBk2z0z8YhwaRfREOj6VdiqY1zPyhc5vBrnc8ioV2B1Jyuz56SGeHq9tmQ5KiYwUPfGSKZ1+p3vWqymEmwOcN7Ym/oOf6ZVLJgrd+NEJM2TNg1xwepmiexQVVwiBWpUDx9/Q6DpQIPpapiyoCDzXZoOMBUxxqw3HhHI1ZWA//utIdWRElWbZ2+Y4umURnZg54HT4PVe6nMzR8t56YXZUB851ly0D6G3WQoySEOkVg46CrL7";
        parameters.cameraName = webcam;

        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        ultimateGoal = this.vuforia.loadTrackablesFromAsset("UltimateGoal");

        BlueTowerGoal = ultimateGoal.get(0/*put the number of the image here*/);
        BlueTowerGoal.setName("BlueTowerGoal");

        //RedTowerGoal = ultimateGoal.get(1/*put the number of the image here*/);
        //RedTowerGoal.setName("RedTowerGoal");

        //Use to set the location of a vumark
        blueGoalLocation = OpenGLMatrix
                //Location
                .translation(mmFTCFieldWidth/2, mmFTCFieldWidth/4, 6*mmPerInch)
                .multiplied(Orientation.getRotationMatrix(
                        //Rotation
                        AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES, 90, 0, -90));
        BlueTowerGoal.setLocation(blueGoalLocation);
        RobotLog.ii("Vuforia Test", "Blue Goal=%s", format(blueGoalLocation));

        //redGoalLocation = OpenGLMatrix
                //Location
               // .translation(-mmFTCFieldWidth/2, 0, 0)
               // .multiplied(Orientation.getRotationMatrix(
                        //Rotation
               //         AxesReference.EXTRINSIC, AxesOrder.XZX,AngleUnit.DEGREES, 90, 90, 0));
        //RedTowerGoal.setLocation(redGoalLocation);
        //RobotLog.ii(RobotLog.TAG, "Red Goal=%s", format(redGoalLocation));

        //Use to set the location of the phone

        cameraLocationOnRobot = OpenGLMatrix
                .translation(webcamForwardDistance,webcamLeftDistance,webcamUpwardDistance)
                .multiplied(Orientation.getRotationMatrix(
                        AxesReference.EXTRINSIC, AxesOrder.YZX,
                        AngleUnit.DEGREES, 0, 0, 0));
        RobotLog.ii("Vuforia Test", "camera=%s", format(cameraLocationOnRobot));

        ((VuforiaTrackableDefaultListener)BlueTowerGoal.getListener()).setPhoneInformation(cameraLocationOnRobot, parameters.cameraDirection);
        //((VuforiaTrackableDefaultListener)RedTowerGoal.getListener()).setPhoneInformation(cameraLocationOnRobot, parameters.cameraDirection);

    }

    @Override
    public void update(Canvas fieldOverlay) {

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(ultimateGoal);

        for (VuforiaTrackable trackable : allTrackables) {

            telemetryData.addData(trackable.getName(), ((VuforiaTrackableDefaultListener) trackable.getListener()).isVisible() ? "Visible" : "Not Visible");

            OpenGLMatrix robotLocationTransform = ((VuforiaTrackableDefaultListener) trackable.getListener()).getUpdatedRobotLocation();

            if (robotLocationTransform != null) {
                location = robotLocationTransform;
                translation = location.getTranslation();
                rotation = Orientation.getOrientation(robotLocationTransform, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

            }

        }

        VuforiaTrackable blue;
        blue = allTrackables.get(0);
        if (((VuforiaTrackableDefaultListener) blue.getListener()).isVisible()) {

            isBlueGoalVisibleLocal = true;

        } else {

            isBlueGoalVisibleLocal = false;

        }

        if (location != null) {
            //  RobotLog.vv(TAG, "robot=%s", format(lastLocation));
            telemetryData.addData("Pos", format(location));
            telemetryData.addData("x", returnVuforiaX());
            telemetryData.addData("y", returnVuforiaY());
            telemetryData.addData("isBlueGoalVisible", isBlueGoalVisible());
            telemetryData.addData("XDistance from X=48", returnDriveX(48));
            telemetryData.addData("YDistance from Y=24", returnDriveY(24));
        } else {
            telemetryData.addData("Pos", "Unknown");
        }
    }

    String format(OpenGLMatrix transformationMatrix) {
        return transformationMatrix.formatAsTransform();
    }

    public VuforiaLocalizer returnVuforiaInstance(){return vuforia;}

    //The subtracted and added values are to account for webcam position
    public float returnVuforiaX(){return (translation.get(0) / mmPerInch) - 8.25f;}
    public float returnVuforiaY(){return (translation.get(1) / mmPerInch) + 7.625f;}

    public boolean isBlueGoalVisible(){return isBlueGoalVisibleLocal;}

    //Untested
    public float returnDriveX(float endXCoordinate){return endXCoordinate - returnVuforiaX();}
    public float returnDriveY(float endYCoordinate){return endYCoordinate - returnVuforiaY();}



    //public float returnVuforiaZ(){return translation.get(2) / mmPerInch;}

    //public double returnVuforiaXRotation(){return rotation.firstAngle;}
    //public double returnVuforiaYRotation(){return rotation.secondAngle;}
    //public double returnVuforiaZRotation(){return rotation.thirdAngle;} //Heading

    //public VectorF returnVuforiaVectorF(){ return translation;}

}
