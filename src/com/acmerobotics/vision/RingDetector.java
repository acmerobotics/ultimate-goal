package com.acmerobotics.vision;

import android.hardware.camera2.CameraManager;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.robomatic.robot.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

public class RingDetector extends Subsystem {
    private static final String KEY = "AdzxQPD/////AAABmWQr+AytbUpprVx2VNTgNiJgawbK313otTyXa3Th2KAhi06wLMwml/nAjh58jdIbDitq5cji21735oTIvYjaoFNdeEZzhQW6aieofpzebPDxtAUTXKVDws1MCES3iCBk2z0z8YhwaRfREOj6VdiqY1zPyhc5vBrnc8ioV2B1Jyuz56SGeHq9tmQ5KiYwUPfGSKZ1+p3vWqymEmwOcN7Ym/oOf6ZVLJgrd+NEJM2TNg1xwepmiexQVVwiBWpUDx9/Q6DpQIPpapiyoCDzXZoOMBUxxqw3HhHI1ZWA//utIdWRElWbZ2+Y4umURnZg54HT4PVe6nMzR8t56YXZUB851ly0D6G3WQoySEOkVg46CrL7";
    private static final String QUAD = "Quad";
    private static final String SINGLE = "Single";

    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfDetector;

    private String ringsVisible;

    private boolean startDetecting = false;

    public RingDetector (HardwareMap hardwareMap){
        super("RingDetector");
        initVuforia(hardwareMap);
        initTf(hardwareMap);
    }

    private void initVuforia(HardwareMap hardwareMap){

        // create parameter instance, don't need cameraMonitorViewId (tf has their own)
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        // set up parameters
        parameters.vuforiaLicenseKey = KEY;
//        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

        // create vuforia instance
        vuforia = ClassFactory.getInstance().createVuforia(parameters);
    }

    private void initTf(HardwareMap hardwareMap){
        // get view id
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        // will display camera feed with tf filter on screen to show obj detection (shows view)
        //TFObjectDetector.Parameters parameters = new TFObjectDetector.Parameters(tfodMonitorViewId); // use for testing
        TFObjectDetector.Parameters parameters = new TFObjectDetector.Parameters(); // use during competition

        // set up parameters, how confident before declaring obj in view as the target
        parameters.minResultConfidence = 0.6f;

        // create tfDetector instance
        tfDetector = ClassFactory.getInstance().createTFObjectDetector(parameters, vuforia);

        // load tf models and label them
        tfDetector.loadModelFromAsset("UltimateGoal.tflite", QUAD, SINGLE);
    }

    public void startDetecting(){
        tfDetector.activate();
        startDetecting = true;
    }

    public void stopDetecting(){
        tfDetector.deactivate();
        startDetecting = false;
    }

    private void detectRings(){
        List<Recognition> recognitions = tfDetector.getUpdatedRecognitions();

        if (recognitions != null) {

            for (Recognition recognition: recognitions) {

                ringsVisible = recognition.getLabel();
            }
        }
    }

    public int detectedRings(){
        if (ringsVisible != null) {
            if (ringsVisible.equals(QUAD)){
                    return 4;
                }

            if (ringsVisible.equals(SINGLE)){
                return 1;
            }

            else {
                return 0;
            }
        }
        else return 0;
    }

    @Override
    public void update(Canvas Overlay){
        if (startDetecting){
            detectRings();
        }
    }

}
