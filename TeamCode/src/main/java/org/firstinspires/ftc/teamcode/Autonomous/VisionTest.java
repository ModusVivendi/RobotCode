package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Autonomous.SleeveDetection;
import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "Autonomie 2022-2023")
public class VisionTest extends LinearOpMode {

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack;
    private EncoderMove encoderMove;

    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam";

    @Override
    public void runOpMode() throws InterruptedException {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);

        leftMotor = hardwareMap.dcMotor.get("FL");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        rightMotorBack = hardwareMap.dcMotor.get("BR");

        encoderMove = new EncoderMove(leftMotor,leftMotorBack,rightMotor,rightMotorBack);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {
            telemetry.addData("ROTATION: ", sleeveDetection.getPosition());
            telemetry.update();
        }

        waitForStart();
        if(sleeveDetection.getPosition().equals("RIGHT"))
        {
            
        }
        else if(sleeveDetection.getPosition().equals("CENTER"))
        {

        }
        else if(sleeveDetection.getPosition().equals("LEFT"))
        {

        }

    }
}
