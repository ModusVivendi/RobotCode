package org.firstinspires.ftc.teamcode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.EncoderMove;
import org.firstinspires.ftc.teamcode.Functions.NewEncoderMove;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "AutonomieBun Stanga")
public class AutonomieBunStanga extends LinearOpMode {

    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack, armMotor;
    private Servo leftServo, rightServo;
    private NewEncoderMove encoderMove;
    private ArmEncoder armEncoder;
    // Name of the Webcam to be set in the config
    private String webcamName = "Webcam";
    private String currentPosition;

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
        armMotor = hardwareMap.dcMotor.get("AM");
        leftServo = hardwareMap.servo.get("LS");
        rightServo = hardwareMap.servo.get("RS");

        encoderMove = new NewEncoderMove(leftMotor,leftMotorBack,rightMotor,rightMotorBack);
        armEncoder = new ArmEncoder(armMotor);


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
        currentPosition = String.valueOf(sleeveDetection.getPosition());
        if(currentPosition=="RIGHT")
        {
            closeServo(leftServo,rightServo);
            sleep(200);
            encoderMove.DriveTo(1250,-1250,-1250,1250,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(990,990,990,990,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(750,-750,-750,750,0.8, opModeIsActive());
            sleep(500);
            armEncoder.goTo(2830,0.8);
            sleep(500);
            encoderMove.DriveTo(130,130,130,130,0.5,opModeIsActive());
            sleep(200);
            openServo(leftServo,rightServo);
            sleep(200);
            encoderMove.DriveTo(-120,-120,-120,-120,0.5,opModeIsActive());
            sleep(200);
            armEncoder.goTo(0,0.5);
            sleep(200);
            encoderMove.DriveTo(-720,720,720,-720,0.8,opModeIsActive());
            sleep(500);
        }
        else if(currentPosition=="CENTER")
        {
            closeServo(leftServo,rightServo);
            sleep(200);
            encoderMove.DriveTo(1250,-1250,-1250,1250,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(990,990,990,990,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(750,-750,-750,750,0.8, opModeIsActive());
            sleep(500);
            armEncoder.goTo(2830,0.8);
            sleep(500);
            encoderMove.DriveTo(130,130,130,130,0.5,opModeIsActive());
            sleep(200);
            openServo(leftServo,rightServo);
            sleep(200);
            encoderMove.DriveTo(-120,-120,-120,-120,0.5,opModeIsActive());
            sleep(200);
            armEncoder.goTo(0,0.5);
            sleep(200);
            encoderMove.DriveTo(-700,700,700,-700,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(1200,1200,1200,1200,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(-1100,1100,1100,-1100,0.8,opModeIsActive());
            sleep(200);
        }
        else if(currentPosition=="LEFT")
        {
            closeServo(leftServo,rightServo);
            sleep(200);
            encoderMove.DriveTo(1250,-1250,-1250,1250,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(990,990,990,990,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(780,-780,-780,780,0.8, opModeIsActive());
            sleep(500);
            armEncoder.goTo(2830,0.8);
            sleep(500);
            encoderMove.DriveTo(130,130,130,130,0.5,opModeIsActive());
            sleep(200);
            openServo(leftServo,rightServo);
            sleep(200);
            encoderMove.DriveTo(-120,-120,-120,-120,0.5,opModeIsActive());
            sleep(200);
            armEncoder.goTo(0,0.5);
            sleep(200);
            encoderMove.DriveTo(-700,700,700,-700,0.8,opModeIsActive());
            sleep(500);
            encoderMove.DriveTo(-900,-900,-900,-900,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(-2500,2500,2500,-2500,0.8,opModeIsActive());
            sleep(200);
            encoderMove.DriveTo(1000,1000,1000,1000,0.8,opModeIsActive());

        }

    }
    private void openServo(Servo _LS, Servo _RS)
    {
        _LS.setPosition(1);
        _RS.setPosition(1);
    }
    private void closeServo(Servo _LS, Servo _RS)
    {
        _LS.setPosition(0);
        _RS.setPosition(0);
    }
}
