package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RRAutonomousBasicLeft extends LinearOpMode {
    private SleeveDetection sleeveDetection;
    private OpenCvCamera camera;
    private Servo clawServo, topServo;
    private ArmEncoder armEncoder;
    private DcMotor armMotorLeft, armMotorRight;
    private String webcamName = "Webcam";
    private String currentPosition;
    private ClawServos clawServos;
    private TopServos topServos;
    @Override
    public void runOpMode() throws InterruptedException {
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        clawServo =  hardwareMap.servo.get("CS");
        topServo = hardwareMap.servo.get("TS");
        topServos = new TopServos(topServo);
        clawServos = new ClawServos(clawServo);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        drive.setPoseEstimate(new Pose2d(-61.70, 33.0, Math.toRadians(180.0)));
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence advancedTrajTryLeft = drive.trajectorySequenceBuilder(new Pose2d(-61.70, 33.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .lineToConstantHeading(new Vector2d(-10.0, 33.0)) // -12 -33
                .addTemporalMarker(0.20, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
//                .lineToConstantHeading(new Vector2d(-12,-33))
                .strafeLeft(12.0)
                .back(1.0)
                .build();

        TrajectorySequence stackTraj1 = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ advancedTrajTryLeft.end())
                .forward(6.0)
                .lineToSplineHeading(new Pose2d(-10.0, 58.0, Math.toRadians(-90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(350, 350);
                })
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTryLeft.end())
                .forward(2.0)
                .strafeRight(32.5)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTryLeft.end())
                .strafeRight(10.0)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTryLeft.end())
                .strafeLeft(14)
                .build();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(432,240, OpenCvCameraRotation.SIDEWAYS_LEFT);
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


        if(currentPosition == "LEFT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTryLeft);

            servosUp(topServo);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

//            drive.followTrajectorySequence(stackTraj1);
//
//            sleep(100);
//            closeServo(clawServo);
//            sleep(200);
//
//            armEncoder.goTo(2800,2800);



            drive.followTrajectorySequence(leftPark);
        }
        else if(currentPosition == "CENTER")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTryLeft);

            servosUp(topServo);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(centerPark);
        }
        else if(currentPosition == "RIGHT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTryLeft);


            servosUp(topServo);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(rightPark);
        }
    }
    private void openServo(Servo _LS)
    {
        _LS.setPosition(0);
    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(1);
    }
    private void servosUp(Servo topLeftServo)
    {
        topLeftServo.setPosition(0);
        //topRightServo.setPosition(0);
    }
    private void servosDown(Servo topLeftServo)
    {
        topLeftServo.setPosition(1);
        //topRightServo.setPosition(1);
    }
}
