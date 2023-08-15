package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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
@Disabled
public class RRAutonomousTryLeft extends LinearOpMode {
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
        TrajectorySequence advancedTrajTry, stackTrajStack, stackTrajPole;

        advancedTrajTry = drive.trajectorySequenceBuilder(new Pose2d(-61.70, 33.0, Math.toRadians(180.0)))
                .strafeLeft(4.0)
                .back(23.0)
                .lineToLinearHeading(new Pose2d(-5.0,38.1,  Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(-10.2, 36.1, Math.toRadians(-133.0)))
                .addTemporalMarker(0.2, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .back(4.0)
                .addDisplacementMarker( () ->
                {
                    openServo(clawServo);
                })
                .strafeRight(5.5)
                .build();

        TrajectorySequence stackTrajStack1 = drive.trajectorySequenceBuilder(advancedTrajTry.end())
                .lineToLinearHeading(new Pose2d(-14.0, 57.0, Math.toRadians(90)))
                .addTemporalMarker(0.2, () ->
                {
                    openServo(clawServo);
                    armEncoder.goTo(350, 350);

                })
                .back(3.0)
                .build();

        TrajectorySequence stackTrajPole1 = drive.trajectorySequenceBuilder(stackTrajStack1.end())
                .lineToLinearHeading(new Pose2d(-10.2, 34.1, Math.toRadians(47.0)))
                .addTemporalMarker(0.15, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .addTemporalMarker(1.0, () ->
                {
                    servosDown(topServo);
                })
                .forward(5.0)
                .build();

        TrajectorySequence stackTrajStack2 = drive.trajectorySequenceBuilder(stackTrajPole1.end())
                .lineToLinearHeading(new Pose2d(-14.0, 56.0, Math.toRadians(90)))
                .addTemporalMarker(0.5, () ->
                {
                    openServo(clawServo);
                    armEncoder.goTo(250, 250);
                })
                .back(3)
                .build();

        TrajectorySequence stackTrajPole2 = drive.trajectorySequenceBuilder(stackTrajStack2.end())
                .lineToLinearHeading(new Pose2d(-10.2, 33.1, Math.toRadians(47.0)))
                .addTemporalMarker(0.15, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .addTemporalMarker(1.0, () ->
                {
                    servosDown(topServo);
                })
                .forward(5.0)
                .build();

        TrajectorySequence stackTrajStack3 = drive.trajectorySequenceBuilder(stackTrajPole2.end())
                .lineToLinearHeading(new Pose2d(-14.0, 56.0, Math.toRadians(90)))
                .addTemporalMarker(0.5, () ->
                {
                    openServo(clawServo);
                    armEncoder.goTo(150, 150);

                })
                .back(3.0)
                .build();

        TrajectorySequence stackTrajPole3 = drive.trajectorySequenceBuilder(stackTrajStack3.end())
                .lineToLinearHeading(new Pose2d(-10.2, 33.1, Math.toRadians(47.0)))
                .addTemporalMarker(0.15, () ->
                {
                    armEncoder.goTo(2800, +2800);
                })
                .addTemporalMarker(1.0, () ->
                {
                    servosDown(topServo);
                })
                .forward(4.5)
                .build();


        TrajectorySequence backFromPole = drive.trajectorySequenceBuilder(stackTrajPole3.end())
                .forward(10.0)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(backFromPole.end())
                .lineToLinearHeading(new Pose2d(-10.0,8.5, Math.toRadians(0)))
                .addTemporalMarker(0.3, () -> {
                    armEncoder.goTo(0,0);
                })
                .build();



        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(stackTrajPole3.end())
                .forward(4.0)
                .lineToLinearHeading(new Pose2d(-10.0, 33.5, Math.toRadians(0)))
                .addTemporalMarker(0.3, () -> {
                    armEncoder.goTo(0,0);
                })
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(stackTrajPole3.end())
                .forward(4.0)
                .lineToLinearHeading(new Pose2d(-10.0,40.0, Math.toRadians(0)))
                .addTemporalMarker(0.3, () -> {
                    armEncoder.goTo(0,0);
                })
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
        if(isStopRequested()) return;

        if(currentPosition == "LEFT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);
            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(stackTrajStack1);

            closeServo(clawServo);
            sleep(250);

            drive.followTrajectorySequence(stackTrajPole1);

            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(150);

            drive.followTrajectorySequence(stackTrajStack2);

            closeServo(clawServo);
            sleep(250);

            drive.followTrajectorySequence(stackTrajPole2);

            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(150);

            drive.followTrajectorySequence(stackTrajStack3);

            closeServo(clawServo);
            sleep(250);

            drive.followTrajectorySequence(stackTrajPole3);
            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(100);
            drive.followTrajectorySequence(backFromPole);
            sleep(100);
            drive.followTrajectorySequence(leftPark);
        }
        if(currentPosition == "CENTER")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);
            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(stackTrajStack1);

            closeServo(clawServo);
            sleep(260);

            drive.followTrajectorySequence(stackTrajPole1);

            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(150);

            drive.followTrajectorySequence(stackTrajStack2);

            closeServo(clawServo);
            sleep(260);

            drive.followTrajectorySequence(stackTrajPole2);

            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(150);

            drive.followTrajectorySequence(stackTrajStack3);

            closeServo(clawServo);
            sleep(260);

            drive.followTrajectorySequence(stackTrajPole3);

            servosUp(topServo);
            sleep(100);

            openServo(clawServo);
            drive.followTrajectorySequence(centerPark);
        }
        if(currentPosition == "RIGHT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);
            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(stackTrajStack1);

            closeServo(clawServo);
            sleep(260);

            drive.followTrajectorySequence(stackTrajPole1);

            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(150);

            drive.followTrajectorySequence(stackTrajStack2);

            closeServo(clawServo);
            sleep(260);

            drive.followTrajectorySequence(stackTrajPole2);

            openServo(clawServo);
            sleep(150);
            servosUp(topServo);
            sleep(150);

            drive.followTrajectorySequence(stackTrajStack3);

            closeServo(clawServo);
            sleep(260);

            drive.followTrajectorySequence(stackTrajPole3);

            servosUp(topServo);
            sleep(100);

            openServo(clawServo);
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
