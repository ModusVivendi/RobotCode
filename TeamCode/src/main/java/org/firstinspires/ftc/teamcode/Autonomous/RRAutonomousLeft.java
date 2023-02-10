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
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class RRAutonomousLeft extends LinearOpMode {
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
        topServos = new TopServos(topServo);
        clawServos = new ClawServos(clawServo);
        clawServo =  hardwareMap.servo.get("CS");
        topServo = hardwareMap.servo.get("TS");
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
        drive.setPoseEstimate(new Pose2d(-61.70, 40.0, Math.toRadians(180.0)));
        TrajectorySequence basictTraj = drive.trajectorySequenceBuilder(new Pose2d(-61.70, 40.0, Math.toRadians(180.0))) //basic traj
                .strafeLeft(24.0)
                .forward(24.0)
                .strafeLeft(12.0)
                .build();

        TrajectorySequence advancedTraj = drive.trajectorySequenceBuilder(new Pose2d(-61.70, 40.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .back(5.0)
                .turn(Math.toRadians(180.0))
                .forward(45.0)
                .build();

        TrajectorySequence advancedTrajTry = drive.trajectorySequenceBuilder(new Pose2d(-61.70, 40.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .lineToConstantHeading(new Vector2d(-12.0, 33.0))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .strafeRight(9.0)
                .build();

        TrajectorySequence forwardToPole = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTry.end())
                .back(2.0)
                .build();

        TrajectorySequence stack1Traj = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPole.end())
                .forward(3.0)
                .lineToLinearHeading(new Pose2d(-12.0, 58.0, Math.toRadians(90.0) +1e-6))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(400, 400);
                })
                .build();

        TrajectorySequence takeConeStack = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stack1Traj.end())
                .back(2.0)
                .build();

        TrajectorySequence backFromStack = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack.end())
                .forward(8.0)
                .build();

        TrajectorySequence poleTraj = drive.trajectorySequenceBuilder(backFromStack.end())
                .lineToLinearHeading(new Pose2d(-11.0, 25.0, Math.toRadians(180.0) -1e-6))
                .build();

        TrajectorySequence forwardToPole2 = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj.end())
                .back(3.0)
                .build();

        TrajectorySequence stack2Traj = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPole2.end())
                .forward(2.0)
                .lineToLinearHeading(new Pose2d(-12.0, 58.0, Math.toRadians(90.0) +1e-6))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(300, 300);
                })
                .build();

        TrajectorySequence stack3Traj = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPole.end())
                .forward(2.0)
                .lineToLinearHeading(new Pose2d(-12.0, 58.0, Math.toRadians(90.0) +1e-6))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(200, 200);
                })
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPole2.end())
                .forward(2.0)
                .strafeLeft(48.0)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPole2.end())
                .forward(2.0)
                .strafeLeft(24.0)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPole2.end())
                .forward(2.0)
                .strafeRight(24.0)
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

        if(currentPosition == "RIGHT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(forwardToPole);
            servosUp(topServo);

            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);


            drive.followTrajectorySequence(stack1Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(400,400);


            drive.followTrajectorySequence(stack2Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(stack3Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            sleep(2000);

        }
        else if(currentPosition=="CENTER")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(forwardToPole);
            servosUp(topServo);

            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);


            drive.followTrajectorySequence(stack1Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);


            drive.followTrajectorySequence(stack2Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(stack3Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            sleep(2000);
        }
        else if(currentPosition=="LEFT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTry);
            drive.followTrajectorySequence(forwardToPole);
            servosUp(topServo);

            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(stack1Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);


            drive.followTrajectorySequence(stack2Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(stack3Traj);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack);

            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(backFromStack);

            drive.followTrajectorySequence(poleTraj);
            drive.followTrajectorySequence(forwardToPole2);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(200);
            armEncoder.goTo(0,0);

            sleep(2000);
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

