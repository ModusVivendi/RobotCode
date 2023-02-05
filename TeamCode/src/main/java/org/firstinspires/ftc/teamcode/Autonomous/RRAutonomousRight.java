package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
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
public class RRAutonomousRight extends LinearOpMode {
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
        drive.setPoseEstimate(new Pose2d(-62.70, -37.5, Math.toRadians(0.0)));

        TrajectorySequence basictTraj = drive.trajectorySequenceBuilder(new Pose2d(-62.70, -37.5, Math.toRadians(0.0))) //basic traj
                .strafeLeft(24.0)
                .forward(24.0)
                .strafeLeft(12.0)
                .build();

        TrajectorySequence secondTraj = drive.trajectorySequenceBuilder(new Pose2d(-60.70, -12.38, Math.toRadians(0.0)))
                .forward(55.0)
                .strafeLeft(5.0)
                .turn(Math.toRadians(50.0))
                .build();

        TrajectorySequence splineTrajForward = drive.trajectorySequenceBuilder(new Pose2d(-12.70, -0.0, Math.toRadians(0.0)))
                .lineToSplineHeading(new Pose2d(60.74, -11.80, Math.toRadians(180.00)))
                .build();

        TrajectorySequence splineTrajBack = drive.trajectorySequenceBuilder(new Pose2d(-12.70, -0.0, Math.toRadians(0.0)))
                .lineToSplineHeading(new Pose2d(32.21, -6.16, Math.toRadians(140.00)))
                .build();

        TrajectorySequence moveToPutCone = drive.trajectorySequenceBuilder(new Pose2d(-35.70, 0.0, Math.toRadians(0.0)))
                .forward(0.6)
                .build();

        TrajectorySequence backfromCone = drive.trajectorySequenceBuilder(new Pose2d(-34.0, 0.0, Math.toRadians(0.0)))
                .back(5.0)
                .build();

        TrajectorySequence backfromConeSecond = drive.trajectorySequenceBuilder(new Pose2d(-34.0, 0.0, Math.toRadians(0.0)))
                .turn(Math.toRadians(60.0))
                .back(5.0)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(new Pose2d(-37.0, 0.0, Math.toRadians(0.0)))
                .back(3.0)
                .strafeRight(15.0)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(new Pose2d(-37.0, 0.0, Math.toRadians(0.0)))
                .back(3.0)
                .strafeRight(15.0)
                .forward(24.0)
                .strafeRight(21.0)
                .build();

        TrajectorySequence rightPark = drive.trajectorySequenceBuilder(new Pose2d(-37.0, 0.0, Math.toRadians(0.0)))
                .back(3.0)
                .strafeRight(10.0)
                .back(24.0)
                .strafeRight(48.0)
                .forward(24.0)
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

//        if(currentPosition=="RIGHT")
//        {
//            closeServo(clawServo);
//            armEncoder.goTo(50,50);
//
//            drive.followTrajectorySequence(basictTraj);
//
//            armEncoder.goTo(2800,2800);
//            servosDown(topServo);
//            drive.followTrajectorySequence(moveToPutCone);
//            openServo(clawServo);
//            sleep(300);
//            servosUp(topServo);
//            sleep(300);
//            armEncoder.goTo(-10,-10);
//
//            drive.followTrajectorySequence(rightPark);
//        }
        if(currentPosition == "RIGHT")
        {
            closeServo(clawServo);
            armEncoder.goTo(50,50);

            drive.followTrajectorySequence(secondTraj);

            armEncoder.goTo(2800,2800);
            servosDown(topServo);
            openServo(clawServo);
            sleep(300);
            servosUp(topServo);
            sleep(300);
            armEncoder.goTo(-10,-10);

            drive.followTrajectorySequence(splineTrajForward);
            armEncoder.goTo(100, 100);


        }
        else if(currentPosition=="CENTER")
        {
            closeServo(clawServo);
            armEncoder.goTo(50,50);

            drive.followTrajectorySequence(basictTraj);
            armEncoder.goTo(1000,1000);

            servosDown(topServo);
            armEncoder.goTo(2900,2900);

            drive.followTrajectorySequence(moveToPutCone);
            openServo(clawServo);
            sleep(300);
            servosUp(topServo);
            sleep(300);
            armEncoder.goTo(-10,-10);

            drive.followTrajectorySequence(moveToPutCone);
            openServo(clawServo);
            drive.followTrajectorySequence(centerPark);
        }
        else if(currentPosition=="LEFT")
        {
            closeServo(clawServo);
            armEncoder.goTo(50,50);

            drive.followTrajectorySequence(basictTraj);
            armEncoder.goTo(1000,1000);

            servosDown(topServo);
            armEncoder.goTo(2900,2900);

            drive.followTrajectorySequence(moveToPutCone);
            openServo(clawServo);
            sleep(300);
            servosUp(topServo);
            sleep(300);
            armEncoder.goTo(-10,-10);
            drive.followTrajectorySequence(moveToPutCone);
            openServo(clawServo);
            drive.followTrajectorySequence(leftPark);
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
