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
        drive.setPoseEstimate(new Pose2d(-61.70, -33.0, Math.toRadians(180.0)));
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        TrajectorySequence basictTraj = drive.trajectorySequenceBuilder(new Pose2d(-61.70, -33.0, Math.toRadians(180.0))) //basic traj
                .strafeLeft(24.0)
                .forward(24.0)
                .strafeLeft(12.0)
                .build();

        TrajectorySequence secondTraj1 = drive.trajectorySequenceBuilder(new Pose2d(-61.70, -33.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .back(5.0)
                .turn(Math.toRadians(180.0))
                .forward(45.0)
                .build();

        TrajectorySequence secondTrajTry = drive.trajectorySequenceBuilder(new Pose2d(-61.70, -33.0, Math.toRadians(180.0)))
                .splineToConstantHeading(new Vector2d(-12.0, -33.0), Math.toRadians(0.0) +1e-6)
                .splineToConstantHeading(new Vector2d(-10.0, -25.0), Math.toRadians(0.0) +1e-6)
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .forward(1.5)
                .build();

        TrajectorySequence secondTraj2 = drive.trajectorySequenceBuilder(/*new Pose2d(-11.2, -35.7, Math.toRadians(0.0))*/ secondTraj1.end())
                .strafeLeft(12.0)
                .forward(3.0)
                .build();

        TrajectorySequence moveToCone = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ secondTrajTry.end())
                .forward(1.5)
                .build();

        TrajectorySequence splineTrajBack = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ moveToCone.end())
                .back(5.0)
                .lineToLinearHeading(new Pose2d(-12.0, -58.0, Math.toRadians(90.0) +1e-6))
                .strafeLeft(2.0)
                .build();

        TrajectorySequence backToCone = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ splineTrajBack.end())
                .back(3.5)
                .build();

        TrajectorySequence splineTrajForward = drive.trajectorySequenceBuilder(backToCone.end())
                .lineToLinearHeading(new Pose2d(-10.0, -26.0, Math.toRadians(0.0) -1e-6))
                .build();

        TrajectorySequence backfromCone = drive.trajectorySequenceBuilder(/*new Pose2d(-34.0, 0.0, Math.toRadians(0.0))*/moveToCone.end())
                .back(5.0)
                .build();

        TrajectorySequence backfromConeSecond = drive.trajectorySequenceBuilder(new Pose2d(-7.70, -8.0, Math.toRadians(45.0)))
                .back(4.0)
                .turn(Math.toRadians(-45.0))
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

        if(currentPosition == "RIGHT")
        {
            closeServo(clawServo);
            sleep(250);
            telemetry.addData("Ticks AMR: ", armMotorRight.getCurrentPosition());
            telemetry.addData("Ticks AML: ", armMotorLeft.getCurrentPosition());
            telemetry.update();
            armEncoder.goTo(50, 50);
            drive.followTrajectorySequence(secondTrajTry);
//            closeServo(clawServo);
//            sleep(200);
//            armEncoder.goTo(80,80);
//
//            drive.followTrajectorySequence(secondTraj1);
//
//            armEncoder.goTo(3000,3000);
//
//
//            drive.followTrajectorySequence(secondTraj2);
            sleep(500);
            servosDown(topServo);
            sleep(500);
            drive.followTrajectorySequence(moveToCone);
            openServo(clawServo);
            sleep(500);
            servosUp(topServo);
            sleep(500);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(splineTrajBack);

            armEncoder.goTo(400, 400);
            sleep(200);
            drive.followTrajectorySequence(backToCone);
            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(splineTrajForward);

            sleep(500);
            servosDown(topServo);
            sleep(500);
            drive.followTrajectorySequence(moveToCone);
            openServo(clawServo);
            sleep(500);
            servosUp(topServo);
            sleep(500);
            armEncoder.goTo(0,0);


//            drive.followTrajectorySequence(splineTrajBack);
//            armEncoder.goTo(500, 500);
//            sleep(200);
//            drive.followTrajectorySequence(backToCone);
//            sleep(500);
//            closeServo(clawServo);
//            sleep(500);
//
//            armEncoder.goTo(3000, 3000);
//            drive.followTrajectorySequence(splineTrajForward);
//
//            sleep(500);
//            servosDown(topServo);
//            sleep(1000);
//            openServo(clawServo);
//            sleep(500);
//            servosUp(topServo);
//            sleep(500);
//            armEncoder.goTo(0,0);
        }
        else if(currentPosition=="CENTER")
        {
            closeServo(clawServo);
            sleep(250);
            telemetry.addData("Ticks AMR: ", armMotorRight.getCurrentPosition());
            telemetry.addData("Ticks AML: ", armMotorLeft.getCurrentPosition());
            telemetry.update();
            armEncoder.goTo(50, 50);
            drive.followTrajectorySequence(secondTrajTry);
//            closeServo(clawServo);
//            sleep(200);
//            armEncoder.goTo(80,80);
//
//            drive.followTrajectorySequence(secondTraj1);
//
//            armEncoder.goTo(3000,3000);
//
//
//            drive.followTrajectorySequence(secondTraj2);
            sleep(500);
            servosDown(topServo);
            sleep(500);
            drive.followTrajectorySequence(moveToCone);
            openServo(clawServo);
            sleep(500);
            servosUp(topServo);
            sleep(500);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(splineTrajBack);

            armEncoder.goTo(400, 400);
            sleep(200);
            drive.followTrajectorySequence(backToCone);
            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(splineTrajForward);

            sleep(500);
            servosDown(topServo);
            sleep(500);
            drive.followTrajectorySequence(moveToCone);
            openServo(clawServo);
            sleep(500);
            servosUp(topServo);
            sleep(500);
            armEncoder.goTo(0,0);
//
//            drive.followTrajectorySequence(splineTrajBack);
//            armEncoder.goTo(500, 500);
//            sleep(200);
//            drive.followTrajectorySequence(backToCone);
//            sleep(500);
//            closeServo(clawServo);
//            sleep(500);
//
//            armEncoder.goTo(3000, 3000);
//            drive.followTrajectorySequence(splineTrajForward);
//
//            sleep(500);
//            servosDown(topServo);
//            sleep(1000);
//            openServo(clawServo);
//            sleep(500);
//            servosUp(topServo);
//            sleep(500);
//            armEncoder.goTo(0,0);
        }
        else if(currentPosition=="LEFT")
        {
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);
            drive.followTrajectorySequence(secondTrajTry);
//            closeServo(clawServo);
//            sleep(200);
//            armEncoder.goTo(80,80);
//
//            drive.followTrajectorySequence(secondTraj1);
//
//            armEncoder.goTo(3000,3000);
//
//
//            drive.followTrajectorySequence(secondTraj2);
            sleep(500);
            servosDown(topServo);
            sleep(500);
            drive.followTrajectorySequence(moveToCone);
            openServo(clawServo);
            sleep(500);
            servosUp(topServo);
            sleep(500);
            armEncoder.goTo(0,0);

            drive.followTrajectorySequence(splineTrajBack);

            armEncoder.goTo(400, 400);
            sleep(200);
            drive.followTrajectorySequence(backToCone);
            sleep(200);
            closeServo(clawServo);
            sleep(300);

            armEncoder.goTo(2800, 2800);

            drive.followTrajectorySequence(splineTrajForward);

            sleep(500);
            servosDown(topServo);
            sleep(500);
            drive.followTrajectorySequence(moveToCone);
            openServo(clawServo);
            sleep(500);
            servosUp(topServo);
            sleep(500);
            armEncoder.goTo(0,0);
//
//            drive.followTrajectorySequence(splineTrajBack);
//            armEncoder.goTo(500, 500);
//            sleep(200);
//            drive.followTrajectorySequence(backToCone);
//            sleep(500);
//            closeServo(clawServo);
//            sleep(500);
//
//            armEncoder.goTo(3000, 3000);
//            drive.followTrajectorySequence(splineTrajForward);
//
//            sleep(500);
//            servosDown(topServo);
//            sleep(1000);
//            openServo(clawServo);
//            sleep(500);
//            servosUp(topServo);
//            sleep(500);
//            armEncoder.goTo(0,0);
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
