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
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

//@Disabled
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
        int ticks = 400;

        // Zona RIGHT

        TrajectorySequence advancedTrajTryRight = drive.trajectorySequenceBuilder(new Pose2d(-61.70, -33.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .lineToConstantHeading(new Vector2d(-8.0, -34.0)) // -12 -33
                .addTemporalMarker(0.20, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .lineToConstantHeading(new Vector2d(-12,-33))
                .strafeRight(9.1)
                .build();

        TrajectorySequence forwardToPoleRight = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTryRight.end())
                .back(2.0)
                .build();

        TrajectorySequence stackTraj1Right = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleRight.end())
                .forward(6.0)
                .lineToSplineHeading(new Pose2d(-10.0, -58.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(350, 350);
                })
                .build();

        TrajectorySequence takeConeStack1Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj1Right.end())
                .back(2.0)
                .build();

        TrajectorySequence backFromStack1Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack1Right.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj1Right = drive.trajectorySequenceBuilder(backFromStack1Right.end())
                .lineToSplineHeading(new Pose2d(-10.5, -23.5, Math.toRadians(180.0) +1e-6)) //13 si 26 cu minus
                .build();

        TrajectorySequence forwardToPoleStack1Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj1Right.end())
                .back(3.8)
                .build();

        TrajectorySequence stackTraj2Right = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack1Right.end())
                .forward(5.0)
                .lineToSplineHeading(new Pose2d(-7.0, -56.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(250, 250);
                })
                .build();

        TrajectorySequence takeConeStack2Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj2Right.end())
                .back(2.2)
                .build();


        TrajectorySequence backFromStack2Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack2Right.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj2Right = drive.trajectorySequenceBuilder(backFromStack2Right.end())
                .lineToSplineHeading(new Pose2d(-8.9, -23.1, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        TrajectorySequence forwardToPoleStack2Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj2Right.end())
                .back(2.0)
                .build();

       TrajectorySequence stackTraj3Right = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack2Right.end())
               .forward(5.5)
               .lineToSplineHeading(new Pose2d(-3.0, -56.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(170, 170);
                })
                .build();

       TrajectorySequence takeConeStack3Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj3Right.end())
                .back(2.1)
                .build();


       TrajectorySequence backFromStack3Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack3Right.end())
                .forward(7.0)
                .build();

       TrajectorySequence poleTraj3Right = drive.trajectorySequenceBuilder(backFromStack3Right.end())
                .lineToSplineHeading(new Pose2d(-9.1, -21.0, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

       TrajectorySequence forwardToPoleStack3Right = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj3Right.end())
                .back(3.2)
                .build();


       TrajectorySequence rightPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPoleStack3Right.end())
                .strafeLeft(34.0)
                .build();

        // Zona CENTER

        TrajectorySequence advancedTrajTryCenter = drive.trajectorySequenceBuilder(new Pose2d(-61.70, -33.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .lineToConstantHeading(new Vector2d(-8.0, -33.5)) // -12 -33
                .addTemporalMarker(0.20, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .lineToConstantHeading(new Vector2d(-12,-33))
                .strafeRight(9.1)
                .build();

        TrajectorySequence forwardToPoleCenter = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTryCenter.end())
                .back(2.0)
                .build();

        TrajectorySequence stackTraj1Center = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleCenter.end())
                .forward(6.0)
                .lineToSplineHeading(new Pose2d(-10.0, -58.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(340, 340);
                })
                .build();

        TrajectorySequence takeConeStack1Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj1Center.end())
                .back(2.0)
                .build();

        TrajectorySequence backFromStack1Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack1Center.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj1Center = drive.trajectorySequenceBuilder(backFromStack1Center.end())
                .lineToSplineHeading(new Pose2d(-10.5, -22.3, Math.toRadians(180.0) +1e-6)) //13 si 26 cu minus
                .build();

        TrajectorySequence forwardToPoleStack1Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj1Center.end())
                .back(3.8)
                .build();

        TrajectorySequence stackTraj2Center = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack1Center.end())
                .forward(5.0)
                .lineToSplineHeading(new Pose2d(-7.0, -56.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(250, 250);
                })
                .build();

        TrajectorySequence takeConeStack2Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj2Center.end())
                .back(2.2)
                .build();


        TrajectorySequence backFromStack2Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack2Center.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj2Center = drive.trajectorySequenceBuilder(backFromStack2Center.end())
                .lineToSplineHeading(new Pose2d(-9.0, -21.6, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        TrajectorySequence forwardToPoleStack2Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj2Center.end())
                .back(2.2)
                .build();

        TrajectorySequence stackTraj3Center = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack2Center.end())
                .forward(5.5)
                .lineToSplineHeading(new Pose2d(-3.0, -56.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(170, 170);
                })
                .build();

        TrajectorySequence takeConeStack3Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj3Center.end())
                .back(2.4)
                .build();


        TrajectorySequence backFromStack3Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack3Center.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj3Center = drive.trajectorySequenceBuilder(backFromStack3Center.end())
                .lineToSplineHeading(new Pose2d(-9.1, -21.0, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        TrajectorySequence forwardToPoleStack3Center = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj3Center.end())
                .back(3.3)
                .build();

        TrajectorySequence centerPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPoleStack3Center.end())
                .strafeLeft(15.0)
                .build();

        //Zona LEFT

        TrajectorySequence advancedTrajTryLeft = drive.trajectorySequenceBuilder(new Pose2d(-61.70, -33.0, Math.toRadians(180.0)))
                .strafeLeft(3.0)
                .lineToConstantHeading(new Vector2d(-8.0, -34.0)) // -12 -33
                .addTemporalMarker(0.20, () ->
                {
                    armEncoder.goTo(2800, 2800);
                })
                .lineToConstantHeading(new Vector2d(-12,-32.0))
                .strafeRight(9.1)
                .build();

        TrajectorySequence forwardToPoleLeft = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ advancedTrajTryLeft.end())
                .back(1.9)
                .build();

        TrajectorySequence stackTraj1Left = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleLeft.end())
                .forward(6.0)
                .lineToSplineHeading(new Pose2d(-10.0, -57.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(350, 350);
                })
                .build();

        TrajectorySequence takeConeStack1Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj1Left.end())
                .back(2.2)
                .build();

        TrajectorySequence backFromStack1Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack1Left.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj1Left = drive.trajectorySequenceBuilder(backFromStack1Left.end())
                .lineToSplineHeading(new Pose2d(-10.5, -22.5, Math.toRadians(180.0) +1e-6)) //13 si 26 cu minus
                .build();

        TrajectorySequence forwardToPoleStack1Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj1Left.end())
                .back(3.8)
                .build();

        TrajectorySequence stackTraj2Left = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack1Left.end())
                .forward(5.0)
                .lineToSplineHeading(new Pose2d(-7.0, -56.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(250, 250);
                })
                .build();

        TrajectorySequence takeConeStack2Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj2Left.end())
                .back(2.6)
                .build();


        TrajectorySequence backFromStack2Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack2Left.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj2Left = drive.trajectorySequenceBuilder(backFromStack2Left.end())
                .lineToSplineHeading(new Pose2d(-9.0, -22.0, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        TrajectorySequence forwardToPoleStack2Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj2Left.end())
                .back(3.0)
                .build();

        TrajectorySequence stackTraj3Left = drive.trajectorySequenceBuilder(/*new Pose2d(-8.5, -25.83, Math.toRadians(0.0))*/ forwardToPoleStack2Left.end())
                .forward(5.5)
                .lineToSplineHeading(new Pose2d(-3.0, -57.0, Math.toRadians(90.0)))
                .addTemporalMarker(1, () ->
                {
                    armEncoder.goTo(170, 170);
                })
                .build();

        TrajectorySequence takeConeStack3Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ stackTraj3Left.end())
                .back(2.5)
                .build();


        TrajectorySequence backFromStack3Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ takeConeStack3Left.end())
                .forward(7.0)
                .build();

        TrajectorySequence poleTraj3Left = drive.trajectorySequenceBuilder(backFromStack3Left.end())
                .lineToSplineHeading(new Pose2d(-9.1, -22.5, Math.toRadians(180.0) +1e-6)) //15 si 27
                .build();

        TrajectorySequence forwardToPoleStack3Left = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ poleTraj3Left.end())
                .back(4.2)
                .build();

        TrajectorySequence leftPark = drive.trajectorySequenceBuilder(/*new Pose2d(-35.70, 0.0, Math.toRadians(0.0))*/ forwardToPoleStack3Left.end())
                .strafeRight(14.0)
                .build();

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(432, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
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
            sleep(200);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTryLeft);
            drive.followTrajectorySequence(forwardToPoleLeft);
            servosUp(topServo);

            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con preloaded


            drive.followTrajectorySequence(stackTraj1Left);

//            sleep(100);

            drive.followTrajectorySequence(takeConeStack1Left);

            sleep(200);
            closeServo(clawServo);
            sleep(260);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack1);

            drive.followTrajectorySequence(poleTraj1Left);
            drive.followTrajectorySequence(forwardToPoleStack1Left);
            servosUp(topServo);

            sleep(150);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 1 Stack

            drive.followTrajectorySequence(stackTraj2Left);

            sleep(150);

            drive.followTrajectorySequence(takeConeStack2Left);

            sleep(200);
            closeServo(clawServo);
            sleep(270);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack2);

            drive.followTrajectorySequence(poleTraj2Left);
            drive.followTrajectorySequence(forwardToPoleStack2Left);
            servosUp(topServo);

            sleep(150);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 2 Stack



            //Park

            drive.followTrajectorySequence(leftPark);

        }
        else if(currentPosition=="CENTER")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTryCenter);
            drive.followTrajectorySequence(forwardToPoleCenter);
            servosUp(topServo);

            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);
            //Con preloaded


            drive.followTrajectorySequence(stackTraj1Center);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack1Center);

            sleep(200);
            closeServo(clawServo);
            sleep(270);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack1);

            drive.followTrajectorySequence(poleTraj1Center);
            drive.followTrajectorySequence(forwardToPoleStack1Center);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 1 Stack

            drive.followTrajectorySequence(stackTraj2Center);

            sleep(200);

            drive.followTrajectorySequence(takeConeStack2Center);

            sleep(200);
            closeServo(clawServo);
            sleep(260);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack2);

            drive.followTrajectorySequence(poleTraj2Center);
            drive.followTrajectorySequence(forwardToPoleStack2Center);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 2 Stack



            drive.followTrajectorySequence(centerPark);
        }
        else if(currentPosition=="RIGHT")
        {
            servosUp(topServo);
            closeServo(clawServo);
            sleep(250);
            armEncoder.goTo(50, 50);

            drive.followTrajectorySequence(advancedTrajTryRight);
            drive.followTrajectorySequence(forwardToPoleRight);
            servosUp(topServo);

            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con preloaded


            drive.followTrajectorySequence(stackTraj1Right);

            sleep(150);

            drive.followTrajectorySequence(takeConeStack1Right);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack1);

            drive.followTrajectorySequence(poleTraj1Right);
            drive.followTrajectorySequence(forwardToPoleStack1Right);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 1 Stack

            drive.followTrajectorySequence(stackTraj2Right);

            sleep(150);

            drive.followTrajectorySequence(takeConeStack2Right);

            sleep(200);
            closeServo(clawServo);
            sleep(250);

            armEncoder.goTo(2800, 2800);
            sleep(200);
//            drive.followTrajectorySequence(backFromStack2);

            drive.followTrajectorySequence(poleTraj2Right);
            drive.followTrajectorySequence(forwardToPoleStack2Right);
            servosUp(topServo);

            sleep(200);
            openServo(clawServo);
            sleep(100);
            armEncoder.goTo(0,0);

            //Con 2 Stack


            //Cone 3 Stack

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
