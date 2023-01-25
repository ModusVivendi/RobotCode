package org.firstinspires.ftc.teamcode.Autonomous;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.trajectorysequence.TrajectorySequence;

@Autonomous
public class RRAutonomous extends LinearOpMode {
    private Servo leftServo;
    private ArmEncoder armEncoder;
    private DcMotor armMotorLeft, armMotorRight;
    @Override
    public void runOpMode() throws InterruptedException {
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        drive.setPoseEstimate(new Pose2d(-60.70, -12.38, Math.toRadians(0.0)));
        TrajectorySequence basictTraj = drive.trajectorySequenceBuilder(new Pose2d(-60.70, -12.38, Math.toRadians(0.0))) //basic traj
                .forward(24.0)
                .strafeLeft(36.0)
                .build();

        TrajectorySequence secondTraj = drive.trajectorySequenceBuilder(new Pose2d(-60.70, -12.38, Math.toRadians(0.0)))
                .forward(48.0)
                .turn(45.0)
                .build();

        TrajectorySequence splineTraj = drive.trajectorySequenceBuilder(new Pose2d(-12.70, -0.0, Math.toRadians(0.0)))
                .lineToSplineHeading(new Pose2d(60.14, -12.09, Math.toRadians(-3.75)))
                .build();
        waitForStart();
        if(isStopRequested()) return;
        drive.followTrajectorySequence(secondTraj);

    }
}
