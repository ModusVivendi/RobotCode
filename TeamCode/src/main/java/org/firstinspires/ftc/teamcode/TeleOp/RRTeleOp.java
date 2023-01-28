package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.advanced.PoseStorage;

@TeleOp(name="RRTeleOp", group = "GAME")
public class RRTeleOp extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;
    private Servo clawServo, topServo;
    private Move move;
    private Rotate rotate;
    private ClawServos clawServos;
    private ArmEncoder armEncoder;
    private TopServos topServos;
    String armCurrentDirection = "up";

    @Override
    public void runOpMode() throws InterruptedException {

        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        clawServo = hardwareMap.servo.get("CS");
        topServo = hardwareMap.servo.get("TS");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        clawServos = new ClawServos(clawServo);
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        topServos = new TopServos(topServo);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        drive.setPoseEstimate(PoseStorage.currentPose);

        servosUp(topServo);

        waitForStart();

        if (isStopRequested()) return;

        Pose2d poseEstimate = drive.getPoseEstimate();
        telemetry.addData("x", poseEstimate.getX());
        telemetry.addData("y", poseEstimate.getY());
        telemetry.addData("heading", poseEstimate.getHeading());
        telemetry.update();

        while(opModeIsActive() && !isStopRequested()) {
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(poseEstimate.getHeading());

                drive.setWeightedDrivePower(
                        new Pose2d(
                                -gamepad1.left_stick_y,
                                -gamepad1.left_stick_x,
                                -gamepad1.right_stick_x
                        )
                );

            drive.update();

            poseEstimate = drive.getPoseEstimate();

            telemetry.addData("x", poseEstimate.getX());
            telemetry.addData("y", poseEstimate.getY());
            telemetry.addData("heading", poseEstimate.getHeading());
            telemetry.update();

            if (gamepad2.x) {
                clawServos.SwitchAndWait(1, getRuntime());
            }
            if (gamepad2.y) {
                int i = 0;
                closeServo(clawServo);
                while (i <= 100) {
                    i++;
                }
                openServo(clawServo);

            }
            if (gamepad1.x) {
                clawServos.SwitchAndWait(1, getRuntime());
            }
            if (gamepad2.dpad_up) // Arm Up
            {
                armCurrentDirection = "up";
                armEncoder.goTo(1000, 1000, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );
                    drive.update();
                }
                servosDown(topServo);
                armEncoder.goTo(3000, 3000, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }

            } else if (gamepad2.dpad_down) //Arm Down
            {
                armCurrentDirection = "down";
                // closeServo(leftServo);
                servosUp(topServo);
                long setTime = System.currentTimeMillis();
                boolean hasRun = false;
                boolean ok = false;
                while(ok==false)
                {
                    if(System.currentTimeMillis() - setTime > 1200 && !hasRun) {
                        hasRun = true;
                        ok = true;
                        armEncoder.goTo(-10, -10, 0.7);
                        while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                            input = new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ).rotated(poseEstimate.getHeading());
                            drive.setWeightedDrivePower(
                                    new Pose2d(
                                            -gamepad1.left_stick_y,
                                            -gamepad1.left_stick_x,
                                            -gamepad1.right_stick_x
                                    )
                            );

                            drive.update();
                        }
                    }
                }

                closeServo(clawServo);
            }
            if (gamepad1.dpad_up) // Arm Up
            {
                armCurrentDirection = "up";
                armEncoder.goTo(1000, 1000, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }
                servosDown(topServo);
                armEncoder.goTo(3000, 3000, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }

            }
            else if(gamepad2.dpad_left) // Arm level 1
            {
                armCurrentDirection = "up";
                armEncoder.goTo(1300, 1300, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }
                servosDown(topServo);

            }
            else if(gamepad2.dpad_right) // Level 2
            {
                armCurrentDirection = "up";
                armEncoder.goTo(1300, 1300, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }
                servosDown(topServo);
                armEncoder.goTo(2000, 2000, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(poseEstimate.getHeading());
                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x,
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }
            }
            else if (gamepad1.dpad_down) //Arm Down
            {
                armCurrentDirection = "down";
                // closeServo(leftServo);
                servosUp(topServo);
                long setTime = System.currentTimeMillis();
                boolean hasRun = false;
                boolean ok = false;
                while(ok==false)
                {
                    if(System.currentTimeMillis() - setTime > 1500 && !hasRun) {
                        hasRun = true;
                        ok = true;
                        armEncoder.goTo(-10, -10, 0.7);
                        while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                            input = new Vector2d(
                                    -gamepad1.left_stick_y,
                                    -gamepad1.left_stick_x
                            ).rotated(poseEstimate.getHeading());
                            drive.setWeightedDrivePower(
                                    new Pose2d(
                                            -gamepad1.left_stick_y,
                                            -gamepad1.left_stick_x,
                                            -gamepad1.right_stick_x
                                    )
                            );

                            drive.update();
                        }
                    }
                }

                closeServo(clawServo);
            }
            if (armCurrentDirection.equals("down")) {
                armMotorLeft.setPower(0);
                armMotorRight.setPower(0);
                armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
        }
    }
    private void openServo(Servo _LS)
    {
        _LS.setPosition(1);
    }
    private void closeServo(Servo _LS)
    {
        _LS.setPosition(0);
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
