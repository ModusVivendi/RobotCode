package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    private ElapsedTime runtime = new ElapsedTime();
    final double END_GAME = 90.0;
    final double HALF_TIME = 60.0;
    final double QUARTER_TIME = 30.0;
    final double HALF_QUARTER_TIME = 15.0;
    final double NO_TIME = 5.0;
    boolean secondHalf = false;
    boolean secondEnd = false;
    boolean secondQuarter = false;
    boolean secondHalfQuarter = false;
    boolean secondNo = false;
    Gamepad.RumbleEffect customRumbleEffectQuarter, customRumbleEffectHalf, customRumbleEffectHalfQuarter, customRumbleEffectEnd; //quarter e cu no
    //half e cu halfquarter

    @Override
    public void runOpMode() throws InterruptedException {
        // Created a three-pulse rumble sequence: RIGHT, LEFT, LEFT
        customRumbleEffectQuarter = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .build();

        customRumbleEffectHalf = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 500)
                .build();

        customRumbleEffectEnd = new Gamepad.RumbleEffect.Builder()
                .addStep(1.0, 1.0, 500)  //  Rumble right motor 100% for 500 mSec
                .addStep(0.0, 0.0, 300)  //  Pause for 300 mSec
                .addStep(1.0, 1.0, 500)  //  Rumble left motor 100% for 250 mSec
                .addStep(0.0, 0.0, 300)
                .addStep(1.0, 1.0, 1000)
                .build();

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
        runtime.reset(); // Start game timer.

        if (isStopRequested()) return;

        while(opModeIsActive() && !isStopRequested()) {
            //Watch the runtime timer, and run the custom rumble when we hit half-time.
            //Make sure we only signal once by setting "secondHalf" flag to prevent further rumbles.
            if ((runtime.seconds() > HALF_TIME) && !secondHalf)  {
                gamepad1.runRumbleEffect(customRumbleEffectHalf);
                secondHalf =true;
            }
            else if ((runtime.seconds() > END_GAME) && !secondEnd)  {
                gamepad1.runRumbleEffect(customRumbleEffectEnd);
                secondEnd =true;
            }
            else if ((runtime.seconds() > NO_TIME) && !secondNo)  {
                gamepad1.runRumbleEffect(customRumbleEffectQuarter);
                secondNo =true;
            }
            else if ((runtime.seconds() > HALF_QUARTER_TIME) && !secondHalfQuarter)  {
                gamepad1.runRumbleEffect(customRumbleEffectHalf);
                secondHalfQuarter =true;
            }
            else if ((runtime.seconds() > QUARTER_TIME) && !secondQuarter)  {
                gamepad1.runRumbleEffect(customRumbleEffectQuarter);
                secondQuarter =true;
            }

            // Display the time remaining while we are still counting down.
            if (!secondHalf) {
                telemetry.addData(">", "Halftime Alert Countdown: %3.0f Sec \n", (HALF_TIME - runtime.seconds()) );
                telemetry.update();
            }


            Pose2d poseEstimate = drive.getPoseEstimate();
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ).rotated(-poseEstimate.getHeading());

            drive.setWeightedDrivePower(
                    new Pose2d(
                            input.getX(),
                            input.getY(),
                            -gamepad1.right_stick_x
                    )
            );
            drive.update();



            if(gamepad1.right_bumper)
            {
                drive.setPoseEstimate(PoseStorage.currentPose);
            }

            if (gamepad2.x) {
                clawServos.SwitchAndWait(1, getRuntime());
            }

            if (gamepad1.x) {
                clawServos.SwitchAndWait(1, getRuntime());
            }

            if(gamepad1.left_bumper)
            {
            }

            if (gamepad2.dpad_up) // Arm Up
            {
                armCurrentDirection = "up";
                armEncoder.goTo(1000, 1000, 1);
                while (armMotorLeft.isBusy() && armMotorRight.isBusy()) {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
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
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }

            }

            else if (gamepad2.dpad_down) //Arm Down
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
                            ).rotated(-poseEstimate.getHeading());

                            drive.setWeightedDrivePower(
                                    new Pose2d(
                                            input.getX(),
                                            input.getY(),
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
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
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
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
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
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
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
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
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
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
                                    -gamepad1.right_stick_x
                            )
                    );

                    drive.update();
                }
            }
            else if (gamepad2.y) {
                armCurrentDirection = "up";
                armEncoder.goTo(100, 100, 1);
                while(armMotorLeft.isBusy() && armMotorRight.isBusy())
                {
                    input = new Vector2d(
                            -gamepad1.left_stick_y,
                            -gamepad1.left_stick_x
                    ).rotated(-poseEstimate.getHeading());

                    drive.setWeightedDrivePower(
                            new Pose2d(
                                    input.getX(),
                                    input.getY(),
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
                            ).rotated(-poseEstimate.getHeading());

                            drive.setWeightedDrivePower(
                                    new Pose2d(
                                            input.getX(),
                                            input.getY(),
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
