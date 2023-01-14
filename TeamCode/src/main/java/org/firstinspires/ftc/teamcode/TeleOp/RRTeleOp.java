package org.firstinspires.ftc.teamcode.TeleOp;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;
import org.firstinspires.ftc.teamcode.Functions.TopServos;
import org.firstinspires.ftc.teamcode.RoadRunner.drive.StandardTrackingWheelLocalizer;

@TeleOp(name="RRTeleOp", group = "GAME")
public class RRTeleOp extends LinearOpMode {

    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;
    private Servo leftServo, topLeftServo, topRightServo;
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
        leftServo = hardwareMap.servo.get("LS");
        topLeftServo = hardwareMap.servo.get("TLS");
        topRightServo = hardwareMap.servo.get("TRS");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        clawServos = new ClawServos(leftServo);
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);
        topServos = new TopServos(topLeftServo,topRightServo);
        StandardTrackingWheelLocalizer localizer = new StandardTrackingWheelLocalizer(hardwareMap);

        waitForStart();

        while(opModeIsActive()) {
            localizer.update();
            Pose2d myPose = localizer.getPoseEstimate();

            telemetry.addData("x", myPose.getX());
            telemetry.addData("y", myPose.getY());
            telemetry.addData("heading", myPose.getHeading());

            if(gamepad2.x)
            {
                clawServos.SwitchAndWait(1,getRuntime());
            }
            if(gamepad1.x)
            {
                clawServos.SwitchAndWait(1,getRuntime());

            }
            if(gamepad2.dpad_up) // Arm Up
            {
                armCurrentDirection = "up";
                armEncoder.goTo(714,714,1);
                if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
                    if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
                        rotate.RotateRaw(2, gamepad1.left_stick_x);
                    }

                }
                else if(gamepad1.right_bumper)
                {
                    move.MoveRaw(4, 1);
                }
                else if(gamepad1.left_bumper)
                {
                    move.MoveRaw(3, 1);
                }
                else{
                    move.MoveStop();
                }

                if(gamepad1.right_trigger>0){
                    move.MoveRaw(1,gamepad1.right_trigger);
                }
                if(gamepad1.left_trigger>0){
                    move.MoveRaw(2,gamepad1.left_trigger);
                }
                if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                    move.MoveStop();
                }
                servosUp(topLeftServo,topRightServo);
                armEncoder.goTo(2142, 2142,1);
                while(armMotorLeft.isBusy() && armMotorRight.isBusy())
                {
                    if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
                        if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
                            rotate.RotateRaw(2, gamepad1.left_stick_x);
                        }

                    }
                    else if(gamepad1.right_bumper)
                    {
                        move.MoveRaw(4, 1);
                    }
                    else if(gamepad1.left_bumper)
                    {
                        move.MoveRaw(3, 1);
                    }
                    else{
                        move.MoveStop();
                    }

                    if(gamepad1.right_trigger>0){
                        move.MoveRaw(1,gamepad1.right_trigger);
                    }
                    if(gamepad1.left_trigger>0){
                        move.MoveRaw(2,gamepad1.left_trigger);
                    }
                    if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                        move.MoveStop();
                    }
                }
            }
            else if(gamepad2.dpad_down) //Arm Down
            {
                armCurrentDirection = "down";
                armEncoder.goTo(0,0,0.8);
                while(armMotorLeft.isBusy() && armMotorRight.isBusy())
                {
                    if(gamepad1.left_stick_x!=0 || gamepad1.left_stick_y!=0){
                        if(Math.abs(gamepad1.left_stick_x)>=Math.abs(gamepad1.left_stick_y)){
                            rotate.RotateRaw(2, gamepad1.left_stick_x);
                        }

                    }
                    else if(gamepad1.right_bumper)
                    {
                        move.MoveRaw(4, 1);
                    }
                    else if(gamepad1.left_bumper)
                    {
                        move.MoveRaw(3, 1);
                    }
                    else{
                        move.MoveStop();
                    }

                    if(gamepad1.right_trigger>0){
                        move.MoveRaw(1,gamepad1.right_trigger);
                    }
                    if(gamepad1.left_trigger>0){
                        move.MoveRaw(2,gamepad1.left_trigger);
                    }
                    if (gamepad1.right_trigger == 0 && gamepad1.left_trigger == 0){
                        move.MoveStop();
                    }
                }
            }
        }
    }
    private void servosUp(Servo topLeftServo, Servo topRightServo)
    {
        topLeftServo.setPosition(1);
        topRightServo.setPosition(1);
    }
}
