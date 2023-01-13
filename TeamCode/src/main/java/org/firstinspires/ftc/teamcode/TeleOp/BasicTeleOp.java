package org.firstinspires.ftc.teamcode.TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.ClawServos;
import org.firstinspires.ftc.teamcode.Functions.Move;
import org.firstinspires.ftc.teamcode.Functions.Rotate;

@TeleOp(name="Basic TeleOp", group="GAME")
public class BasicTeleOp extends OpMode {
    private DcMotor leftMotor, rightMotor, leftMotorBack, rightMotorBack;
    private DcMotor armMotorLeft, armMotorRight;
    private Servo leftServo, rightServo;
    private Move move;
    private Rotate rotate;
    private ClawServos clawServos;
    private ArmEncoder armEncoder;
    int armTarget = 0;
    double armSpeed = 0 ;
    String armCurrentDirection = "up";
    @Override
    public void init() {
        leftMotor = hardwareMap.dcMotor.get("FL");
        rightMotor = hardwareMap.dcMotor.get("FR");
        leftMotorBack = hardwareMap.dcMotor.get("BL");
        rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
        leftServo = hardwareMap.servo.get("LS");
        rightServo = hardwareMap.servo.get("RS");
        move = new Move(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        rotate = new Rotate(leftMotor, rightMotor, leftMotorBack, rightMotorBack);
        clawServos = new ClawServos(leftServo, rightServo);
        armEncoder = new ArmEncoder(armMotorLeft, armMotorRight);



        rightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }

    @Override
    public void loop() {
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

            armEncoder.goTo(2855, 2855,1);
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
        if(gamepad1.dpad_up) // Arm Up
        {
            armCurrentDirection = "up";

            armEncoder.goTo(2855,2855,1);
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
        else if(gamepad1.dpad_down) //Arm Down
        {
            armCurrentDirection = "down";
            armEncoder.goTo(0,0,0.8);
            while (armMotorLeft.isBusy() && armMotorRight.isBusy())
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
        if(armCurrentDirection.equals("down"))
        {
            armMotorLeft.setPower(0);
            armMotorRight.setPower(0);
            armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }
    }
}
