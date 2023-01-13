package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmEncoder {
    /**
     * Made by David
     */
    private DcMotor armMotorLeft, armMotorRight;
    private int armLeftPos, armRightPos;
    public void Init()
    {
        /**
         * Runs the motor using encoders.
         */
        armMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
       // armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armLeftPos = 0;
        armRightPos = 0;
    }
    public ArmEncoder(DcMotor _AML, DcMotor _AMR)
    {
        armMotorLeft= _AML;
        armMotorRight= _AMR;
        Init();
    }
    public void goTo(int armLeftTarget, int armRightTarget, double power)
    {
      //  armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armLeftPos = 0;
        armRightPos = 0;

        armLeftPos+=armLeftTarget;
        armRightPos+=armRightTarget;

        armMotorLeft.setTargetPosition(armLeftPos);
        armMotorRight.setTargetPosition(armRightPos);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotorLeft.setPower(power);
        armMotorRight.setPower(power);

//        while(armMotor.isBusy())
//        {
//
//        }


    }
}