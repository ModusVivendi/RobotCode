package org.firstinspires.ftc.teamcode.Functions;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ArmEncoder {
    /**
     * Made by David
     */
    private DcMotor armMotor;
    private int armPos;
    public void Init()
    {
        /**
         * Runs the motor using encoders.
         */
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armPos = 0;
    }
    public ArmEncoder(DcMotor _AM)
    {
        armMotor = _AM;
        Init();
    }

    public void goTo(int armTarget, double power)
    {
      //  armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        armPos = 0;

        armPos+=armTarget;

        armMotor.setTargetPosition(armPos);

        armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        armMotor.setPower(power);

        while(armMotor.isBusy())
        {

        }


    }
}