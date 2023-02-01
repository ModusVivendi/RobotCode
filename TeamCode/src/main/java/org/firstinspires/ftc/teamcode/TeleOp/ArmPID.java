package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Functions.PIDController;

@Config
@TeleOp
public class ArmPID extends LinearOpMode{
    private PIDController controller;
    private DcMotor armMotorLeft, armMotorRight;

//    public static double p = 0, i = 0, d = 0;
//    public static double f = 0;

    public static int target = 100;

    public final double ticks_in_cm = 384.5/11.2;


    @Override
    public void runOpMode() throws InterruptedException {
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");

        controller = new PIDController();

        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        while(opModeIsActive())
        {
            double commandLeft = controller.PIDControl(target, armMotorLeft.getCurrentPosition());
            double commandRight = controller.PIDControl(target, armMotorRight.getCurrentPosition());
            armMotorRight.setPower(-commandRight);
            armMotorLeft.setPower(-commandLeft);
        }
    }

}
