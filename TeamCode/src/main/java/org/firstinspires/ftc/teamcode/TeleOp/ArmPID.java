package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp
public class ArmPID extends LinearOpMode{
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public final double ticks_in_inch = 700/180.0;
    private DcMotor armMotorLeft, armMotorRight;

    @Override
    public void runOpMode() throws InterruptedException {
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");

        armMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
