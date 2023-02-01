package org.firstinspires.ftc.teamcode.Functions;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
@Config
public class PIDController {

    DcMotorEx armMotorLeft, armMotorRight;
    double integralSum = 0;
    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;
    public static double kf = 0;

    ElapsedTime timer = new ElapsedTime();
    private double lastError = 0;
    public PIDController()
    {
    }

//    @Override
//    public void runOpMode() throws InterruptedException {
//        armMotorLeft = hardwareMap.get(DcMotorEx.class, "AML");
//        armMotorRight = hardwareMap.get(DcMotorEx.class, "AMR");
//
//        armMotorLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        armMotorRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//
//        waitForStart();
//
//        while(opModeIsActive()) {
//            double powerRight = PIDControl(1000, armMotorRight.getVelocity());
//            double powerLeft = PIDControl(1000, armMotorLeft.getVelocity());
//            armMotorRight.setPower(-powerRight);
//            armMotorLeft.setPower(-powerLeft);
//        }
//
//    }

    public double PIDControl(double reference, double state) {
        double error = reference - state;
        integralSum += error * timer.seconds();
        double derivative = (error - lastError) / timer.seconds();
        lastError = error;

        timer.reset();

        double output = (error * kp) + (derivative * kd) + (integralSum * ki) + (reference * kf);
        return output;
    }
}
