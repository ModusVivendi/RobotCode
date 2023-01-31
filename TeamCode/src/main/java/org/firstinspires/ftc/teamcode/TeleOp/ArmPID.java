package org.firstinspires.ftc.teamcode.TeleOp;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.controller.PIDController;

@Config
@TeleOp
public class ArmPID extends OpMode {
    private PIDController controller;

    public static double p = 0, i = 0, d = 0;
    public static double f = 0;

    public static int target = 0;

    public final double ticks_in_degree = 700/180.0;
    private DcMotor armMotorLeft, armMotorRight;

    @Override
    public void init() {
        controller = new PIDController(p, i, d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        armMotorLeft = hardwareMap.dcMotor.get("AML");
        armMotorRight = hardwareMap.dcMotor.get("AMR");
    }

    @Override
    public void loop() {
        controller.setPID(p,i,d);

        int armPosLeft = armMotorLeft.getCurrentPosition();
        int armPosRight = armMotorRight.getCurrentPosition();

        double pidLeft = controller.calculate(armPosLeft, target);
        double pidRight = controller.calculate(armPosRight, target);

        double ff = Math.cos(Math.toRadians(target/ticks_in_degree)) * f;

        double powerLeft = pidLeft + ff;
        double powerRight = pidRight + ff;

        armMotorLeft.setPower(powerLeft);
        armMotorRight.setPower(powerRight);

        telemetry.addData("posLeft: ", armPosLeft);
        telemetry.addData("posRight: ", armPosRight);
        telemetry.update();
    }
}
