package org.firstinspires.ftc.teamcode.Autonomous;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Functions.ArmEncoder;
import org.firstinspires.ftc.teamcode.Functions.NewEncoderMove;

@Autonomous(name="EncoderNew", group = "Concept")
//@Disabled
public class TestNewEncoder extends LinearOpMode {

   // private DcMotor leftMotor, leftMotorBack, rightMotor, rightMotorBack;
    private DcMotor armMotor;
    private NewEncoderMove encoderMove;
    private ArmEncoder armEncoder;

    @Override
    public void runOpMode()
    {
      //  leftMotor = hardwareMap.dcMotor.get("FL");
       // leftMotorBack = hardwareMap.dcMotor.get("BL");
       // rightMotor = hardwareMap.dcMotor.get("FR");
       // rightMotorBack = hardwareMap.dcMotor.get("BR");
        armMotor = hardwareMap.dcMotor.get("AM");
       // encoderMove = new NewEncoderMove(leftMotor,leftMotorBack,rightMotor,rightMotorBack);

        armEncoder = new ArmEncoder(armMotor);

        waitForStart();

        armEncoder.goTo(1540, 0.5);
        sleep(1000);
       // encoderMove.DriveTo(1000,1000,1000,1000,0.5, opModeIsActive());
      //  sleep(1000);
      //  encoderMove.DriveTo(-1000,1000,1000,-1000,0.5,opModeIsActive());
      //  sleep(500);
    }
}

