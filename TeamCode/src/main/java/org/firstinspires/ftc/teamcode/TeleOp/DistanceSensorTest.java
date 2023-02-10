import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Functions.ColorSensorV3;


@TeleOp(name = "DistanceSensorTest", group = "Tests")
public class DistanceSensorTest extends LinearOpMode {
    private ColorSensorV3 distanceSensor;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");

        waitForStart();

        while (opModeIsActive()) {
            double distance = distanceSensor.getDistance();
            telemetry.addData("Distance", distance);
            telemetry.update();
        }
    }
}
