package org.firstinspires.ftc.teamcode.experimental;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.utils.MathUtils;

/* COEFFICIENTS:
 *      d = 0.0001
 *      f = 0.1
 *      i = 0
 *      p = 0.004
 */

@Config
@TeleOp(name = "Experimental Arm Tuner", group = "experimental")
public class ExperimentalArmTuner extends LinearOpMode {

    public static double p = 0, i = 0, d = 0, f = 0;
    public static int target = 0;
    public static double MAX_POWER = 0.75;
    public static double ticksPerDegree = 700 / 180.0;

    private DcMotorEx armMotor;
    private PIDController controller;

    @Override
    public void runOpMode() {
        controller = new PIDController(p, i, d, 0);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        armMotor = hardwareMap.get(DcMotorEx.class, "armmotor");
        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        waitForStart();

        armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while(opModeIsActive() && !isStopRequested()) {
            controller.setPID(p, i, d);
            int armPos = armMotor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target / ticksPerDegree)) * f;

            armMotor.setPower(MathUtils.clamp(pid + ff, -MAX_POWER, MAX_POWER));
            telemetry.addData("pos", armPos);
            telemetry.addData("target", target);
            telemetry.update();
        }
    }
}
