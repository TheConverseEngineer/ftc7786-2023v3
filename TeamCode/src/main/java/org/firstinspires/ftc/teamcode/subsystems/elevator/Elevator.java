package org.firstinspires.ftc.teamcode.subsystems.elevator;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Elevator implements Subsystem {

    // The math for this is super convoluted, so it has all been moved to this other class
    private final LiftStateManager liftStateManager;

    private final DcMotor motor1, motor2;
    private double lastRecordedPos;

    public Elevator(DcMotor liftMotorWithEncoder, DcMotor otherLiftMotor, double startPos) {
        liftStateManager = new LiftStateManager(startPos);
        motor1 = liftMotorWithEncoder;
        motor2 = otherLiftMotor;
        lastRecordedPos = startPos;
    }

    @Override
    public void periodic() {
        double position = motor1.getCurrentPosition() / ElevatorConstants.TICKS_PER_INCH;
        lastRecordedPos = position;
        if (position < ElevatorConstants.MIN_POS) {
            motor1.setPower(.25);
            motor2.setPower(.25);
        } else if (position > ElevatorConstants.MAX_POS) {
            motor1.setPower(-.25);
            motor2.setPower(-.25);
        } else {
            double power = liftStateManager.getMotorPower(position);
            motor1.setPower(power);
            motor2.setPower(power);
        }
    }

    public void goToPosition(double newPos) {
        this.liftStateManager.setNewTarget(newPos);
    }

    public double getPosition() {
        return lastRecordedPos;
    }

    public Command goToPositionCommand(double position) {
        return new Command() {
            @Override
            public void init() {
                goToPosition(MathUtils.clamp(position, 0, ElevatorConstants.MAX_POS));
            }
            @Override public void loop() { }
            @Override public void end() { }
            @Override public boolean isComplete() {
                return Math.abs(lastRecordedPos - position) < 0.02;
            }
        };
    }

    public Command goToPositionAsyncCommand(double position) {
        return new Command() {
            @Override
            public void init() {
                goToPosition(MathUtils.clamp(position, 0, ElevatorConstants.MAX_POS));
            }
            @Override public void loop() { }
            @Override public void end() { }
            @Override public boolean isComplete() {
                return true;
            }
        };
    }
}
