package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.subsystems.elevator.ElevatorConstants;
import org.firstinspires.ftc.teamcode.utils.MathUtils;

public class Arm implements Subsystem {
    private final DcMotor armMotor;

    private final ElapsedTime timer;

    // target positions
    private double armMotorTargetPos;

    // PID helper values
    private double lastI = 0;
    private double armPos = 0;
    private double lastError = 0;
    private double lastTime;

    /** Assumes that the arm is originally facing forward
     *
     * @param armMotor      The dc motor controlling the arm
     * @param armStartPosition  The default starting position of the arm
     */
    public Arm(DcMotor armMotor, double armStartPosition) {
        this.armMotor = armMotor;

        timer = new ElapsedTime();
        timer.reset();
        armMotorTargetPos = armStartPosition;
    }

    public void setTargetPos(double armMotorTargetPos) {
        // Set PID I value to zero when the motor target changes
        lastI = 0;
        this.armMotorTargetPos = MathUtils.clamp(armMotorTargetPos, ArmConstants.ARM_MIN_POS, ArmConstants.ARM_MAX_POS);
    }

    public double getTargetPos() {
        return this.armMotorTargetPos;
    }


    @Override
    public void periodic() {
        //Update position values
        armPos = armMotor.getCurrentPosition();
        if (armPos < ArmConstants.ARM_MIN_POS) armMotor.setPower(.25);
        else if (armPos > ArmConstants.ARM_MAX_POS) armMotor.setPower(-.25);
        else loopArmPID();
    }

    public void loopArmPID(){
        if (armMotorTargetPos < 10 && armPos < 200 && armPos > 50) {
            armMotor.setPower(ArmConstants.MIN_END_SPEED);
            return;
        }
        double error = armMotorTargetPos - armPos;
        double deltaTime = timer.seconds()-lastTime;
        //If error changes sign set I to zero
        if(error*lastError<=0) lastI = 0;
        else lastI += error*deltaTime;

        double IVal = MathUtils.clamp(ArmConstants.I_PID*lastI, -ArmConstants.I_CAP, ArmConstants.I_CAP);
        double DVal = (error-lastError)/deltaTime;
        double output = (error* ((armPos<=685) ? ((error>=0)?ArmConstants.P_PID_UP:ArmConstants.P_PID_DOWN) : ((error>=0)?ArmConstants.P_PID_DOWN:ArmConstants.P_PID_UP))) + (IVal) + (DVal*ArmConstants.D_PID);

        lastError = error;
        lastTime += deltaTime;
        armMotor.setPower(MathUtils.clamp(output + ((armPos > 700)?-ArmConstants.STATIC_PID:((armPos < 600)?ArmConstants.STATIC_PID:0)), ArmConstants.MIN_ARM_POWER, ArmConstants.MAX_ARM_POWER));
    }

    /** The arm controller can be a bit finicky, so it might be a god idea to run this command with
     * a timeout attached to it
     */
    public Command goToPositionCommand(double position) {
        return new Command() {
            @Override
            public void init() {
                setTargetPos(MathUtils.clamp(position, ArmConstants.ARM_MIN_POS, ArmConstants.ARM_MAX_POS));
            }
            @Override public void loop() { }
            @Override public void end() { }
            @Override public boolean isComplete() {
                return Math.abs(armPos - position) < 25;
            }
        };
    }

    public Command goToPositionAsyncCommand(double position) {
        return new Command() {
            @Override
            public void init() {
                setTargetPos(MathUtils.clamp(position, ArmConstants.ARM_MIN_POS, ArmConstants.ARM_MAX_POS));
            }
            @Override public void loop() { }
            @Override public void end() { }
            @Override public boolean isComplete() {
                return true;
            }
        };
    }
}