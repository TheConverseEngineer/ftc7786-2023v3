package org.firstinspires.ftc.teamcode.subsystems.arm;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.Subsystem;

public class Flipper implements Subsystem {
    Servo flipServo;

    public Flipper(Servo flipServo){
        this.flipServo = flipServo;
    }

    @Override
    public void periodic() {

    }

    public void flipForward() {
        this.flipServo.setPosition(ArmConstants.FLIP_SERVO_FORWARD);
    }

    public void flipReverse() {
        this.flipServo.setPosition(ArmConstants.FLIP_SERVO_REVERSE);
    }
}
