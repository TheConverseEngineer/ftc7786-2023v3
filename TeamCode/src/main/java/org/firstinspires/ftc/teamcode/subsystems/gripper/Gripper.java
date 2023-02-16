package org.firstinspires.ftc.teamcode.subsystems.gripper;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.command.Subsystem;

import static org.firstinspires.ftc.teamcode.subsystems.gripper.GripperConstants.*;

public class Gripper implements Subsystem {
    Servo leftGripServo, rightGripServo;

    public Gripper(Servo left, Servo right){
        //Initialize hardware
        leftGripServo = left;
        rightGripServo = right;
    }

    @Override
    public void periodic() {

    }

    //Methods to open and close the claw
    public void close(){
        leftGripServo.setPosition(leftGripClosePos);
        rightGripServo.setPosition(rightGripClosePos);
    }
    public void open(){
        leftGripServo.setPosition(leftGripOpenPos);
        rightGripServo.setPosition(rightGripOpenPos);
    }
}