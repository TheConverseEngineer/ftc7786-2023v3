package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.CommandOpMode;
import org.firstinspires.ftc.teamcode.command.CommandScheduler;
import org.firstinspires.ftc.teamcode.command.prefabs.SequentialCommand;
import org.firstinspires.ftc.teamcode.command.prefabs.WaitCommand;
import org.firstinspires.ftc.teamcode.drive.AutoDrive;
import org.firstinspires.ftc.teamcode.subsystems.arm.Arm;
import org.firstinspires.ftc.teamcode.subsystems.arm.Flipper;
import org.firstinspires.ftc.teamcode.subsystems.elevator.Elevator;
import org.firstinspires.ftc.teamcode.subsystems.gripper.Gripper;

public class Auto extends CommandOpMode {
    Arm arm;
    Gripper gripper;
    Elevator elevator;
    Flipper flipper;
    AutoDrive drive;

    Trajectory trajectory;

    @Override
    public void init(CommandScheduler master) {
        // Hardware get calls go here
        // make trajectory

        master.registerSubsystem(arm, gripper, elevator, flipper, drive);
        master.ScheduleCommand(
                new SequentialCommand(
                        SequentialCommand.runAsync(new SequentialCommand(
                                new WaitCommand(100)
                        )),
                        drive.followTrajectoryCommand(trajectory)
                )
        );
    }
}
