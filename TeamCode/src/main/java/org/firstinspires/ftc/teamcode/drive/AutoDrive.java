package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.command.Command;
import org.firstinspires.ftc.teamcode.command.Subsystem;
import org.firstinspires.ftc.teamcode.quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.quickstart.trajectorysequence.TrajectorySequence;

/** Small extension of SampleMecanumDrive that bridges the gap to command-based */
public class AutoDrive extends SampleMecanumDrive implements Subsystem {
    public AutoDrive(HardwareMap hardwareMap) {
        super(hardwareMap);
    }

    @Override
    public void periodic() {
        update();
    }

    /**
     * The returned command will cause the robot to follow the trajectory without
     * interrupting command execution
     */
    public Command followTrajectoryAsyncCommand(Trajectory trajectory) {
        return new Command() {
            @Override
            public void init() {
                followTrajectoryAsync(trajectory);
            }
            @Override public void loop() { }
            @Override public void end()  { }
            @Override public boolean isComplete() {
                return true;
            }
        };
    }

    /**
     * The returned command will cause the robot to follow the trajectory and pause
     * execution of the sequential command thread until the trajectory is complete
     */
    public Command followTrajectoryCommand(Trajectory trajectory) {
        return new Command() {
            @Override
            public void init() {
                followTrajectoryAsync(trajectory);
            }
            @Override public void loop() { }
            @Override public void end()  { }
            @Override public boolean isComplete() {
                return isBusy();
            }
        };
    }
}
