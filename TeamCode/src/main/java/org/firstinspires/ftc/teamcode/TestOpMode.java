package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.quickstart.trajectorysequence.TrajectorySequence;

@Autonomous(name = "TestAuto", group = "default")
@Config
public class TestOpMode extends LinearOpMode {
    public static double head = 30, fx = 25, fy = 48;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(30,3,0))
                .lineToLinearHeading(new Pose2d(30,4, Math.toRadians(34)))
                .lineToSplineHeading(new Pose2d(30, 45, Math.toRadians(34)))
                .lineToLinearHeading(new Pose2d(fx,fy, Math.toRadians(head)))
                .build();

        waitForStart();

        drive.followTrajectorySequence(t1);

    }
}
