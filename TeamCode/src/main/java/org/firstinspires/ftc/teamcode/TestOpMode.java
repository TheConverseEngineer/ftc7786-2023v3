package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.quickstart.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.quickstart.trajectorysequence.TrajectorySequence;

@Autonomous(name = "TestAuto", group = "default")
@Config
public class TestOpMode extends LinearOpMode {
    public static double head = 30, fx = 25, fy = 47, tx=30, ty=45, endHead = -20;

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PhotonCore.enable();

        TrajectorySequence t1 = drive.trajectorySequenceBuilder(new Pose2d(0,0,0))
                .lineToLinearHeading(new Pose2d(30,3,0))
                .turn(Math.toRadians(head))
                .lineToConstantHeading(new Vector2d(tx, ty))
                .splineToConstantHeading(new Vector2d(fx, fy), Math.toRadians(endHead))
                .build();

        waitForStart();

        drive.followTrajectorySequence(t1);

    }
}
