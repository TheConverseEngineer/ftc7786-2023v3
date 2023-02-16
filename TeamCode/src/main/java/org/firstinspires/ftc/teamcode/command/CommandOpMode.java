package org.firstinspires.ftc.teamcode.command;

import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


/** OpMode template to help simplify the creation of Command-Based code */
public abstract class CommandOpMode extends LinearOpMode {

    @Override
    public final void runOpMode() {
        // Setup Command stuff
        CommandScheduler master = CommandScheduler.getInstance();
        master.reset();
        init(master);

        PhotonCore.enable();

        waitForStart();

        start(master);

        while (opModeIsActive() && !isStopRequested()) {
            loop(master);
            master.run();
        }
        end();
        master.reset();
    }

    /** Initialize commands, subsystems, and triggers here!
     * NOTE: DO NOT FORGET to register subsystems/triggers and queue start-up commands
     * @param master    The CommandScheduler instance
     */
    public abstract void init(CommandScheduler master);

    /** Can be used to add additional non-Command behavior to opMode loops.
     *  Mainly intended for drive code and such.
     *  AVOID BLOCKING LOOPS LIKE THE PLAGUE! */
    public void loop(CommandScheduler master) {

    }

    /** Can be used to add on-OpMode-end behaviors.
     * AVOID BLOCKING LOOPS LIKE THE PLAGUE! */
    public void end() {

    }

    /** Can be used to add behaviors when start is pressed */
    public void start(CommandScheduler master) {

    }
}