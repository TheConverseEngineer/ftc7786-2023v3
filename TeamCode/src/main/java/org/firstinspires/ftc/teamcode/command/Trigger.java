package org.firstinspires.ftc.teamcode.command;

public abstract class Trigger {

    private boolean last = false;

    /** Internal function run only by CommandScheduler */
    public final void update() {
        boolean current = value();
        onValue(current);
        if (current && !last) onPress();
        else if (!current && last) onRelease();
        last = current;
    }

    /** The current value of the trigger */
    abstract boolean value();

    /** Runs every iteration
     *
     * @param val   the current value of the input
     */
    public void onValue(boolean val) {}

    /** Runs once every time this trigger activates */
    public void onPress() {}

    /** Runs once every time this trigger activates */
    public void onRelease() {}
}
