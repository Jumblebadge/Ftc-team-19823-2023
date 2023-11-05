package org.firstinspires.ftc.teamcode.utility;

public class ButtonDetector {

    public boolean current = false;
    public boolean last = false;

    public boolean toggle(boolean currentState) {
        if (currentState && !last) {
            current = !current;
        }

        last = currentState;
        return current;
    }

    public boolean risingEdge(boolean currentState) {
        boolean state = currentState && !last;
        last = currentState;
        return state;
    }

    public void toFalse() {
        current = false;
    }
}

