package org.firstinspires.ftc.bobot.utilities;

import com.acmerobotics.dashboard.config.Config;

import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class ButtonUtil {

    public static double DEBOUNCE = 20;
    public static double CLICK_TIMEOUT = 250;
    public static double HOLD_TIMEOUT = 350;

    private final ButtonInterface buttonInterface;

    private final ElapsedTime pressTimer;
    private final ElapsedTime releaseTimer;

    private boolean previousButtonValue = false;
    private boolean expectingDC = false;
    private boolean processDC = false;
    private boolean ignoreHolding = false;
    private boolean ignoreRelease = false;
    private boolean processSC = true;

    public ButtonUtil(ButtonInterface buttonInterface) {
        this.buttonInterface = buttonInterface;

        pressTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        releaseTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    }

    public ButtonEvent getButtonEvent() {
        ButtonEvent event = ButtonEvent.NONE;

        boolean currentButtonValue = buttonInterface.getButton();
        if (currentButtonValue && !previousButtonValue && releaseTimer.time() >= DEBOUNCE) {
            pressTimer.reset();
            processDC = releaseTimer.time() <= CLICK_TIMEOUT && expectingDC && !processDC;

            ignoreHolding = false;
            ignoreRelease = false;
            expectingDC = false;
            processSC = true;
        } else if (!currentButtonValue && previousButtonValue && pressTimer.time() >= DEBOUNCE && !ignoreRelease) {
            releaseTimer.reset();

            if (!processDC) expectingDC = true;
            else {
                expectingDC = false;
                processDC = false;
                processSC = false;

                event = ButtonEvent.DOUBLE_CLICK;
            }
        }

        if (!currentButtonValue && releaseTimer.time() >= CLICK_TIMEOUT && processSC && expectingDC && !processDC) {
            expectingDC = false;

            event = ButtonEvent.SINGLE_CLICK;
        }

        if (currentButtonValue && pressTimer.time() >= HOLD_TIMEOUT && !ignoreHolding) {
            ignoreRelease = true;
            ignoreHolding = true;
            expectingDC = false;
            processDC = false;

            event = ButtonEvent.HOLDING;
        }

        previousButtonValue = currentButtonValue;
        return event;
    }

    public enum ButtonEvent {NONE, SINGLE_CLICK, DOUBLE_CLICK, HOLDING}
}
