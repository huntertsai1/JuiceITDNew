package org.firstinspires.ftc.teamcode.util;

import static java.util.Map.entry;

import java.util.HashMap;
import java.util.Map;

public class StateKeeper {
    public static HashMap<Double, Boolean> specimenHighRungMap = (HashMap<Double, Boolean>) Map.ofEntries(
            entry(9.0, false),
            entry(8.0, false),
            entry(7.0, false),
            entry(6.0, false),
            entry(5.0, false),
            entry(4.0, false),
            entry(3.0, false),
            entry(2.0, false),
            entry(1.0, false),
            entry(0.0, false),
            entry(-1.0, false),
            entry(-2.0, false),
            entry(-3.0, false),
            entry(-4.0, false),
            entry(-5.0, false),
            entry(-6.0, false),
            entry(-7.0, false),
            entry(-8.0, false),
            entry(-9.0, false)
    );;

    /**
     * Finds the top priority open slot for depositing specimen. <b>Returns -16236 if no slots are open</b>
     * @return The x-coordinate of the best open slot
     */
    public static double findOpenHighRung() {
        for (Map.Entry<Double, Boolean> slot : specimenHighRungMap.entrySet()) {
            if (!slot.getValue()) {
                return slot.getKey();
            }
        }

        return -16236;
    }

    /**
     * Enters slots that you indicate you have deposited onto as taken onto the immortal record
     * @param slots Slots you have deposited onto and want to mark as taken
     */
    public static void putSpecimenHighRung(Double... slots) {
        for (double slot : slots) {
            specimenHighRungMap.replace(slot, true);
        }
    }
}
