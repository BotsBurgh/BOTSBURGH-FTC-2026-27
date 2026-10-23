package org.firstinspires.ftc.teamcode.vision2.blobvision;

import java.util.function.BooleanSupplier;

/**
 * Decides how many artifacts were just picked up.
 *
 * Default is {@link #INFERRED} (count one each time the intake push finishes on a confirmed target).
 * A real sensor is far more reliable: {@code PickupDetector.sensor(() -> beamBreak.isBroken(), 80)}.
 */
public interface PickupDetector {

    final class Context {
        /** The blind intake push just ended this loop. */
        public final boolean pushFinished;
        /** The artifact we were pushing into had been confirmed by the tracker. */
        public final boolean targetConfirmed;
        public final long nowMs;

        public Context(boolean pushFinished, boolean targetConfirmed, long nowMs) {
            this.pushFinished = pushFinished;
            this.targetConfirmed = targetConfirmed;
            this.nowMs = nowMs;
        }
    }

    /** @return number of NEW artifacts picked up since the last call (usually 0 or 1). */
    int poll(Context ctx);

    /** Vision-only estimate: one pickup per completed push on a confirmed target. */
    PickupDetector INFERRED = new PickupDetector() {
        @Override
        public int poll(Context ctx) {
            return (ctx.pushFinished && ctx.targetConfirmed) ? 1 : 0;
        }
    };

    /** Counts rising edges of a sensor (beam break, color sensor, current spike...) with debounce. */
    static PickupDetector sensor(final BooleanSupplier triggered, final long debounceMs) {
        return new PickupDetector() {
            private boolean last = false;
            private long lastCountMs = Long.MIN_VALUE / 2;

            @Override
            public int poll(Context ctx) {
                boolean now = triggered.getAsBoolean();
                int n = 0;
                if (now && !last && ctx.nowMs - lastCountMs >= debounceMs) {
                    n = 1;
                    lastCountMs = ctx.nowMs;
                }
                last = now;
                return n;
            }
        };
    }
}
