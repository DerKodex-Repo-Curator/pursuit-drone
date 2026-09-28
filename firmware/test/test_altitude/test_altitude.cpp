/* Altitude sensing and hold, tested on the host.
 *
 * The rangefinder's tilt correction and the hover-throttle learner are both
 * pure maths, and both fail in ways that look like flying problems rather
 * than software problems: an uncorrected bank reads high and the aircraft
 * quietly descends; a learner that adapts during a climb teaches itself that
 * climb throttle is hover throttle and never stops climbing.
 */
#include <unity.h>

#include <cmath>

#include "alt_hold.h"
#include "rangefinder.h"

static const float DT = 1.0f / 30.0f;   /* the rangefinder's rate */
static const float MAX_TILT = 35.0f;


/* --------------------------------------------- pack-voltage compensation */

/* Hold station at zero error so the output IS the hover baseline, and read
 * what different pack voltages do to it. Thrust goes as (duty*V)^2: a full
 * pack needs LESS duty than reference, a sagged one MORE. Before this
 * existed the baseline was a constant, so a FAILSAFE descent late in the
 * pack commanded a duty that could not hover (AUDIT_REVD.md C3). */
static float steady_output_at(float volts)
{
    AltitudeHold ah;
    ah.arm(1.0f);
    if (volts > 0.0f) ah.set_pack_voltage(volts);
    float out = 0.0f;
    for (int i = 0; i < 5; i++)
        out = ah.update(1.0f, 1.0f, true, 0.5f, DT);
    return out;
}

void test_sagged_pack_commands_more_duty_than_full(void)
{
    const float full   = steady_output_at(4.20f);
    const float ref    = steady_output_at(3.70f);
    const float sagged = steady_output_at(3.30f);
    TEST_ASSERT_TRUE(full < ref);
    TEST_ASSERT_TRUE(ref < sagged);
    /* And the scale is the physics, not a fudge: V_REF/V within clamps. */
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.69f * (3.70f / 4.20f), full);
    TEST_ASSERT_FLOAT_WITHIN(0.02f, 0.69f * (3.70f / 3.30f), sagged);
}

void test_implausible_voltage_disables_compensation(void)
{
    const float base = steady_output_at(0.0f);       /* never fed */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, base, steady_output_at(0.0f + 1e-6f)); /* 0V: off */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, base, steady_output_at(9.0f));         /* junk: off */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, base, steady_output_at(NAN));          /* NaN: off */
}

void test_learner_stores_the_reference_voltage_baseline(void)
{
    /* Learn while hovering on a SAGGED pack, then ask for the baseline. If
     * the learner stored the compensated duty instead of normalising it
     * back to V_REF, the reference baseline would creep upward and a later
     * full-pack flight would hover too high. */
    AltitudeHold ah;
    ah.arm(1.0f);
    ah.set_pack_voltage(3.30f);
    for (int i = 0; i < 3000; i++)
        ah.update(1.0f, 1.0f, true, 0.5f, DT);       /* 100 s of clean hover */
    TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.69f, ah.hover_throttle());
}

/* ------------------------------------------------------- tilt correction */

void test_level_reads_through_unchanged(void)
{
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 1.20f,
        project_range_to_height(1.20f, 0.0f, 0.0f, MAX_TILT));
}

void test_bank_is_projected_back_to_vertical(void)
{
    /* Banked 30 degrees over a floor 1m below, the beam travels 1/cos(30) =
     * 1.155m. Without this correction the aircraft believes it is 15% higher
     * than it is and descends to fix an error that does not exist — so every
     * bank becomes an unintended sink. */
    const float slant = 1.0f / cosf(30.0f * 3.14159265f / 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.0f,
        project_range_to_height(slant, 30.0f, 0.0f, MAX_TILT));

    /* Same in pitch, and the two combine multiplicatively. */
    const float slant_p = 1.0f / cosf(20.0f * 3.14159265f / 180.0f);
    TEST_ASSERT_FLOAT_WITHIN(0.005f, 1.0f,
        project_range_to_height(slant_p, 0.0f, 20.0f, MAX_TILT));

    const float both = 1.0f / (cosf(15.0f * 3.14159265f / 180.0f) *
                               cosf(15.0f * 3.14159265f / 180.0f));
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 1.0f,
        project_range_to_height(both, 15.0f, 15.0f, MAX_TILT));
}

void test_correction_is_symmetric(void)
{
    const float a = project_range_to_height(1.5f, 25.0f, 0.0f, MAX_TILT);
    const float b = project_range_to_height(1.5f, -25.0f, 0.0f, MAX_TILT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, a, b);
}

void test_extreme_bank_is_rejected(void)
{
    /* Past the limit the beam is pointing at a wall or a chair, not the
     * floor. No trigonometry recovers a measurement of the wrong surface. */
    TEST_ASSERT_TRUE(project_range_to_height(2.0f, 50.0f, 0.0f, MAX_TILT) < 0);
    TEST_ASSERT_TRUE(project_range_to_height(2.0f, 0.0f, 60.0f, MAX_TILT) < 0);
    TEST_ASSERT_TRUE(project_range_to_height(2.0f, 30.0f, 30.0f, MAX_TILT) < 0);
}

void test_bad_input_is_rejected(void)
{
    const float nan_v = std::nan("");
    TEST_ASSERT_TRUE(project_range_to_height(nan_v, 0, 0, MAX_TILT) < 0);
    TEST_ASSERT_TRUE(project_range_to_height(1.0f, nan_v, 0, MAX_TILT) < 0);
    TEST_ASSERT_TRUE(project_range_to_height(-1.0f, 0, 0, MAX_TILT) < 0);
    TEST_ASSERT_TRUE(project_range_to_height(0.0f, 0, 0, MAX_TILT) < 0);
}

/* ------------------------------------------------------------ alt hold */

/* Toy vertical model: throttle above hover climbs, below it sinks, with drag.
 * Enough to check the loop converges and does not run away. */
struct Airframe {
    float height = 0.0f;
    float vel = 0.0f;
    /* Matches the DERIVED baseline in alt_hold.cpp: sqrt(66 g / 140 g) using
     * the README's own mass and thrust figures. The 0.30 that briefly lived
     * here modelled an airframe making 733 g of thrust from four 8520s. */
    float true_hover = 0.69f;

    void step(float throttle, float dt) {
        const float accel = (throttle - true_hover) * 22.0f - vel * 1.6f;
        vel += accel * dt;
        height += vel * dt;
        if (height < 0.0f) { height = 0.0f; if (vel < 0) vel = 0.0f; }
    }
};

void test_holds_a_commanded_height(void)
{
    AltitudeHold alt;
    Airframe af;
    af.height = 1.0f;
    alt.arm(af.height);

    for (int i = 0; i < 900; i++) {       /* 30 seconds */
        const float t = alt.update(1.0f, af.height, true, 0.3f, DT);
        af.step(t, DT);
    }
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.12f, 1.0f, af.height,
        "altitude hold did not settle on the commanded height");
}

void test_hover_throttle_is_learned(void)
{
    /* The point of the baseline: after holding station the loop should know
     * roughly what throttle hover takes, so the PID is only ever supplying
     * the difference rather than manufacturing hover from its integrator. */
    AltitudeHold alt;
    Airframe af;
    /* An airframe a little heavier than the baseline assumes. Close enough
     * that the loop can hold station, which is the ONLY condition under which
     * learning runs -- see the deadlock test below. */
    af.true_hover = 0.77f;
    af.height = 1.0f;
    alt.arm(af.height);

    for (int i = 0; i < 3000; i++) {
        const float t = alt.update(1.0f, af.height, true, 0.3f, DT);
        af.step(t, DT);
    }
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.08f, 0.77f, alt.hover_throttle(),
        "hover throttle did not converge on the airframe's actual value");
}

void test_a_badly_wrong_baseline_is_NOT_rescued_by_the_learner(void)
{
    /* Documents a real limitation. Note it cuts BOTH ways: this test used to
     * probe an airframe far ABOVE the baseline, but the same deadlock traps an
     * airframe far BELOW it, which is exactly the hole a hover_ of 0.30 would
     * have dropped a 0.69 airframe into.
     *
     * Learning is gated on holding_ -- small error AND small climb rate. An
     * airframe whose true hover is far from the baseline never settles, so
     * holding_ never goes true, so the baseline never moves. The learner
     * refines a nearly-right guess; it cannot rescue a wrong one.
     *
     * This is why hover_ must be MEASURED rather than modelled, and why
     * getting it wrong upward is the dangerous direction: the loop opens at
     * that throttle and has no mechanism to climb down from it quickly.
     */
    AltitudeHold alt;
    Airframe af;
    af.true_hover = 0.25f;        /* far BELOW the 0.69 baseline */
    af.height = 1.0f;
    alt.arm(af.height);

    for (int i = 0; i < 3000; i++) {
        const float t = alt.update(1.0f, af.height, true, 0.3f, DT);
        af.step(t, DT);
    }
    /* It gets nowhere near. Asserting the FAILURE so that if someone later
     * makes the learner able to cross this gap, this test fails loudly and
     * the comment above gets revisited rather than silently going stale. */
    TEST_ASSERT_TRUE_MESSAGE(alt.hover_throttle() > 0.50f,
        "learner unexpectedly crossed a large hover gap -- re-check the "
        "claim that hover_ must start close");
}

void test_climb_does_not_corrupt_the_learned_hover(void)
{
    /* Learning during a climb teaches the aircraft that climb throttle is
     * hover throttle — and it then climbs forever. */
    AltitudeHold alt;
    Airframe af;
    af.height = 0.5f;
    alt.arm(af.height);

    for (int i = 0; i < 400; i++) {       /* settle first */
        const float t = alt.update(0.5f, af.height, true, 0.6f, DT);
        af.step(t, DT);
    }
    const float settled = alt.hover_throttle();

    for (int i = 0; i < 200; i++) {       /* now climb hard */
        const float t = alt.update(2.0f, af.height, true, 0.6f, DT);
        af.step(t, DT);
    }
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.05f, settled, alt.hover_throttle(),
        "hover baseline drifted while climbing");
}

void test_setpoint_is_rate_limited(void)
{
    /* A commanded jump must become a climb, not an instant full-throttle
     * demand. */
    AltHoldConfig c = default_alt_hold_config();
    AltitudeHold alt;
    alt.configure(c);
    Airframe af;
    af.height = 0.5f;
    alt.arm(af.height);

    float peak_rate = 0.0f;
    float prev = af.height;
    for (int i = 0; i < 300; i++) {
        const float t = alt.update(3.0f, af.height, true, 0.6f, DT);
        af.step(t, DT);
        peak_rate = fmaxf(peak_rate, (af.height - prev) / DT);
        prev = af.height;
    }
    /* Some overshoot past the setpoint slew is expected from the airframe's
     * own momentum, but not several times it. */
    TEST_ASSERT_TRUE_MESSAGE(peak_rate < c.max_climb_ms * 3.0f,
        "climb rate far exceeded the setpoint slew limit");
}

void test_falls_back_when_the_rangefinder_drops_out(void)
{
    /* An aircraft that cannot measure height must still be flyable — just not
     * hands-off. Holding an altitude it can no longer see is how it drifts
     * silently into the floor. */
    AltitudeHold alt;
    alt.arm(1.0f);
    for (int i = 0; i < 100; i++) alt.update(1.0f, 1.0f, true, 0.6f, DT);

    const float out = alt.update(1.0f, 0.0f, false, 0.42f, DT);
    TEST_ASSERT_FLOAT_WITHIN(1e-5, 0.42f, out);
    TEST_ASSERT_FALSE(alt.holding());
}

void test_output_stays_within_limits(void)
{
    AltHoldConfig c = default_alt_hold_config();
    AltitudeHold alt;
    alt.configure(c);
    alt.arm(0.0f);

    const float heights[] = {0.0f, 0.05f, 1.0f, 3.5f, 100.0f};
    const float targets[] = {0.0f, 0.4f, 2.0f, 50.0f};
    for (float h : heights)
        for (float t : targets)
            for (int i = 0; i < 60; i++) {
                const float out = alt.update(t, h, true, 0.6f, DT);
                TEST_ASSERT_TRUE(out >= c.min_throttle - 1e-6f);
                TEST_ASSERT_TRUE(out <= c.max_throttle + 1e-6f);
                TEST_ASSERT_FALSE(std::isnan(out));
            }
}

void test_arming_does_not_command_a_step(void)
{
    /* Arming at 1.5m with a stale 0m setpoint would demand a dive. */
    AltitudeHold alt;
    alt.arm(1.5f);
    const float first = alt.update(1.5f, 1.5f, true, 0.6f, DT);
    TEST_ASSERT_FLOAT_WITHIN_MESSAGE(0.08f, alt.hover_throttle(), first,
        "engaging altitude hold produced a throttle step");
}

void test_survives_degenerate_dt(void)
{
    AltitudeHold alt;
    alt.arm(1.0f);
    TEST_ASSERT_FALSE(std::isnan(alt.update(1.0f, 1.0f, true, 0.6f, 0.0f)));
    TEST_ASSERT_FALSE(std::isnan(alt.update(1.0f, 1.0f, true, 0.6f, -1.0f)));
    TEST_ASSERT_FALSE(std::isnan(alt.update(1.0f, 1.0f, true, 0.6f, 99.0f)));
    const float nan_v = std::nan("");
    TEST_ASSERT_FALSE(std::isnan(alt.update(nan_v, 1.0f, true, 0.6f, DT)));
}

/* --------------------------------------------------- commanded ceiling */

/* The numbers from config.h. Repeated rather than included because config.h
 * is board wiring and this is a host build; the point of the tests is the
 * BEHAVIOUR at a ceiling, not the particular metres. */
static constexpr float CEIL_TOF = 2.50f;
static constexpr float CEIL_BARO = 9.50f;

void test_without_a_barometer_the_ceiling_is_the_rangefinder(void)
{
    /* The failure this prevents is quiet: with no barometer, a climb past
     * ~3.5m loses the floor and altitude hold degrades to open-loop throttle
     * rather than to an error, so the aircraft simply drifts off at whatever
     * height it stopped being able to see. */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, CEIL_TOF,
        pursuit_clamp_alt_setpoint(9.0f, false, CEIL_TOF, CEIL_BARO));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, CEIL_TOF,
        pursuit_clamp_alt_setpoint(100.0f, false, CEIL_TOF, CEIL_BARO));
}

void test_with_a_barometer_thirty_feet_is_allowed(void)
{
    /* 30 ft is 9.14m — the number this whole sensor was fitted for. */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 9.14f,
        pursuit_clamp_alt_setpoint(9.14f, true, CEIL_TOF, CEIL_BARO));
}

void test_a_reachable_setpoint_passes_through_untouched(void)
{
    const float reachable[] = {0.0f, 0.4f, 1.2f, 2.5f};
    for (float m : reachable)
        TEST_ASSERT_FLOAT_WITHIN(1e-4f, m,
            pursuit_clamp_alt_setpoint(m, false, CEIL_TOF, CEIL_BARO));
}

void test_a_garbled_setpoint_descends_rather_than_climbing(void)
{
    /* Both directions of nonsense must fail toward the ground. Clamping a NaN
     * to the CEILING would turn one corrupt float into a full-power climb. */
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f,
        pursuit_clamp_alt_setpoint(std::nan(""), true, CEIL_TOF, CEIL_BARO));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f,
        pursuit_clamp_alt_setpoint(-5.0f, true, CEIL_TOF, CEIL_BARO));
    TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f,
        pursuit_clamp_alt_setpoint(HUGE_VALF, false, CEIL_TOF, CEIL_BARO));
}

/* ----------------------------------------------------------------- main */

int main(int, char **)
{
    UNITY_BEGIN();

    RUN_TEST(test_level_reads_through_unchanged);
    RUN_TEST(test_bank_is_projected_back_to_vertical);
    RUN_TEST(test_correction_is_symmetric);
    RUN_TEST(test_extreme_bank_is_rejected);
    RUN_TEST(test_bad_input_is_rejected);

    RUN_TEST(test_holds_a_commanded_height);
    RUN_TEST(test_sagged_pack_commands_more_duty_than_full);
    RUN_TEST(test_implausible_voltage_disables_compensation);
    RUN_TEST(test_learner_stores_the_reference_voltage_baseline);
    RUN_TEST(test_hover_throttle_is_learned);
    RUN_TEST(test_a_badly_wrong_baseline_is_NOT_rescued_by_the_learner);
    RUN_TEST(test_climb_does_not_corrupt_the_learned_hover);
    RUN_TEST(test_setpoint_is_rate_limited);
    RUN_TEST(test_falls_back_when_the_rangefinder_drops_out);
    RUN_TEST(test_output_stays_within_limits);
    RUN_TEST(test_arming_does_not_command_a_step);
    RUN_TEST(test_survives_degenerate_dt);

    RUN_TEST(test_without_a_barometer_the_ceiling_is_the_rangefinder);
    RUN_TEST(test_with_a_barometer_thirty_feet_is_allowed);
    RUN_TEST(test_a_reachable_setpoint_passes_through_untouched);
    RUN_TEST(test_a_garbled_setpoint_descends_rather_than_climbing);

    return UNITY_END();
}
