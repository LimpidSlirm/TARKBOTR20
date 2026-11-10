#include "turn_control.h"
#include <float.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static unsigned checks;
#define CHECK(x) do { ++checks; if (!(x)) { \
    fprintf(stderr, "FAIL line %d: %s\n", __LINE__, #x); exit(1); \
} } while (0)

static int near(double a, double b)
{
    return fabs(a - b) < 0.000001;
}

static TurnConfig example_config(void)
{
    TurnConfig config = {0.35f, 0.10f, 0.60f, 0.20f};
    return config;
}

static void basics(void)
{
    TurnConfig config = example_config();
    TurnTarget left;
    TurnTarget right;
    CHECK(Turn_Left(&config, 0.10f, 25.0f, &left) == TURN_OK);
    CHECK(near(left.forward_mps, 0.10));
    CHECK(near(left.yaw_rate_radps, 0.15));
    CHECK(Turn_Right(&config, 0.10f, 25.0f, &right) == TURN_OK);
    CHECK(near(right.forward_mps, left.forward_mps));
    CHECK(near(right.yaw_rate_radps, -left.yaw_rate_radps));
    CHECK(Turn_Left(&config, 0.10f, 0.0f, &left) == TURN_OK);
    CHECK(near(left.forward_mps, 0.10) && left.yaw_rate_radps == 0.0f);
    CHECK(Turn_Right(&config, 0.0f, 100.0f, &right) == TURN_LIMITED);
    CHECK(right.forward_mps == 0.0f && right.yaw_rate_radps == 0.0f);
    CHECK(Turn_Left(&config, 0.0f, 0.0f, &left) == TURN_OK);
    CHECK(left.forward_mps == 0.0f && left.yaw_rate_radps == 0.0f);
    CHECK(Turn_Left(&config, 1.0f, 0.0f, &left) == TURN_LIMITED);
    CHECK(near(left.forward_mps, 0.10));
    CHECK(Turn_Left(&config, 0.10f, 100.0f, &left) == TURN_LIMITED);
    CHECK(near(left.yaw_rate_radps, 2.0 * left.forward_mps / config.effective_track_m));
}

static void wheel_limit(void)
{
    TurnConfig config = example_config();
    TurnTarget target;
    double left;
    double right;
    config.max_wheel_mps = 0.08f;
    CHECK(Turn_Left(&config, 0.10f, 50.0f, &target) == TURN_LIMITED);
    left = target.forward_mps - target.yaw_rate_radps * (double)config.effective_track_m / 2.0;
    right = target.forward_mps + target.yaw_rate_radps * (double)config.effective_track_m / 2.0;
    CHECK(left >= 0.0 && near(right, 0.08));
    CHECK(near(target.yaw_rate_radps / target.forward_mps, 3.0));
    CHECK(Turn_Right(&config, 0.10f, 0.0f, &target) == TURN_LIMITED);
    CHECK(near(target.forward_mps, 0.08) && target.yaw_rate_radps == 0.0f);
}

static void invalid_inputs(void)
{
    TurnConfig config = example_config();
    TurnConfig bad;
    TurnTarget target;
    float invalid[] = {-1.0f, NAN, INFINITY, -INFINITY};
    float bad_limits[] = {0.0f, -1.0f, NAN, INFINITY, -INFINITY};
    unsigned i;
    unsigned j;
    CHECK(Turn_Left(&config, 0.1f, 20.0f, NULL) == TURN_INVALID_ARGUMENT);
    CHECK(Turn_Right(&config, 0.1f, 20.0f, NULL) == TURN_INVALID_ARGUMENT);
    target.forward_mps = 9.0f;
    target.yaw_rate_radps = 9.0f;
    CHECK(Turn_Left(NULL, 0.1f, 20.0f, &target) == TURN_INVALID_ARGUMENT);
    CHECK(target.forward_mps == 0.0f && target.yaw_rate_radps == 0.0f);
    for (i = 0; i < sizeof(invalid) / sizeof(invalid[0]); ++i) {
        target.forward_mps = target.yaw_rate_radps = 9.0f;
        CHECK(Turn_Left(&config, invalid[i], 20.0f, &target) == TURN_INVALID_ARGUMENT);
        CHECK(target.forward_mps == 0.0f && target.yaw_rate_radps == 0.0f);
        target.forward_mps = target.yaw_rate_radps = 9.0f;
        CHECK(Turn_Right(&config, 0.1f, invalid[i], &target) == TURN_INVALID_ARGUMENT);
        CHECK(target.forward_mps == 0.0f && target.yaw_rate_radps == 0.0f);
    }
    CHECK(Turn_Left(&config, 0.1f, 100.01f, &target) == TURN_INVALID_ARGUMENT);
    CHECK(target.forward_mps == 0.0f && target.yaw_rate_radps == 0.0f);
    for (i = 0; i < sizeof(bad_limits) / sizeof(bad_limits[0]); ++i) {
        for (j = 0; j < 4; ++j) {
            bad = config;
            if (j == 0) bad.effective_track_m = bad_limits[i];
            if (j == 1) bad.max_forward_mps = bad_limits[i];
            if (j == 2) bad.max_yaw_rate_radps = bad_limits[i];
            if (j == 3) bad.max_wheel_mps = bad_limits[i];
            target.forward_mps = target.yaw_rate_radps = 9.0f;
            CHECK(Turn_Right(&bad, 0.1f, 20.0f, &target) == TURN_INVALID_ARGUMENT);
            CHECK(target.forward_mps == 0.0f && target.yaw_rate_radps == 0.0f);
        }
    }
}

static void sweep_invariants(void)
{
    TurnConfig config = example_config();
    TurnConfig before;
    TurnTarget left;
    TurnTarget right;
    TurnStatus ls;
    TurnStatus rs;
    double vl;
    double vr;
    double previous;
    int speed;
    int strength;
    int cap;
    for (cap = 1; cap <= 3; ++cap) {
        config.max_wheel_mps = (float)cap * 0.05f;
        before = config;
        for (speed = 0; speed <= 20; ++speed) {
            previous = -1.0;
            for (strength = 0; strength <= 100; ++strength) {
                ls = Turn_Left(&config, (float)speed / 100.0f, (float)strength, &left);
                rs = Turn_Right(&config, (float)speed / 100.0f, (float)strength, &right);
                CHECK(ls != TURN_INVALID_ARGUMENT && ls == rs);
                CHECK(memcmp(&before, &config, sizeof(config)) == 0);
                CHECK(near(left.forward_mps, right.forward_mps));
                CHECK(near(left.yaw_rate_radps, -right.yaw_rate_radps));
                CHECK(left.forward_mps >= 0.0f && left.forward_mps <= config.max_forward_mps);
                CHECK(left.yaw_rate_radps >= 0.0f && left.yaw_rate_radps <= config.max_yaw_rate_radps);
                CHECK(left.yaw_rate_radps + 0.000001 >= previous);
                previous = left.yaw_rate_radps;
                vl = left.forward_mps - left.yaw_rate_radps * (double)config.effective_track_m / 2.0;
                vr = left.forward_mps + left.yaw_rate_radps * (double)config.effective_track_m / 2.0;
                CHECK(vl >= -0.000001 && vr >= vl);
                CHECK(vr <= config.max_wheel_mps + 0.000001);
            }
        }
    }
}

static void finite_extremes(void)
{
    TurnConfig config = {FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX};
    TurnTarget target;
    CHECK(Turn_Left(&config, FLT_MAX, 100.0f, &target) != TURN_INVALID_ARGUMENT);
    CHECK(isfinite(target.forward_mps) && isfinite(target.yaw_rate_radps));
    config.effective_track_m = FLT_MIN;
    CHECK(Turn_Right(&config, FLT_MAX, 100.0f, &target) != TURN_INVALID_ARGUMENT);
    CHECK(isfinite(target.forward_mps) && isfinite(target.yaw_rate_radps));
}

int main(void)
{
    basics();
    wheel_limit();
    invalid_inputs();
    sweep_invariants();
    finite_extremes();
    printf("PASS: %u checks (actual turn_control.c, host only)\n", checks);
    return 0;
}
