/*
 * ARM FP32 software baseline for the frozen dynamic-beta-v1 and RGRSF MARG
 * estimators.  This program is intentionally independent of the RTL: it
 * uses IEEE-754 single precision and standard square-root normalization.
 *
 * Target runtime: Zynq-7000 Cortex-A9 Linux (hard-float, NEON-capable).
 * Host execution is permitted for functional validation only; its timing is
 * not an ARM measurement.
 */
#ifndef _WIN32
#define _POSIX_C_SOURCE 200809L
#endif

#include <errno.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#include <windows.h>
#else
#include <time.h>
#endif

#define EPSILON 1.0e-8f
#define DEFAULT_BETA0 0.0075f
#define DEFAULT_REPEATS 200

typedef struct { float x, y, z; } vec3_t;
typedef struct { float w, x, y, z; } quat_t;

typedef struct {
    vec3_t accel;          /* Already mapped to the SAAM core convention. */
    vec3_t mag;
    vec3_t gyro_rad_s;
    quat_t q_truth_mapped; /* [qx_true, -qw_true, qz_true, -qy_true]. */
    float dt_s;
    uint8_t disturbance_flags;
} sample_t;

typedef enum { METHOD_DYNAMIC, METHOD_RGRSF } method_t;
typedef enum { MODE_MARG = 0, MODE_IMU_TILT = 1, MODE_GYRO_ONLY = 2 } rgrsf_mode_t;

typedef struct {
    float beta0;
    float acc_normal_low, acc_normal_high, acc_hard_low, acc_hard_high;
    float mag_normal_low, mag_normal_high, mag_hard_low, mag_hard_high;
    float mn2_normal, mn2_reject, qdot_normal, qdot_reject;
    float acc_dir_normal, acc_dir_reject;
    float mag_dir_normal_cos2, mag_dir_reject_cos2;
    int bad_frame_count, good_frame_count;
} parameters_t;

typedef struct {
    quat_t q;
    int have_state;
    int beta_level; /* 0 reject, 1 quarter, 2 half, 3 nominal. */
    int beta_bad_count, beta_good_count;
    int mode;
    int mode_bad_count, mode_good_count;
} estimator_t;

typedef struct {
    int available;
    quat_t q_saam;
    vec3_t a_unit, m_unit;
    float mn2;
} observation_t;

typedef struct {
    double sum_sq_all, sum_sq_clean, sum_sq_disturbed;
    float max_all, max_clean, max_disturbed;
    size_t count_all, count_clean, count_disturbed;
    unsigned long beta_level_count[4];
    unsigned long mode_count[3];
    float *errors_deg;
} evaluation_t;

typedef struct {
    double elapsed_s;
    double us_per_frame;
    double frames_per_s;
} benchmark_t;

static const parameters_t FROZEN = {
    DEFAULT_BETA0,
    0.90f, 1.10f, 0.75f, 1.25f,
    0.80f, 1.20f, 0.55f, 1.45f,
    0.20f, 0.05f, 0.995f, 0.980f,
    0.990f, 0.940f,
    0.9698463103929541f, 0.8213938048432696f,
    1, 5
};

static float clampf(float x, float lo, float hi) {
    return x < lo ? lo : (x > hi ? hi : x);
}

static float vec_dot(vec3_t a, vec3_t b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
static float vec_norm2(vec3_t a) { return vec_dot(a, a); }

static int vec_normalize(vec3_t in, vec3_t *out) {
    const float n2 = vec_norm2(in);
    if (!(n2 > EPSILON) || !isfinite(n2)) return 0;
    const float inv = 1.0f / sqrtf(n2);
    out->x = in.x * inv; out->y = in.y * inv; out->z = in.z * inv;
    return 1;
}

static float quat_dot(quat_t a, quat_t b) { return a.w*b.w + a.x*b.x + a.y*b.y + a.z*b.z; }

static int quat_normalize(quat_t in, quat_t *out) {
    const float n2 = quat_dot(in, in);
    if (!(n2 > EPSILON) || !isfinite(n2)) return 0;
    const float inv = 1.0f / sqrtf(n2);
    out->w = in.w * inv; out->x = in.x * inv; out->y = in.y * inv; out->z = in.z * inv;
    return 1;
}

static quat_t quat_mul(quat_t a, quat_t b) {
    quat_t r = {
        a.w*b.w - a.x*b.x - a.y*b.y - a.z*b.z,
        a.w*b.x + a.x*b.w + a.y*b.z - a.z*b.y,
        a.w*b.y - a.x*b.z + a.y*b.w + a.z*b.x,
        a.w*b.z + a.x*b.y - a.y*b.x + a.z*b.w
    };
    return r;
}

static quat_t sign_align(quat_t reference, quat_t observation) {
    if (quat_dot(reference, observation) < 0.0f) {
        observation.w = -observation.w; observation.x = -observation.x;
        observation.y = -observation.y; observation.z = -observation.z;
    }
    return observation;
}

static quat_t fuse(quat_t qgyro, quat_t qref, float beta) {
    quat_t mix = {
        (1.0f-beta)*qgyro.w + beta*qref.w,
        (1.0f-beta)*qgyro.x + beta*qref.x,
        (1.0f-beta)*qgyro.y + beta*qref.y,
        (1.0f-beta)*qgyro.z + beta*qref.z
    };
    quat_t normalized;
    return quat_normalize(mix, &normalized) ? normalized : qgyro;
}

static quat_t gyro_propagate(quat_t q, vec3_t gyro, float dt) {
    const float h = 0.5f * dt;
    quat_t pre = {
        q.w - h*(gyro.x*q.x + gyro.y*q.y + gyro.z*q.z),
        q.x + h*(gyro.x*q.w + gyro.z*q.y - gyro.y*q.z),
        q.y + h*(gyro.y*q.w - gyro.z*q.x + gyro.x*q.z),
        q.z + h*(gyro.z*q.w + gyro.y*q.x - gyro.x*q.y)
    };
    quat_t normalized;
    return quat_normalize(pre, &normalized) ? normalized : q;
}

static observation_t saam_observation(vec3_t a_raw, vec3_t m_raw) {
    observation_t obs;
    memset(&obs, 0, sizeof(obs));
    if (!vec_normalize(a_raw, &obs.a_unit) || !vec_normalize(m_raw, &obs.m_unit)) return obs;

    const vec3_t a = obs.a_unit, m = obs.m_unit;
    const float md = clampf(vec_dot(a, m), -1.0f, 1.0f);
    obs.mn2 = fmaxf(0.0f, 1.0f - md*md);
    if (!(obs.mn2 > EPSILON)) return obs;
    const float mn = sqrtf(obs.mn2);
    const quat_t qtilde = {
        (a.z - 1.0f)*(mn + m.x) + a.x*(md - m.z),
        (a.z - 1.0f)*m.y + a.y*(md - m.z),
        a.z*md - a.x*mn - m.z,
        a.x*m.y - a.y*(mn + m.x)
    };
    /* Equation (6) is followed by the paper-mandated [q3,q0,q1,q2] reorder. */
    const quat_t q_reordered = {qtilde.z, qtilde.w, qtilde.x, qtilde.y};
    obs.available = quat_normalize(q_reordered, &obs.q_saam);
    return obs;
}

static void estimator_reset(estimator_t *state) {
    memset(state, 0, sizeof(*state));
    state->q.w = 1.0f;
    state->beta_level = 3;
    state->mode = MODE_MARG;
}

static float beta_for_level(float beta0, int level) {
    if (level >= 3) return beta0;
    if (level == 2) return 0.5f*beta0;
    if (level == 1) return 0.25f*beta0;
    return 0.0f;
}

static int dynamic_candidate(const parameters_t *p, vec3_t a_raw, vec3_t m_raw,
                             float mn2, float qdot_abs) {
    int soft = 0, hard = 0;
    const float an2 = vec_norm2(a_raw), mn_raw2 = vec_norm2(m_raw);
    if (an2 < p->acc_normal_low || an2 > p->acc_normal_high) {
        if (an2 < p->acc_hard_low || an2 > p->acc_hard_high) hard = 1; else ++soft;
    }
    if (mn_raw2 < p->mag_normal_low || mn_raw2 > p->mag_normal_high) {
        if (mn_raw2 < p->mag_hard_low || mn_raw2 > p->mag_hard_high) hard = 1; else ++soft;
    }
    if (mn2 < p->mn2_normal) {
        if (mn2 <= p->mn2_reject) hard = 1; else ++soft;
    }
    if (qdot_abs < p->qdot_normal) {
        if (qdot_abs < p->qdot_reject) hard = 1; else ++soft;
    }
    if (hard) return 0;
    return soft == 0 ? 3 : (soft == 1 ? 2 : 1);
}

static void update_beta_hysteresis(estimator_t *s, const parameters_t *p, int candidate) {
    if (candidate < s->beta_level) {
        s->beta_good_count = 0;
        ++s->beta_bad_count;
        if (s->beta_bad_count >= p->bad_frame_count) {
            s->beta_level = candidate;
            s->beta_bad_count = 0;
        }
    } else if (candidate == 3 && s->beta_level < 3) {
        s->beta_bad_count = 0;
        ++s->beta_good_count;
        if (s->beta_good_count >= p->good_frame_count) {
            ++s->beta_level;
            s->beta_good_count = 0;
        }
    } else {
        s->beta_bad_count = 0;
        s->beta_good_count = 0;
    }
}

static vec3_t predicted_gravity(quat_t q) {
    vec3_t g = {
        2.0f*(q.x*q.z - q.w*q.y),
        2.0f*(q.y*q.z + q.w*q.x),
        1.0f - 2.0f*(q.x*q.x + q.y*q.y)
    };
    return g;
}

static void horizontal_heading(quat_t q, float *hx, float *hy) {
    *hx = 1.0f - 2.0f*(q.y*q.y + q.z*q.z);
    *hy = 2.0f*(q.w*q.z + q.x*q.y);
}

static int qacc_reference(quat_t qgyro, vec3_t a_unit, quat_t *out) {
    float hx, hy;
    horizontal_heading(qgyro, &hx, &hy);
    const float ct2 = a_unit.y*a_unit.y + a_unit.z*a_unit.z;
    const float rho2 = hx*hx + hy*hy;
    if (!(ct2 > 0.001f) || !(rho2 > 0.001f)) return 0;
    const float ct = sqrtf(ct2), rho = sqrtf(rho2);
    quat_t qyaw = {rho + hx, 0.0f, 0.0f, hy};
    quat_t qpitch = {1.0f + ct, 0.0f, -a_unit.x, 0.0f};
    quat_t qroll = {ct + a_unit.z, a_unit.y, 0.0f, 0.0f};
    quat_t n0, n1, n2;
    if (!quat_normalize(qyaw, &n0) || !quat_normalize(qpitch, &n1) || !quat_normalize(qroll, &n2)) return 0;
    return quat_normalize(quat_mul(quat_mul(n0, n1), n2), out);
}

static quat_t dynamic_step(estimator_t *s, const parameters_t *p, const sample_t *sample,
                           int *beta_level, int *mode) {
    observation_t obs = saam_observation(sample->accel, sample->mag);
    quat_t qgyro;
    if (!s->have_state) {
        if (!obs.available) { *beta_level = s->beta_level; *mode = MODE_GYRO_ONLY; return s->q; }
        /* Match the RTL: the first valid SAAM quaternion is an absolute-state seed. */
        s->q = obs.q_saam; s->have_state = 1;
        *beta_level = s->beta_level; *mode = MODE_MARG; return s->q;
    } else {
        qgyro = gyro_propagate(s->q, sample->gyro_rad_s, sample->dt_s);
    }
    if (!obs.available) {
        s->q = qgyro; s->have_state = 1; s->beta_level = 0;
        *beta_level = 0; *mode = MODE_GYRO_ONLY; return s->q;
    }
    quat_t qref = sign_align(qgyro, obs.q_saam);
    const float qdot = fabsf(quat_dot(qgyro, qref));
    const int candidate = dynamic_candidate(p, sample->accel, sample->mag, obs.mn2, qdot);
    update_beta_hysteresis(s, p, candidate);
    s->q = fuse(qgyro, qref, beta_for_level(p->beta0, s->beta_level));
    s->have_state = 1;
    *beta_level = s->beta_level; *mode = MODE_MARG;
    return s->q;
}

static void update_mode_hysteresis(estimator_t *s, const parameters_t *p, int desired) {
    if (desired > s->mode) {
        s->mode_good_count = 0;
        ++s->mode_bad_count;
        if (s->mode_bad_count >= p->bad_frame_count) {
            s->mode = desired;
            s->mode_bad_count = 0;
        }
    } else if (desired < s->mode) {
        s->mode_bad_count = 0;
        ++s->mode_good_count;
        if (s->mode_good_count >= p->good_frame_count) {
            --s->mode; /* gradual recovery: gyro -> IMU -> MARG */
            s->mode_good_count = 0;
        }
    } else {
        s->mode_bad_count = 0;
        s->mode_good_count = 0;
    }
}

static quat_t rgrsf_step(estimator_t *s, const parameters_t *p, const sample_t *sample,
                         int *beta_level, int *mode) {
    observation_t obs = saam_observation(sample->accel, sample->mag);
    quat_t qgyro;
    if (!s->have_state) {
        if (!obs.available) { *beta_level = 0; *mode = MODE_GYRO_ONLY; return s->q; }
        /* Match the RTL: initialize from the first valid absolute MARG observation. */
        s->q = obs.q_saam; s->have_state = 1; s->mode = MODE_MARG;
        *beta_level = s->beta_level; *mode = s->mode; return s->q;
    } else {
        qgyro = gyro_propagate(s->q, sample->gyro_rad_s, sample->dt_s);
    }
    if (!obs.available) {
        s->q = qgyro; s->have_state = 1; s->beta_level = 0; s->mode = MODE_GYRO_ONLY;
        *beta_level = 0; *mode = s->mode; return s->q;
    }

    quat_t qsaam = sign_align(qgyro, obs.q_saam);
    const float qdot = fabsf(quat_dot(qgyro, qsaam));
    int candidate = dynamic_candidate(p, sample->accel, sample->mag, obs.mn2, qdot);
    const float accel_dir = clampf(vec_dot(obs.a_unit, predicted_gravity(qgyro)), -1.0f, 1.0f);
    float gx, gy, sx, sy;
    horizontal_heading(qgyro, &gx, &gy);
    horizontal_heading(qsaam, &sx, &sy);
    const float cross = gx*sx + gy*sy;
    const float heading_product = (gx*gx + gy*gy)*(sx*sx + sy*sy);
    const float heading_cos2 = heading_product > EPSILON ? (cross*cross)/heading_product : 0.0f;
    const float accel_n2 = vec_norm2(sample->accel), mag_n2 = vec_norm2(sample->mag);

    int desired = MODE_MARG;
    if (accel_n2 < p->acc_hard_low || accel_n2 > p->acc_hard_high || accel_dir < p->acc_dir_reject) {
        desired = MODE_GYRO_ONLY;
    } else if (mag_n2 < p->mag_hard_low || mag_n2 > p->mag_hard_high || obs.mn2 <= p->mn2_reject ||
               cross <= 0.0f || heading_cos2 < p->mag_dir_reject_cos2) {
        desired = MODE_IMU_TILT;
    }
    if (accel_dir < p->acc_dir_normal && candidate > 1) candidate = 2;
    if (cross <= 0.0f || heading_cos2 < p->mag_dir_normal_cos2) {
        if (candidate > 1) candidate = 2;
    }
    update_mode_hysteresis(s, p, desired);

    if (s->mode == MODE_MARG) {
        update_beta_hysteresis(s, p, candidate);
        s->q = fuse(qgyro, qsaam, beta_for_level(p->beta0, s->beta_level));
    } else if (s->mode == MODE_IMU_TILT) {
        quat_t qacc;
        s->beta_level = 0; s->beta_bad_count = s->beta_good_count = 0;
        if (qacc_reference(qgyro, obs.a_unit, &qacc)) {
            qacc = sign_align(qgyro, qacc);
            s->q = fuse(qgyro, qacc, 0.25f*p->beta0);
        } else {
            s->q = qgyro;
            s->mode = MODE_GYRO_ONLY;
        }
    } else {
        s->beta_level = 0; s->beta_bad_count = s->beta_good_count = 0;
        s->q = qgyro;
    }
    s->have_state = 1;
    *beta_level = s->beta_level; *mode = s->mode;
    return s->q;
}

static quat_t estimator_step(estimator_t *s, const parameters_t *p, method_t method,
                             const sample_t *sample, int *beta_level, int *mode) {
    return method == METHOD_DYNAMIC ? dynamic_step(s, p, sample, beta_level, mode)
                                     : rgrsf_step(s, p, sample, beta_level, mode);
}

static float geodesic_deg(quat_t q, quat_t truth) {
    q = sign_align(truth, q);
    const float d = clampf(fabsf(quat_dot(q, truth)), 0.0f, 1.0f);
    return 114.59155902616465f * acosf(d); /* 2 * 180/pi */
}

static int cmp_float(const void *a, const void *b) {
    const float da = *(const float *)a, db = *(const float *)b;
    return da < db ? -1 : (da > db ? 1 : 0);
}

static float percentile95(const evaluation_t *e, const sample_t *samples, size_t n, int disturbed) {
    size_t count = 0, i;
    for (i = 0; i < n; ++i) if (((samples[i].disturbance_flags != 0) ? 1 : 0) == disturbed) ++count;
    if (count == 0) return NAN;
    float *tmp = (float *)malloc(count * sizeof(*tmp));
    if (!tmp) return NAN;
    size_t j = 0;
    for (i = 0; i < n; ++i) if (((samples[i].disturbance_flags != 0) ? 1 : 0) == disturbed) tmp[j++] = e->errors_deg[i];
    qsort(tmp, count, sizeof(*tmp), cmp_float);
    const size_t index = (size_t)ceilf(0.95f*(float)count) - 1U;
    const float answer = tmp[index < count ? index : count-1U];
    free(tmp);
    return answer;
}

static void evaluate(method_t method, const parameters_t *p, const sample_t *samples, size_t n, evaluation_t *e) {
    memset(e, 0, sizeof(*e));
    e->errors_deg = (float *)calloc(n, sizeof(*e->errors_deg));
    if (!e->errors_deg) { fprintf(stderr, "Out of memory for errors.\n"); exit(2); }
    estimator_t state; estimator_reset(&state);
    for (size_t i = 0; i < n; ++i) {
        int level = 0, mode = 0;
        quat_t q = estimator_step(&state, p, method, &samples[i], &level, &mode);
        const float error = geodesic_deg(q, samples[i].q_truth_mapped);
        const int disturbed = samples[i].disturbance_flags != 0;
        e->errors_deg[i] = error; e->sum_sq_all += (double)error*error; ++e->count_all;
        if (error > e->max_all) e->max_all = error;
        if (disturbed) {
            e->sum_sq_disturbed += (double)error*error; ++e->count_disturbed;
            if (error > e->max_disturbed) e->max_disturbed = error;
        } else {
            e->sum_sq_clean += (double)error*error; ++e->count_clean;
            if (error > e->max_clean) e->max_clean = error;
        }
        if (level >= 0 && level <= 3) ++e->beta_level_count[level];
        if (mode >= MODE_MARG && mode <= MODE_GYRO_ONLY) ++e->mode_count[mode];
    }
}

static double now_seconds(void) {
#ifdef _WIN32
    LARGE_INTEGER freq, counter;
    QueryPerformanceFrequency(&freq); QueryPerformanceCounter(&counter);
    return (double)counter.QuadPart/(double)freq.QuadPart;
#else
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (double)ts.tv_sec + 1.0e-9*(double)ts.tv_nsec;
#endif
}

static benchmark_t benchmark(method_t method, const parameters_t *p, const sample_t *samples,
                             size_t n, int repeats) {
    volatile float sink = 0.0f;
    const double begin = now_seconds();
    for (int r = 0; r < repeats; ++r) {
        estimator_t state; estimator_reset(&state);
        for (size_t i = 0; i < n; ++i) {
            int level, mode;
            quat_t q = estimator_step(&state, p, method, &samples[i], &level, &mode);
            sink += q.w * 1.0e-9f;
        }
    }
    const double elapsed = now_seconds() - begin;
    benchmark_t result = {elapsed, 1.0e6*elapsed/((double)n*repeats), ((double)n*repeats)/elapsed};
    if (sink == -1.0f) fprintf(stderr, "unreachable benchmark sink\n");
    return result;
}

static int split_csv(char *line, char **fields, int maximum) {
    int count = 0;
    char *cursor = line;
    while (cursor && count < maximum) {
        fields[count++] = cursor;
        char *comma = strchr(cursor, ',');
        if (!comma) break;
        *comma = '\0'; cursor = comma + 1;
    }
    return count;
}

static int load_dataset(const char *path, sample_t **out_samples, size_t *out_count) {
    FILE *stream = fopen(path, "rb");
    if (!stream) { fprintf(stderr, "Cannot open %s: %s\n", path, strerror(errno)); return 0; }
    char line[8192];
    if (!fgets(line, sizeof(line), stream)) { fclose(stream); return 0; } /* CSV header */
    size_t capacity = 20000, count = 0;
    sample_t *samples = (sample_t *)malloc(capacity*sizeof(*samples));
    if (!samples) { fclose(stream); return 0; }
    while (fgets(line, sizeof(line), stream)) {
        char *f[48];
        const int columns = split_csv(line, f, 48);
        if (columns < 41) { fprintf(stderr, "Malformed CSV row %lu (%d columns).\n", (unsigned long)(count+2), columns); free(samples); fclose(stream); return 0; }
        if (count == capacity) {
            capacity *= 2;
            sample_t *next = (sample_t *)realloc(samples, capacity*sizeof(*samples));
            if (!next) { free(samples); fclose(stream); return 0; }
            samples = next;
        }
        sample_t *s = &samples[count++];
        /* Coordinate adapter must match the existing SAAM RTL preparation. */
        s->accel.x = -strtof(f[20], NULL); s->accel.y = -strtof(f[21], NULL); s->accel.z = -strtof(f[22], NULL);
        s->gyro_rad_s.x = strtof(f[23], NULL); s->gyro_rad_s.y = strtof(f[24], NULL); s->gyro_rad_s.z = strtof(f[25], NULL);
        s->mag.x = strtof(f[26], NULL); s->mag.y = strtof(f[27], NULL); s->mag.z = strtof(f[28], NULL);
        const float qw = strtof(f[7], NULL), qx = strtof(f[8], NULL), qy = strtof(f[9], NULL), qz = strtof(f[10], NULL);
        s->q_truth_mapped.w = qx; s->q_truth_mapped.x = -qw; s->q_truth_mapped.y = qz; s->q_truth_mapped.z = -qy;
        s->dt_s = 0.001f;
        s->disturbance_flags = (uint8_t)((atoi(f[38]) ? 1 : 0) | (atoi(f[39]) ? 2 : 0) | (atoi(f[40]) ? 4 : 0));
    }
    fclose(stream); *out_samples = samples; *out_count = count; return count > 0;
}

static const char *method_name(method_t method) { return method == METHOD_DYNAMIC ? "dynamic_beta_v1_fp32" : "rgrsf_fp32"; }

static void write_json_string(FILE *out, const char *text) {
    fputc('"', out);
    for (const unsigned char *p = (const unsigned char *)text; *p; ++p) {
        if (*p == '"' || *p == '\\') { fputc('\\', out); fputc(*p, out); }
        else if (*p == '\n') fputs("\\n", out);
        else if (*p == '\r') fputs("\\r", out);
        else if (*p == '\t') fputs("\\t", out);
        else if (*p < 0x20) fprintf(out, "\\u%04x", (unsigned)*p);
        else fputc(*p, out);
    }
    fputc('"', out);
}

static void write_json_number(FILE *out, double value) {
    if (isfinite(value)) fprintf(out, "%.8f", value);
    else fputs("null", out);
}

static void write_method_json(FILE *out, method_t method, const evaluation_t *e, const sample_t *samples,
                              size_t n, int repeats, benchmark_t b, int comma) {
    const double all_rmse = sqrt(e->sum_sq_all/(double)e->count_all);
    const double clean_rmse = e->count_clean ? sqrt(e->sum_sq_clean/(double)e->count_clean) : NAN;
    const double disturbed_rmse = e->count_disturbed ? sqrt(e->sum_sq_disturbed/(double)e->count_disturbed) : NAN;
    const float disturbed_p95 = e->count_disturbed ? percentile95(e, samples, n, 1) : NAN;
    fprintf(out,
        "    {\n"
        "      \"method\": \"%s\",\n"
        "      \"frames\": %lu,\n"
        "      \"accuracy_deg\": {\"rmse\": ", method_name(method), (unsigned long)n);
    write_json_number(out, all_rmse); fprintf(out, ", \"max\": "); write_json_number(out, e->max_all);
    fprintf(out, ", \"clean_rmse\": "); write_json_number(out, clean_rmse); fprintf(out, ", \"clean_max\": ");
    write_json_number(out, e->count_clean ? e->max_clean : NAN); fprintf(out, ", \"disturbed_rmse\": ");
    write_json_number(out, disturbed_rmse); fprintf(out, ", \"disturbed_p95\": "); write_json_number(out, disturbed_p95);
    fprintf(out, ", \"disturbed_max\": "); write_json_number(out, e->count_disturbed ? e->max_disturbed : NAN);
    fprintf(out,
        "},\n"
        "      \"gating\": {\"beta_level_0\": %lu, \"beta_level_1\": %lu, \"beta_level_2\": %lu, \"beta_level_3\": %lu, \"mode_marg\": %lu, \"mode_imu_tilt\": %lu, \"mode_gyro_only\": %lu},\n"
        "      \"compute_benchmark\": {\"repeats\": %d, \"elapsed_seconds\": %.9f, \"microseconds_per_frame\": %.9f, \"frames_per_second\": %.3f}\n"
        "    }%s\n",
        e->beta_level_count[0], e->beta_level_count[1], e->beta_level_count[2], e->beta_level_count[3],
        e->mode_count[0], e->mode_count[1], e->mode_count[2], repeats, b.elapsed_s, b.us_per_frame, b.frames_per_s,
        comma ? "," : "");
}

static void usage(const char *program) {
    fprintf(stderr, "Usage: %s --input DATA.csv [--method dynamic|rgrsf|both] [--repeats N] [--output RESULT.json]\n", program);
}

int main(int argc, char **argv) {
    const char *input = NULL, *output = NULL, *method_arg = "both";
    int repeats = DEFAULT_REPEATS;
    for (int i = 1; i < argc; ++i) {
        if (!strcmp(argv[i], "--input") && i+1 < argc) input = argv[++i];
        else if (!strcmp(argv[i], "--output") && i+1 < argc) output = argv[++i];
        else if (!strcmp(argv[i], "--method") && i+1 < argc) method_arg = argv[++i];
        else if (!strcmp(argv[i], "--repeats") && i+1 < argc) repeats = atoi(argv[++i]);
        else { usage(argv[0]); return 2; }
    }
    if (!input || repeats < 1 || (strcmp(method_arg, "dynamic") && strcmp(method_arg, "rgrsf") && strcmp(method_arg, "both"))) {
        usage(argv[0]); return 2;
    }
    sample_t *samples = NULL; size_t count = 0;
    if (!load_dataset(input, &samples, &count)) return 2;
    const int do_dynamic = !strcmp(method_arg, "dynamic") || !strcmp(method_arg, "both");
    const int do_rgrsf = !strcmp(method_arg, "rgrsf") || !strcmp(method_arg, "both");
    evaluation_t dynamic_eval, rgrsf_eval;
    benchmark_t dynamic_bench, rgrsf_bench;
    if (do_dynamic) { evaluate(METHOD_DYNAMIC, &FROZEN, samples, count, &dynamic_eval); dynamic_bench = benchmark(METHOD_DYNAMIC, &FROZEN, samples, count, repeats); }
    if (do_rgrsf) { evaluate(METHOD_RGRSF, &FROZEN, samples, count, &rgrsf_eval); rgrsf_bench = benchmark(METHOD_RGRSF, &FROZEN, samples, count, repeats); }
    FILE *out = output ? fopen(output, "wb") : stdout;
    if (!out) { fprintf(stderr, "Cannot write %s: %s\n", output, strerror(errno)); free(samples); return 2; }
    fprintf(out,
        "{\n"
        "  \"artifact\": \"ARM FP32 software baseline\",\n"
        "  \"evidence_boundary\": \"Timing is valid only on the machine that executed this binary. A host run is not a Cortex-A9 measurement.\",\n"
        "  \"input\": ");
    write_json_string(out, input);
    fprintf(out,
        ",\n"
        "  \"parameters\": {\"beta0\": %.7f, \"bad_frame_count\": %d, \"good_frame_count\": %d},\n"
        "  \"methods\": [\n", FROZEN.beta0, FROZEN.bad_frame_count, FROZEN.good_frame_count);
    if (do_dynamic) write_method_json(out, METHOD_DYNAMIC, &dynamic_eval, samples, count, repeats, dynamic_bench, do_rgrsf);
    if (do_rgrsf) write_method_json(out, METHOD_RGRSF, &rgrsf_eval, samples, count, repeats, rgrsf_bench, 0);
    fprintf(out, "  ]\n}\n");
    if (output) fclose(out);
    if (do_dynamic) free(dynamic_eval.errors_deg);
    if (do_rgrsf) free(rgrsf_eval.errors_deg);
    free(samples);
    return 0;
}
