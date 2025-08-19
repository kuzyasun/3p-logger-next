#include "alt_fuser.h"

#include <math.h>
#include <string.h>

#ifndef isnan
#define isnan(x) ((x) != (x))
#endif

static float clampf(float v, float lo, float hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static float lp_step(bool *init, float prev, float v, float tau, float dt) {
    if (tau <= 0.0f) return v;
    if (!*init) {
        *init = true;
        return v;
    }
    float a = 1.0f - expf(-dt / tau);
    return prev + a * (v - prev);
}

static bool gps_quality_ok(int sats, float hdop, int minSats, float maxHdop) {
    bool satsOk = (sats < 0) || (sats >= minSats);
    bool hdopOk = isnan(hdop) || (hdop <= maxHdop);
    return satsOk && hdopOk;
}

static bool gate_zero(int *zeroRun, int zeroDebounce, float v) {
    if (v == 0.0f) {
        (*zeroRun)++;
        return (*zeroRun) >= zeroDebounce;  // приймаємо 0 лише якщо тримається N тика
    }
    *zeroRun = 0;
    return true;
}

static bool gate_jump_rate(float last, float v, float dt, float maxJump, float maxRate) {
    float dz = v - last;
    if (fabsf(dz) > maxJump) return false;
    if (dt > 0.0f && fabsf(dz / dt) > (maxRate * 1.5f)) return false;
    return true;
}

void alt_fuser_reset(AltFuser *f) {
    AltFuserCfg cfg = f->cfg;  // зберегти конфіг
    memset(f, 0, sizeof(*f));
    f->cfg = cfg;
}

void alt_fuser_init(AltFuser *f, const AltFuserCfg *cfg) {
    const AltFuserCfg def = {
        .gpsMinSats = 6,
        .gpsMaxHdop = 2.5f,
        .zeroDebounce = 3,
        .maxJumpM = 60.0f,
        .maxRateMps = 15.0f,
        .holdMaxS = 2.0f,
        .wGps = 0.25f,
        .tauBaroS = 0.25f,
        .tauGpsS = 1.2f,
        .fusedAlpha = 0.35f,
        .fusedAlphaSingle = 0.45f,
        .freshTimeoutS = 1.0f,
        .adaptAlpha = 0.20f  // "швидкість" / сек: step = 1-exp(-dt*adaptAlpha)
    };
    f->cfg = cfg ? *cfg : def;
    alt_fuser_reset(f);
}

float alt_fuser_update(AltFuser *f, float now_s, bool gpsValid, float gpsAltM, int sats, float hdop, bool barValid, float barAltM, bool *usedGps,
                       bool *usedBaro) {
    if (usedGps) *usedGps = false;
    if (usedBaro) *usedBaro = false;

    // dt
    float dt;
    if (!f->hasT) {
        f->hasT = true;
        f->tPrev = now_s;
        dt = 0.02f;
    } else {
        dt = now_s - f->tPrev;
        f->tPrev = now_s;
    }
    dt = clampf(dt, 0.005f, 0.25f);

    // === GPS validate ===
    bool gpsOk = false;
    if (gpsValid && !isnan(gpsAltM)) {
        bool qOk = gps_quality_ok(sats, hdop, f->cfg.gpsMinSats, f->cfg.gpsMaxHdop);
        if (qOk) {
            bool zeroOk = gate_zero(&f->gpsZeroRun, f->cfg.zeroDebounce, gpsAltM);
            if (zeroOk) {
                if (f->hasGoodGps)
                    gpsOk = gate_jump_rate(f->lastGoodGps, gpsAltM, dt, f->cfg.maxJumpM, f->cfg.maxRateMps);
                else
                    gpsOk = true;
            }
        }
    }
    if (gpsOk) {
        f->gpsLP = lp_step(&f->gpsLPInit, f->gpsLP, gpsAltM, f->cfg.tauGpsS, dt);
        f->lastGoodGps = f->gpsLP;
        f->hasGoodGps = true;
        f->tLastGoodGps = now_s;
    } else if (f->hasGoodGps && (now_s - f->tLastGoodGps) < f->cfg.holdMaxS) {
        // коастимо GPS, якщо недавно був добрий
        gpsOk = true;
    }

    // === BARO validate ===
    bool barOk = false;
    if (barValid && !isnan(barAltM)) {
        bool zeroOk = gate_zero(&f->baroZeroRun, f->cfg.zeroDebounce, barAltM);
        if (zeroOk) {
            if (f->hasGoodBaro)
                barOk = gate_jump_rate(f->lastGoodBaro, barAltM, dt, f->cfg.maxJumpM, f->cfg.maxRateMps);
            else
                barOk = true;
        }
    }
    if (barOk) {
        f->baroLP = lp_step(&f->baroLPInit, f->baroLP, barAltM, f->cfg.tauBaroS, dt);
        f->lastGoodBaro = f->baroLP;
        f->hasGoodBaro = true;
        f->tLastGoodBaro = now_s;
    } else if (f->hasGoodBaro && (now_s - f->tLastGoodBaro) < f->cfg.holdMaxS) {
        barOk = true;
    }

    // === Автоадаптація ваги GPS ===
    // свіжість GPS незалежно від поточного gpsOk
    float freshGps = (f->hasGoodGps && ((now_s - f->tLastGoodGps) < f->cfg.freshTimeoutS)) ? 1.0f : 0.0f;

    // цільова "надійність" GPS
    float relTarget = gpsOk ? 1.0f : (freshGps > 0.5f ? 0.5f : 0.0f);

    // екпоненційне зближення до цілі: step = 1-exp(-dt*adaptAlpha)
    float a = 1.0f - expf(-dt * clampf(f->cfg.adaptAlpha, 0.0f, 10.0f));
    f->gpsReliability += a * (relTarget - f->gpsReliability);
    f->gpsReliability = clampf(f->gpsReliability, 0.0f, 1.0f);

    float wGpsEff = f->cfg.wGps * f->gpsReliability;

    // === Мікс ===
    float target;
    if (barOk && gpsOk) {
        target = (1.0f - wGpsEff) * f->lastGoodBaro + wGpsEff * f->lastGoodGps;
        if (usedBaro) *usedBaro = true;
        if (usedGps) *usedGps = (wGpsEff > 0.0f);
    } else if (barOk) {
        target = f->lastGoodBaro;
        if (usedBaro) *usedBaro = true;
    } else if (gpsOk) {
        target = f->lastGoodGps;
        if (usedGps) *usedGps = true;
    } else {
        target = f->fusedInit ? f->fused : 0.0f;  // все погано — тримаємо останнє
    }

    // === Вихідне згладжування ===
    float alpha = ((barOk ^ gpsOk) ? f->cfg.fusedAlphaSingle : f->cfg.fusedAlpha);
    alpha = clampf(alpha, 0.0f, 1.0f);
    if (!f->fusedInit) {
        f->fused = target;
        f->fusedInit = true;
    }
    f->fused = (1.0f - alpha) * f->fused + alpha * target;

    return f->fused;
}
