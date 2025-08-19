#pragma once
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    // Якість GPS для первинного гейту
    int gpsMinSats;    // мін. кількість супутників, щоб дозволити GPS
    float gpsMaxHdop;  // макс. HDOP, щоб дозволити GPS (NAN = ігнорувати)

    // Anti-glitch гейти джерел
    int zeroDebounce;  // приймати 0 лише після N послідовних тиках 0
    float maxJumpM;    // відсікати разовий стрибок > X м
    float maxRateMps;  // відсікати |dz/dt| > X м/с
    float holdMaxS;    // тримати останнє "добре" до X секунд, коли потік пропав

    // Базова вага GPS у міксі (коли GPS стабільний)
    float wGps;  // 0..1 (напр. 0.25 означає 25% GPS, 75% BARO)

    // LP-фільтри на джерелах
    float tauBaroS;  // τ LP для баро (сек)
    float tauGpsS;   // τ LP для GPS  (сек)

    // LP на виході (уникати ступінчастості при перемиканні джерел)
    float fusedAlpha;        // 0..1 — для випадку, коли обидва джерела валідні
    float fusedAlphaSingle;  // 0..1 — коли валідне лише одне джерело (можна швидше)

    // Автоадаптація ваги GPS
    float freshTimeoutS;  // якщо GPS давно не оновлювався (> це), вважаємо "несвіжим"
    float adaptAlpha;     // швидкість адаптації надійності GPS (Гц): step = 1-exp(-dt*adaptAlpha)
} AltFuserCfg;

typedef struct {
    AltFuserCfg cfg;

    // Вихідний стан
    bool fusedInit;
    float fused;

    // LP стани джерел
    bool gpsLPInit, baroLPInit;
    float gpsLP, baroLP;

    // Останні "добрі" значення та час їх появи
    bool hasGoodGps, hasGoodBaro;
    float lastGoodGps, lastGoodBaro;
    float tLastGoodGps, tLastGoodBaro;

    // Лічильники нульових послідовностей
    int gpsZeroRun, baroZeroRun;

    // Час
    bool hasT;
    float tPrev;

    // Автоадаптивна "надійність" GPS 0..1 (множник для cfg.wGps)
    float gpsReliability;
} AltFuser;

void alt_fuser_init(AltFuser *f, const AltFuserCfg *cfg);  // cfg=NULL → дефолти
void alt_fuser_reset(AltFuser *f);

/**
 * Оновлює ф’юзер та повертає злиту висоту (м).
 *
 * now_s     — монотонний час у секундах
 * gpsValid  — зовнішній gate: чи вважати GPS придатним у цей тик (свіжий/якісний)
 * gpsAltM   — GPS-висота (м), уже декодована/без офсету (може бути 0)
 * sats,hdop — показники якості GPS; якщо HDOP невідомий — передай NAN
 * barValid  — зовнішній gate для баро (наприклад, баро!=0 і свіже)
 * barAltM   — баро-висота (м)
 * usedGps/usedBaro — опційні вихідні прапорці: які джерела реально увійшли у мікс
 */
float alt_fuser_update(AltFuser *f, float now_s, bool gpsValid, float gpsAltM, int sats, float hdop, bool barValid, float barAltM, bool *usedGps,
                       bool *usedBaro);

#ifdef __cplusplus
}
#endif
