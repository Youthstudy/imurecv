/*
 * energy_switch.c
 *
 * Energy-consistent switching between Impedance and Admittance controllers.
 * Portable C suitable for STM32 with minimal changes (replace sensor/actuator stubs).
 *
 * Author: ChatGPT (GPT-5 Thinking mini)
 *
 * Key features:
 * - Compute total energy before switch and align target controller to match energy.
 * - Preserve velocity sign to avoid unnatural reversal.
 * - Minimum dwell time to prevent chatter.
 * - Small-energy handling (avoid sqrt of negative/small numbers).
 *
 * Usage:
 * - Replace SensorRead_Position/Velocity/ExternalForce and ActuatorSend_Force
 *   with platform-specific implementations (HAL ADC, encoder read, etc).
 * - Call ControlLoopStep periodically at fixed dt (e.g., from SysTick or timer ISR).
 */

#include <math.h>
#include <stdint.h>
#include <stdbool.h>

/* ---------- Tunable parameters ---------- */
#define DT                (0.001f)    /* control timestep [s] */
#define MIN_DWELL_TIME    (0.05f)     /* minimum time [s] to stay in a mode after switching */
#define ENERGY_EPS        (1e-6f)     /* small energy floor to avoid division by zero */
#define SMALL_ENERGY_TH   (1e-4f)     /* consider as near-zero energy */

/* ---------- Controller data structures ---------- */
typedef struct {
    float K;        /* stiffness */
    float D;        /* damping */
    float M;        /* virtual inertia (for energy calc) */
    float x_ref;    /* virtual equilibrium */
    float x;        /* measured/est position used by controller */
    float v;        /* velocity (measured) */
    float F_out;    /* output force/torque command */
} ImpedanceCtrl;

typedef struct {
    float K;        /* virtual stiffness */
    float B;        /* virtual damping */
    float M;        /* virtual inertia */
    float x;        /* virtual mass position state */
    float v;        /* virtual mass velocity state */
    float acc;      /* virtual mass acceleration state */
    float dt;       /* integration timestep */
    float F_out;    /* output force */
} AdmittanceCtrl;

/* ---------- Switching state ---------- */
typedef enum { MODE_IMP = 0, MODE_ADM = 1 } ControlMode;

typedef struct {
    ControlMode mode;
    float last_switch_time;   /* system time when last switched (s) */
} Switcher;

/* ---------- Timekeeping (user must update) ---------- */
static float system_time = 0.0f; /* seconds, increment DT every step */

/* ---------- Helper math ---------- */
static inline float kinetic_energy(float M, float v) {
    return 0.5f * M * v * v;
}
static inline float potential_energy(float K, float x, float x_ref) {
    float dx = x - x_ref;
    return 0.5f * K * dx * dx;
}
static inline float signf_safe(float x) {
    return (x >= 0.0f) ? 1.0f : -1.0f;
}

/* ---------- Controller update functions ---------- */

/* Impedance: F = K*(x_ref - x) - D*v
 * Additionally we simulate simple plant integration for demo (user should use real plant)
 */
float Impedance_Update(ImpedanceCtrl *imp) {
    imp->F_out = imp->K * (imp->x_ref - imp->x) - imp->D * imp->v;
    return imp->F_out;
}

/* Admittance: M*a = F_ext - B*v - K*x  (virtual mass)
 * integrate state, return command (e.g., desired position force)
 */
float Admittance_Update(AdmittanceCtrl *adm, float F_ext, float x_ref) {
    float dt = adm->dt;
    adm->acc = (F_ext - adm->B * adm->v - adm->K * (adm->x - x_ref)) / adm->M;
    adm->v += adm->acc * dt;
    adm->x += adm->v * dt;
    adm->F_out = adm->K * adm->x + adm->B * adm->v; /* mapping to actuator */
    return adm->F_out;
}

/* ---------- Energy compute / align functions ---------- */

float ComputeEnergyImpedance(const ImpedanceCtrl *imp) {
    float Ek = kinetic_energy(imp->M, imp->v);
    float Ep = potential_energy(imp->K, imp->x, imp->x_ref);
    return Ek + Ep;
}
float ComputeEnergyAdmittance(const AdmittanceCtrl *adm) {
    float Ek = kinetic_energy(adm->M, adm->v);
    /* For admittance we treat potential around zero reference (virtual spring center = 0) */
    float Ep = potential_energy(adm->K, adm->x, 0.0f);
    return Ek + Ep;
}

/* Align admittance state so its total energy equals E_target.
 * Keep adm->x if possible; adjust adm->v magnitude to match kinetic residual.
 * Preserve sign of velocity based on source_sign to keep direction.
 */
void AlignAdmittanceToEnergy(AdmittanceCtrl *adm, float E_target, float source_v_sign) {
    if (E_target <= SMALL_ENERGY_TH) {
        adm->v = 0.0f;
        adm->x = 0.0f;
        return;
    }

    float Ep = potential_energy(adm->K, adm->x, 0.0f);
    if (E_target > Ep + ENERGY_EPS) {
        float Ek_needed = E_target - Ep;
        if (Ek_needed < 0.0f) Ek_needed = 0.0f;
        adm->v = signf_safe(source_v_sign) * sqrtf( fmaxf(0.0f, 2.0f * Ek_needed / adm->M) );
    } else {
        /* Not enough total energy to keep current x: scale x down and zero velocity */
        adm->v = 0.0f;
        float currentEp = Ep;
        float scale = 0.0f;
        if (currentEp > ENERGY_EPS) scale = sqrtf(E_target / (currentEp + ENERGY_EPS));
        adm->x *= scale;
    }
}

/* Align impedance state to E_target; keep x_ref, adjust imp->v or imp->x accordingly.
 * Preserve source velocity sign to keep direction.
 */
void AlignImpedanceToEnergy(ImpedanceCtrl *imp, float E_target, float source_v_sign) {
    if (E_target <= SMALL_ENERGY_TH) {
        imp->v = 0.0f;
        imp->x = imp->x_ref; /* rest at equilibrium */
        return;
    }

    float Ep = potential_energy(imp->K, imp->x, imp->x_ref);
    if (E_target > Ep + ENERGY_EPS) {
        float Ek_needed = E_target - Ep;
        if (Ek_needed < 0.0f) Ek_needed = 0.0f;
        imp->v = signf_safe(source_v_sign) * sqrtf( fmaxf(0.0f, 2.0f * Ek_needed / imp->M) );
    } else {
        /* Reduce displacement toward ref (scale dx) */
        imp->v = 0.0f;
        float dx = imp->x - imp->x_ref;
        float Ep_current = Ep;
        float scale = 0.0f;
        if (Ep_current > ENERGY_EPS) scale = sqrtf(E_target / (Ep_current + ENERGY_EPS));
        imp->x = imp->x_ref + dx * scale;
    }
}

/* ---------- Energy-consistent switching routine ---------- */
void EnergyConsistentSwitch(Switcher *sw,
                            ImpedanceCtrl *imp,
                            AdmittanceCtrl *adm,
                            ControlMode new_mode) {
    if (sw->mode == new_mode) return; /* nothing to do */

    /* Enforce minimum dwell time before allowing another switch (debounce) */
    if ((system_time - sw->last_switch_time) < MIN_DWELL_TIME) {
        return;
    }

    /* compute energy from source */
    float E_before = 0.0f;
    float source_v_sign = 1.0f;
    if (sw->mode == MODE_IMP) {
        E_before = ComputeEnergyImpedance(imp);
        source_v_sign = signf_safe(imp->v);
    } else {
        E_before = ComputeEnergyAdmittance(adm);
        source_v_sign = signf_safe(adm->v);
    }

    /* small guard */
    if (E_before < 0.0f) E_before = 0.0f;

    /* align target controller to E_before */
    if (new_mode == MODE_IMP) {
        AlignImpedanceToEnergy(imp, E_before, source_v_sign);
    } else {
        AlignAdmittanceToEnergy(adm, E_before, source_v_sign);
    }

    /* commit switch */
    sw->mode = new_mode;
    sw->last_switch_time = system_time;
}

/* ---------- Simple anti-chatter logic helper ---------- */
bool CanSwitch(Switcher *sw) {
    return ((system_time - sw->last_switch_time) >= MIN_DWELL_TIME);
}

/* ---------- Hardware/Platform abstraction (STUBS) ----------
 * Replace these with actual implementations:
 * - SensorRead_Position(): read measured position (m or rad)
 * - SensorRead_Velocity(): read measured velocity
 * - SensorRead_ExternalForce(): read external force sensor (or estimate)
 * - ActuatorSend_Force(f): send force/torque command to motor driver
 */
float SensorRead_Position(void) {
    /* TODO: replace: read encoder/position sensor */
    /* For demo we return 0: position handled in local plant sim below */
    return 0.0f;
}
float SensorRead_Velocity(void) {
    /* TODO: replace */
    return 0.0f;
}
float SensorRead_ExternalForce(void) {
    /* TODO: replace with force sensor reading or estimator */
    return 0.0f;
}
void ActuatorSend_Force(float f_cmd) {
    /* TODO: replace: send to motor controller (e.g. voltage/current loop) */
    (void)f_cmd;
}

/* ---------- Example plant simulation (for desktop testing) ----------
 * You can comment out when running on real hardware. This sim integrates a
 * simple mass subject to commanded force so you can observe behavior when
 * running on PC. On STM32 use real sensors instead.
 */
static float plant_x = 0.0f;
static float plant_v = 0.0f;
static const float plant_M = 1.0f;
void PlantSim_Step(float f_cmd, float dt) {
    /* simple double integrator: M*a = f_cmd - b* v (b=0.1) */
    float b = 0.1f;
    float a = (f_cmd - b * plant_v) / plant_M;
    plant_v += a * dt;
    plant_x += plant_v * dt;
}
float PlantSim_ReadPosition(void) { return plant_x; }
float PlantSim_ReadVelocity(void) { return plant_v; }

/* ---------- Example control system and main loop ---------- */
int main(void) {
    /* Initialize controllers */
    ImpedanceCtrl imp = {
        .K = 150.0f,
        .D = 10.0f,
        .M = 1.0f,
        .x_ref = 0.05f, /* desired virtual equilibrium */
        .x = 0.0f,
        .v = 0.0f,
        .F_out = 0.0f
    };
    AdmittanceCtrl adm = {
        .K = 150.0f,
        .B = 10.0f,
        .M = 1.0f,
        .x = 0.0f,
        .v = 0.0f,
        .acc = 0.0f,
        .dt = DT,
        .F_out = 0.0f
    };
    Switcher sw = {
        .mode = MODE_IMP,
        .last_switch_time = -MIN_DWELL_TIME /* allow immediate switch if needed */
    };

    /* Example: start in impedance mode, then at t=1.0s switch to admittance */
    const float SWITCH_TIME = 1.0f;

    /* For demonstration: apply external force step at t=1.0 (simulate contact) */
    const float EXT_FORCE_STEP = 5.0f;

    /* main loop - simulation style */
    const int total_steps = (int)(2.0f / DT);
    for (int k = 0; k < total_steps; ++k) {
        /* advance system time */
        system_time += DT;

        /* read sensors (replace with real sensor reads on target) */
        float measured_pos = PlantSim_ReadPosition(); /* SensorRead_Position(); */
        float measured_vel = PlantSim_ReadVelocity(); /* SensorRead_Velocity(); */

        /* update controller internal measured states */
        imp.x = measured_pos;
        imp.v = measured_vel;
        /* adm.x and adm.v are internal virtual states, not overwritten here */

        /* Example switching condition:
         * - switch to admittance at SWITCH_TIME
         * - switch back to impedance at SWITCH_TIME + 0.7s
         * In real usage, condition could be force threshold or mode request.
         */
        if (system_time >= SWITCH_TIME && sw.mode == MODE_IMP && CanSwitch(&sw)) {
            /* before switching, compute energy and align admittance */
            EnergyConsistentSwitch(&sw, &imp, &adm, MODE_ADM);
        }
        if (system_time >= (SWITCH_TIME + 0.7f) && sw.mode == MODE_ADM && CanSwitch(&sw)) {
            EnergyConsistentSwitch(&sw, &imp, &adm, MODE_IMP);
        }

        /* external force applied to admittance (e.g., contact) */
        float F_ext = 0.0f;
        if (system_time >= SWITCH_TIME) F_ext = EXT_FORCE_STEP;

        /* control step */
        float cmd_force = 0.0f;
        if (sw.mode == MODE_IMP) {
            cmd_force = Impedance_Update(&imp);
        } else {
            cmd_force = Admittance_Update(&adm, F_ext, 0);
        }

        /* send to actuator (or plant sim) */
        /* ActuatorSend_Force(cmd_force); */
        PlantSim_Step(cmd_force, DT);

        /* Optional: logging or telemetry here (send imp.F_out, adm.F_out, plant_x) */
    }

    /* End */
    return 0;
}
