import config
import io
import numpy as np
import matplotlib.pyplot as plt
import datetime as date

PRINT_TO_CONSOLE: bool = True


def main() -> None:
    out: str = f"\nAnaliza silnika krokowego {date.datetime.now()}:\n"

    # --- Analiza momentu odcięcia w funkcji prędkości ---

    step_angle = 360.0 / config.HS174401.steps_per_revolution
    rotor_teeth_count = 90 / step_angle

    # Peak torque at maximum load with two phases excited
    # because the holding torque is defined as the maximum torque at rated current with two phases energized
    peak_torque = config.HS174401.holding_torque

    # Average torque over one step at maximum load
    low_speed_peak_torque = 2*np.sqrt(2)*peak_torque/np.pi

    R_phase_total = config.HS174401.phase_resistance + config.A4988.on_resistance
    L_phase = config.HS174401.phase_inductance
    V_s_required = R_phase_total * config.HS174401.rated_current
    V_s = config.GENERIC_SUPPLY.supply_voltage
    V_s_margin = V_s - V_s_required
    V_fundamental = V_s * 4 / np.pi

    out += f"   Margines napięcia zasilania: {V_s_margin:.2f} V\n"

    # Solved at minimum speed where the back-EMF is negligible, so the supply voltage is mostly dropped across the phase resistance
    psi_m_peak = low_speed_peak_torque * \
        R_phase_total / (rotor_teeth_count * V_fundamental)

    # Pull-out torque curve calculation
    # Torque in steppers is dependent only on the back-EMF and the supply current through the phases
    step_per_second = np.linspace(0, 1000, 100)
    pull_out_torque: np.ndarray = np.zeros_like(step_per_second)
    for i, step in enumerate(step_per_second):
        omega_e = (np.pi * step) / 2  # Electrical angular velocity (rad/s)

        impedance_sq = R_phase_total**2 + \
            (omega_e * L_phase)**2

        term1 = (rotor_teeth_count * psi_m_peak *
                 V_fundamental) / np.sqrt(impedance_sq)
        term2 = (rotor_teeth_count * omega_e * (psi_m_peak**2)
                 * R_phase_total) / impedance_sq

        pull_out_torque[i] = term1 - term2

    # Plotting the pull-out torque curve
    plt.figure(figsize=(10, 6))
    plt.plot(step_per_second, pull_out_torque *
             1000, label="Moment odcięcia (mNm)")

    plt.plot(0, low_speed_peak_torque*1000, 'ro',
             label="Moment odcięcia przy niskiej prędkości")

    plt.plot(0, peak_torque*1000, 'go', label="Moment trzymania")

    plt.title("Moment odcięcia silnika krokowego HS174401")
    plt.xlabel("Prędkość (kroki/s)")
    plt.ylabel("Moment odcięcia (mNm)")
    plt.grid()
    plt.legend()
    plt.savefig(
        f"pull_out_torque_curve_{date.datetime.now().strftime('%Y%m%d_%H%M%S')}.png")

    # --- Inertia analysis for the turntable and barrel loads ---

    # Moment of inertia for the turntable load alone (modeled as a solid cylinder)
    # Almost certainly overspec but that's fine
    m_a = 2.0  # Mass of the turntable load (kg)
    r_a = 0.2  # Radius of the turntable load (m)
    I_load_a = 0.5 * m_a * r_a**2

    # Moment of inertia for the barrel load alone (modeled as a rod with small offset)
    m_b = 1  # Mass of the barrel load (kg)
    l_b = 0.4  # Length of the barrel load (m)
    # Distance from the center of mass of the barrel load to the axis of rotation (m)
    d_b = 0.05
    I_load_b = (1/12) * m_b * l_b**2 + m_b * d_b**2

    # Desired output speed (both for motor A and B)
    omega_out = 0.5 * np.pi * 1  # Desired output angular velocity (rad/s)

    # Constants for optimization
    t_accel = 0.25  # seconds to reach steady state assuming a trapezoidal velocity profile
    ratios = np.linspace(1, 15, 100)  # test ratios from 1:1 to 15:1

    I_test_cases = [I_load_a+I_load_b, I_load_b]
    optimal_Ns = []
    safety_margins = []
    for I_load in I_test_cases:

        optimal_N = 0
        max_safety_margin = -np.inf

        for N in ratios:
            # 1. Calculate Required Motor Speed (rad/s)
            motor_omega = omega_out * N
            # Convert to steps/sec for your torque lookup
            steps_sec = (
                motor_omega * config.HS174401.steps_per_revolution) / (2 * np.pi)

            # 2. Find Available Torque at this speed (interpolate from your pull_out_torque array)
            # Or just recalculate using your term1 - term2 formula
            available_torque = np.interp(
                steps_sec, step_per_second, pull_out_torque)

            # 3. Calculate Required Torque to accelerate the load
            total_inertia = config.HS174401.rotor_inertia + \
                ((I_load) / (N**2))
            alpha_motor = (omega_out * N) / t_accel
            required_torque = total_inertia * alpha_motor

            # 4. Check Safety Margin
            margin = available_torque - required_torque

            if margin > max_safety_margin and steps_sec < 2500:  # Stay within reliable speeds
                max_safety_margin = margin
                optimal_N = N

        optimal_Ns.append(optimal_N)
        safety_margins.append(max_safety_margin)

    out += f"\n   Optymalne parametry silnika A: \n"
    out += f"       Przełożenie: {optimal_Ns[0]:.2f}:1 @ {(optimal_Ns[0]*omega_out*config.HS174401.steps_per_revolution)/(2*np.pi):.2f} kroków/s\n"
    available_torque_a = np.interp(
        optimal_Ns[0]*omega_out*config.HS174401.steps_per_revolution/(2*np.pi), step_per_second, pull_out_torque)
    out += f"       Dostępny moment siły: {available_torque_a*1000:.2f} mNm z marginesem bezpieczeństwa {safety_margins[0]*1000:.2f} mNm\n"

    out += f"\n   Optymalne parametry silnika B: \n"
    out += f"       Przełożenie: {optimal_Ns[1]:.2f}:1 @ {(optimal_Ns[1]*omega_out*config.HS174401.steps_per_revolution)/(2*np.pi):.2f} kroków/s\n"
    available_torque_b = np.interp(
        optimal_Ns[1]*omega_out*config.HS174401.steps_per_revolution/(2*np.pi), step_per_second, pull_out_torque)
    out += f"       Dostępny moment siły: {available_torque_b*1000:.2f} mNm z marginesem bezpieczeństwa {safety_margins[1]*1000:.2f} mNm\n"

    if PRINT_TO_CONSOLE:
        print(out)
    else:
        with io.open(config.ANALYSIS_OUTPUT_PATH, "a") as out_file:
            out_file.write(out)


if __name__ == "__main__":
    main()
