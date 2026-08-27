import config
import io
import numpy as np
import datetime as date

PRINT_TO_CONSOLE: bool = True


def main() -> None:
    out: str = f"\nAnaliza napędu {date.datetime.now()}:\n"

    # --- Analiza charakterystyki układu napędowego ---
    # Based on motor_analysis.py results and SDP-SI materials

    M_peak = config.HS174401.holding_torque  # at startup

    gear_ratios = [4.82, 2.84]  # [m]
    d_centers = [0.08, 0.16]  # [m]

    # Backup offset values in case the center distance is too small for the given pulley sizes
    gear_offsets = [0.02, 0.02]  # [m]
    gear_in_teeth = 20

    # Calculated from the number of teeth and module (m = d / z)
    gear_in_diameter = 12.73 / 1000  # [m]

    t_accel = 0.25
    steps_in = [240.91, 141.92]
    omega_in = [2*np.pi * step /
                config.HS174401.steps_per_revolution for step in steps_in]
    M_accel = [72.97e-3, 35.15e-3]  # [Nm]

    for i in range(len(gear_ratios)):
        ratio = gear_ratios[i]

        pulley_out = gear_in_diameter * ratio
        pulley_in = gear_in_diameter

        out += f"\n   Średnica koła napędzanego: {pulley_out * 1000:.2f} mm, Średnica koła napędowego: {pulley_in * 1000:.2f} mm\n"

        if d_centers[i] < (pulley_out + pulley_in) / 2:
            d_center = (pulley_out + pulley_in) / 2 + gear_offsets[i]
        else:
            d_center = d_centers[i]

        out += f"   Odległość między osiami: {d_center * 1000:.2f} mm\n"

        # Teeth in mesh calculation
        TIM = gear_in_teeth * (0.5 - (pulley_out - pulley_in)/(6*d_center))

        TIM_factors = {2: 0.2, 3: 0.4, 4: 0.6, 5: 0.8, 6: 1.0}
        TIM_factor = 0.0
        for key, factor in TIM_factors.items():
            if TIM >= key:
                TIM_factor = factor

        out += f"   Przełożenie: {ratio:.2f}, TIM: {TIM:.2f}, Współczynnik TIM: {TIM_factor:.2f}\n"

        M_design = M_peak / TIM_factor

        # Approximate pitch length calculation
        d_pitch = (2 * d_center) + (1.57 * (pulley_in + pulley_out)
                                    ) + ((pulley_out - pulley_in)**2)/(4*d_center)

        out += f"   Moment projektowy: {M_design:.2f} Nm, Długość pasa: {d_pitch * 1000:.2f} mm\n"

        # Effective tension on the belt (on the smaller pulley)
        T_belt = M_design / (pulley_in / 2)

        out += f"   Naprężenie efektywne na pasie: {T_belt:.2f} N\n"

        # Peak power calculation at the output shaft
        # Normally occuring at acceleration from 0 to max speed
        # Linear acceleration assumption

        # Old calcuation, not accounting for the fact that the motor won't be providing holding torque at operational speeds
        # Motor won't put out full torque at high speeds, so we apply a safety factor
        # P_peak = 0.8 * M_design * (omega_in[i] / t_accel)

        # New calculation, using the actual torque at the given speed
        P_out = 0.95 * M_accel[i] * (omega_in[i])  # small correction factor

        out += f"   Szacunkowa moc na wale wyjściowym: {P_out:.2f} W ~ {P_out * 0.001341022:.4f} hp\n"

    if PRINT_TO_CONSOLE:
        print(out)
    else:
        with open(config.ANALYSIS_OUTPUT_PATH, "w") as f:
            f.write(out)


if __name__ == "__main__":
    main()
