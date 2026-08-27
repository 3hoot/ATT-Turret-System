from dataclasses import dataclass

ANALYSIS_OUTPUT_PATH: str = "./out.txt"


@dataclass
class Stepper:
    """Parametry silnika krokowego"""
    rated_current: float        # [A]
    phase_resistance: float     # [Ohm]
    phase_inductance: float     # [H]
    holding_torque: float       # [Nm]
    detent_torque: float        # [Nm]
    rotor_inertia: float        # [kg*m^2]
    steps_per_revolution: int   # [steps/rev]


HS174401 = Stepper(
    rated_current=1.7,
    phase_resistance=1.5,
    phase_inductance=2.8e-3,
    holding_torque=40.0e-2,
    detent_torque=2.2e-2,
    rotor_inertia=54.0e-7,
    steps_per_revolution=200
)


@dataclass
class Supply:
    """Parametry zasilania układu"""
    supply_voltage: float      # [V]
    power: float               # [W]


GENERIC_SUPPLY = Supply(
    supply_voltage=24.0,
    power=200.0
)


@dataclass
class Driver:
    """Parametry sterownika silnika krokowego"""
    max_current: float         # [A]
    max_microstepping: int     # [microsteps/step]
    on_resistance: float       # [Ohm]


A4988 = Driver(
    max_current=2.0,
    max_microstepping=16,
    on_resistance=0.430
)
