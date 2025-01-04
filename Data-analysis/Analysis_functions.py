import numpy as np

def specific_heat(temps: np.ndarray) -> np.ndarray:
    """
    This function calculates the specific heat of water for a given temperature or an array of temperatures.
    The function takes in a temperature or an array of temperatures in Celsius and returns the specific heat value(s) in J/gC.
    """
    z0 = 4.2184
    z1 = -2.8218 * 10**(-3)
    z2 = 7.3478 * 10**(-5)
    z3 = -9.4712 * 10**(-7)
    z4 = 7.2869 * 10**(-9)
    z5 = -2.8098 * 10**(-11)
    z6 = 4.4008 * 10**(-14)

    return z0 + z1*temps + z2*temps**2 + z3*temps**3 + z4*temps**4 + z5*temps**5 + z6*temps**6

def power(time_data: np.array, temp_data: np.array, water_mass: float) -> np.array:
    """
    This function calculates the power input to the system for all data points.
    The function takes in the time data, temperature data, and mass of water,
    and returns the power input in watts for each time interval.
    """
    
    specific_heat_values = specific_heat(temp_data)[1:]  # J/gC (specific heat of water at the current temperature)

    # Calculate the temperature difference
    temp_diff = np.diff(temp_data)

    # Calculate the time difference
    time_diff = np.diff(time_data)

    # Calculate the heat absorbed
    heat_absorbed = water_mass * specific_heat_values * temp_diff / time_diff

    return heat_absorbed

def energy_stored(temp_data: np.array, water_mass: float) -> float:
    """
    This function calculates the energy stored in the system.
    The function takes in the temperature data and mass of water, and returns the energy stored in the system in J.
    """
    
    specific_heat_values = specific_heat(temp_data)  # J/gC (specific heat of water at the current temperature)
    
    # Calculate the heat absorbed
    energy_harvested = water_mass * np.trapezoid(specific_heat_values, x=temp_data)

    return energy_harvested