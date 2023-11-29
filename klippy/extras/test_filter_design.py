#test_filter_design.py

import scipy.signal as signal

def butterworth_filter_coeffs(cutoff, order):
    """
    Calculate the filter coefficients for a Butterworth filter.

    Parameters:
        cutoff (float): Cutoff frequency.
        order (int): Order of the Butterworth filter.

    Returns:
        (b, a) tuple: Numerator and denominator coefficients.
    """
    from math import sqrt

    r = tan(pi * cutoff)
    c = 1.0 + 2.0**0.5
    a_denom = 1.0 + c * r + r**2
    b_nom = [1.0, -2.0, 1.0]
    a_denom *= 2.0**0.5

    b = [val * (r**order) for val in b_nom]
    a = [val / a_denom for val in (1.0, -2.0, 1.0)]

    return b, a

