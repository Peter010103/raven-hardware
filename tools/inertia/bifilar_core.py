"""Offline bifilar routines adapted from sanity-hardware (MIT)."""
# Copyright (c) 2026 Prorok Lab, University of Cambridge.
# See licenses/SANITY-MIT.txt and LICENSES.md.
import math
import numpy as np
from scipy.optimize import least_squares
from scipy.signal import find_peaks,savgol_filter

def geometry(mass_g, length_cm, top_cm, bottom_cm):
    values = np.array([mass_g, length_cm, top_cm, bottom_cm], dtype=float)
    if not np.isfinite(values).all() or np.any(values <= 0):
        raise ValueError("Mass and dimensions must be finite and positive")
    m, length, a, b = mass_g / 1000, length_cm / 100, top_cm / 200, bottom_cm / 200
    if length <= abs(a-b):
        raise ValueError("Strings too short for the attachment spacing")
    height = math.sqrt(length**2 - (a-b)**2)
    stiffness = m * 9.80665 * a * b / height
    return dict(mass_total_kg=m, string_length_m=length, upper_full_spacing_m=2*a,
                lower_full_spacing_m=2*b, vertical_separation_m=height,
                gravity_m_s2=9.80665, stiffness_Nm_rad=stiffness,
                inertia_per_T_squared=stiffness/(4*math.pi**2))

def fit_oscillation(t, angle, min_frequency=.2, max_frequency=10.):
    if not (0 < min_frequency < max_frequency):
        raise ValueError("Require 0 < min_frequency < max_frequency")
    if len(t) < 100 or t[-1]-t[0] < 8:
        raise ValueError("Need at least eight seconds of usable free oscillation")
    if not np.isfinite(angle).all() or not np.isfinite(t).all() or np.any(np.diff(t) <= 0):
        raise ValueError("Angles and strictly increasing timestamps must be finite")
    if np.max(np.diff(t)) > .1:
        raise ValueError("Tracking gap exceeds 100 ms inside the fit window; select a continuous window")
    # Frequency is fitted against source timestamps, not network arrival timing.
    dt = max(float(np.median(np.diff(t))), .02)
    grid = np.arange(0., t[-1]-t[0], dt)
    y = np.interp(grid, t-t[0], angle)
    centred = grid-grid.mean()
    trend = np.polyfit(centred, y, 1)
    detrended = y-np.polyval(trend, centred)
    if np.std(detrended) < math.radians(.15):
        raise ValueError("No resolvable torsional oscillation (less than 0.15 deg RMS)")
    frequencies = np.fft.rfftfreq(len(grid), dt)
    power = abs(np.fft.rfft(detrended*np.hanning(len(grid))))**2
    indices = np.flatnonzero((frequencies >= min_frequency) & (frequencies <= max_frequency))
    if not len(indices):
        raise ValueError("Fit window has no resolvable frequency in the requested range")
    starts = indices[np.argsort(power[indices])[-5:]]

    def solve(parameters, return_model=False):
        frequency, decay = parameters
        envelope = np.exp(-decay*grid)
        phase = 2*math.pi*frequency*grid
        matrix = np.column_stack([np.ones(len(grid)), centred,
                                  envelope*np.cos(phase), envelope*np.sin(phase)])
        coefficients = np.linalg.lstsq(matrix, y, rcond=None)[0]
        model = matrix@coefficients
        return (model, coefficients) if return_model else model-y

    solutions = [least_squares(solve, [frequencies[i], .02],
                               bounds=([min_frequency, 0.], [max_frequency, 1.]),
                               xtol=1e-10, ftol=1e-10, gtol=1e-10)
                 for i in starts]
    result = min(solutions, key=lambda x: float(x.fun@x.fun))
    model, coefficients = solve(result.x, True)
    frequency, decay = result.x
    oscillation = model-(coefficients[0]+coefficients[1]*centred)
    residual_rms = float(np.sqrt(np.mean((model-y)**2)))
    oscillation_rms = float(np.sqrt(np.mean(oscillation**2)))
    amplitude = float(np.hypot(*coefficients[2:]))
    ratio = residual_rms / max(oscillation_rms, 1e-12)
    failures = []
    warnings = []
    if not result.success:
        failures.append("Oscillation fit did not converge")
    if frequency*grid[-1] < 6:
        failures.append("Fewer than six fitted cycles")
    if ratio > .15:
        failures.append("Fit residual exceeds 15% of oscillation RMS; inspect release, swing and tracking")
    if math.degrees(amplitude) < .5:
        failures.append("Fitted starting amplitude below 0.5 deg")
    if frequency < min_frequency*1.025 or frequency > max_frequency*.995 or decay > .98:
        failures.append("Fit reached frequency/decay search boundary")
    if math.degrees(amplitude) > 12:
        warnings.append("Amplitude above 12 deg: small-angle approximation needs review")
    if decay < 1e-5:
        warnings.append("Decay is at zero bound; damping was not resolved")
    corrected = y-(coefficients[0]+coefficients[1]*centred)
    window = max(5, int(.12/frequency/dt) | 1)
    window = min(window, (len(y)-1)//2*2+1)
    smooth = savgol_filter(corrected, window, 2)
    peaks, _ = find_peaks(smooth, distance=max(1, int(.65/frequency/dt)),
                          prominence=max(math.radians(.1), .15*amplitude*np.exp(-decay*grid[-1])))
    periods = np.diff(grid[peaks])
    periods = periods[(periods > .7/frequency) & (periods < 1.3/frequency)]
    summary = dict(frequency_damped_hz=float(frequency), period_damped_s=float(1/frequency),
                   decay_per_s=float(decay), amplitude_at_fit_start_deg=math.degrees(amplitude),
                   residual_rms_deg=math.degrees(residual_rms), residual_to_oscillation_rms=ratio,
                   fitted_cycles=float(frequency*grid[-1]),
                   peak_period_median_s=float(np.median(periods)) if len(periods) else None,
                   peak_period_sd_s=float(np.std(periods, ddof=1)) if len(periods)>1 else None,
                   peak_period_count=len(periods), failures=failures, warnings=warnings)
    return summary, grid, y, model
