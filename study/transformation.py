#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

# time constance
t = np.linspace(0, 0.02, 120)
# signal, 110V/60Hz
amplitude = 110 * np.sqrt(2)
frequency = 60
omega =  2 * np.pi * frequency

def clark_transform(a, b, c, transform_type='amplitude'):
    '''
    Clark transformation:
      Converts a three-phase stationary coordinate system (abc) to a two-phase stationary coordinate system (αβ).

    parameters:
        a, b, c - Three-phase voltage/current signals
        transform_type:
            'amplitude' - Amplitude-preserving transformation (default)
            'power'     - Power-invariant transformation

    return:
        alpha, beta

    '''

    if transform_type == 'amplitude':
        k = 2/3
    elif transform_type == 'power':
        k = np.sqrt(2/3)
    else:
        raise ValueError("Invalid transform type. Choose 'amplitude' or 'power'")

    alpha = k * (va - 0.5*vb - 0.5*vc)
    beta = k * (np.sqrt(3)/2*vb - np.sqrt(3)/2*vc)
    return alpha, beta

def inverse_clark_transform(alpha, beta, transform_type='amplitude'):
    '''
    Inverse Clark transformation:
      Converts the αβ coordinate system to the abc coordinate system.

    parameters:
       alpha, beta - two phase signals
       transform_type:
            'amplitude' - Amplitude-preserving transformation (default)
            'power'     - Power-invariant transformation

    return:
       a, b, c - three phase signal

    '''

    if transform_type == 'amplitude':
        k = 1
    elif transform_type == 'power':
        k = np.sqrt(2/3)
    else:
        raise ValueError("Invalid transform type. Choose 'amplitude' or 'power'")

    a = k * alpha
    b = k * (-0.5*alpha + np.sqrt(3)/2*beta)
    c = k * (-0.5*alpha - np.sqrt(3)/2*beta)
    return a, b, c

def park_transform(alpha, beta, gamma):
    '''
    Park transformation:
      Converts a stationary coordinate system (αβ) to a rotating coordinate system (dq).

    parameters:
        alpha, beta - two phase signals
        gamma - Rotation angle in radians

    return：
        d, q - direct and quadrature axes
    '''

    cos_c = np.cos(gamma)
    sin_c = np.sin(gamma)

    d = cos_c * alpha + sin_c * beta
    q = -sin_c * alpha + cos_c * beta
    return d, q

def inverse_park_transform(d, q, gamma):
    '''
    Inverse Park transformation:
      Converts the dq coordinate system to the αβ coordinate system.

    parameters:
        d, q - direct and quadrature axes signals
        gamma - Rotation angle in radians

    return：
        alpha, beta
    '''

    cos_c = np.cos(gamma)
    sin_c = np.sin(gamma)

    alpha = cos_c * d - sin_c * q
    beta = sin_c * d + cos_c * q
    return alpha, beta


'''
A balanced three-phase system consists of three sinusoidal voltages or currents of equal magnitude and frequency,
displaced by 120 degrees from each other. This results in the sum of the three phases being zero at all times.

Mathematically, this can be represented as:
   Va + Vb + Vc = zero
'''
va = amplitude * np.sin(omega * t)
vb = amplitude * np.sin(omega * t - 2 * np.pi / 3)
vc = amplitude * np.sin(omega * t - 4 * np.pi / 3)

# Clark transformation
v_alpha, v_beta = clark_transform(a=va, b=vb, c=vc)

# Park transformation
gamma = omega * t
v_d, v_q = park_transform(alpha=v_alpha, beta=v_beta, gamma=gamma)

# plot
plt.figure(figsize=(10, 12))

plt.subplot(3, 1, 1)
plt.plot(t, va, label=r'$V_{a}$', linewidth=2)
plt.plot(t, vb, label=r'$V_{b}$', linewidth=2)
plt.plot(t, vc, label=r'$V_{c}$', linewidth=2)
plt.legend(loc='upper right')
plt.title('Original abc')

plt.subplot(3, 1, 2)
plt.plot(t, v_alpha, label=r'$V_{\alpha}$', linewidth=2)
plt.plot(t, v_beta, label=r'$V_{\beta}$', linewidth=2)
plt.legend(loc='upper right')
plt.title('Clark Transform to α-β')

plt.subplot(3, 1, 3)
plt.plot(t, v_d, label=r'$V_{d}$', linewidth=2)
plt.plot(t, v_q, label=r'$V_{q}$', linewidth=2)
plt.ylim([(amplitude * -1.2), 50])
plt.legend(loc='upper right')
plt.title('Park Transform to d-q')

plt.tight_layout()
plt.subplots_adjust(
    left=0.08,
    right=0.95,
    bottom=0.05,
    top=0.95,
    wspace=0.25,
    hspace=0.33
)

plt.show()
