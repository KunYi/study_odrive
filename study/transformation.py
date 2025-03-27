#!/usr/bin/python

import numpy as np
import matplotlib.pyplot as plt

# time constance
t = np.linspace(0, 0.02, 120)
# signal, 110V/60Hz
amplitude = 110 * np.sqrt(2)
frequency = 60
omega =  2 * np.pi * frequency

# three phase sine waves
va = amplitude * np.sin(omega * t)
vb = amplitude * np.sin(omega * t - 2 * np.pi / 3)
vc = amplitude * np.sin(omega * t - 4 * np.pi / 3)

# Clark transformation
v_alpha = (2/3) * (va - 0.5 * vb - 0.5 * vc)
v_beta = (2/3) * ((np.sqrt(3) / 2) * vb - (np.sqrt(3) / 2) * vc)
v_zero = (2/3) * (0.5 * va + 0.5 * vb + 0.5 * vc)

# Park transformation
gamma = omega * t
v_d = np.cos(gamma) * v_alpha + np.sin(gamma) * v_beta
v_q = -np.sin(gamma) * v_alpha + np.cos(gamma) * v_beta

# plot
plt.figure(figsize=(10, 12))

plt.subplot(3, 1, 1)
plt.plot(t, va, label=r'$V_{a}$', linewidth=2)
plt.plot(t, vb, label=r'$V_{b}$', linewidth=2)
plt.plot(t, vc, label=r'$V_{c}$', linewidth=2)
plt.legend()
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
plt.legend()
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
