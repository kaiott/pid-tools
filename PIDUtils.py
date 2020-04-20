import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt


def modulo_criterion(s, poles, zeros, K):
	p_mods = [np.abs(s-p) for p in poles]
	z_mods = [np.abs(s-z) for z in zeros]
	prod_p = np.prod(p_mods)
	prod_z = np.prod(z_mods)
	kC = prod_p/(prod_z*np.abs(K))
	return kC


def argument_criterion_bad(s, poles, zeros):
	n = len(poles)
	m = len(zeros)
	p_args = [np.angle(s-p) for p in poles]
	z_args = [np.angle(s-z) for z in zeros]
	sum_p = np.sum(p_args)
	sum_z = np.sum(z_args)
	z = -np.imag(s)/np.tan(sum_p - sum_z - np.pi) + np.real(s)
	return z


def argument_function(z, *args):
	s, poles, zeros = args
	p_args = [np.angle(s-p) for p in poles]
	z_args = [np.angle(s-z) for z in zeros]
	return np.sum(p_args) - np.sum(z_args) - np.angle(s - z) - np.pi

def argument_criterion(s, poles, zeros):
	z0 = np.real(s)
	z = fsolve(argument_function, z0, args=(s, poles, zeros))
	return z


def plot_crit_arg(z, s, poles, zeros):
	# Plot it
	z = np.linspace(z*2, z/2, 100)
	plt.plot(z, argument_function(z, s, poles, zeros))
	plt.xlabel("z")
	plt.ylabel("expression value")
	plt.grid()
	plt.show()


def print_text(z, s, poles, zeros):
	print(f'Polo de diseño s = { s:.2f}')
	print(f'Con el criterio del argumento: \u2211arg(s-pi) - \u2211arg(s-zi) = (2p+1)\u03C0, p \u220A \u2124')

	string = ''
	for pole in poles:
		string += f' + arg(s-({ pole }))'
	for zero in zeros:
		string += f' - arg(s-({ zero }))'
	string += f' -\u03C0'
	print(f'arg(s-z) = { string }')

	string = ''
	for pole in poles:
		string += f' + {np.angle(s-pole):.2f}'
	for zero in zeros:
		string += f' - {np.angle(s-zero):.2f}'
	string += f' -\u03C0'
	print(f'arg(s-z) ={ string }')

	string = ''
	for pole in poles:
		string += f' + {np.angle(s-pole,deg=True):.2f}°'
	for zero in zeros:
		string += f' - {np.angle(s-zero,deg=True):.2f}°'
	string += f' -\u03C0'
	print(f'arg(s-z) ={ string }')

	print(f'arg(s-z)= {np.angle(s-z)[0]:.2f}')
	print(f'arg(s-z)= {np.angle(s-z,deg=True)[0]:.2f}°')
	print(f'solution at z = { z[0]:.2f}')

	zeros = np.append(zeros, z)

	string = ''
	for pole in poles:
		string += f' {np.abs(s-pole):.2f},'
	print(f'Distance of poles: { string }')
	string = ''
	for zero in zeros:
		string += f' {np.abs(s-zero):.2f},'
	print(f'Distance of zeros: { string }	!!possibly needs reordering according to order of angles!!')


	def pid_f(kp, z1, z2, b, c):
		return 0

