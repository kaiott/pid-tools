import PIDUtils as pid
import numpy as np
import matplotlib.pyplot as plt
import control as co

class PIDControl():
	def __init__(self, k, process_zeros, process_poles, ts, delta, ep = 0):
		# contructor arguments
		self.k = k
		self.process_zeros = process_zeros
		self.process_poles = process_poles
		self.ts = ts
		self.delta = delta
		self.ep = ep

		# derived variables
		self.process = k * co.tf(np.poly(process_zeros), np.poly(process_poles))
		self.sigma = 4 / ts
		self.tan_phi = -np.pi / np.log(delta)
		self.wp = self.sigma * self.tan_phi
		self.s = -self.sigma + self.wp * 1j

		self.pid_zeros = np.array([])
		self.pid_poles = np.array([])
		if ep == 0:
			self.pid_poles = np.append(self.pid_poles, [0])

		self.kp = 1
		self._update_pid()

		self.poles = np.append(self.process_poles, self.pid_poles)
		self.zeros = np.append(self.process_zeros, self.pid_zeros)

	def add_zero_to_controller(self, zero):
		self.pid_zeros = np.append(self.pid_zeros, zero)
		self.zeros = np.append(self.zeros, zero)
		self._update_pid()

	def set_kp_of_controller(self, kp):
		self.kp = kp
		self._update_pid()

	def add_zero_by_criterium(self):
		z = pid.argument_criterion(s=self.s, poles=self.poles, zeros=self.zeros)
		pid.print_text(z=z, s=self.s, poles=self.poles, zeros=self.zeros)
		self.add_zero_to_controller(z)	

	def set_kp_by_criterium(self):
		kp = pid.modulo_criterion(s=self.s, poles=self.poles, zeros=self.zeros, K=self.k)
		self.set_kp_of_controller(kp)
		print(f'solution at kC = { kp:.2f}')

	def _update_pid(self):
		self.pid = self.kp * co.tf(np.poly(self.pid_zeros),np.poly(self.pid_poles)).minreal(tol=0.001)
		self.closed_loop = co.feedback(self.process*self.pid).minreal(tol=0.001)

	def draw_root_locus(self):
		co.root_locus(co.feedback(0.00001*self.process*self.pid), Plot=True, grid=False)

		st = -self.sigma + self.wp * 1j
		poles_and_zeros = np.append(np.append(self.zeros, self.poles), [st, np.conj(st)])
		ymin=np.min(np.imag(poles_and_zeros)) - 2
		ymax=np.max(np.imag(poles_and_zeros)) + 2
		xmin=np.min(np.real(poles_and_zeros)) -2
		xmax=np.max(np.real(poles_and_zeros)) + 2
		plt.ylim(ymin, ymax)
		plt.xlim(xmin, xmax)

		for root in co.pole(self.closed_loop):
			plt.plot(np.real(root), np.imag(root), color='red', marker='*')
			plt.text(np.real(root)+0.1, np.imag(root) + (ymax-ymin)/20*(1 if np.imag(root) > 0 else -1), f'{root:.2f}', color='red')

		x = np.real(st)
		y = np.imag(st)
		plt.fill_between([0, xmax*1.1], ymin*1.1, ymax*1.1, color='red', alpha=0.1)
		plt.fill_between([xmin*1.1, 0], ymax*1.1, [-self.tan_phi*xmin*1.1, 0], color='red', alpha=0.1)
		plt.fill_between([xmin*1.1, 0], [self.tan_phi*xmin*1.1, 0], ymin*1.1, color='red', alpha=0.1)
		plt.fill_between([x, 0], [y, 0], [-y, 0], color='red', alpha=0.1)

		plt.grid()
		plt.show()

	def print_Gp(self):
		print(self.process)

	def print_PID(self):
		print(self.pid)

	def print_Gcl(self):
		print(self.closed_loop)
