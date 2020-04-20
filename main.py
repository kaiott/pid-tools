import PIDUtils as pid
import numpy as np
import matplotlib.pyplot as plt
import control as co


# specifications, ep=0 assumed
te = 2.65
delta = 0.1

# Process
K=7
process_poles = np.array([0, -12])
process_zeros = np.array([])

# Initial guess PID
pid_poles = np.array([0])
pid_zeros = np.array([])

# limit pole
s = -1.50943+2.0594j

# specifications
sigma = 4/te
y_p_x = -np.pi/np.log(delta)
poles = np.append(process_poles, pid_poles)
zeros = np.append(process_zeros, pid_zeros)

#argument and modulo criterion to figure out paramters of PID
z = pid.argument_criterion(s=s, poles=poles, zeros=zeros)
pid.print_text(z=z, s=s, poles=poles, zeros=zeros)
pid_zeros = np.append(pid_zeros, z)
zeros = np.append(zeros, z)

kC = pid.modulo_criterion(s=s, poles=poles, zeros=zeros, K=K)
print(f'solution at kC = { kC:.2f}')

G = K * co.tf(np.poly(process_zeros), np.poly(process_poles))

PI = co.tf(np.poly(pid_zeros),np.poly(pid_poles))
print(G)
print(PI)

Gcl = co.feedback(0.0001*G*PI)
co.root_locus(Gcl, Plot=True, grid=False)
Gcl=Gcl.minreal(tol=0.01)
print(Gcl)
#co.root_locus(Gcl, Plot=True, grid=False)

PI = kC*PI
Gcl = co.feedback(G*PI)
Gcl=Gcl.minreal(tol=0.01)
print(Gcl)
print(co.pole(Gcl))
print(co.zero(Gcl))

xmin, xmax, ymin, ymax = plt.axis()
allthings = np.append(np.append(np.append(zeros, poles), s), np.conj(s))
bottom=np.max([np.min(np.imag(allthings)) -2, ymin])
top=np.min([np.max(np.imag(allthings)) + 2, ymax])
left=np.max([np.min(np.real(allthings)) -2, xmin])
right=np.min([np.max(np.real(allthings)) + 2, xmax])
plt.ylim(bottom, top)
plt.xlim(left, right)
xmin, xmax, ymin, ymax = plt.axis()

for root in co.pole(Gcl):
	plt.plot(np.real(root), np.imag(root), color='red', marker='*')
	plt.text(np.real(root)+0.1, np.imag(root) + (ymax-ymin)/20*(1 if np.imag(root) > 0 else -1), f'{root:.2f}', color='red')

x = np.real(s)
y = np.imag(s)
plt.fill_between([0, xmax*1.1], ymin*1.1, ymax*1.1, color='red', alpha=0.1)
plt.fill_between([xmin*1.1, 0], ymax*1.1, [-y_p_x*xmin*1.1, 0], color='red', alpha=0.1)
plt.fill_between([xmin*1.1, 0], [y_p_x*xmin*1.1, 0], ymin*1.1, color='red', alpha=0.1)
plt.fill_between([x, 0], [y, 0], [-y, 0], color='red', alpha=0.1)

plt.grid()
plt.show()