import numpy as np
import matplotlib.pyplot as plt

"""
2000/04/29 - Hannes Richter
This is a skript calculating the inverse kinematics for the ALP-dog.
It comes with a lot of functions for illustration which will be removed on
deployment.
"""

xaxis = np.array([1, 0, 0, 1])
yaxis = np.array([0, 1, 0, 1])
zaxis = np.array([0, 0, 1, 1])
zero = np.array([0, 0, 0, 1])

l1, l2, l3, l4, l5, l6, l7 = (20, 20, 100, 110, 40, 25, 50)

π = np.pi

def rotationMatrix(ω, θ):
	ω = ω / np.linalg.norm(ω[0:3])
	ω1, ω2, ω3 = ω[0], ω[1], ω[2]
	cθ, sθ = np.cos(θ), np.sin(θ)

	R = np.zeros((3, 3))
	R[0, 0] = cθ + ω1**2*(1 - cθ)
	R[1, 0] = ω1*ω2*(1 - cθ) + ω3*sθ
	R[2, 0] = ω1*ω3*(1 - cθ) - ω2*sθ
	R[0, 1] = ω1*ω2*(1 - cθ) - ω3*sθ
	R[1, 1] = cθ + ω2**2*(1 - cθ)
	R[2, 1] = ω2*ω3*(1 - cθ) + ω1*sθ
	R[0, 2] = ω1*ω3*(1 - cθ) + ω2*sθ
	R[1, 2] = ω2*ω3*(1 - cθ) - ω1*sθ
	R[2, 2] = cθ + ω3**2*(1 - cθ)
	return R

def transformationMatrix(R, p):
        T = np.vstack((R, np.zeros(3)))
        p = np.expand_dims(p, axis=1)
        T = np.hstack((T, p))
        return T

def transformationsDH(joints, axis, θ):
	T = transformationMatrix(rotationMatrix(axis[0], θ[0]), zero)
	transformations = [T]

	for index in range(1, len(joints)-1):
		Rl = rotationMatrix(axis[index], θ[index])
		Tl = transformationMatrix(Rl, joints[index-1])
		T = T * Tl
		transformations.append(T)

	Tl = transformationMatrix(np.eye(3), joints[len(joints)-1])
	T = T * Tl
	transformations.append(T)

	return transformations

def computePoints(joints, axis, θ):
	R01 = rotationMatrix(axis[0], θ[0])
	T01 = transformationMatrix(R01, zero)
	R12 = rotationMatrix(axis[1], θ[1])
	T12 = transformationMatrix(R12, joints[0])
	T02 = T01.dot(T12)
	R23 = rotationMatrix(axis[2], θ[2])
	T23 = transformationMatrix(R23, joints[1])
	T03 = T02.dot(T23)
	T34 = transformationMatrix(np.eye(3), joints[2])
	T04 = T03.dot(T34)

	P = np.c_[T01[:, 3], T02[:, 3], T03[:, 3], T04[:, 3]]
	return P

def plotLeg(x, y, z, θs):
	P = computePoints(joints, axis, θs)
	α = θs[3]
	β = θs[4]
	θ1 = θs[0]
	θ2 = θs[1]
	θ3 = θs[2]
	ϵs = θs[5]
	S1 = np.array([l1 + l6*np.cos(α), l2*np.cos(θ1) + l6*np.sin(α)])
	S2 = np.array([l1 + l6*np.cos(α+π/2), l2*np.cos(θ1) + l6*np.sin(α+π/2)])
	S3 = np.array([l6*np.cos(β), l6*np.sin(β)])
	E = np.array([l1 + l3*np.cos(θ2 - π/2), l2*np.cos(θ1) + l3*np.sin(θ2 - π/2)])

	# y-z plot
	plt.subplot(121)
	plt.plot(P[1, :], P[2, :])
	plt.plot([0, S3[0], S2[0], l1, S1[0], E[0] + l6*np.cos(α), E[0]],
		 [0, S3[1], S2[1], l2, S1[1], E[1] + l6*np.sin(α), E[1]])
	plt.scatter([y], [z])
	plt.axis("equal")
	plt.xlabel("Y")
	plt.ylabel("Z")

	# x-z plot
	plt.subplot(122)
	plt.plot(P[0, :], P[2, :])
	plt.scatter([x], [z])
	plt.axis("equal")
	plt.xlabel("X")

axis = np.array([yaxis, xaxis, xaxis])

joint1 = np.array([l5, l1, l2, 1.0])
joint2 = np.array([0, 0, -l3, 1.0])
joint3 = np.array([0, -l4, 0, 1.0])
joints = np.array([joint1, joint2, joint3])

def cosTheorem(adj1, adj2, opp):
	return np.arccos((adj1**2 + adj2**2 - opp**2) / (2*adj1*adj2))

def inverseKinematics(x, y, z):
	d4 = np.sqrt(x**2 + z**2)
	ν = np.arccos(l5/d4)
	ω = -np.arctan2(z, x)
	θ1 = ω - ν

	𝓎 = y - l1
	𝓏 = d4 * np.sin(-ν) - l2
	d1 = np.sqrt(𝓎**2 + 𝓏**2)
	α = cosTheorem(l3, l4, d1)
	β = np.arctan2(𝓏, 𝓎)
	γ = cosTheorem(d1, l3, l4)

	θ2 = β + γ + π/2
	θ3 = α - π/2

	α = θ2 + θ3
	δ = α + π/4

	# shoulder servo angle
	d3 = np.sqrt(l1**2 + l2**2)
	d2 = np.sqrt((d3 + l6*np.cos(δ))**2 + (l6*np.sin(δ))**2)

	ϵ = cosTheorem(d2, d3, l6) + cosTheorem(l6, d2, l7)
	ϵf = ϵ + π/4
	ϵs = 3/2*π - ϵf
	return np.array([θ1, θ2, θ3, α, ϵf, ϵs])


plt.figure(figsize=(8, 4))

for t in range(100):
	ψ = t/100*2*π
	x, y, z = 0, 50*np.cos(ψ), - 100 + 50*np.sin(ψ)
	θs = inverseKinematics(x, y, z)
	plotLeg(x, y, z, θs)
	plt.pause(0.05)
	plt.clf()

plt.show()
