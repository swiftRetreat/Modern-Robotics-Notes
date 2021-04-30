import numpy as np

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
	return np.array([θ1, θ2, ϵs])

for t in range(100):
	ψ = t/100*2*π
	x, y, z = 0, 50*np.cos(ψ), - 100 + 50*np.sin(ψ)
	θs = inverseKinematics(x, y, z)
	print(θs)
