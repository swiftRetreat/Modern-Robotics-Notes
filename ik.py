import numpy as np
import time
from adafruit_servokit import ServoKit

kit = ServoKit(channels=16)

"""
2000/04/29 - Hannes Richter
This is a skript calculating the inverse kinematics for the ALP-dog.
It comes with a lot of functions for illustration which will be removed on
deployment.
"""

shoulderT = 12
kit.servo[shoulderT].set_pulse_width_range(500, 2500)
shoulderB = 13
kit.servo[shoulderB].set_pulse_width_range(500, 2500)
elbow = 14
kit.servo[elbow].set_pulse_width_range(500, 2500)

l1, l2, l3, l4, l5, l6, l7 = (20, 20, 100, 110, 40, 25, 50)

π = np.pi

def deg(rad):
	return 180/π*rad

def cosTheorem(adj1, adj2, opp):
	return np.arccos((adj1**2 + adj2**2 - opp**2) / (2*adj1*adj2))

def inverseKinematics(x, y, z):
	d4 = np.sqrt(x**2 + z**2)
	ν = np.arccos(l5/d4)
	ω = -np.arctan2(z, x)
	θ1 = ω - ν
	θ1s = θ1 + π/2

	𝓎 = y - l1
	𝓏 = d4 * np.sin(-ν) - l2
	d1 = np.sqrt(𝓎**2 + 𝓏**2)
	α = cosTheorem(l3, l4, d1)
	β = np.arctan2(𝓏, 𝓎)
	γ = cosTheorem(d1, l3, l4)

	θ2 = β + γ + π/2
	θ2s = θ2 + π/2
	θ3 = α - π/2

	α = θ2 + θ3
	δ = α + π/4

	# shoulder servo angle
	d3 = np.sqrt(l1**2 + l2**2)
	d2 = np.sqrt((d3 + l6*np.cos(δ))**2 + (l6*np.sin(δ))**2)

	ϵ = cosTheorem(d2, d3, l6) + cosTheorem(l6, d2, l7)
	ϵf = ϵ + π/4
	ϵs = ϵf - π/2
	return deg(θ1s), deg(θ2s), deg(ϵs)

for t in range(100):
	ψ = t/100*2*π
	x, y, z = 0, 50*np.cos(ψ), - 100 + 50*np.sin(ψ)
	θ1, θ2, θ3 = inverseKinematics(x, y, z)
	time.sleep(0.05)
	print(θ1, θ2, θ3)
	kit.servo[shoulderT].angle = θ1
	kit.servo[shoulderB].angle = θ2
	kit.servo[elbow].angle = θ3




