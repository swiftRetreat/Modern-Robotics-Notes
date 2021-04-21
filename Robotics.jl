# imports
using Plots
using LinearAlgebra
using Distributions

# Robotics
# `scarafleus`, `blueSky`, (`Kirite`), (`Albert Wigmore`). December 2020 - ?"

# Commonly used vectors in their homogeneous space representation.
xhat = [1.0, 0.0, 0.0, 1.0]
yhat = [0.0, 1.0, 0.0, 1.0]
zhat = [0.0, 0.0, 1.0, 1.0]
zero = [0.0, 0.0, 0.0, 1.0]

# Matrices for computations

# Identity matrix
# Computes the identity matrix of dimension `n` by `n`.
eye(n) = Matrix(1.0I, n, n)

# Computes the identity matrix of dimension `n` by `m`.
eye(n, m) = Matrix(1.0I, n, m)

# Rotation Matrix
# Computes a rotation matrix by combining a rotation axis `ω` with an angle `θ`.
function rotationMatrix(ω, θ)
	ω = ω/sqrt(ω[1]^2 + ω[2]^2 + ω[3]^2)
	return rotationMatrix(ω[1], ω[2], ω[3], θ)
end

# Computes a rotation matrix by combining a rotation axis given by `ω1`, `ω2`, `ω3` with an angle `θ`.
function rotationMatrix(ω1, ω2, ω3, θ)
	cθ = cos(θ)
	sθ = sin(θ)

	R = zeros(3, 3)
	R[1, 1] = cθ + ω1^2*(1 - cθ)
	R[2, 1] = ω1*ω2*(1 - cθ) + ω3*sθ
	R[3, 1] = ω1*ω3*(1 - cθ) - ω2*sθ
	R[1, 2] = ω1*ω2*(1 - cθ) - ω3*sθ
	R[2, 2] = cθ + ω2^2*(1 - cθ)
	R[3, 2] = ω2*ω3*(1 - cθ) + ω1*sθ
	R[1, 3] = ω1*ω3*(1 - cθ) + ω2*sθ
	R[2, 3] = ω2*ω3*(1 - cθ) - ω1*sθ
	R[3, 3] = cθ + ω3^2*(1 - cθ)
	return R
end

# Transformation Matrix
# Computes the 4x4 dimensional transformation matrix from combining a rotation matrix `R` with an offset vector `p`.
function transformationMatrix(R, p)
	T = zeros(4, 4)
	T[1:3, 1:3] = R
	T[1:3, 4] = p[1:3]
	T[4, 4] = 1
	return T
end

# Computes the 4x4 dimensional transformation matrix from combining a rotation matrix given by a rotation axis `ω` and an angle `θ` with an offset vector `p`.
transformationMatrix(ω, θ, p) = transformationMatrix(rotationMatrix(ω, θ), p)

# Extracts the rotation matrix from a transformation matrix `T`.
R(T) = T[1:3, 1:3]

# Extracts the offset vector from a transformation matrix `T`.
p(T) = T[:, 4]

# Computes the inverse of a transformationMatrix `T` efficiently.
function inverseTransformationMatrix(T::Array{Float64, 2})
	Ti = zeros(4, 4)
	Ti[1:3, 1:3] = Transpose(R(T))
	Ti[1, 4] = -dot(R(T)[1:3, 1], p(T))
	Ti[2, 4] = -dot(R(T)[1:3, 2], p(T))
	Ti[3, 4] = -dot(R(T)[1:3, 3], p(T))
	Ti[4, 4] = 1
	return Ti
end

# Skew Matrix
# Computes the skew matrix of a vector or matrix `v` as defined in the book.
function skew(v)
	if length(v) == 6
		S = v
		ω = S[1:3]
		ν = S[4:6]
		return [skew(ω) ν; zeros(1, 3) 0]
	else
		ω = v
		return [0 -ω[3] ω[2]; ω[3] 0 -ω[1]; -ω[2] ω[1] 0]
	end
end

# Adjoint Matrix
# Computes the adjoint matrix of the transformation matrix `T`. Do not use a capital 'a'.
function adjoint(T::Array{Float64, 2})
	return [R(T) zeros(3, 3); skew(p(T))*R(T) R(T)]
end

# Kinematic chains
struct KinematicChain
	joints::Array{Array{Float64, 1}, 1}
	axis::Array{Array{Float64, 1}, 1}
	range::Float64
end

# 3R robot as seen in the book p. 138.
function createRRR(L1::Float64, L2::Float64, L3::Float64)
	return KinematicChain([L1*xhat, L2*xhat, L3*xhat], [yhat, yhat, yhat], L1+L2+L3)
end
RRR = createRRR(1.0, 1.0, 1.0)

# KUKA LBR iiwa 7 d.o.f robot, as seen in task 4.1 and 5.1 in the exercise manual.
function createKUKA(L1::Float64, L2::Float64, L3::Float64, L4::Float64)
	return KinematicChain([
			[0.0, 0.0, L1, 1.0],
			[0.0, 0.0, 0.0, 1.0],
			[0.0, 0.0, L2, 1.0],
			[0.0, 0.0, 0.0, 1.0],
			[0.0, 0.0, L3, 1.0],
			[0.0, 0.0, 0.0, 1.0],
			[0.0, 0.0, L4, 1.0]
		],
		[
			zhat,
			xhat,
			zhat,
			xhat,
			zhat,
			xhat,
			zhat
		], L1+L2+L3+L4)
end
KUKA = createKUKA(0.34, 0.4, 0.4, 0.15)

# PUMA 6 d.o.f. robot arm which has isolated translation and rotation components making it useful for inverse kinematics.
function createPUMA(L1::Float64, L2::Float64, L3::Float64)
	return KinematicChain([
			[0.0, 0.0, L1, 1.0],
			[0.0, 0.0, L2, 1.0],
			[0.0, 0.0, L3, 1.0],
			[0.0, 0.0, 0.0, 1.0],
			[0.0, 0.0, 0.0, 1.0],
			[0.0, 0.0, 0.0, 1.0]
		],
		[
			xhat,
			xhat,
			xhat,
			xhat,
			yhat,
			zhat
		], L1+L2+L3)
end
PUMA = createPUMA(1.0, 1.0, 1.0)

# Classical Leg for quadruped robots. Not anatomically correct, they would need to have an additional elbow."

# Returns the `transformation matrices` from {s} to {joint} for given joint `axis`, `joint` representations in their own reference frame and joint parameters `θ`. `Tl` is the transformation matrix from {s} to {last joint} and `T` is the transformation matrix from {last joint} to {next joint}."
function transformationsDH(joints::Array{Array{Float64, 1}, 1}, axis::Array{Array{Float64, 1}, 1}, θ)

	ground::Array{Float64, 1} = [0.0, 0.0, 0.0, 1.0]
	T::Array{Float64, 2} = transformationMatrix(axis[1], θ[1], ground)
	transformations = [T]

	for index = 2:length(joints)
		Tl::Array{Float64, 2} = transformationMatrix(axis[index], θ[index], joints[index-1])
		T = T * Tl
		push!(transformations, T)
	end

	Tl = transformationMatrix(eye(3), joints[length(joints)])
	T = T * Tl
	push!(transformations, T)

	return transformations
end

transformationsDH(C::KinematicChain, θ) = transformationsDH(C.joints, C.axis, θ)

# Returns the `transformation matrix T` that specifies the `end-effectors` position.
function transformationEE(C::KinematicChain, θ)
	return last(transformationsDH(C::KinematicChain, θ))
end

# Returns the EndEffectors location
function EELocation(C::KinematicChain, θ)
	return p(last(transformationsDH(C::KinematicChain, θ)))
end

# Plotting

# This function takes an array of transformation matrices `Ts` and plots a graph of the coordinate frames at each joint as well as the joints. This is set within the plot `limits`."
function plotKC(Ts::Array{Array{Float64, 2}, 1}, limits)
	l = length(Ts)
	p = plotJoints(Ts, limits)
	for index = 1:l
		showAxis!(p, Ts[index], limits)
	end
	p
end

# This function takes a `KinematicChain` and parameters `θ` and plots a graph of the coordinate frames at each joint as well as the joints. You can provide offset parameters `θ0`.
function plotKC(C::KinematicChain, θ, θ0=[])
	plotKC(transformationsDH(C, length(θ) == length(θ0) ? θ - θ0 : θ), (-C.range, C.range))
end

# Displays the axis at a reference frame given by a transformation matrix `T`. Takes the parameter `plot` for the plot to draw in, this plot has to be actively displayed afterwards. Also takes the plot `limits` which dictate the length of the axis to make sure it is adequate.
function showAxis!(plot, T::Array{Float64, 2}, limits)
	l::Float64 = (limits[2] - limits[1]) / 20
	xAxis = T * [l, 0.0, 0.0, 1.0]
	yAxis = T * [0.0, l, 0.0, 1.0]
	zAxis = T * [0.0, 0.0, l, 1.0]
	origin = T * zero

	plot!(plot, [origin xAxis][1, :], [origin xAxis][2, :], [origin xAxis][3, :], color=:red)
	plot!(plot, [origin yAxis][1, :], [origin yAxis][2, :], [origin yAxis][3, :], color=:blue)
	plot!(plot, [origin zAxis][1, :], [origin zAxis][2, :], [origin zAxis][3, :], color=:green)
end

# Plots the joints of a robot which is represented by transformation matrices `Ts`. The joint coordinates are extracted from the matrices offset vector.
function plotJoints(Ts::Array{Array{Float64, 2}, 1}, lims)
	l = length(Ts)
	xCord = [Ts[index][1, 4] for index = 1:l]
	yCord = [Ts[index][2, 4] for index = 1:l]
	zCord = [Ts[index][3, 4] for index = 1:l]
	plot(xCord, yCord, zCord, lims=lims, color=:black, linewidth=4, legend=false)
	scatter!(xCord, yCord, zCord, color=:white)
end

# Compute the screw axis of the `joints` and `axis` in {s}.
function spaceScrews(joints::Array{Array{Float64, 1}, 1}, axis::Array{Array{Float64, 1}, 1})
	n = length(joints)
	Ts = transformationsDH(joints, axis, zeros(n))
	S::Array{Float64, 2} = zeros(6, n)
	for i = 1:length(Ts)-1
		S[1:3, i] = axis[i][1:3]
		S[4:6, i] = -cross(axis[i][1:3], p(Ts[i])[1:3])
	end
	return S
end

# Compute the screw axis of the kinematic chain `C` in {s}.
spaceScrews(C::KinematicChain) = spaceScrews(C.joints, C.axis)

# Compute the screw axis of the `joints` and `axis` in {b}.
function bodyScrews(joints::Array{Array{Float64, 1}, 1}, axis::Array{Array{Float64, 1}, 1})
	n = length(joints)
	Ts = transformationsDH(-reverse(joints), reverse(axis), zeros(n))
	B::Array{Float64, 2} = zeros(6, n)
	for i = 1:length(Ts)-1
		B[1:3, i] = axis[n+1-i][1:3]
		B[4:6, i] = -cross(axis[n+1-i][1:3], p(Ts[n+1-i])[1:3])
	end
	return B
end

# Compute the screw axis of the kinematic chain `C` in {b}.
bodyScrews(C::KinematicChain) = bodyScrews(C.joints, C.axis)

# Space Jacobian from screws
# Computes the Jacobian in {s} using the `screws` and `θ`.
function spaceJacobian(screws::Array{Float64, 2}, θ::Array{Float64, 1})
	n = length(θ)
	Jacobian::Array{Float64, 2} = eye(6, n)
	for index in 1:n
		Jacobian[:, index] = spaceJacobian(screws, θ, index)
	end
	return Jacobian
end

# Returns the `i`th column of the jacobian.
function spaceJacobian(screws::Array{Float64, 2}, θ::Array{Float64, 1}, i::Int64)
	Jsn = eye(4)
	for index = 1:i-1
		Jsn = Jsn * exp(skew(screws[:, index]) * θ[index])
	end
	return adjoint(Jsn) * screws[:, i]
end

spaceJacobian(C::KinematicChain, θ::Array{Float64, 1}) = spaceJacobian(spaceScrews(C), θ)

# Body Jacobian from screws
# Computes the Jacobian in {b} using the `screws` and `θ`.
function bodyJacobian(screws::Array{Float64, 2}, θ::Array{Float64, 1})
	n = length(θ)
	Jacobian::Array{Float64, 2} = eye(6, n)
	for index in 1:n
		Jacobian[:, index] = bodyJacobian(screws, θ, index)
	end
	return Jacobian
end

# Returns the `i`th column of the jacobian.
function bodyJacobian(screws::Array{Float64, 2}, θ::Array{Float64, 1}, i::Int64)
	Jsn = eye(4)
	for index = length(θ):-1:i+1
		Jsn = Jsn * exp(-skew(screws[:, index]) * θ[index])
	end
	return adjoint(Jsn) * screws[:, i]
end

bodyJacobian(C::KinematicChain, θ::Array{Float64, 1}) = bodyJacobian(bodyScrews(C), θ)

# Inverse Kinematics
# Computes the inverse kinematics for a given kinematic chain `C`, initial parameters `θ_0` and desired position `x_d`. The inital parameters already have to be quite close to a solution for this algorithm to converge.
function inversePositionKinematics(C::KinematicChain, θ_0::Array{Float64, 1}, x_d::Array{Float64, 1})
	maxIterations = 200
	iterations = 0
	θ_i::Array{Float64, 1} = θ_0
	x_a::Array{Float64, 1} = p(transformationEE(C, θ_i))
	e = (x_d - x_a)[1:3]

	while iterations < maxIterations && norm(e) > tol
		x_a = p(transformationEE(C, θ_i))
		pJ_i = pinv(spaceJacobian(C, θ_i)[4:6, :])
		e = (x_d - x_a)[1:3]
		Δθ = pJ_i * e
		θ_i += Δθ
		iterations += 1
	end
	return θ_i
end

# Robot Dog Leg
function createLeg()
	L1 = [36, 57, -18, 1.0]
	L2 = [0, 0, -120, 1.0]
	L3 = [5.9, 0, -136.6, 1.0]
	return KinematicChain([
		L1,
		L2,
		L3
	], [
		xhat,
		yhat,
		yhat
	], norm(L1+L2+L3))
end
Leg = createLeg()

struct MassPoint
	position::Array{Float64, 1}
	mass::Float64
end

# rigid body consisting of discrete mass points
struct RigidBody
	massPoints::Array{MassPoint, 1}
end

# plotting rigid bodies
function plotRB(rigidBody::RigidBody)
	pscatter([mp.position for mp in rigidBody.massPoints])
end

function plotRB!(rigidBody::RigidBody)
	pscatter!([mp.position for mp in rigidBody.massPoints])
end

function rotationalInertiaMatrix(rigidBody::RigidBody)
	return sum([mp.mass * skew(mp.position)^2 for mp in rigidBody.massPoints])
end

ℐ(rigidBody::RigidBody) = rotationalInertiaMatrix(rigidBody)

function totalMass(rigidBody::RigidBody)
	return sum([mp.mass for mp in rigidBody.massPoints])
end

function spatialInertiaMatrix(rigidBody::RigidBody)
	return [rotationalInertiaMatrix(rigidBody) zeros(3, 3)
			zeros(3, 3) totalMass(rigidBody) * eye(3)]
end

𝒢(rigidBody::RigidBody) = spatialInertiaMatrix(rigidBody)

function spatialMomentum(spatialInertiaMatrix, twist)
	return spatialInertiaMatrix * twist
end

𝒫(spatialInertiaMatrix, twist) = spatialMomentum(spatialInertiaMatrix, twist)

# Helpers

X(A::Array{Array{Float64, 1}, 1}) = [A[index][1] for index = 1:length(A)]
X(A::Array{Float64, 1}) = [A[1]]
Y(A::Array{Array{Float64, 1}, 1}) = [A[index][2] for index = 1:length(A)]
Y(A::Array{Float64, 1}) = [A[2]]
Z(A::Array{Array{Float64, 1}, 1}) = [A[index][3] for index = 1:length(A)]
Z(A::Array{Float64, 1}) = [A[3]]

pplot(points) = plot(X(points), Y(points), Z(points))
pplot!(points) = plot!(X(points), Y(points), Z(points))
pscatter(points) = scatter(X(points), Y(points), Z(points))
pscatter!(points) = scatter!(X(points), Y(points), Z(points))

function inverseALPLeg(x, y, z)
	l1::Float64 = 20  # horizontal offset from servos
	l2::Float64 = 20  # vertical offset from servos
	l3::Float64 = 100 # upper leg length
	l4::Float64 = 110 # lower leg length
	l5::Float64 = 25  # servo arm length
	l6::Float64 = 50  # first servo linkage length
	l7::Float64 = 40  # depth difference between shoulder servo and rest of leg
	f = norm([z, y])
	ξ = atan(-sqrt(f^2 - l7^2), l7) - atan(y, z)
	𝓍 = x - l1 # offset due to shoulder not being at origin
	𝓎 = y - l2 + z*sin(ξ) + y*cos(ξ) - y # offset due to shoulder and x-axis rotation (θ1) causing a y offset
	γ = atan(𝓎, 𝓍)
	e = norm([𝓍, 𝓎])
	δ = acos((l3^2 + e^2 - l4^2) / (2*l3*e))
	ϵ = acos((l3^2 + l4^2 - e^2) / (2*l3*l4))
	origin = [0.0, 0.0]
	shoulder = [l1, l2]

	θ1 = -acos(l7/f) - atan(y, z)
	θ2 = π/2 + γ + δ
	θ3 =  ϵ - π/2

	knee = shoulder + l3*[cos(θ2-π/2), sin(θ2-π/2)]
	foot = knee + l4*[cos(θ2+θ3-π), sin(θ2+θ3-π)]
	points = [origin, shoulder, knee, foot]

	α = θ2 + θ3 + π/2
	Q = shoulder + l5 * unit(α)
	S = shoulder + l5 * unit(α - π/2)
	T = knee + l5 * unit(θ2 + θ3)

	d = norm(shoulder)
	n = norm(Q - origin)

	β1 = acos((l5^2 + n^2 - l6^2) / (2*l5*n))
	β2 = acos((d^2 + n^2 - l5^2) / (2*d*n))
	β = β1 + β2 + π/4

	R = l5 * unit(β)
	return [θ1, θ2, θ3, α, ξ]
end

# cut out but may be useful later

"""
function sampleWorkspace(C::KinematicChain, iterations::Int64)
	points::Array{Array{Float64, 1}, 1} = []
	for index = 1:iterations
		θ = rand(-π:0.00001:π, length(C.joints))
		push!(points, p(last(transformationsDH(C.joints, C.axis, θ))))
	end
	scatter(X(points), Y(points), Z(points), legend=false, color=:blue)
end
"""
