### A Pluto.jl notebook ###
# v0.14.1

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    quote
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : missing
        el
    end
end

# ╔═╡ 9575c5a5-526b-4adb-8f72-6479b63db823
begin
	using Plots
	using PlutoUI
end

# ╔═╡ c3a94772-986c-11eb-32b7-632a4157a5e6
include("Robotics.jl")

# ╔═╡ 3d377bb4-9876-11eb-09cb-33c4b1dc37ce
plotKC(KUKA, ones(7))

# ╔═╡ 5bf76cb0-986e-11eb-064d-69c7e1a77b3f
rigidBody1 = RigidBody([MassPoint([x, 0.0, 0.0, 1.0], 1) for x=-2:0.1:2])

# ╔═╡ fc7fe034-9870-11eb-054f-dbc90a4d14df
rigidBody2 = RigidBody([MassPoint([1.0, 0.0, 0.0, 1.0], 1), MassPoint([0.0, 0.0, 0.0, 1.0], 1), MassPoint([0.0, 1.0, 0.0, 1.0], 1), MassPoint([1.0, 1.0, 0.0, 1.0], 1)])

# ╔═╡ 276302d8-9874-11eb-1118-55416578b304
# Exercise 8.6
rigidBody3 = RigidBody([MassPoint([0.0, 3*sin(2π*θ), 3*cos(2π*θ), 1.0], 0.01) for θ=0:0.001:1])

# ╔═╡ e40e5788-9875-11eb-3b1c-27197e950e8d
begin
	plotRB(rigidBody1)
	plotRB!(rigidBody2)
	plotRB!(rigidBody3)
end

# ╔═╡ bb7f6ab8-9871-11eb-0a42-4fdd37372713
ℐ(rigidBody2)

# ╔═╡ 65c86218-9874-11eb-3b55-c71b1e35bcf9
𝒢(rigidBody3)

# ╔═╡ 1b90affb-18d1-452b-a430-a3126fb60702
md"### Spotmicro Inverse Kinematics"

# ╔═╡ 1bee6643-9be6-48d5-b885-cb2518a86bc3


# ╔═╡ b2264f78-fa69-41ad-bfb3-c9d6caecb506
begin
	l1 = 25
	l2 = 20
	l3 = 80
	l4 = 80
end

# ╔═╡ a589b8eb-8de1-415c-9e65-483008de73c5
@bind x Slider(-100:100)

# ╔═╡ 210f38e6-567b-4564-b42b-ddb568073600
@bind y Slider(-100:100)

# ╔═╡ e046a833-2429-4f72-bd03-47f7a8687b54
begin
	α = atan(y, x)
	d = sqrt(x^2 + y^2)
	θ1 = acos(l2 / d) + α
end

# ╔═╡ aa0fbda5-d991-40ea-87f6-a369a138359a
begin
	plot([0, x], [0, y], aspect_ratio=:equal)
	plot!([0, x], [0, 0])
	scatter!([0, d*cos(α)], [0, d*sin(α)])
	plot!([0, l2*cos(θ1), x], [0, l2*sin(θ1), y])
	plot!([l2 * cos(ψ) for ψ = 0:0.1:θ1], [l2 * sin(ψ) for ψ = 0:0.1:θ1])
end

# ╔═╡ e717092b-8e20-4e30-bafc-c853a893d86d
@bind z Slider(-100:100)

# ╔═╡ 47cbe95a-c208-44b3-9110-b26c657f0c30
begin
	𝓏 = z - l1
	γ = atan(y, 𝓏)
	e = sqrt(𝓏^2 + y^2)
	β = acos(1 - e^2/(2l3^2))
	δ = (π - β)/2
	θ2 = γ + δ
	θ3 = π - β
end

# ╔═╡ 51d66ae3-9bed-47eb-8b9c-e3f432dc3f64
begin
	plot([0, l1, z], [0, 0, y], aspect_ratio=:equal)
	
	scatter!([0, l1, l1 + l3*cos(θ2), l1 + l3*cos(θ2) + l3*cos(θ2 - θ3)], [0, 0, l3*sin(θ2), l3*sin(θ2) + l3*sin(θ2 - θ3)])
	
	plot!([l1, l1 + l3*cos(θ2), l1 + l3*cos(θ2) + l3*cos(θ2 - θ3)], [0, l3*sin(θ2), l3*sin(θ2) + l3*sin(θ2 - θ3)])
	
	plot!([l1, l1 + e*cos(γ)], [0, e*sin(γ)])
	
	plot!([l1 + l1 * cos(ψ) for ψ = 0:0.01:θ2], [l1 * sin(ψ) for ψ = 0:0.01:θ2])
	
	plot!([l1 + l3*cos(θ2) + l1 * cos(ψ) for ψ = (-π+θ2):0.01:(-π+θ2+β)], [l3*sin(θ2) + l1 * sin(ψ) for ψ = (-π+θ2):0.01:(-π+θ2+β)])
end

# ╔═╡ Cell order:
# ╠═c3a94772-986c-11eb-32b7-632a4157a5e6
# ╠═9575c5a5-526b-4adb-8f72-6479b63db823
# ╠═3d377bb4-9876-11eb-09cb-33c4b1dc37ce
# ╠═5bf76cb0-986e-11eb-064d-69c7e1a77b3f
# ╠═fc7fe034-9870-11eb-054f-dbc90a4d14df
# ╠═276302d8-9874-11eb-1118-55416578b304
# ╠═e40e5788-9875-11eb-3b1c-27197e950e8d
# ╠═bb7f6ab8-9871-11eb-0a42-4fdd37372713
# ╠═65c86218-9874-11eb-3b55-c71b1e35bcf9
# ╟─1b90affb-18d1-452b-a430-a3126fb60702
# ╠═1bee6643-9be6-48d5-b885-cb2518a86bc3
# ╠═b2264f78-fa69-41ad-bfb3-c9d6caecb506
# ╠═e046a833-2429-4f72-bd03-47f7a8687b54
# ╠═aa0fbda5-d991-40ea-87f6-a369a138359a
# ╠═a589b8eb-8de1-415c-9e65-483008de73c5
# ╠═210f38e6-567b-4564-b42b-ddb568073600
# ╠═e717092b-8e20-4e30-bafc-c853a893d86d
# ╠═47cbe95a-c208-44b3-9110-b26c657f0c30
# ╠═51d66ae3-9bed-47eb-8b9c-e3f432dc3f64
