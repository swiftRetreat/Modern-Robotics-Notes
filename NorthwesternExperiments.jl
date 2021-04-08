### A Pluto.jl notebook ###
# v0.12.21

using Markdown
using InteractiveUtils

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

# ╔═╡ Cell order:
# ╠═c3a94772-986c-11eb-32b7-632a4157a5e6
# ╠═3d377bb4-9876-11eb-09cb-33c4b1dc37ce
# ╠═5bf76cb0-986e-11eb-064d-69c7e1a77b3f
# ╠═fc7fe034-9870-11eb-054f-dbc90a4d14df
# ╠═276302d8-9874-11eb-1118-55416578b304
# ╠═e40e5788-9875-11eb-3b1c-27197e950e8d
# ╠═bb7f6ab8-9871-11eb-0a42-4fdd37372713
# ╠═65c86218-9874-11eb-3b55-c71b1e35bcf9
