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

# ╔═╡ fbdcd897-e143-4469-92f3-b6ec14f8cebe
md"## Robot Dog Invere Kinematics"

# ╔═╡ 3cc13168-a6ac-48a9-923d-5501402af13c
md"### Inversing the knee actuation chain"

# ╔═╡ f52b7004-faba-468e-8bdc-88bd6107bc3f
begin
	l1 = 20 # horizontal offset from servos
	l2 = 20 # vertical offset from servos
	l3 = 100 # upper leg length
	l4 = 110 # lower leg length
	l5 = 25 # servo arm length
	l6 = 50 # first servo linkage length
end

# ╔═╡ 05822716-fa08-4854-8592-c08ebc010e24
θ1 = 0

# ╔═╡ a9de1a0d-d180-441c-b583-46701b48d1ff
@bind θ2 Slider(-π:0.01:π)

# ╔═╡ 5aad6085-f4d1-49b1-ac47-0f9c1e0848e7
@bind θ3 Slider(-π:0.01:π)

# ╔═╡ ee5e3dbe-aa1a-42dd-a2d9-e0c9098fd4c5
[θ1, θ2, θ3]

# ╔═╡ dfa61f5c-d3ca-4b34-a6ce-3c97570be50b
begin
	origin = [0.0, 0.0]
	shoulder = [l1, l2]
	knee = shoulder + l3*[cos(θ2-π/2), sin(θ2-π/2)]
	foot = knee + l4*[cos(θ2+θ3-π), sin(θ2+θ3-π)]
	points = [origin, shoulder, knee, foot]
end

# ╔═╡ 6aee61ec-5984-4f66-872a-55daec0c69e4
function circle(P, r)
	return [[P[1] + r*cos(θ), P[2] + r*sin(θ)] for θ = 0:0.05:(2π+0.05)]
end

# ╔═╡ ecebd2ea-235a-4853-8534-683e33dedd60
@bind αhat Slider(0:0.01:2π)

# ╔═╡ f022e164-77c9-47f8-ad97-764afe10937f
begin
	α0 = π/4 # offset since d is not parallel to x
	α = αhat + α0
	
	Q = l5 * [cos(α), sin(α)]
	
	d = sqrt(l1^2 + l2^2)
	b = sqrt((Q[1] - l1)^2 + (Q[2] - l2)^2)
	
	β = acos((l6^2 + b^2 - l5^2) / (2*l6*b))
	δ = acos((b^2 + d^2 - l5^2) / (2*b*d))
	
	P1 = Q + l6*[cos(β - δ + α0), sin(β - δ + α0)]
	P2 = Q + l6*[cos(-β - δ + α0), sin(-β - δ + α0)]
end

# ╔═╡ 6905010f-ebab-4433-93c1-b580f53bdc19
begin
	scatter(X(points), Y(points), aspect_ratio=:equal, legend=false)
	plot!(X(points[2:4]), Y(points[2:4]), color=:black, linewidth=2)
	plot!(X(circle(origin, 25)), Y(circle(origin, 25)), line=:dot)
	plot!(X(circle(shoulder, l5)), Y(circle(shoulder, l5)), line=:dot)
	plot!(X([[0, 0], Q, P1, Q, P2]), Y([[0, 0], Q, P1, Q, P2]))
	plot!(X(circle(Q, l6)), Y(circle(Q, l6)), line=:dot)
	plot!(X(circle(knee, l5)), Y(circle(knee, l5)), line=:dot)
end

# ╔═╡ 2774a8ae-1b08-4fc5-8362-59e06fae5bb6
[α, β, δ]

# ╔═╡ Cell order:
# ╠═c3a94772-986c-11eb-32b7-632a4157a5e6
# ╠═9575c5a5-526b-4adb-8f72-6479b63db823
# ╟─fbdcd897-e143-4469-92f3-b6ec14f8cebe
# ╟─3cc13168-a6ac-48a9-923d-5501402af13c
# ╠═f52b7004-faba-468e-8bdc-88bd6107bc3f
# ╠═05822716-fa08-4854-8592-c08ebc010e24
# ╠═a9de1a0d-d180-441c-b583-46701b48d1ff
# ╠═5aad6085-f4d1-49b1-ac47-0f9c1e0848e7
# ╠═ee5e3dbe-aa1a-42dd-a2d9-e0c9098fd4c5
# ╠═dfa61f5c-d3ca-4b34-a6ce-3c97570be50b
# ╟─6aee61ec-5984-4f66-872a-55daec0c69e4
# ╠═6905010f-ebab-4433-93c1-b580f53bdc19
# ╠═ecebd2ea-235a-4853-8534-683e33dedd60
# ╠═2774a8ae-1b08-4fc5-8362-59e06fae5bb6
# ╠═f022e164-77c9-47f8-ad97-764afe10937f
