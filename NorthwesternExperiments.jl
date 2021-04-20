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

# ╔═╡ f52b7004-faba-468e-8bdc-88bd6107bc3f
begin
	l1 = 20 # horizontal offset from servos
	l2 = 20 # vertical offset from servos
	l3 = 100 # upper leg length
	l4 = 110 # lower leg length
	l5 = 25 # servo arm length
	l6 = 50 # first servo linkage length
end

# ╔═╡ e9f61724-c9d9-4925-a225-532f1a0ebfe7
@bind x Slider(-100:100)

# ╔═╡ 95924372-8a44-4492-807a-9f753a832d93
@bind y Slider(-170:0)

# ╔═╡ abc52a8e-11ba-4125-9aba-5aad5607b4fa
md"#### 2D inverse kinematics"

# ╔═╡ 3cc13168-a6ac-48a9-923d-5501402af13c
md"#### Inverting the knee actuation chain"

# ╔═╡ 44656382-db76-4699-8a34-7cf450ae1dd6
norm(P) = sqrt(sum(P.^2))

# ╔═╡ 204da9c8-b7e1-4a60-9039-abbf9bfec24b
begin
	𝓍 = x - l1
	𝓎 = y - l2
	γ = atan(𝓎, 𝓍)
	e = norm([𝓍, 𝓎])
	δ = acos((l3^2 + e^2 - l4^2) / (2*l3*e))
	ϵ = acos((l3^2 + l4^2 - e^2) / (2*l3*l4))
end

# ╔═╡ a9de1a0d-d180-441c-b583-46701b48d1ff
θ2 = π/2 + γ + δ

# ╔═╡ ecebd2ea-235a-4853-8534-683e33dedd60
θ3 = ϵ - π/2 #@bind θ3 Slider(-π:0.01:π)

# ╔═╡ f71fa539-e509-44a0-a930-aebbf1f05153
α = θ2 + θ3 + π/2

# ╔═╡ dfa61f5c-d3ca-4b34-a6ce-3c97570be50b
begin
	origin = [0.0, 0.0]
	shoulder = [l1, l2]
	knee = shoulder + l3*[cos(θ2-π/2), sin(θ2-π/2)]
	foot = knee + l4*[cos(θ2+θ3-π), sin(θ2+θ3-π)]
	points = [origin, shoulder, knee, foot]
end

# ╔═╡ c9042f68-66d5-4691-9a10-25d648596cb8
unit(θ) = [cos(θ), sin(θ)]

# ╔═╡ 02c872eb-9678-4f9c-a002-36e9d1dc8cc5
begin
	Q = shoulder + l5 * unit(α)
	S = shoulder + l5 * unit(α - π/2)
	T = knee + l5 * unit(θ2 + θ3)
	
	d = norm(shoulder)
	n = norm(Q - origin)
	
	β1 = acos((l5^2 + n^2 - l6^2) / (2*l5*n))
	β2 = acos((d^2 + n^2 - l5^2) / (2*d*n))
	β = β1 + β2 + π/4
	
	R = l5 * unit(β)
end

# ╔═╡ 6aee61ec-5984-4f66-872a-55daec0c69e4
function circle(P, r)
	return [[P[1] + r*cos(θ), P[2] + r*sin(θ)] for θ = 0:0.01:(2π+0.01)]
end

# ╔═╡ c3492a8b-608f-4377-ac7d-619de88f094c
function plotCircle!(P, r)
	plot!(X(circle(P, r)), Y(circle(P, r)), line=:dot)
end

# ╔═╡ 4811574f-0bd2-4d3d-8cd7-5db27c2e828e
function arc(P, r, θstart, length)
	if length >= 0
		return [[P[1] + r*cos(θ), P[2] + r*sin(θ)] for θ = θstart:0.01:(θstart + length + 0.01)]
	else
		return [[P[1] + r*cos(θ), P[2] + r*sin(θ)] for θ = (θstart + length):0.01:(θstart+0.01)]
	end
end

# ╔═╡ 5f0e3206-d7ca-4210-b408-e5eefa5f36b4
function plotArc!(P, r, θstart, length)
	plot!(X(arc(P, r, θstart, length)), Y(arc(P, r, θstart, length)))
end

# ╔═╡ cd84d23f-5d6f-4c9a-a8d0-a3087a198ede
pplot2!(points) = plot!(X(points), Y(points), color=:black, linewidth=1.5)

# ╔═╡ 6905010f-ebab-4433-93c1-b580f53bdc19
begin
	scatter(X(points), Y(points), aspect_ratio=:equal, legend=false)
	plot!(X(points[1:2]), Y(points[1:2]), color=:black, linewidth=1.5)
	plot!(X(points[2:4]), Y(points[2:4]), color=:black, linewidth=3)
	
	pplot2!([origin, R, Q, shoulder, S, T, knee])
	
	#plotArc!(origin, 10, 0, π/4)
	#plotArc!(origin, 10, π/4, β1)
	#plotArc!(origin, 10, π/4+β1, β2)
	#plotCircle!(Q, l6)
	
	plotArc!(shoulder, 10, 3/2*π, θ2)
	plotArc!(knee, 10, θ2+π, θ3)
	plotArc!(shoulder, l5/2, α-π/2, π/2)
	scatter!([x], [y])
	#scatter!([𝓍], [𝓎])
end

# ╔═╡ Cell order:
# ╠═c3a94772-986c-11eb-32b7-632a4157a5e6
# ╠═9575c5a5-526b-4adb-8f72-6479b63db823
# ╟─fbdcd897-e143-4469-92f3-b6ec14f8cebe
# ╠═f52b7004-faba-468e-8bdc-88bd6107bc3f
# ╟─6905010f-ebab-4433-93c1-b580f53bdc19
# ╠═e9f61724-c9d9-4925-a225-532f1a0ebfe7
# ╠═95924372-8a44-4492-807a-9f753a832d93
# ╠═a9de1a0d-d180-441c-b583-46701b48d1ff
# ╠═ecebd2ea-235a-4853-8534-683e33dedd60
# ╟─abc52a8e-11ba-4125-9aba-5aad5607b4fa
# ╠═204da9c8-b7e1-4a60-9039-abbf9bfec24b
# ╟─3cc13168-a6ac-48a9-923d-5501402af13c
# ╠═f71fa539-e509-44a0-a930-aebbf1f05153
# ╠═dfa61f5c-d3ca-4b34-a6ce-3c97570be50b
# ╠═02c872eb-9678-4f9c-a002-36e9d1dc8cc5
# ╟─44656382-db76-4699-8a34-7cf450ae1dd6
# ╟─c9042f68-66d5-4691-9a10-25d648596cb8
# ╟─6aee61ec-5984-4f66-872a-55daec0c69e4
# ╟─c3492a8b-608f-4377-ac7d-619de88f094c
# ╟─4811574f-0bd2-4d3d-8cd7-5db27c2e828e
# ╟─5f0e3206-d7ca-4210-b408-e5eefa5f36b4
# ╟─cd84d23f-5d6f-4c9a-a8d0-a3087a198ede
