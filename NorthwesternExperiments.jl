### A Pluto.jl notebook ###
# v0.14.2

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
	using PlutoUI
	using Plots
end

# ╔═╡ c3a94772-986c-11eb-32b7-632a4157a5e6
include("Robotics.jl")

# ╔═╡ fbdcd897-e143-4469-92f3-b6ec14f8cebe
md"## Robot Dog Invere Kinematics"

# ╔═╡ 924616b7-4fc2-4329-b91c-ef9fe7244b08
Toff1 = [eye(3) [-50, 0, 0]; [0 0 0] 1]

# ╔═╡ a20fe9fa-5cc7-4d38-b4f7-7833b4ce90e1
Toff2 = [eye(3) [50, 0, 0]; [0 0 0] 1]

# ╔═╡ f64aecfa-ab4d-4679-af80-ad8877fb61a3
Toff3 = [eye(3) [-50, 200, 0]; [0 0 0] 1]

# ╔═╡ 03b7a16e-9f28-49f5-9567-8c2e0bbb9e93
Toff4 = [eye(3) [50, 200, 0]; [0 0 0] 1]

# ╔═╡ e9f61724-c9d9-4925-a225-532f1a0ebfe7
@bind xglobal Slider(-100:100, default=0, show_value=true)

# ╔═╡ 85b203e4-8c4a-4aff-93f3-0cd164c861b5
@bind yglobal Slider(-70:70, default=0, show_value=true)

# ╔═╡ 95924372-8a44-4492-807a-9f753a832d93
@bind zglobal Slider(-170:0, default=-100, show_value=true)

# ╔═╡ 8c55ead2-938c-46c2-85c2-25a676712aaf
θ1 = inverseALPLeg(xglobal, yglobal, zglobal, eye(4))

# ╔═╡ b2c2c899-7fb7-42eb-97be-c44ea3cd8833
θ2 = inverseALPLeg(xglobal, yglobal, zglobal, eye(4))

# ╔═╡ a5280457-3dd6-4fd1-a4e8-9f02ffce37c7
θ3 = inverseALPLeg(xglobal, yglobal, zglobal, eye(4))

# ╔═╡ 20056452-fc6b-4394-ba67-1efd1e2d553b
θ4 = inverseALPLeg(xglobal, yglobal, zglobal, eye(4))

# ╔═╡ 37e7bae3-aa16-484a-b470-4f0a9d98fc12
begin
	plotKC(ALPLeg, θ1, Toff1)
	plotKC!(ALPLeg, θ2, Toff2)
	plotKC!(ALPLeg, θ3, Toff3)
	plotKC!(ALPLeg, θ4, Toff4)
	plot!([50, -50, -50, 50, 50], [0, 0, 200, 200, 0], zeros(5), color=:black, linewidth=3)
	#scatter!([xglobal], [yglobal], [zglobal])
end

# ╔═╡ Cell order:
# ╠═c3a94772-986c-11eb-32b7-632a4157a5e6
# ╠═9575c5a5-526b-4adb-8f72-6479b63db823
# ╟─fbdcd897-e143-4469-92f3-b6ec14f8cebe
# ╠═924616b7-4fc2-4329-b91c-ef9fe7244b08
# ╠═8c55ead2-938c-46c2-85c2-25a676712aaf
# ╠═a20fe9fa-5cc7-4d38-b4f7-7833b4ce90e1
# ╠═b2c2c899-7fb7-42eb-97be-c44ea3cd8833
# ╠═f64aecfa-ab4d-4679-af80-ad8877fb61a3
# ╠═a5280457-3dd6-4fd1-a4e8-9f02ffce37c7
# ╠═03b7a16e-9f28-49f5-9567-8c2e0bbb9e93
# ╠═20056452-fc6b-4394-ba67-1efd1e2d553b
# ╟─e9f61724-c9d9-4925-a225-532f1a0ebfe7
# ╟─85b203e4-8c4a-4aff-93f3-0cd164c861b5
# ╟─95924372-8a44-4492-807a-9f753a832d93
# ╟─37e7bae3-aa16-484a-b470-4f0a9d98fc12
