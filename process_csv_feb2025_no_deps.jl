### A Pluto.jl notebook ###
# v0.20.6

using Markdown
using InteractiveUtils

# This Pluto notebook uses @bind for interactivity. When running this notebook outside of Pluto, the following 'mock version' of @bind gives bound variables a default value (instead of an error).
macro bind(def, element)
    #! format: off
    return quote
        local iv = try Base.loaded_modules[Base.PkgId(Base.UUID("6e696c72-6542-2067-7265-42206c756150"), "AbstractPlutoDingetjes")].Bonds.initial_value catch; b -> missing; end
        local el = $(esc(element))
        global $(esc(def)) = Core.applicable(Base.get, el) ? Base.get(el) : iv(el)
        el
    end
    #! format: on
end

# ╔═╡ 7666f464-9c6e-11ef-2b1d-835a4afa5427
begin
using Rotations
using StaticArrays
using LinearAlgebra
using Distributions
#using WGLMakie
using CairoMakie
CairoMakie.activate!()
using MakieExtra
using AlgebraOfGraphics
using PairPlots
import AlgebraOfGraphics: density
const AoG = AlgebraOfGraphics
using StatsBase
using PrettyTables
using OhMyThreads: tmap
using LinearAlgebra: cholesky
#using SimpleNonlinearSolve
#import SimpleNonlinearSolve: SimpleTrustRegion
using NonlinearSolve
import DimensionalData: DimArray, @d, rebuild
using DimensionalData
using IntervalSets
using CSV, DataFrames
using PlutoUI
using StatisticalMeasures
using XLSX, Geodesy
using ForwardDiff
using MvNormalCalibration
using ProbabilisticParameterEstimators
end; md"Imports..."

# ╔═╡ 55d90c1a-94f5-4140-9b52-557e256c86ba
html"""<style>
main {
    max-width: 900px;
}
"""

# ╔═╡ 8ab9a7a1-55ad-4472-a55f-47cce4e8bbea
LinearAlgebra.cholesky(::UniformScaling) = (I, I)

# ╔═╡ 017d2e77-b7ab-4a57-83fe-b124a39494a3
struct CameraPoint{T <: Real} <: FieldVector{3, T}
    x::T
    y::T
    z::T
end; "Camera centric coordinate system"

# ╔═╡ a12ffe64-f7ec-4d12-add2-4e8b2713e785
struct WorldPoint{T <: Real} <: FieldVector{3, T}
    x::T
    y::T
    z::T
end; "World coordinate system."

# ╔═╡ 4604d95f-15aa-457b-b9b5-abe3b78a45d1
function cam_pt_to_world_pt(cam_pos::WorldPoint, cam_rot::Rotations.RotZYX, pt::CameraPoint)
    R = cam_rot; t = cam_pos
    R * pt + t
end

# ╔═╡ 2344d560-d75b-41a4-abd5-ada19c2a2f4c
struct ProjectionPoint{T <: Real} <: FieldVector{2, T}
    x::T
    y::T
end; "Projection coordiante system"

# ╔═╡ d748d115-e6f8-40ed-9646-8d8443384a17
function compute_statistic(cam_pos::WorldPoint, cam_rot::Rotations.RotZYX, pts::AbstractVector{<:WorldPoint},
                           z::AbstractVector{<:ProjectionPoint}, sigmas::AbstractVector{<:Real})
	Σ = Diagonal(repeat(sigmas.^2, outer=length(pts)))
	compute_statistic(cam_pos, cam_rot, pts, z, Σ)
end



# ╔═╡ 216b830a-d081-4cf7-a4dc-679f4740bc67
function world_pt_to_cam_pt(cam_pos::WorldPoint, cam_rot::Rotations.RotZYX, pt::WorldPoint)
    R = cam_rot; t = cam_pos
    CameraPoint(R' * (pt - t))
end

# ╔═╡ 0b2d8ffb-354c-487a-a174-bc768e942603
function project(cam_pos::WorldPoint, cam_rot::RotZYX, pt::WorldPoint;
                 focal_length::Real=0.04)
    pt_ = world_pt_to_cam_pt(cam_pos, cam_rot, pt)
    scale = let focal_length = 25e-3, pixel_size = 0.00345e-3 / 1  # m / pxl
        focal_length / pixel_size
    end
    ProjectionPoint((scale .* pt_[2:3] ./ pt_[1])...)
end

# ╔═╡ e202b700-9dd6-4d09-82ac-0ed8b15206a0
function compute_H(cam_pos::WorldPoint, cam_rot::Rotations.RotZYX, pts::AbstractVector{<:WorldPoint})
    H_pos = vcat((
        ForwardDiff.jacobian(cam_pos_->project(WorldPoint(cam_pos_...), cam_rot, pt), cam_pos)
        for pt in pts
    )...)
    H_rot = vcat((
        # `params(::RotZYX)` gives yaw, pitch roll, which is the same order it goes in again (by design)
        ForwardDiff.jacobian(cam_rot_->project(cam_pos, RotZYX(cam_rot_...), pt), Rotations.params(cam_rot))
        for pt in pts
    )...)
    H = [H_pos H_rot]
end

# ╔═╡ a64d427c-baf0-4419-930c-c5c7abc5041f
function compute_residual(
	  cam_pos::WorldPoint,
	  cam_rot::Rotations.RotZYX,
      pts::AbstractVector{<:WorldPoint},
      zs::AbstractVector{<:ProjectionPoint})
    z0s = [project(cam_pos, cam_rot, pt) for pt in pts]
    H = compute_H(cam_pos, cam_rot, pts)

    zs_ = vcat(zs...)
    z0s_ = vcat(z0s...)
    delta_z_ = zs_ - z0s_
    r = (I - H * pinv(H)) * delta_z_
end

# ╔═╡ 13745817-3643-4958-a11e-308cc93c6d51
function compute_statistic(cam_pos::WorldPoint, cam_rot::Rotations.RotZYX, pts::AbstractVector{<:WorldPoint},
                           z::AbstractVector{<:ProjectionPoint}, Σ::Union{Symmetric, Diagonal})
    # TODO maybe divide r here by sigma?
	#(r'*Diag(sig, sig)) * (Diag(sig, sig) * r)
	#r' * Σ * r
	# r' * L * U * r
	# U\r
	#Σ = Diagonal(repeat(sigmas.^2, outer=length(pts)))
	L, U = cholesky(Σ)
    r′ = compute_residual(cam_pos, cam_rot, pts, z)
	r = U'\r′
    n = 2*length(pts)
    m = 2*3 # minimum number of points to solve the problem
    # @show r'r
    cdf(Chisq(n - m), r'r)
end



# ╔═╡ 49ac8a61-f967-4544-83a3-1f17406d510b
function estimate_pose(pts::AbstractVector{<:WorldPoint}, zs::AbstractVector{<:ProjectionPoint};
                       Σ=I, initial_guess=[-6000, 0, 500, 0, 0, 0])
    function loss(cam_pos_rot, (; pts, zs, Σ))
        cam_pos = WorldPoint(cam_pos_rot[1:3]...)
		cam_rot = RotZYX(roll=cam_pos_rot[4], pitch=cam_pos_rot[5], yaw=cam_pos_rot[6])
        zs_est = [project(cam_pos, cam_rot, pt) for pt in pts]
		L, U = cholesky(Σ)
        U'\vcat((zs_est .- zs)...)
    end
    prob = NonlinearLeastSquaresProblem{false}(loss, initial_guess, (; pts, zs, Σ))
    sol = solve(prob, LevenbergMarquardt();
        abstol=5e-3,
        reltol=1e-3)
    # sol = solve(prob, TrustRegion())
    #@assert NonlinearSolve.SciMLBase.successful_retcode(sol) "$(sol.retcode), $(sol.resid)"
    return (; cam_pos=WorldPoint(sol.u[1:3]...), cam_rot=RotZYX(roll=sol.u[4], pitch=sol.u[5], yaw=sol.u[6]), sol)
end




# ╔═╡ 56e9cc5b-516b-4685-a497-11367da9dbac
function estimate_pos(pts::AbstractVector{<:WorldPoint}, zs::AbstractVector{<:ProjectionPoint}, gt_rot::Rotations.RotZYX;
                       Σ=I, initial_guess=[-6000, 0, 500])
    function loss(cam_pos_rot, (; pts, zs, Σ))
        cam_pos = WorldPoint(cam_pos_rot[1:3]...)
        cam_rot = gt_rot
        zs_est = [project(cam_pos, cam_rot, pt) for pt in pts]
		L, U = cholesky(Σ)
        U'\vcat((zs_est .- zs)...)
    end
    prob = NonlinearLeastSquaresProblem{false}(loss, initial_guess, (; pts, zs, Σ))
    sol = solve(prob, LevenbergMarquardt();
        abstol=5e-3,
        reltol=1e-3)
    # sol = solve(prob, TrustRegion())
    #@assert NonlinearSolve.SciMLBase.successful_retcode(sol) "$(sol.retcode), $(sol.resid)"
    return (; cam_pos=WorldPoint(sol.u[1:3]...), cam_rot=gt_rot, sol)
end




# ╔═╡ ffd58e7f-e101-4c81-9d53-17add3ddd979
world_pt_to_cam_pt(WorldPoint(-10,0,0), RotZYX(I), WorldPoint(0,0,0))

# ╔═╡ d954ee07-14b8-4742-b853-dccbea2760db
parsef32(x::AbstractString) = parse(Float32, replace(x, ','=>""))

# ╔═╡ 58e78698-04e8-4cc1-a5ca-d185edbdd96f
parsef32(x::Real) = convert(Float32, x)

# ╔═╡ 47948653-c83f-40a8-b96b-88b1fcd2221c
begin
	dir = Dim{:dir}([:x, :y, :z])
	corner = Dim{:corner}([:near_left, :near_right, :far_left, :far_right])
	img_dir = Dim{:img_dir}([:x, :y])
end

# ╔═╡ 0e150a3f-d15a-4aa0-8548-0ba11df4311f
# ╠═╡ disabled = true
#=╠═╡
is_available = convert.(Bool, df.availability)
  ╠═╡ =#

# ╔═╡ 614d06cd-1686-40a8-87c5-5e8c95a8cb08
# ╠═╡ disabled = true
#=╠═╡
ConfusionMatrix()(is_available, limited_error)
  ╠═╡ =#

# ╔═╡ 705863b2-f634-489e-a196-7a294761bc73
σ_correction = 1//1

# ╔═╡ 79e948ff-e782-42db-b63c-880eddec399b


# ╔═╡ 27582b37-ed60-433c-beb6-4272049c6d0a
function load_runways(runway_file=joinpath("data", "2307 A3 Reference Data_v2.xlsx"))
    XLSX.readxlsx(runway_file)["Sheet1"] |> XLSX.eachtablerow |> DataFrame
end

# ╔═╡ c7479ca3-2a39-4132-a2c4-d1d68dffdb88
all_runways = load_runways();

# ╔═╡ 92204af1-7ff7-414a-88ec-5d2e46375bb8
[rwy for rwy in all_runways.ICAO_RWY if occursin("KTUS", rwy)]

# ╔═╡ cf9e078c-558c-4161-9558-7dc254881dd8
yaw_correction(icao_rwy::AbstractString) = let
  if icao_rwy == "KTUS_30"
	  icao_rwy = "KTUS_29L"
  end
  matches = all_runways[all_runways.ICAO_RWY .== icao_rwy, "True Bearing"]
  if length(matches) == 1
	  return matches[1]
  else
	  @warn "Not found: $(icao_rwy)"
	  return 0
  end
end

# ╔═╡ 108f501e-3b01-476b-b632-a57a99085be3
df, is_available = let
	#df_ = CSV.read("./data/ConfusionMatrix_Results.csv", DataFrame)
	#df_ = CSV.read("./data/mpvs_results_60_approaches.csv", DataFrame)
	#df_ = CSV.read("./data/VLA_two_landings.csv", DataFrame)
	df_ = CSV.read("./data/VLA_TEST_v14_f0_90pct_success.csv", DataFrame)
	is_available_ = convert.(Bool, df_.availability)
    idx_something_missing = (
        ismissing.(df_.pred_kp_bottom_left_x_px) .||
        ismissing.(df_.pred_kp_bottom_left_y_px) .||
        ismissing.(df_.pred_kp_bottom_right_x_px) .||
        ismissing.(df_.pred_kp_bottom_right_y_px) .||
        ismissing.(df_.pred_kp_top_left_x_px) .||
        ismissing.(df_.pred_kp_top_left_y_px) .||
        ismissing.(df_.pred_kp_top_right_x_px) .||
        ismissing.(df_.pred_kp_top_right_y_px))
    #df = df_[is_available, :]
    df = df_[.!idx_something_missing, :]
	df.gt_yaw_deg .= parsef32.(df.gt_yaw_deg)
	df.gt_yaw_deg .-= yaw_correction.(df.airport_runway)
	is_available = is_available_[.!idx_something_missing]
	(df, is_available)
end;

# ╔═╡ 108721c7-6b9d-462c-8966-c6d75c47bae2
begin
idx_avail = findall(is_available)
idx_unavail = findall(.!is_available)
end;

# ╔═╡ c9a98058-67da-419d-9ea4-dd565c8f9977
df.airport_runway

# ╔═╡ 767c425e-8c65-4a03-999d-7823bbb020f8
df.active_runway

# ╔═╡ 4a747cda-96d9-4bec-9057-67d2b45765b6
sum(is_available)

# ╔═╡ f021f1da-fbcc-441b-8600-a7b1689b7985
nrow(df)

# ╔═╡ 9d3ceb51-6c34-4a0f-988e-06ee1e12651d
preds = DimArray(
	[df.pred_kp_bottom_left_x_px df.pred_kp_bottom_left_y_px ;;;
     df.pred_kp_bottom_right_x_px df.pred_kp_bottom_right_y_px ;;;
	 df.pred_kp_top_left_x_px df.pred_kp_top_left_y_px ;;;
	 df.pred_kp_top_right_x_px df.pred_kp_top_right_y_px] .|> parsef32,
	(i=(:), img_dir=([:x, :y]), corner=([:near_left, :near_right, :far_left, :far_right]));
	name="preds")


# ╔═╡ 81e826c0-7bb2-48ea-b8d7-1f6233f57aff
preds

# ╔═╡ 18df6509-c7c7-417d-b60a-188c88ef2b51
gts = DimArray(
	[df.gt_label_runway_bottom_left_x_px df.gt_label_runway_bottom_left_y_px ;;;
     df.gt_label_runway_bottom_right_x_px df.gt_label_runway_bottom_right_y_px ;;;
	 df.gt_label_runway_top_left_x_px df.gt_label_runway_top_left_y_px ;;;
	 df.gt_label_runway_top_right_x_px df.gt_label_runway_top_right_y_px] .|> parsef32,
	(i=(:), img_dir=([:x, :y]), corner=([:near_left, :near_right, :far_left, :far_right]));
	name="gts")

# ╔═╡ fb577f71-ae98-420e-a9b0-b746c98fe082
errors = @d preds .- gts name=:errors

# ╔═╡ 9b2868c9-5f93-46cf-ae27-39107d735fdb
limited_error = limited_errors = [all(abs.(slice) .< 300) for slice in eachslice(errors; dims=:i)]

# ╔═╡ fc24fc4c-7c98-44c4-b91b-3ee9de33abb0
median(errors)

# ╔═╡ cebc069f-6cda-46d3-90d0-441977444e7f
filter_idx = Where(i->all(errors[i=i] .<= 1_000))
#filter_idx = findall(df.recording_uq .== "FT216_4")

# ╔═╡ c1f29135-2a04-42c0-92b1-92ce8020810f
let
  # plt = data(errors[i=is_available])
 # plt = data(errors[i=filter_idx])
  plt = data(errors[i=limited_errors])
  plt *= mapping(:errors=>"errors [pxl]", color=:img_dir, layout=:corner)
  plt *= (
	 #histogram(normalization=:pdf, bins=50) * visual(alpha=0.5) +
	 #density(; npoints=200, datalimits=(-20)) +
     density()
  )
  draw(plt;
       figure=(; size=(800, 800)),
       facet=(; linkxaxes=:none, linkyaxes=:none),
	   #axis=(; limits=(-50, 50, nothing, nothing))
	   axis=(;
	   #xminorticks=IntervalsBetween(4),
	   #limits=(-15, 15, nothing, nothing),
	   xminorticksvisible = true, xminorgridvisible = true),
  )
end

# ╔═╡ e57d58de-6152-4ef9-8cbe-f21cfc5954b9
std(errors; dims=:i)[1,:,:]

# ╔═╡ fbaba5c1-07e6-4a85-9e86-6e87fe244094
std(errors; dims=:i)[1,:,:] |> mean

# ╔═╡ 84b5eab2-8e83-4858-9fe0-614f921ae672
quantile.(eachslice(errors; dims=(:img_dir, :corner)), [0.05]),
quantile.(eachslice(errors; dims=(:img_dir, :corner)), [0.95])

# ╔═╡ 759dd20a-12d3-4576-b3d5-ec3366705bc6
let df = DimTable(errors[i=filter_idx, img_dir=At(:y)];
                  layersfrom=:corner) |> DataFrame
	df_normalized = mapcols(df[!, Not(:i)]) do col; col ./ std(col); end
	pairplot(df_normalized)
end

# ╔═╡ 63d18b43-4a82-4841-aa1b-4064f3e62c94
let df_x = DimTable(errors[img_dir=At(:x)];
                    layersfrom=:corner) |> DataFrame,
    df_y = DimTable(errors[img_dir=At(:y)];
                    layersfrom=:corner) |> DataFrame

	df_x_normalized = mapcols(df_x[!, Not(:i)]) do col; col ./ std(col); end
	df_y_normalized = mapcols(df_y[!, Not(:i)]) do col; col ./ std(col); end

	pairplot(df_x_normalized, df_y_normalized)
end

# ╔═╡ 645d7f1d-1cc9-4455-ad98-0c638236fac9
mean(errors[i=is_available].^2) |> sqrt

# ╔═╡ 74f305e9-50e9-4314-832f-fe98c565125c
var_preds = DimArray(
	[df.pred_kp_uncertainty_bottom_left_xx_px2 df.pred_kp_uncertainty_bottom_left_yy_px2 ;;;
	 df.pred_kp_uncertainty_bottom_right_xx_px2 df.pred_kp_uncertainty_bottom_right_yy_px2 ;;;
	 df.pred_kp_uncertainty_top_left_xx_px2 df.pred_kp_uncertainty_top_left_yy_px2 ;;;
	 df.pred_kp_uncertainty_top_right_xx_px2 df.pred_kp_uncertainty_top_right_yy_px2] .|> parsef32,
	(i=(:), img_dir=([:x, :y]), corner=([:near_left, :near_right, :far_left, :far_right]));
	name="vars")

# ╔═╡ 394906d4-33b5-4b6b-9aec-0a5c01ac43b6
zvals = @d errors ./ (σ_correction .* sqrt.(var_preds)) name=:zvals

# ╔═╡ eb911259-3d35-4efe-aa5c-4c759210dd7d
let
	plt = data(
		zvals[i=filter_idx]
	) * mapping(
		:zvals, color=:img_dir, layout=:corner
	) * density()
	fig = draw(plt)
	for i in 1:2, j in 1:2
		lines!(fig.figure[i,j], -3:0.01:3, x->pdf(Normal(), x); color=:red)
	end
	Label(fig.figure[0,1:2]; text="z-values with correction $(σ_correction)", tellwidth=false, font=:bold)
	fig
end

# ╔═╡ 9a8d2623-79d0-41cf-9f96-eb83f4c4f7fe
Markdown.parse("""
!!! warning "Fraction of samples with less than 1_000 pixels of error:"
    $(size(zvals[i=Where(i->all(errors[i=i] .<= 1_000))], :i)) / $(size(zvals, :i)).
""")

# ╔═╡ b0319e8e-9c2b-495b-b4ed-9ebd1dc5cee2
let
	preds = [
		MvNormal(collect(μs), Diagonal(collect(vars)))
		for (μs, vars) in zip(
			eachrow(mergedims(preds, (:img_dir, :corner)=>:corner_flat)),
			eachrow(mergedims(σ_correction^2 .* var_preds, (:img_dir, :corner)=>:corner_flat))
		)
	]
	gts = collect.(
		eachrow(mergedims(gts, (:img_dir, :corner)=>:corner_flat))
	)
	pvals, calibrationvals = computecalibration(preds[i=filter_idx], gts[i=filter_idx])
	fig = lines(pvals, calibrationvals;
				linewidth=5,
				axis=(; xlabel="Predicted coverage", ylabel="Empirical coverage",
				        title="Calibration with correction $(σ_correction)"),
			   figure=(; fontsize=25))
	lines!(fig.figure[1,1], 0:1, 0:1; linestyle=:dash)
	fig
end

# ╔═╡ a716eba0-4b2a-4d43-a082-9dcff54d68fb
let chosen_img_dir=:x, chosen_corner=:far_right
	preds = [
		Normal(μ, sqrt(var)) for (μ, var) in zip(
			mergedims(preds[i=filter_idx,
							img_dir=Where(==(chosen_img_dir)),
							corner=Where(==(chosen_corner))],
					  (:i, :img_dir, :corner)=>:corner_flat),
			mergedims(σ_correction^2 .* var_preds[i=filter_idx,
												  img_dir=Where(==(chosen_img_dir)),
												  corner=Where(==(chosen_corner))],
					  (:i, :img_dir, :corner)=>:corner_flat)
		)
	]
	gts = mergedims(gts[i=filter_idx,
						img_dir=Where(==(chosen_img_dir)),
						corner=Where(==(chosen_corner))],
					(:i, :img_dir, :corner)=>:corner_flat)
	pvals, calibrationvals = computecalibration(preds, gts)
	fig = lines(pvals, calibrationvals;
				linewidth=5,
				axis=(; xlabel="Predicted coverage", ylabel="Empirical coverage",
				        title="Flattened calibration with correction $(σ_correction), $(chosen_img_dir), $(chosen_corner)"),
			   figure=(; fontsize=20))
	lines!(fig.figure[1,1], 0:1, 0:1; linestyle=:dash)
	fig
end

# ╔═╡ b68c8641-836d-4679-82bd-4af9e0b656b4
extrema(var_preds[i=filter_idx])

# ╔═╡ ba191996-f543-4214-a0b2-19549a7fd719
var_preds

# ╔═╡ be47de88-964a-4b3a-922d-cd50d8fdd2a0
sum(df.recording_uq .== "FT113_22")

# ╔═╡ b2f10478-b059-4d89-8e41-6b75c076b471
let
	σ_xx = sqrt.(parsef32.(df.pred_kp_uncertainty_top_right_xx_px2))
	hist(σ_xx[σ_xx .< 100])
end

# ╔═╡ 45dc6a3a-2f0e-4594-bb5b-4becd9db26ad
names(df)

# ╔═╡ 1a40bc77-3575-4d3f-a268-06898baac76a
let df = DimTable(errors[i=is_available, img_dir=At(:x)];
                  layersfrom=:corner) |> DataFrame
	df_normalized = mapcols(df[!, Not(:i)]) do col; col ./ std(col); end
	pairplot(df_normalized)
end

# ╔═╡ b7cef624-b4e3-4eaf-a40e-9990bda0d076
n_preds = sum(is_available)
#n_preds=500

# ╔═╡ 20e591ad-38a4-4771-b954-50f4447b54e6
#idx = findall(is_available .&& limited_errors)[1:n_preds]
idx = findall(is_available)[1:n_preds]
#idx = 1:n_preds

# ╔═╡ 14a5b1a3-32b4-4d1b-923f-5031ea49ffd8
global_sigmas = 0.5

# ╔═╡ 681e8abf-e233-4679-81d4-608bdec3f082
df.flight_test |> unique

# ╔═╡ 884e7926-f07f-4b2f-9269-f77165a6e6ba
names(df)

# ╔═╡ 39415484-8169-470b-8d59-b76a2e33f18f
pos_gts = [
	WorldPoint(row.gt_along_track_distance_m |> parsef32,
	           row.gt_cross_track_distance_m |> parsef32,
	           row.gt_height_m |> parsef32)
	for row in eachrow(df)
]

# ╔═╡ e6359781-af8a-46e6-b2c4-b09d23b69462
pos_preds_theirs = hcat(
	(df.pred_along_track_distance_m .|> parsef32),
	(df.pred_cross_track_distance_m .|> parsef32),
	(df.pred_height_m .|> parsef32)) |> eachrow;

# ╔═╡ 7700edaa-82bb-4655-92b9-70b7863b8056
# ╠═╡ disabled = true
#=╠═╡
rot_preds = getfield.(pose_preds, :cam_rot)
  ╠═╡ =#

# ╔═╡ f64570bc-c026-4328-831b-85a9495db12e
rot_gts_deg = eachrow([parsef32.(df.gt_yaw_deg) parsef32.(df.gt_pitch_deg) parsef32.(df.gt_roll_deg)])

# ╔═╡ be58a503-1f71-406d-bbeb-4ac2898e9e78
begin
yaw_deg(R::RotZYX) = -Rotations.params(R)[1] |> rad2deg
pitch_deg(R::RotZYX) = -Rotations.params(R)[2] |> rad2deg
roll_deg(R::RotZYX) = Rotations.params(R)[3] |> rad2deg
end

# ╔═╡ 732e0c99-cae1-4661-9f24-bd2cc9d21448
names(df)

# ╔═╡ f6cfd372-74ea-48e9-a1f0-9df661176f3d
reverse_stat(p) = let
	n = 2*4 # length(pts)
	m = 2*3
    quantile(Chisq(n - m), p)
end

# ╔═╡ a8b98efa-05f5-4cf6-9333-e5d26556b784
filter(x->occursin("confusion", x), names(df)) |> collect

# ╔═╡ c455180c-ff3f-4728-aecf-f55d58a9ae3c
"""
    visualize_runway_corners(df::DataFrame, global_sigma::Real; row_idx=1, fig=nothing, ax=nothing)

Visualizes runway corners from a DataFrame row with uncertainty ellipses using Makie.

Args:
    df (DataFrame): DataFrame containing corner coordinates.
                    Expects columns like 'gt_label_runway_bottom_left_x_px', etc.
    global_sigma (Real): Standard deviation for x and y uncertainty (radius of circle).
    row_idx (Int): Index of the row in the DataFrame to visualize.
    fig (Figure): Optional existing Makie Figure.
    ax (Axis): Optional existing Makie Axis.

Returns:
    Tuple{Figure, Axis}: The Makie Figure and Axis used for plotting.
"""
function visualize_runway_corners_with_zoom(
    df,
	proj_preds,
    global_sigma::Real;
    row_idx=1,
    margin_factor=0.1,
    fig=nothing
    )

	CAM_WIDTH_PX = 4096
    CAM_HEIGHT_PX = 3000

    # Extract corner coordinates for the specified row
    row = df[row_idx, :]
    corners_gt = Point2f[
        parsef32.((row.gt_label_runway_bottom_left_x_px, row.gt_label_runway_bottom_left_y_px)),
        parsef32.((row.gt_label_runway_bottom_right_x_px, row.gt_label_runway_bottom_right_y_px)),
        parsef32.((row.gt_label_runway_top_right_x_px, row.gt_label_runway_top_right_y_px)),
        parsef32.((row.gt_label_runway_top_left_x_px, row.gt_label_runway_top_left_y_px))
    ]
	corners_theirs = Point2f[
		[row.pred_kp_bottom_left_x_px,row.pred_kp_bottom_left_y_px] .|> parsef32,
		[row.pred_kp_bottom_right_x_px,row.pred_kp_bottom_right_y_px] .|> parsef32,
		[row.pred_kp_top_left_x_px,row.pred_kp_top_left_y_px] .|> parsef32,
		[row.pred_kp_top_right_x_px,row.pred_kp_top_right_y_px] .|> parsef32,
    ][[1, 2, 4, 3]]
	corners_ours = Point2f.(proj_preds[row_idx][[1,2,4,3]])
    corners_ours = [Point2f(-x, -y) for (x, y) in corners_ours] .+ [Point2f(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]

    # Create figure if not provided
    if isnothing(fig)
        fig = Figure(size = (1200, 500)) # Adjust size for two plots
    end

    # --- Axis 1: Overview ---
    ax1 = Axis(fig[1, 1],
               aspect = DataAspect(),
               title = "Overview (Row $row_idx)",
               limits = (0, CAM_WIDTH_PX, 0, CAM_HEIGHT_PX), # Set limits directly
               yreversed = true)

    # --- Axis 2: Zoomed View ---
    # Calculate bounding box of corners
    xs = [c[1] for c in corners_theirs]
    ys = [c[2] for c in corners_theirs]
    min_x, max_x = extrema(xs)
    min_y, max_y = extrema(ys)

    # Calculate margins
    bbox_width = max_x - min_x
    bbox_height = max_y - min_y
    margin_x = bbox_width * margin_factor
    margin_y = bbox_height * margin_factor

    # Ensure margins are not zero if corners align
    margin_x = max(margin_x, global_sigma * 5) # At least show full ellipse
    margin_y = max(margin_y, global_sigma * 5)

    # Calculate zoom limits
    zoom_xlim = (min_x - margin_x, max_x + margin_x)
    zoom_ylim = (min_y - margin_y, max_y + margin_y)

    ax2 = Axis(fig[1, 2],
               aspect = DataAspect(),
               title = "Zoomed View",
               limits = (zoom_xlim..., zoom_ylim...), # Apply calculated limits
               yreversed = true)

	zoom_lines!(ax1, ax2)

    # --- Plotting on both axes ---
    n_ellipse_points = 50
    t = range(0, 2π, length=n_ellipse_points)

    for ax in [ax1, ax2]
        # Plot corner points
		lines!(ax, [corners_gt..., corners_gt[1]], color=:red, label=nothing)
        scatter!(ax, corners_gt, color=:red, markersize=8, label="Corners gt")

		lines!(ax, [corners_theirs..., corners_theirs[1]], color=:green, label=nothing)
        scatter!(ax, corners_theirs, color=:green, markersize=8, label="Corners theirs")

		lines!(ax, [corners_ours..., corners_ours[1]], color=:blue, label=nothing)
        scatter!(ax, corners_ours, color=:blue, markersize=8, label="Corners ours")

        # Plot uncertainty ellipses (circles)
        for (i, corner) in enumerate(corners_theirs)
            ellipse_x = corner[1] .+ global_sigma .* cos.(t)
            ellipse_y = corner[2] .+ global_sigma .* sin.(t)
            lines!(ax, ellipse_x, ellipse_y, color=:green, linewidth=1.5,
                   label = (i==1 && ax==ax1 ? "Uncertainty (σ=$global_sigma)" : nothing) # Label only once on ax1
                  )
        end
    end
    # Optional: Add legend to the first axis
    axislegend(ax1)

    return fig
end


# ╔═╡ 4c183314-0765-4fc5-8a2b-1005f3974d99
@bind CONFUSION_MATRIX_PLT_TYPE PlutoUI.Select(["TRUE_POSITIVE", "TRUE_NEGATIVE", "FALSE_POSITIVE_KEYPOINT_BOTTOM", "FALSE_POSITIVE_KEYPOINT_TOP", "FALSE_POSITIVE_IOU", "FALSE_POSITIVE_EDGE", "FALSE_POSITIVE_EDGE_MID"])

# ╔═╡ ab07a5ec-af45-45b1-b30d-efff964c6eca
begin
confusionsorter = sorter(["TRUE_POSITIVE", "TRUE_NEGATIVE", "FALSE_POSITIVE_KEYPOINT_BOTTOM", "FALSE_POSITIVE_KEYPOINT_TOP", "FALSE_POSITIVE_IOU", "FALSE_POSITIVE_EDGE", "FALSE_POSITIVE_EDGE_MID"])
colors=splat(Pair).(zip(
	confusionsorter.(["TRUE_POSITIVE", "TRUE_NEGATIVE", "FALSE_POSITIVE_KEYPOINT_BOTTOM", "FALSE_POSITIVE_KEYPOINT_TOP", "FALSE_POSITIVE_IOU", "FALSE_POSITIVE_EDGE", "FALSE_POSITIVE_EDGE_MID"]),
    Makie.ColorSchemes.tab10))
end

# ╔═╡ 8d00d6f9-22f6-451d-86c4-ac0e7ba112a2
load_runways();

# ╔═╡ c6c45d35-bf9a-4dae-99c8-44a583eadf53
yaw_corrections = map(eachrow(df)) do row
	yaw_correction(row.airport_runway)
end

# ╔═╡ 8f571bdf-63b3-494a-8a0c-afb157b5145a
df.recording_uq |> unique

# ╔═╡ 95e07d3f-fb66-484b-9ab0-3df3164cb228
n = 2 * 4; m = 2 * 3

# ╔═╡ f4706e8a-d63d-48f9-b59a-50227ece8c92
type_ = df.confusion_matrix_value

# ╔═╡ be243081-c4c8-4177-876a-f46b78322ba5
"Number of points to be flagged: $(sum(type_ .== :confusion_matrix_False_Positive))"

# ╔═╡ 84869098-ab36-40b7-a170-c0f8f6828c30


# ╔═╡ 43dfd472-d2c8-4c48-b356-12b2e543d57e
FP_indices = findall(df.confusion_matrix_value .== "False Positive")

# ╔═╡ e1c252c5-1584-4990-80a5-33f1960613e8
let row = eachrow(df)[13]
	[
    parsef32(row.gt_label_runway_top_left_x_px)-parsef32(row.pred_kp_top_left_x_px);
    parsef32(row.gt_label_runway_top_right_x_px)-parsef32(row.pred_kp_top_right_x_px);
    parsef32(row.gt_label_runway_bottom_left_x_px)-parsef32(row.pred_kp_bottom_left_x_px);
    parsef32(row.gt_label_runway_bottom_right_x_px)-parsef32(row.pred_kp_bottom_right_x_px)
    ]
end

# ╔═╡ d8b0b31a-6308-4c72-9694-5c650d781bfc
df.filtered_along_track_distance_m

# ╔═╡ e162e99e-2c72-4ae0-a164-2d17482d33f3
MvNormal

# ╔═╡ 55ccafe6-f4fb-48a9-9d11-2d3043e4ffcd
Base.getindex(D::Distributions.MvNormal, idx::UnitRange) = MvNormal(mean(D)[idx], cov(D)[idx, idx])

# ╔═╡ 57fe6f63-0453-4d0f-81fa-f3be2a1d64e0
Base.getindex(D::Distributions.MvNormal, idx::Integer) = Normal(mean(D)[idx], sqrt(cov(D)[idx, idx]))

# ╔═╡ ba932239-3587-4ab7-a994-bf9d852d8c37
md"""
# Abstract: Integrity Analysis on Real Data
In this notebook we showcase the RAIM algorithm using real data in a autonomous landing scenario.
We (re)implement the RAIM equations and a basic pose estimation framework.
Then we consider a dataset of real approaches.

In the first part, we explore the dataset, visualizing error distributions and correlations.
In the second part we apply RAIM to the dataset and explore the resulting statistics.

We note that this is a follow-up notebook to the "integrity_mwp" notebook, which details the RAIM implementation in more detail.

This work was done as part of the $A^3$ <> Stanford SISL collaboration during Fall 2024, by Romeo Valentin (romeov@stanford.edu) and Sydney Katz (smkatz@stanford.edu).
"""

# ╔═╡ 09bd158d-9a3f-4e6c-9ad3-75194d4dcb53
md"""
## Running this notebook yourself
To reproduce this notebook, install julia [1], and the Pluto.jl [2] package.
Then run Pluto, and use it to open this notebook. All other package management will be taken care of for you by Pluto.

[1]: [https://julialang.org/download]()
[2]: [https://plutojl.org/#install]()
"""

# ╔═╡ 1c63c8ac-ec6d-48cf-b249-611762ac9c31
md"# Table of contents"; PlutoUI.TableOfContents()

# ╔═╡ 3dd13658-8cd0-11ef-1494-3bb9f1532797
md"""
# Building an integrity metric.

!!! info "This section is better explored in the previous notebook `integrity_mwp`

Let's consider

$y = H x + v + f$

-  $y$: Computer vision output
-  $v$: Predicted noise
-  $f$: Fault vector (in only a  few elements)
-  $H$: Jacobian of projection matrix
"""

# ╔═╡ 17874c41-59cb-44e8-b131-3347aa6c97d2
md"""
!!! info "This section is better explored in the previous notebook `integrity_mwp`"
    Here we will only put the code, unstructured.
"""

# ╔═╡ 8fcd3665-e7c2-4dbb-acf3-d5a72b7dae7a
md"""
We will start with some basic functionality, such as some coordinate system definitions and transformations, and a projection function.

We also include a small least squares solver to estimate our position based on minimize image reprojection error.

Then we compute the jacobian of the projection, $H$, and use that to compute our residual metric.

Finally, we derive the test statistic, and run a small simulation study.
"""

# ╔═╡ 9b567edd-027a-447b-8c21-01da9116f13c
md"# Exploring the dataset"

# ╔═╡ f08f9dfa-46bd-4638-bd5b-0e35ab06c16d
md"## Loading the data"

# ╔═╡ 3dbb6033-9b86-4aa8-a690-954a26714aa8
md"## Preprocess data"

# ╔═╡ 19209391-89d3-4b9d-9184-aafcc6be28db
md"### Extract predictions and ground truth pixel values"

# ╔═╡ 0fa22457-56c8-44fa-abe5-11ddd56768de
md"""
## Availability and limited errors
The dataset includes an "availability" binary flag for each sample.
However, for the "available" samples we find that some errors are unaccetably large (and mess up our plotting). So we also introduce a subset of samples with `limited_errors` where the pixel error is less than 50.
"""

# ╔═╡ b8f78292-e219-46a7-b87d-55676b480806
md"""
Let's investigate if the availability and limited errors agree with each other.
Here, "true" means "available", and vice versa.
We consider the `avaiability` feature to be the prediction, and our `limited error` metric to be the ground truth.
"""

# ╔═╡ 06263122-84f9-4f8a-b3c5-53997c98c033
md"""
We can see that in fact the `availability` metric manages to flag $1840/1898$ features, i.e. about $97\%$.
"""

# ╔═╡ 23b5c76e-9c61-4972-a859-ffae4503c7ef
md"## Visualizing prediction errors"

# ╔═╡ af58639a-e090-4db2-87e7-00ae44f85430
md"""
We can see that most errors are between -10 and 10 pixels, and that the error distributions are all looking similar.
"""

# ╔═╡ 2d3a7b83-0f84-4232-973f-0a7b984b5a57
md"""
## Checking out the predicted variances
"""

# ╔═╡ 9e6bf632-e7c8-4641-abb0-bcc66f678a95
md"### Calibration of position"

# ╔═╡ a3910a7f-498a-4f95-b24c-ae3cb055b0f2
md"""
### Error distribution
Let's compute the standard deviation of the errors
"""

# ╔═╡ fd846d1c-d129-4848-924a-bab632d15e81
md"and the average of all of those"

# ╔═╡ d3664410-bff8-409d-8253-76af5769bddb
md"as well as the 5th and 95th percentiles"

# ╔═╡ 97838f16-2d86-46b3-b76a-889ac12b300a
md"It looks like we can assume a standard pixel error of five pixels for our predictions."

# ╔═╡ a1892357-7cb9-4465-90bb-56de853e7f8d
md"""
## Error correlations
We are able to see that errors are reasonably uncorrelated, except for perhaps the x/x near left and near right, and same for y/y.
"""

# ╔═╡ 57092346-8b5f-49b8-9568-5593eccceabd
md"### projection errors left/right"

# ╔═╡ be7699cb-e5be-46c0-9e90-242a52e062c7
md"### projection errors up/down"

# ╔═╡ 72ce8d8a-e061-4f1e-aace-3793b63970fa
md"### Let's see correlations between x and y also"

# ╔═╡ 292d8301-9c96-4d28-8914-827358b3d67e
md"""
# Making pose predictions
Let's now start actually making some pose predictions, and then trying to apply RAIM.
"""

# ╔═╡ cdfa16b9-a97a-4de1-b235-058c7cc13e82
md"""
Let's for now pick indices which are "available" and don't have huge error.
"""

# ╔═╡ fb8f58e7-8891-493f-91c0-6b4e3e8d085c
md"""
Let's compute our own pose predictions. We need to
- setup the runway (assuming just a rectangle with to altitude change)
- translate the coordinate system of the pixels (using the known camera pixel size)
- decide on a standard pixel error
- run our pose estimation
Notice that we have already coded in the camera focal length and pixel size into our projection function at the top of this file.
"""

# ╔═╡ ef81c3a7-12cf-4105-abb1-bfd87d1402e6
md"""
### Loading runway data
"""

# ╔═╡ a24f68e3-4243-4fdf-90e3-1b521763b979
md"""
## Pose preds 3dof
"""

# ╔═╡ 30844016-07e1-4a9f-b0ef-e4d06b48c0ba
md"""
## Projection norm comparison
"""

# ╔═╡ 5f0c3c39-b126-44ac-89d8-9cfbbd3e23d4
md"""
## Visualizing pose preds

We can now try to compare our pose estimations against the ground truth.
"""

# ╔═╡ 81ad4103-313b-4977-8933-2a070fdd861a
md"""
### Position predictions
We first overlay our position predictions over the ground truth values reported in the dataframe.
"""

# ╔═╡ d06e78be-a015-456e-9ef3-93bddfa11ac1
getidx(idx...) = arr->getindex(arr, idx...)

# ╔═╡ 7571c77a-64d2-4492-8dd2-4652dba94985
md"""
We can see that the general trends are correct.
However, there seems to be a constant offset error of about 6km for the alongtrack distance, and the altitude is also significantly off.
"""

# ╔═╡ 9aa886a4-fb33-4cac-8292-a4ad404fc742
md"""
### Rotation predictions
"""

# ╔═╡ db765d77-c213-4d51-9c6b-a2a689a5d0dc
md"""
We visualize pitch and yaw from the dataset, and for our predictions.
Notably, we need to invert the sign of one of the two, since we have an inverted "sense of rotation".
"""

# ╔═╡ 72188aa8-1e12-4559-8b6f-984bbdf6b4be
md"""
Again we can see that the trajectory looks quite similar, but is off by a constant offset in both directions.
Indeed, their "yaw" is huge all the time. We estimate a much more moderate yaw.
"""

# ╔═╡ a6433067-bfe5-4d79-a125-28a8df6cd1c6
md"""
requirement for GPS: 1e-3 misdetection
how to define a bad measurement:
  - dynamic UQ undervalued by more than $1\sigma$
"""

# ╔═╡ 9d128ec0-440b-49ba-8514-7f31214fb61f
md"""
## Plot stats!
"""

# ╔═╡ 99abcd76-02a7-4068-82bb-fcb3e15277e2
md"""
Let's look at the result.
Recall that the distribution of stats relies on how we set `global_sigma`.
"""

# ╔═╡ 87deb7c7-5573-443f-9905-861fc8da1b07
md"""
Positive -> We think there is a runway that we can infer
  -> our prediction may be true or false
Negative -> There is no runway
  ->our prediction may be true or false


KP error (or edge error) -> 15 pixels
"""

# ╔═╡ 07ac85fa-8050-496e-817e-633b4c6ed0c1
md"Plot only available: $(@bind plot_only_available CheckBox(default=false))"

# ╔═╡ 40a46895-ff21-4aba-a905-109ed292510e
md"## Error visualization"

# ╔═╡ d4b1cfde-fa64-4233-b2df-35b000e49bdf
md"Data subindex: $(@bind sl_keypoint PlutoUI.NumberField(1:sum(df.confusion_matrix_value.==CONFUSION_MATRIX_PLT_TYPE)))"

# ╔═╡ 41c54145-db34-4274-966d-555c97fc3dfe
md"Confusion matrix"

# ╔═╡ f81f6528-0dd7-49ed-bdfd-023a93f7de74
md"""Or another way:"""

# ╔═╡ 7dc3864d-7b45-480b-b997-246eaecd9d95
md"""
### Plotting relative errors and whether we flag
"""

# ╔═╡ 2efcbcad-2fc8-454b-b33a-9da3bcfb4bc3
md"""
TODO: also plot the rotations so we can see why we flag a bunch of these points.
"""

# ╔═╡ f176f523-8ba7-4fae-9078-7eab58070675
md"""
TODO: Filter out high angular rate cases
-> e.g. finite difference on yaw
The hypothesis here is that the ground truth is lagging because of the EKF, and the measurements are actually correct.
""";

# ╔═╡ 5761a668-e49e-4284-ab20-a0cf339c4e67
md"""
As we can notice, some of the outliers don't get flagged! Why not?
"""

# ╔═╡ bd790498-0168-4b7d-8d39-83b237b21fbe
md"""
Let's look e.g. at index 13. It has large crosstrack error, but is not flagged.
We find that the x prediction errors are highly correlated! So in this case, RAIM has no chance!
"""

# ╔═╡ 641d7f45-1681-41b9-822f-76b2aa9fb4b8
md"""
Otherwise, it is perhaps justified to say that we should not flag any other points, because actually the predictions are quite accurate.
"""

# ╔═╡ 802c7cba-1e84-43b9-8fe4-65fa0bcd02f5
 md"## KF Approach"

# ╔═╡ d219e732-a9ee-4504-ae47-a7b5c2efffe2
md"# Compute uncertainty"

# ╔═╡ dccf4cce-d3f4-40e9-a09d-880a1155e1e9
issomething(x) = !isnothing(x)

# ╔═╡ 354a0869-1d9e-44c0-a942-ed6765b57c0d
gt_pos = map(eachrow(df)) do row
	[row.gt_along_track_distance_m,
		      row.gt_cross_track_distance_m,
		      row.gt_height_m] .|> parsef32 .|> x->convert(Float64, x)
end

# ╔═╡ 64d35b92-b6da-48eb-8395-112ccab1d202
length(gt_pos)

# ╔═╡ fb8a439c-6f42-4b91-b71a-5c9d2922fadf
md"# Some clarifications"

# ╔═╡ f8d20369-2233-4397-8a84-5a24ec5f2345
md"## RotXYZ, RotZYX, roll, pitch, yaw, etc"

# ╔═╡ 7d2bbf84-aab6-4bda-8bcd-3a073be1753f
RotXYZ(1, 2, 3) == RotXYZ(roll=1, pitch=2, yaw=3)

# ╔═╡ 3bb6cdc4-5706-4845-a036-14a346f7f125
RotZYX(3, 2, 1) == RotZYX(roll=1, pitch=2, yaw=3)
# this is because roll always X, pitch always Y, yaw always Z.
# the question is just how to construct the rotation matrix.

# ╔═╡ 540e28e7-f216-4eb9-8c29-80d87f4166ef
RotXYZ(3, 2, 1) == RotXYZ(yaw=1, pitch=2, roll=3)

# ╔═╡ 30a6009e-bd92-4bfe-a79c-a6ae0fd2fa05
# this is of course all in rad
RotXYZ(pi, pi, pi) ≈ I(3)

# ╔═╡ ccc65d37-0172-4a43-80a6-b0924651b2ce
RotXYZ(pi, pi, pi)

# ╔═╡ 49c7a512-5188-4855-946f-d9d852cc7f07
RotZYX(0, 0, pi/2) * [1;0;0]
# this is because we first rotate about X, then 0 rotation about Y and Z

# ╔═╡ b084a8e3-83b5-476e-a0da-9f0833707998
τ=2π

# ╔═╡ 0cffbb9e-551d-4bee-bf75-ff8392f3bfdc
md"""## A quick refresher on coordinate system, yaw-pitch-roll, and airplane applications

We need some way to define the rotations that we do. Broadly speaking, there's two choices to make: intrinsic vs extrinsic, and order of axes rotations.

In aerospace, we typicall do yaw-pitch-roll intrinsic rotation.
Here, X=roll, Y=pitch, and Z=yaw. Thus, we apply first yaw (about Z), then pitch (about Y), then roll (about X).
However, crucially we don't always rotate about the world frame axes. Instead, after doing our first rotation, we consider a "new" coordinate frame in the airplane, and then rotate about pitch. Same again for yaw.
Thus, this is an "intrinsic" rotation.

We want such an intrinsic rotation, and the `RotZYX` function implements that.
"""

# ╔═╡ 563884d3-55b8-47cd-bde9-f37394076477
md"""
Although `RotZYX` mirrors the "correct" roll-pitch-yaw structure, we must be careful when retrieving the correct order of roll, pitch, yaw using `Rotations.params`:
```julia
yaw, pitch, roll = Rotations.params(RotZYX(rand(3)...))
```
This is because `params` just gives back `theta1, theta2, theta3` with no regard for the order.
`theta3` is always what we rotate about first. Here, we rotate about `X` first, i.e. roll, so we get that back last. Conversely, `theta1` is what we rotate about last, so we get it back first.
"""

# ╔═╡ ac87ed4d-6cf6-4b14-8c74-dde316b751ed
yaw, pitch, roll = Rotations.params(RotZYX(roll=1, pitch=2.0, yaw=3.0))

# ╔═╡ 4fb8fe27-1dfb-46e3-b9b3-bbc7272c2282
RotZYX(Rotations.params(RotZYX(1, 2, 3))...) == RotZYX(1, 2, 3)

# ╔═╡ 79a6dcfa-cffb-4257-aa5b-0e216488e026
df.all_errors_within_allowance

# ╔═╡ ebcf941d-2521-4a7c-aa6b-42559b54cccc
load_runways()

# ╔═╡ e085258f-84a9-46f8-ae06-2844d9d204cc
function get_far_threshold_elevation_change(all_runways, icao_rwy)
	m = match(r"([a-zA-Z]{4})_([0-9]{2})([LR]?)", icao_rwy)
	bearing = parse(Int, m.captures[2])
	bearing_op = mod(bearing+18, 36)
	icao_rwy_op = m.captures[1]*"_"*string(bearing_op)*(m.captures[3] == "" ? "" : (m.captures[3] == "L" ? "R" : "L"))

	near_row = all_runways[all_runways.ICAO_RWY.==icao_rwy, "THR Elev"]
	if !(length(near_row)==1)
		@warn "Couldn't find unique height delta for runway $(icao_rwy)." maxlog=1 _id=icao_rwy
		return 0
	end
	elev_near = near_row[1] |> parsef32
	far_row = all_runways[all_runways.ICAO_RWY.==icao_rwy_op, "THR Elev"]
	if !(length(far_row)==1)
		@warn "Couldn't find unique height delta for runway $(icao_rwy)." maxlog=1
	return 0
	end
	elev_far = far_row[1] |> parsef32
	return elev_far - elev_near
end

# ╔═╡ a99535e7-7add-413f-8faa-dbda72e7c111
#pose_preds = tmap(eachrow(df)[collect(limited_errors)]) do row
pose_preds = tmap(eachrow(df)) do row
	rwy_w = row.active_runway_width_m |> parsef32
	rwy_l = row.active_runway_length_m |> parsef32

	Δh = get_far_threshold_elevation_change(all_runways, row.airport_runway)
	h = 0
    pts = WorldPoint[
        [0, rwy_w/2, h],
        [0,  -rwy_w/2, h],
        [rwy_l, rwy_w/2, h+Δh],
        [rwy_l, -rwy_w/2, h+Δh],
    ]

    # noise model chosen by looking at the typical error standard deviations
    sigmas = global_sigmas * ones(2)

	#optical_center_u_px:2047.5,"optical_center_v_px":1499.5,
	CAM_WIDTH_PX, CAM_HEIGHT_PX = (4096-1), (3000-1)
	# (near_left, near_right, far_left, far_right)

	ys = ProjectionPoint[
		[row.pred_kp_bottom_left_x_px,row.pred_kp_bottom_left_y_px] .|> parsef32,
		[row.pred_kp_bottom_right_x_px,row.pred_kp_bottom_right_y_px] .|> parsef32,
		[row.pred_kp_top_left_x_px,row.pred_kp_top_left_y_px] .|> parsef32,
		[row.pred_kp_top_right_x_px,row.pred_kp_top_right_y_px] .|> parsef32,
	] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]

	# ys = ProjectionPoint[
	# 	[row.gt_label_runway_bottom_left_x_px,row.gt_label_runway_bottom_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_bottom_right_x_px,row.gt_label_runway_bottom_right_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_left_x_px,row.gt_label_runway_top_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_right_x_px,row.gt_label_runway_top_right_y_px] .|> parsef32,
	# ] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]


	# ^ center coordinate system. their origin is top left
	# v flip coordinate system
	# ours points x->left, y->up. theirs points x->right, y->down.
	ys = [ProjectionPoint(-x, -y) for (x, y) in ys]
	gt_pos = [row.gt_along_track_distance_m,
		      row.gt_cross_track_distance_m,
		      row.gt_height_m] .|> parsef32 .|> x->convert(Float64, x)
	gt_rot = [row.gt_yaw_deg, row.gt_pitch_deg, row.gt_roll_deg]
	estimate_pose(pts, ys;
				  Σ=Diagonal(repeat(sigmas, outer=4).^2),
	              initial_guess = [gt_pos .+ [100; 10; 5] .* randn(3); zeros(3)]
			  )
end


# ╔═╡ ba3a2565-b587-45eb-ac79-b5da74a1137e
proj_norms6dof = map(pose_preds) do pred
	  norm(pred.sol.resid)
	end

# ╔═╡ 4adcff64-6234-4c58-be4c-d31918c6f8d0
pos_preds = getfield.(pose_preds, :cam_pos)

# ╔═╡ 8528ddec-ea1b-420f-ac72-21ec23c1fdb6
pos_preds

# ╔═╡ 04bb2b2a-cad1-44e6-84c6-5b9741c6e3bf
let
	plt = (data((; cam_pos=pos_preds)) * mapping(; color=direct(:pred)) +
	       data((; cam_pos=pos_gts)) * mapping(; color=direct(:gt)) +
	       data((; cam_pos=pos_preds_theirs)) * mapping(; color=direct(:their_pred))

	)
	plt *= mapping([:cam_pos=>getidx(1)=>"position [m]", :cam_pos=>getidx(2)=>"position [m]", :cam_pos=>getidx(3)=>"position [m]"];
	               row=AoG.dims(1)=>renamer(["alongtrack", "crosstrack", "altitude"]))
	plt *= visual(Lines)
	fig3 = draw(plt; figure=(; size=(800,400)), facet=(; linkyaxes=:none))
	lineargs = (; color=:red, linestyle=:dash)
	idx_avail = findall(is_available)
	idx_unavail = findall(.!is_available)
	for row in 1:3
		vspan!(content(fig3.figure[row, 1]), idx_avail, idx_avail.+1; color=(:green, 0.1))
		vspan!(content(fig3.figure[row, 1]), idx_unavail, idx_unavail.+1; color=(:red, 0.1))
		#vlines!(fig3.figure[1,col], FP_indices; lineargs...)
	end
	idx_limits = (1, nrow(df))
	#limits!(fig3.figure[1, 1], ylims=(-10, 10))    # alongtrack plot
    limits!(content(fig3.figure[2, 1]), idx_limits, (-200, 200))      # crosstrack plot
    limits!(content(fig3.figure[3, 1]), idx_limits, (0, 500))      # altitude plot

	fig3
end

# ╔═╡ acde1499-e1a9-41bb-8153-c9149c2cc1f3
rot_preds = getfield.(pose_preds, :cam_rot)

# ╔═╡ f48ffdc5-1349-4fb7-9035-e874c682f956
Rotations.params(rot_preds[1]) .|> rad2deg

# ╔═╡ 44a0fddc-14ae-463d-bc8b-c2b45662e37e
let i = 510
[
	Rotations.params(rot_preds[i]) .|> rad2deg;;
	getindex.([eachrow(df)[i]], [:gt_yaw_deg, :gt_pitch_deg, :gt_roll_deg])
]
end

# ╔═╡ 67667732-ff12-45d3-afbb-91a8d2e32595
rot_preds_deg = map(v->rad2deg.(v), Rotations.params.(rot_preds)) |> collect

# ╔═╡ 037a6234-0547-4bc9-b5b6-0dad84827206
let yaw_correction_ = eachrow([yaw_corrections fill(0, length(rot_gts_deg)) fill(0, length(rot_gts_deg))])
data(
	(; x=((rot_preds_deg .- rot_gts_deg) .- 0*yaw_correction_), foo=df.airport_runway)
) * mapping(
	:x=>x->getindex(x, 1),
	:x=>x->getindex(x, 1);
	marker=:foo, color=:foo
) * visual(Scatter) |> draw
end

# ╔═╡ 4a6fd421-7226-4b3e-b1e3-504a3646121c
let yaw_correction_ = eachrow([fill(0, length(rot_gts_deg)) fill(0, length(rot_gts_deg)) yaw_corrections])
	idx = findall(df.confusion_matrix_value .== "TRUE_POSITIVE")

map(v->mod.(v, 360), (rot_preds_deg .- (rot_gts_deg .- yaw_correction_)))
end

# ╔═╡ 29a802e6-3624-4b73-aa70-58c5fe68cca0
let
plt = data((; preds=rot_preds[limited_errors], i=eachindex(rot_preds[limited_errors]), is_available=limited_errors))
plt *= mapping(
	:preds=>roll_deg=> "roll [deg]",
	:preds=>pitch_deg => "pitch [deg]",
	color=:i,
	linestyle=direct("pred"))#, linestyle=direct("pred"))
plt *= visual(Lines)
plt_gt = data(df[is_available, :])
plt_gt *= mapping(
	:gt_roll_deg=>parsef32 => "roll [deg]",
	:gt_pitch_deg=>parsef32 => "pitch [deg]",
	color=direct(eachindex(eachrow(df))),
	linestyle=direct("gt"))
plt_gt *= visual(Lines)
(plt + plt_gt) |> draw
end

# ╔═╡ c91eeab7-fb43-447c-b695-fd68b3f44467
let
plt = data((; preds=rot_preds, i=eachindex(rot_preds)))
plt *= mapping(
	:preds=>yaw_deg => "yaw [deg]",
	color=:i, linestyle=direct("pred"))
plt *= visual(Lines)
plt_gt = data(df)
plt_gt *= mapping(
	:gt_yaw_deg=>parsef32 => "yaw [deg]",
	color=direct(eachindex(eachrow(df))),
	linestyle=direct("gt"))
plt_gt *= visual(Lines)
fig = draw((plt + plt_gt); figure=(; size=(1200, 600)))
vspan!(content(fig.figure[1, 1]), idx_avail, idx_avail.+1; color=(:green, 0.1))
vspan!(content(fig.figure[1, 1]), idx_unavail, idx_unavail.+1; color=(:red, 0.1))
limits!(content(fig.figure[1,1]), (1, nrow(df)), (-10, 10))
fig
end

# ╔═╡ 92b655f8-98c5-45ba-b46f-040f5e1b702e
let
plt = data((; preds=rot_preds, i=eachindex(rot_preds)))
plt *= mapping(
	:preds=>pitch_deg => "pitch [deg]",
	color=:i, linestyle=direct("pred"))
plt *= visual(Lines)
plt_gt = data(df)
plt_gt *= mapping(
	:gt_pitch_deg=>parsef32 => "pitch [deg]",
	color=direct(eachindex(eachrow(df))),
	linestyle=direct("gt"))
plt_gt *= visual(Lines)
fig = draw((plt + plt_gt); figure=(; size=(1200, 600)))
vspan!(content(fig.figure[1, 1]), idx_avail, idx_avail.+1; color=(:green, 0.1))
vspan!(content(fig.figure[1, 1]), idx_unavail, idx_unavail.+1; color=(:red, 0.1))
limits!(content(fig.figure[1,1]), (1, nrow(df)), (-10, 10))

	fig
end

# ╔═╡ cce44102-3a6b-498f-b572-129d770f2788
let
plt = data((; preds=rot_preds, i=eachindex(rot_preds)))
plt *= mapping(
	:preds=>roll_deg => "roll [deg]",
	color=:i, linestyle=direct("pred"))
plt *= visual(Lines)
plt_gt = data(df)
plt_gt *= mapping(
	:gt_roll_deg=>parsef32 => "roll [deg]",
	color=direct(eachindex(eachrow(df))),
	linestyle=direct("gt"))
plt_gt *= visual(Lines)
fig = draw((plt + plt_gt); figure=(; size=(1200, 600)))
vspan!(content(fig.figure[1, 1]), idx_avail, idx_avail.+1; color=(:green, 0.1))
vspan!(content(fig.figure[1, 1]), idx_unavail, idx_unavail.+1; color=(:red, 0.1))
limits!(content(fig.figure[1,1]), (1, nrow(df)), (-20, 20))
fig
end

# ╔═╡ 6094ce1c-db71-4269-98d1-9dddcb00e8a6
#pos_preds3dof = tmap(eachrow(df)[collect(limited_errors)]) do row
pos_preds3dof = tmap(eachrow(df)) do row
	rwy_w = row.active_runway_width_m |> parsef32
	rwy_l = row.active_runway_length_m |> parsef32

	Δh = get_far_threshold_elevation_change(all_runways, row.airport_runway)
	h = 0
    pts = WorldPoint[
        [0, rwy_w/2, h],
        [0,  -rwy_w/2, h],
        [rwy_l, rwy_w/2, h+Δh],
        [rwy_l, -rwy_w/2, h+Δh],
    ]

    # noise model chosen by looking at the typical error standard deviations
    sigmas = global_sigmas * ones(2)

	#optical_center_u_px:2047.5,"optical_center_v_px":1499.5,
	CAM_WIDTH_PX, CAM_HEIGHT_PX = (4096-1), (3000-1)
	# (near_left, near_right, far_left, far_right)

	ys = ProjectionPoint[
		[row.pred_kp_bottom_left_x_px,row.pred_kp_bottom_left_y_px] .|> parsef32,
		[row.pred_kp_bottom_right_x_px,row.pred_kp_bottom_right_y_px] .|> parsef32,
		[row.pred_kp_top_left_x_px,row.pred_kp_top_left_y_px] .|> parsef32,
		[row.pred_kp_top_right_x_px,row.pred_kp_top_right_y_px] .|> parsef32,
	] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]

	# ys = ProjectionPoint[
	# 	[row.gt_label_runway_bottom_left_x_px,row.gt_label_runway_bottom_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_bottom_right_x_px,row.gt_label_runway_bottom_right_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_left_x_px,row.gt_label_runway_top_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_right_x_px,row.gt_label_runway_top_right_y_px] .|> parsef32,
	# ] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]


	# ^ center coordinate system. their origin is top left
	# v flip coordinate system
	# ours points x->left, y->up. theirs points x->right, y->down.
	ys = [ProjectionPoint(-x, -y) for (x, y) in ys]
	gt_pos = [row.gt_along_track_distance_m,
		      row.gt_cross_track_distance_m,
		      row.gt_height_m] .|> parsef32 .|> x->convert(Float64, x)
	gt_rot = RotZYX(roll=deg2rad(row.gt_roll_deg), pitch=-deg2rad(row.gt_pitch_deg), yaw=-deg2rad(row.gt_yaw_deg))
	estimate_pos(pts, ys, gt_rot;
				  Σ=Diagonal(repeat(sigmas, outer=4).^2),
	              initial_guess = gt_pos .+ [100; 10; 5] .* randn(3)
			  )
end


# ╔═╡ 50c175ed-5dfa-40d1-9b4e-df72afc69107
proj_norms3dof = map(pos_preds3dof) do pred
	norm(pred.sol.resid)
end

# ╔═╡ 8fa3fcb7-e7ec-47c0-9370-6bfbca95d642
let
	plt = data((;
	  proj_norms = [proj_norms6dof[limited_errors]; proj_norms3dof[limited_errors]],
	  case = [repeat([:six], length(proj_norms6dof[limited_errors]));
			  repeat([:three], length(proj_norms3dof[limited_errors]))
			 ]
	 )) * mapping(:proj_norms=>(x->x/8)=>"Mean projection error [pxl]", color=:case=>"DOF solved") * histogram(; bins=100, normalization=:pdf)
    draw(plt; figure=(; title=raw"Projection error with and without predefined attitude", subtitle="FT113_22 and FT216_4, limited errors"))
end

# ╔═╡ 2e5bc6d7-8e97-4378-8418-0765fd5567cd
let
	idx = (eachrow(df).flight_test .== "FT216") .&& limited_errors
	plt = data((;
	  proj_norms = [proj_norms6dof[idx]; proj_norms3dof[idx]],
	  case = [repeat([:six], length(proj_norms6dof[idx]));
			  repeat([:three], length(proj_norms3dof[idx]))
			 ]
	 )) * mapping(:proj_norms=>(x->x/8)=>"Mean projection error [pxl]", color=:case=>"DOF solved") * histogram(; bins=100, normalization=:pdf)
    draw(plt; figure=(; title=raw"Reprojection error with and without predefined attitude", subtitle="FT216_4, limited errors"))
end

# ╔═╡ 5025810d-12d5-4f38-93a9-a88dc9f827ae
#for (pos, rot, row) in zip(pos_preds, rot_preds, eachrow(df))
stats = tmap((collect∘zip)(pos_preds, rot_preds, eachrow(df))) do (pos, rot, row)
    CAM_WIDTH_PX, CAM_HEIGHT_PX = (4096-1), (3000-1)
	zs = ProjectionPoint[
		[row.pred_kp_bottom_left_x_px,row.pred_kp_bottom_left_y_px] .|> parsef32,
		[row.pred_kp_bottom_right_x_px,row.pred_kp_bottom_right_y_px] .|> parsef32,
		[row.pred_kp_top_left_x_px,row.pred_kp_top_left_y_px] .|> parsef32,
		[row.pred_kp_top_right_x_px,row.pred_kp_top_right_y_px] .|> parsef32,
	] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]

	# zs = ProjectionPoint[
	# 	[row.gt_label_runway_bottom_left_x_px,row.gt_label_runway_bottom_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_bottom_right_x_px,row.gt_label_runway_bottom_right_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_left_x_px,row.gt_label_runway_top_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_right_x_px,row.gt_label_runway_top_right_y_px] .|> parsef32,
	# ] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]


	# ^ center coordinate system. their origin is top left
	# v flip coordinate system
	# ours points x->left, y->up. theirs points x->right, y->down.
	zs = [ProjectionPoint(-x, -y) for (x, y) in zs]

	rwy_w = row.active_runway_width_m |> parsef32
	rwy_l = row.active_runway_length_m |> parsef32


	Δh = get_far_threshold_elevation_change(all_runways, row.airport_runway)
	h = 0
    pts = WorldPoint[
        [0, rwy_w/2, h],
        [0,  -rwy_w/2, h],
        [rwy_l, rwy_w/2, h+Δh],
        [rwy_l, -rwy_w/2, h+Δh],
    ]

	compute_statistic(pos, rot, pts, zs, Diagonal(global_sigmas * ones(8)))
end


# ╔═╡ 3851d39f-07a9-4805-8f5c-3fa35d725bcb
md"""
Percentage of True Positive samples that are rejected: $(round(sum((plot_only_available ? is_available : true) .&& stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE") /
 sum((plot_only_available ? is_available : true) .&& df.confusion_matrix_value .== "TRUE_POSITIVE"); sigdigits=3))
"""

# ╔═╡ c0b1656b-2fed-40ba-a015-6d12b4dbac74
statsfig = let
	n = 2*4
	m = 2*3
plt2 = if plot_only_available
	data((; stats=stats[is_available], type=df.confusion_matrix_value[is_available],))
else
	data((; stats, type=df.confusion_matrix_value,))
end
plt2 *= mapping(:stats=>reverse_stat, color=:type=>confusionsorter, row=:type=>confusionsorter) * histogram(; bins=100, normalization=:pdf)
fig = draw(plt2, scales(Color=(; palette=colors)); figure=(; size=(800, 600)))
for axentry in fig.grid
vlines!(axentry.axis, [quantile(Chisq(n-m), 1-0.05)], color=(:red, 0.7))
end
	fig
end

# ╔═╡ d918c54b-ed57-404f-8004-3bcac12f01e8
if plot_only_available
  save("figs/statsfig_difficult_available.svg", statsfig)
else
  save("figs/statsfig_difficult_all.svg", statsfig)
end


# ╔═╡ a8d77cd5-b5c9-4738-b000-8393f875cc81
let
  stat = stats[findall(df.confusion_matrix_value.==CONFUSION_MATRIX_PLT_TYPE)[sl_keypoint]]
  admonition_type = (stat <= 0.95 ? "correct" : "danger")
  admonition_title = (stat <= 0.95 ? "Passed!" : "Failed.")
	  str = if stat <= 0.95
		  "Passed RAIM check with ``$(round(stat; digits=2))`` ``< 0.95``!"
	  else
		  "Failed RAIM check with ``$(round(stat; digits=2))`` ``\\nleq 0.95``!"
	  end

  Markdown.MD(Markdown.Admonition(admonition_type, admonition_title, Markdown.parse.([str])))
end

# ╔═╡ 7939d533-f11d-42f2-bb07-950f07f70d42
let
	n = 2*4
	m = 2*3
    plt2 = data((; stats, pos_err=(pos_preds-pos_gts), type=df.confusion_matrix_value,)
	) * mapping(
		:pos_err=>norm, color=:type, stack=:type
	) * histogram(; bins=100, normalization=:pdf)
    fig = draw(plt2; figure=(; size=(800, 600)))
    #vlines!(fig.figure[1,1], [quantile(Chisq(n-m), 1-0.05)], color=(:red, 0.7))
    fig
end

# ╔═╡ 29adafd9-be0c-42c8-8daa-d1f21cd72d70
let
	n = 2*4
	m = 2*3
	idx = findall(df.confusion_matrix_value .== "TRUE_POSITIVE")
    plt2 = data((; stats=stats, pos_err=(pos_preds-pos_gts), type=df.confusion_matrix_value, flagged=(stats .> (1-0.05)),)
	) * mapping(
		:pos_err=>norm, color=:flagged, stack=:type
	) * histogram(; bins=100, normalization=:pdf)
    fig = draw(plt2; figure=(; size=(800, 600)))
    #vlines!(fig.figure[1,1], [quantile(Chisq(n-m), 1-0.05)], color=(:red, 0.7))
    fig
end

# ╔═╡ 30d47391-1925-48b3-a26b-341a4c7220b1
let
	n = 2*4
	m = 2*3
	yaw_correction_ = eachrow([yaw_corrections fill(0, length(rot_gts_deg)) fill(0, length(rot_gts_deg))])
	idx = findall(df.confusion_matrix_value .== "TRUE_POSITIVE")
    plt2 = data((; stats=stats,
	               rot_err=map(v->mod.(v, 360), (rot_preds_deg .- (rot_gts_deg .- yaw_correction_))), type=df.confusion_matrix_value, flagged=(stats .> (1-0.05)),)
	) * mapping(
		:rot_err=>x->getindex(x, 1), color=:flagged, stack=:type
	) * histogram(; normalization=:pdf)
    fig = draw(plt2; figure=(; size=(800, 600)))
    #vlines!(fig.figure[1,1], [quantile(Chisq(n-m), 1-0.05)], color=(:red, 0.7))
    fig
end

# ╔═╡ 99c519a1-d157-4c1a-8695-465916b4c8d7
(preds - gts)[i=findall(stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE")][i=101]

# ╔═╡ 18b9e53a-21ae-47ff-98dc-7f48be13e473
pos_preds[stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE"][1]

# ╔═╡ 0b501e39-b0e0-41cd-9674-ae9bfd78b046
(rot_preds_deg .- rot_gts_deg)[findall(stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE")]

# ╔═╡ b65bc626-f24e-41c3-9aaf-777d690a7f75
(pos_preds .- pos_gts)[stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE"]

# ╔═╡ a0ebea13-1b1a-42a7-b46c-8a59077dee9a
let
	n = 2*4
	m = 2*3
plt2 = data((; stats, type=df.confusion_matrix_value,)) * mapping(:stats, color=:type, stack=:type) * histogram(; bins=100, normalization=:pdf)
fig = draw(plt2; figure=(; size=(800, 600)))
vlines!(fig.figure[1,1], [1-0.05], color=(:red, 0.7))
fig
end

# ╔═╡ bf2d32bb-6d92-419c-8a69-4988637f08a2
is_flagged = reverse_stat(stats) .> quantile(Chisq(n-m), 1-0.05)

# ╔═╡ e5dba065-3135-4a28-b09d-52fef6112b7c
"True Flag: $(mean(is_flagged .&& (type_ .== :confusion_matrix_False_Positive)))"

# ╔═╡ 7417408e-2643-4a01-a5af-46e23161deaa
"False Flag: $(mean(is_flagged .&& (type_ .== :confusion_matrix_True_Positive)))"

# ╔═╡ b08ffc79-dade-4422-bae2-41c2eee504f8
"True Nonflag: $(mean(.!is_flagged .&& (type_ .== :confusion_matrix_True_Positive)))"

# ╔═╡ 5acd232a-4c4e-4d8b-8c4e-8b1551fbd805
"False Nonflag: $(mean(.!is_flagged .&& (type_ .== :confusion_matrix_False_Positive)))"

# ╔═╡ 7a6c363f-2471-4f0d-8b37-6c342873c4f6
"Relative False Nonflag: $(mean(.!is_flagged .&& (type_ .== :confusion_matrix_False_Positive)) / mean(type_.==:confusion_matrix_False_Positive))"

# ╔═╡ 26cbd568-dfac-4021-9d08-df07967b7b1d
"Relative True Flag: $(mean(is_flagged .&& (type_ .== :confusion_matrix_False_Positive)) / mean(type_.==:confusion_matrix_False_Positive))"

# ╔═╡ 58eb0abb-60c8-45f1-bee3-59a2add0dfdc
sum(is_flagged)

# ╔═╡ 748e8dd2-c0ff-4b22-88a2-05010750097e
begin
	plt = data((; x=eachindex(pos_preds), pos_err=(pos_preds .- pos_gts)./(getindex.(pos_gts,1)),
	              is_flagged))
	plt *= mapping(:x, [:pos_err=>getidx(1), :pos_err=>getidx(2), :pos_err=>getidx(3)],
	               row=AoG.dims(1)=>renamer(["alongtrack", "crosstrack", "altitude"]),
	               color = :is_flagged,
		           #marker=:is_flagged
	)
	plt *= visual(Scatter)
	fig3 = draw(plt; figure=(; size=(800,400)), facet=(; linkyaxes=:none))
	#lineargs = (; color=:red, linestyle=:dash)
	#for col in 1:3
	#	vlines!(fig3.figure[1,col], FP_indices; lineargs...)
	#end
	fig3
end

# ╔═╡ aa888e9b-f04b-4634-8cf1-fb2248a97c48
md"""
Percentage of True Positive samples that are rejected: $(round(sum(is_available .&& stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE") /
 sum(is_available .&& df.confusion_matrix_value .== "TRUE_POSITIVE"); sigdigits=3))
"""

# ╔═╡ 8812cce6-1388-48f6-82d2-363e5fe71bed
(sum(is_available .&& stats .> (1-0.05) .&& df.confusion_matrix_value .== "TRUE_POSITIVE" .&& df.all_errors_within_allowance) /
 sum(is_available .&& df.confusion_matrix_value .== "TRUE_POSITIVE" .&& df.all_errors_within_allowance))

# ╔═╡ e0dbb5e8-1976-40d4-965f-c49687a18925
proj_preds = map(zip(eachrow(df), pos_preds, rot_preds)) do (row, pos_pred, rot_pred)
	rwy_w = row.active_runway_width_m |> parsef32
	rwy_l = row.active_runway_length_m |> parsef32

	Δh = get_far_threshold_elevation_change(all_runways, row.airport_runway)
	h = 0
    pts = WorldPoint[
        [0, rwy_w/2, h],
        [0,  -rwy_w/2, h],
        [rwy_l, rwy_w/2, h+Δh],
        [rwy_l, -rwy_w/2, h+Δh],
    ]

	[project(pos_pred, rot_pred, pt) for pt in pts]
end

# ╔═╡ d9bff3db-799e-440a-bd9b-02268d7cb4b4
visualize_runway_corners_with_zoom(df, proj_preds, global_sigmas; row_idx=findall(df.confusion_matrix_value.==CONFUSION_MATRIX_PLT_TYPE)[sl_keypoint])

# ╔═╡ c52d0f64-2e3c-4696-a58e-469c8668df68
foo = map(eachrow(df)[1:1]) do row
	rwy_w = row.active_runway_width_m |> parsef32
	rwy_l = row.active_runway_length_m |> parsef32

	Δh = get_far_threshold_elevation_change(all_runways, row.airport_runway)
	h = 0
    pts = WorldPoint[
        [0, rwy_w/2, h],
        [0,  -rwy_w/2, h],
        [rwy_l, rwy_w/2, h+Δh],
        [rwy_l, -rwy_w/2, h+Δh],
    ]

    # noise model chosen by looking at the typical error standard deviations
    sigmas = global_sigmas * ones(2)

	#optical_center_u_px:2047.5,"optical_center_v_px":1499.5,
	CAM_WIDTH_PX, CAM_HEIGHT_PX = (4096-1), (3000-1)
	# (near_left, near_right, far_left, far_right)

	ys = ProjectionPoint[
		[row.pred_kp_bottom_left_x_px,row.pred_kp_bottom_left_y_px] .|> parsef32,
		[row.pred_kp_bottom_right_x_px,row.pred_kp_bottom_right_y_px] .|> parsef32,
		[row.pred_kp_top_left_x_px,row.pred_kp_top_left_y_px] .|> parsef32,
		[row.pred_kp_top_right_x_px,row.pred_kp_top_right_y_px] .|> parsef32,
	] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]

	# ys = ProjectionPoint[
	# 	[row.gt_label_runway_bottom_left_x_px,row.gt_label_runway_bottom_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_bottom_right_x_px,row.gt_label_runway_bottom_right_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_left_x_px,row.gt_label_runway_top_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_right_x_px,row.gt_label_runway_top_right_y_px] .|> parsef32,
	# ] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]


	# ^ center coordinate system. their origin is top left
	# v flip coordinate system
	# ours points x->left, y->up. theirs points x->right, y->down.
	ys = [ProjectionPoint(-x, -y) for (x, y) in ys]
	kf_pos = [row.filtered_along_track_distance_m,
		      row.filtered_cross_track_distance_m,
		      row.filtered_height_m] .|> parsef32 .|> x->convert(Float64, x)
	gt_rot = [row.gt_yaw_deg, row.gt_pitch_deg, row.gt_roll_deg]
	projs = project.([WorldPoint(kf_pos)], [RotZYX(roll=deg2rad(row.gt_roll_deg), pitch=deg2rad(row.gt_pitch_deg), yaw=deg2rad(row.gt_yaw_deg))], pts)
	projs .- ys
end


# ╔═╡ 7386af44-d400-4631-8816-23023dac8d22
#pose_preds = tmap(eachrow(df)[collect(limited_errors)]) do row
pose_preds_uq(dof) = tmap(eachrow(df)) do row
	rwy_w = row.active_runway_width_m |> parsef32
	rwy_l = row.active_runway_length_m |> parsef32

	Δh = get_far_threshold_elevation_change(all_runways, row.airport_runway)
	h = 0
    pts = WorldPoint[
        [0, rwy_w/2, h],
        [0,  -rwy_w/2, h],
        [rwy_l, rwy_w/2, h+Δh],
        [rwy_l, -rwy_w/2, h+Δh],
    ]

    # noise model chosen by looking at the typical error standard deviations
    sigmas = global_sigmas * ones(2)

	#optical_center_u_px:2047.5,"optical_center_v_px":1499.5,
	CAM_WIDTH_PX, CAM_HEIGHT_PX = (4096-1), (3000-1)
	# (near_left, near_right, far_left, far_right)

	ys = ProjectionPoint[
		[row.pred_kp_bottom_left_x_px,row.pred_kp_bottom_left_y_px] .|> parsef32,
		[row.pred_kp_bottom_right_x_px,row.pred_kp_bottom_right_y_px] .|> parsef32,
		[row.pred_kp_top_left_x_px,row.pred_kp_top_left_y_px] .|> parsef32,
		[row.pred_kp_top_right_x_px,row.pred_kp_top_right_y_px] .|> parsef32,
	] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]

	# ys = ProjectionPoint[
	# 	[row.gt_label_runway_bottom_left_x_px,row.gt_label_runway_bottom_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_bottom_right_x_px,row.gt_label_runway_bottom_right_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_left_x_px,row.gt_label_runway_top_left_y_px] .|> parsef32,
	# 	[row.gt_label_runway_top_right_x_px,row.gt_label_runway_top_right_y_px] .|> parsef32,
	# ] .- [ProjectionPoint(CAM_WIDTH_PX÷2, CAM_HEIGHT_PX÷2)]


	# ^ center coordinate system. their origin is top left
	# v flip coordinate system
	# ours points x->left, y->up. theirs points x->right, y->down.
	ys = [ProjectionPoint(-x, -y) for (x, y) in ys]
	gt_pos = [row.gt_along_track_distance_m,
		      row.gt_cross_track_distance_m,
		      row.gt_height_m] .|> parsef32 .|> x->convert(Float64, x)
	gt_rot = [row.gt_yaw_deg, row.gt_pitch_deg, row.gt_roll_deg]

	initial_guess = if dof==:six
		[gt_pos .+ [100; 10; 5] .* randn(3); zeros(3)]
	else
		gt_pos .+ [100; 10; 5]
	end

	noisemodel = let row = row
		obsnoises = [
			MvNormal(zeros(2), Symmetric(parsef32.([
				row.pred_kp_uncertainty_bottom_left_xx_px2 row.pred_kp_uncertainty_bottom_left_xy_px2;
				row.pred_kp_uncertainty_bottom_left_xy_px2 row.pred_kp_uncertainty_bottom_left_yy_px2
			]))),
		    MvNormal(zeros(2), Symmetric(parsef32.([
				row.pred_kp_uncertainty_bottom_right_xx_px2	row.pred_kp_uncertainty_bottom_right_xy_px2;
				row.pred_kp_uncertainty_bottom_right_xy_px2 row.pred_kp_uncertainty_bottom_right_yy_px2
			]))),
		    MvNormal(zeros(2), Symmetric(parsef32.([
				row.pred_kp_uncertainty_top_left_xx_px2	row.pred_kp_uncertainty_top_left_xy_px2;
				row.pred_kp_uncertainty_top_left_xy_px2 row.pred_kp_uncertainty_top_left_yy_px2
			]))),
		    MvNormal(zeros(2), Symmetric(parsef32.([
				row.pred_kp_uncertainty_top_right_xx_px2 row.pred_kp_uncertainty_top_right_xy_px2;
				row.pred_kp_uncertainty_top_right_xy_px2 row.pred_kp_uncertainty_top_right_yy_px2
			])))
		]
	    noisemodel = UncorrGaussianNoiseModel(obsnoises)
	end
	gt_rot = RotZYX(roll=deg2rad(row.gt_roll_deg),
					pitch=-deg2rad(row.gt_pitch_deg),
					yaw=-deg2rad(row.gt_yaw_deg))

	function f(x, p)
		project(WorldPoint(p[1:3]...), (dof==:six ? RotZYX(p[4:6]...) : gt_rot), x)
	end
	if dof == :six
		prior = MvNormal(initial_guess, Diagonal([1000^2, 200^2, 200^2, 0.2^2, 0.2^2, 0.2^2]))
	elseif dof == :three
		prior = MvNormal(initial_guess, Diagonal([1000^2, 200^2, 200^2]))
	end
	est = LinearApproxEstimator(; solvealg=(; kwargs...)->LevenbergMarquardt(; kwargs...))
	try
		predictdist(est, f, pts, ys, prior, noisemodel)
	catch
		nothing
	end
end

# ╔═╡ 9a5923c8-a71d-4c6d-8450-91548b0904e2
pose_preds_uq6dof = pose_preds_uq(:six)

# ╔═╡ 07c6c3ac-5329-4129-b76d-70b35af40fdf
pos_preds_uq6dof = map(pose_preds_uq6dof) do pred
	issomething(pred) ? pred[1:3] : nothing
end

# ╔═╡ 37b2376f-9eaf-49b6-b8fe-a792c31c476c
pose_preds_uq3dof = pose_preds_uq(:three)

# ╔═╡ ac6bc30c-7627-4b03-b45a-763385adc6f2
pos_preds_uq3dof = map(pose_preds_uq3dof) do pred
	issomething(pred) ? pred[1:3] : nothing
end

# ╔═╡ aca80745-5678-4002-bffe-76583ffc1dc0
pos_preds_uq = Dict(
	:three => pos_preds_uq3dof,
	:six => pos_preds_uq6dof
)

# ╔═╡ 8146ca63-f102-4461-bf57-a4d26d7da6f6
 let
	 fig = Figure()
	 for (i, dof) in enumerate([:three, :six])
		 idx = issomething.(pos_preds_uq[dof])
		 ax = Axis(fig[1,i]; title="$dof")
	    calib_vals = computecalibration(identity.(pos_preds_uq[dof][idx]), gt_pos[idx])
		plot!(ax, calib_vals.pvals, calib_vals.calibrationvals)
		lines!(ax, 0:1, 0:1)
	 end
	fig
end

# ╔═╡ 30cf7768-9c12-4831-936b-1dc9ad2bcd8a
 let
	 label(i) = if i == 1; "alongtrack"; elseif i==2; "crosstrack" else "height" end
	 fig = Figure()
	 for (i, dof) in enumerate([:three, :six])
	     idx = issomething.(pos_preds_uq[dof])
		 for j in 1:3
			 xlabel = (i==2 ? "Predicted coverage" : "")
			 ylabel = (j==1 ? "Empirical coverage" : "")
			 ax = Axis(fig[i,j]; title=label(j), xlabel, ylabel)
			 preds = getindex.(pos_preds_uq[dof][idx], [j])
			 gt = getindex.(gt_pos[idx], [j])
		     calib_vals_pos6dof = computecalibration(identity.(preds), gt)
			 plot!(ax, calib_vals_pos6dof.pvals, calib_vals_pos6dof.calibrationvals)
			 lines!(ax, 0:1, 0:1)
		 end
		 Label(fig[i,4], text="dof = $dof", rotation=deg2rad(-90), tellheight=false)
	end
	Label(fig[0, 1:3], text="Calibration plots for three and six dof solver", font=:bold, fontsize=18)
	fig

end

# ╔═╡ a7980f3c-f8ea-4610-9f99-35c536abfc7f
 let
	 label(i) = if i == 1; "alongtrack"; elseif i==2; "crosstrack" else "height" end
	 fig = Figure(; )
	 for (i, dof) in enumerate([:three, :six])
     idx = issomething.(pos_preds_uq[dof])
	 for j in 1:3
		 limits = limits=(-30, 30, 0, nothing)
		 ax = Axis(fig[i,j]; title=label(j), xlabel="z-value", limits)
		 preds = getindex.(pos_preds_uq[dof][idx], [j])
		 gt = getindex.(gt_pos[idx], [j])

		 hist!(ax, (mean.(preds) .- gt) ./ std.(preds); bins=200, normalization=:pdf)
		 lines!(ax, -3:0.01:3, x->pdf(Normal(), x); color=Makie.wong_colors()[2], linewidth=3, alpha=0.8)
	 end
		 Label(fig[i,4], text="dof = $dof", rotation=deg2rad(-90), tellheight=false)
	 end
	 Label(fig[0, 1:3], text="z-values for three and six dof solver", font=:bold, fontsize=18)
	 fig

end

# ╔═╡ fa615e7f-16c4-415c-90ac-5c37023e56b7
get_far_threshold_elevation_change(all_runways, "AYPY_32R")

# ╔═╡ 9a34c847-b38d-4c8e-93ab-f14581d4425c
all_runways[all_runways.ICAO_RWY .∈ [["KSWF_27", "KSBN_27L"]], :]
