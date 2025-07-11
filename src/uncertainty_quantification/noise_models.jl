"""
Noise models for uncertainty quantification in pose estimation.

This module provides noise model implementations for representing
pixel-level uncertainties and their propagation through the estimation pipeline.
"""

using Distributions
using LinearAlgebra

"""
    UncorrGaussianNoiseModel

Uncorrelated Gaussian noise model for pixel observations.

This model assumes that each pixel coordinate has independent Gaussian noise
with potentially different standard deviations.

# Fields
- `noisedistributions::Vector{Normal}`: Vector of Normal distributions for each observation

# Constructor
```julia
UncorrGaussianNoiseModel(distributions::Vector{Normal{T}}) where T
```

# Examples
```julia
# Create noise model for 4 runway corners (8 pixel coordinates)
pixel_std = 2.5  # pixels
noise_dists = [Normal(0.0, pixel_std) for _ in 1:8]
noise_model = UncorrGaussianNoiseModel(noise_dists)

# Access individual distributions
println("First coordinate std: ", std(noise_model.noisedistributions[1]))
```
"""
struct UncorrGaussianNoiseModel{T}
    noisedistributions::Vector{Normal{T}}
    
    function UncorrGaussianNoiseModel(distributions::Vector{Normal{T}}) where T
        if isempty(distributions)
            throw(ArgumentError("Noise distributions cannot be empty"))
        end
        
        # Validate that all distributions have zero mean
        for (i, dist) in enumerate(distributions)
            if abs(mean(dist)) > 1e-10
                @warn "Distribution $i has non-zero mean: $(mean(dist))"
            end
        end
        
        new{T}(distributions)
    end
end

"""
    CorrGaussianNoiseModel

Correlated Gaussian noise model for pixel observations.

This model allows for correlations between different pixel coordinates,
represented by a full covariance matrix.

# Fields
- `distribution::MvNormal`: Multivariate normal distribution

# Examples
```julia
# Create correlated noise model
n_obs = 8  # 4 corners × 2 coordinates
mean_vec = zeros(n_obs)
cov_matrix = 2.5^2 * I(n_obs)  # Diagonal covariance (uncorrelated case)

# Add some correlation between x and y coordinates of same corner
for i in 1:2:n_obs-1
    cov_matrix[i, i+1] = 0.5  # Small correlation between x and y
    cov_matrix[i+1, i] = 0.5
end

noise_model = CorrGaussianNoiseModel(MvNormal(mean_vec, cov_matrix))
```
"""
struct CorrGaussianNoiseModel{T}
    distribution::MvNormal{T}
    
    function CorrGaussianNoiseModel(distribution::MvNormal{T}) where T
        # Validate that mean is approximately zero
        if norm(mean(distribution)) > 1e-10
            @warn "Noise model has non-zero mean: $(mean(distribution))"
        end
        
        new{T}(distribution)
    end
end

"""
    get_covariance_matrix(noise_model::UncorrGaussianNoiseModel) -> Matrix

Extract covariance matrix from uncorrelated noise model.

# Arguments
- `noise_model::UncorrGaussianNoiseModel`: Noise model

# Returns
- `Matrix`: Diagonal covariance matrix

# Examples
```julia
noise_dists = [Normal(0.0, 2.5) for _ in 1:8]
noise_model = UncorrGaussianNoiseModel(noise_dists)
cov_matrix = get_covariance_matrix(noise_model)
println("Covariance matrix size: ", size(cov_matrix))
```
"""
function get_covariance_matrix(noise_model::UncorrGaussianNoiseModel)
    n = length(noise_model.noisedistributions)
    cov_matrix = zeros(n, n)
    
    # Fill diagonal with variances
    for i in 1:n
        cov_matrix[i, i] = var(noise_model.noisedistributions[i])
    end
    
    return cov_matrix
end

"""
    get_covariance_matrix(noise_model::CorrGaussianNoiseModel) -> Matrix

Extract covariance matrix from correlated noise model.

# Arguments
- `noise_model::CorrGaussianNoiseModel`: Noise model

# Returns
- `Matrix`: Full covariance matrix

# Examples
```julia
mvn = MvNormal(zeros(8), 2.5^2 * I(8))
noise_model = CorrGaussianNoiseModel(mvn)
cov_matrix = get_covariance_matrix(noise_model)
```
"""
function get_covariance_matrix(noise_model::CorrGaussianNoiseModel)
    return cov(noise_model.distribution)
end

"""
    sample_noise(noise_model::UncorrGaussianNoiseModel, n_samples::Int=1) -> Matrix

Generate noise samples from uncorrelated Gaussian model.

# Arguments
- `noise_model::UncorrGaussianNoiseModel`: Noise model
- `n_samples::Int`: Number of samples to generate

# Returns
- `Matrix`: Noise samples (n_observations × n_samples)

# Examples
```julia
noise_dists = [Normal(0.0, 2.5) for _ in 1:8]
noise_model = UncorrGaussianNoiseModel(noise_dists)
samples = sample_noise(noise_model, 100)
println("Sample size: ", size(samples))
```
"""
function sample_noise(noise_model::UncorrGaussianNoiseModel, n_samples::Int=1)
    n_obs = length(noise_model.noisedistributions)
    samples = zeros(n_obs, n_samples)
    
    for i in 1:n_obs
        samples[i, :] = rand(noise_model.noisedistributions[i], n_samples)
    end
    
    return samples
end

"""
    sample_noise(noise_model::CorrGaussianNoiseModel, n_samples::Int=1) -> Matrix

Generate noise samples from correlated Gaussian model.

# Arguments
- `noise_model::CorrGaussianNoiseModel`: Noise model
- `n_samples::Int`: Number of samples to generate

# Returns
- `Matrix`: Noise samples (n_observations × n_samples)

# Examples
```julia
mvn = MvNormal(zeros(8), 2.5^2 * I(8))
noise_model = CorrGaussianNoiseModel(mvn)
samples = sample_noise(noise_model, 100)
```
"""
function sample_noise(noise_model::CorrGaussianNoiseModel, n_samples::Int=1)
    return rand(noise_model.distribution, n_samples)
end
"""
Noise models for uncertainty quantification in runway pose estimation.

This module defines noise models that can be used with pose estimation
and integrity monitoring. It integrates with ProbabilisticParameterEstimators
for consistent noise modeling.
"""

using Distributions
using LinearAlgebra

# Re-export noise models from ProbabilisticParameterEstimators
using ..ProbabilisticParameterEstimators: UncorrGaussianNoiseModel, CorrGaussianNoiseModel
export UncorrGaussianNoiseModel, CorrGaussianNoiseModel

"""
    create_pixel_noise_model(pixel_std::Real, n_corners::Int) -> UncorrGaussianNoiseModel

Create uncorrelated Gaussian noise model for pixel observations.

# Arguments
- `pixel_std`: Standard deviation of pixel noise (in pixels)
- `n_corners`: Number of runway corners being observed

# Returns
- `UncorrGaussianNoiseModel` with 2*n_corners Normal distributions

# Examples
```julia
# 2-pixel noise for 4 corners (8 observations total)
noise_model = create_pixel_noise_model(2.0, 4)
```
"""
function create_pixel_noise_model(pixel_std::Real, n_corners::Int)
    distributions = [Normal(0.0, pixel_std) for _ in 1:(2*n_corners)]
    return UncorrGaussianNoiseModel(distributions)
end

"""
    create_corner_noise_model(corner_covariances::AbstractVector{<:AbstractMatrix}) -> UncorrGaussianNoiseModel

Create noise model from individual corner covariance matrices.

# Arguments
- `corner_covariances`: Vector of 2×2 covariance matrices, one per corner

# Returns
- `UncorrGaussianNoiseModel` with MvNormal distributions for each corner

# Examples
```julia
# Different noise for each corner
corner_covs = [
    [4.0 0.5; 0.5 4.0],  # Corner 1: correlated x,y noise
    [2.0 0.0; 0.0 2.0],  # Corner 2: uncorrelated noise
    [3.0 1.0; 1.0 3.0],  # Corner 3: different correlation
    [1.0 0.0; 0.0 1.0]   # Corner 4: low noise
]
noise_model = create_corner_noise_model(corner_covs)
```
"""
function create_corner_noise_model(corner_covariances::AbstractVector{<:AbstractMatrix})
    distributions = [MvNormal(zeros(2), cov) for cov in corner_covariances]
    return UncorrGaussianNoiseModel(distributions)
end

"""
    create_full_correlated_noise_model(covariance_matrix::AbstractMatrix) -> CorrGaussianNoiseModel

Create fully correlated noise model for all pixel observations.

# Arguments
- `covariance_matrix`: Full covariance matrix for all observations

# Returns
- `CorrGaussianNoiseModel` with full correlation structure

# Examples
```julia
# 8×8 covariance matrix for 4 corners (8 pixel coordinates)
Σ = create_block_diagonal_covariance([
    [4.0 0.5; 0.5 4.0],  # Corner 1
    [2.0 0.0; 0.0 2.0],  # Corner 2
    [3.0 1.0; 1.0 3.0],  # Corner 3
    [1.0 0.0; 0.0 1.0]   # Corner 4
])
noise_model = create_full_correlated_noise_model(Σ)
```
"""
function create_full_correlated_noise_model(covariance_matrix::AbstractMatrix)
    n = size(covariance_matrix, 1)
    return CorrGaussianNoiseModel(MvNormal(zeros(n), covariance_matrix))
end

"""
    create_block_diagonal_covariance(corner_covariances::AbstractVector{<:AbstractMatrix}) -> Matrix

Create block-diagonal covariance matrix from individual corner covariances.

# Arguments
- `corner_covariances`: Vector of 2×2 covariance matrices, one per corner

# Returns
- Block-diagonal matrix with corner covariances on the diagonal

# Examples
```julia
corner_covs = [
    [4.0 0.5; 0.5 4.0],
    [2.0 0.0; 0.0 2.0]
]
Σ = create_block_diagonal_covariance(corner_covs)
# Results in 4×4 matrix with 2×2 blocks on diagonal
```
"""
function create_block_diagonal_covariance(corner_covariances::AbstractVector{<:AbstractMatrix})
    n_corners = length(corner_covariances)
    total_size = 2 * n_corners
    
    Σ = zeros(total_size, total_size)
    
    for (i, cov) in enumerate(corner_covariances)
        row_start = 2*(i-1) + 1
        row_end = 2*i
        col_start = row_start
        col_end = row_end
        
        Σ[row_start:row_end, col_start:col_end] = cov
    end
    
    return Σ
end

"""
    extract_noise_model_from_uncertainties(
        uncertainties::AbstractVector{<:AbstractMatrix}
    ) -> UncorrGaussianNoiseModel

Extract noise model from uncertainty matrices (e.g., from computer vision predictions).

# Arguments
- `uncertainties`: Vector of 2×2 uncertainty matrices, one per corner

# Returns
- `UncorrGaussianNoiseModel` based on the uncertainty matrices

# Examples
```julia
# From computer vision uncertainty predictions
uncertainties = [
    [σ²_x1 σ_xy1; σ_xy1 σ²_y1],  # Corner 1 uncertainty
    [σ²_x2 σ_xy2; σ_xy2 σ²_y2],  # Corner 2 uncertainty
    # ...
]
noise_model = extract_noise_model_from_uncertainties(uncertainties)
```
"""
function extract_noise_model_from_uncertainties(
    uncertainties::AbstractVector{<:AbstractMatrix}
)
    distributions = [MvNormal(zeros(2), unc) for unc in uncertainties]
    return UncorrGaussianNoiseModel(distributions)
end
