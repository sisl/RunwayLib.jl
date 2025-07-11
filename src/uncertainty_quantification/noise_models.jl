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
