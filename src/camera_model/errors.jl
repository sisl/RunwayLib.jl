"""
    BehindCameraException

Exception thrown when the projection point is behind the camera
"""
struct BehindCameraException{T} <: Exception
    x::T
end
function Base.showerror(io::IO, ex::BehindCameraException)
    return print(io, "Point to be projected is $(ex.x) behind the camera.")
end
