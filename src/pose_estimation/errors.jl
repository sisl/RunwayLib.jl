struct OptimizationFailedError{R, S} <: Exception
    retcode::R
    sol::S
end
function Base.showerror(io::IO, ex::OptimizationFailedError)
    return print(io, "Optimization failed with retcode $(ex.retcode).")
end
