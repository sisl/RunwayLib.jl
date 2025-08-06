using PackageCompiler
PackageCompiler.create_library(
    ".",
    "RunwayLibCompiled";
    lib_name = "libposeest",
    precompile_statements_file = ["$(@__DIR__)/additional_precompile.jl"],
    incremental = false,
    filter_stdlibs = true,
    force = true,
    header_files = ["$(@__DIR__)/libposeest.h"],
)
