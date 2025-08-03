JULIA ?= $(shell which julia)
SRCDIR = ../src
SRC = $(shell find $(SRCDIR) -name "*.jl")

CC = gcc
CFLAGS = -Wall -O2
RPATH_FLAGS = -Wl,-rpath,'$$ORIGIN' -Wl,-rpath,'$$ORIGIN/julia'
LDFLAGS = -L. -Ljulia
LIBS = -lposeest -ljulia

main: main.jl Project.toml Manifest.toml $(SRC)
	JULIAC="./juliac/juliac.jl" ; \
	$(JULIA) --project=. --depwarn=error "$$JULIAC" --experimental --trim=unsafe-warn --output-exe main main.jl

libposeest.so: lib.jl Project.toml Manifest.toml $(SRC) julia
	JULIAC="./juliac/juliac.jl" ; \
	$(JULIA) --project=. --depwarn=error "$$JULIAC" --experimental --trim=unsafe-warn --compile-ccallable --output-lib libposeest lib.jl --export-abi abiinfo.json --relative-rpath

Manifest.toml: Project.toml ../Project.toml
	$(JULIA) --project=. -e 'using Pkg; Pkg.instantiate()'
	@touch $@ # Pkg.instantiate doesn't update the mtime if there are no changes

mainc: main.c libposeest.h libposeest.so julia
	$(CC) $(CFLAGS) $(RPATH_FLAGS) -o mainc main.c $(LDFLAGS) $(LIBS)

install-julialibs:
	@echo "Copying Julia libraries..."
	cp -r $$($(JULIA) -e "print(joinpath(Sys.BINDIR, \"..\", \"lib\"))") ./julia
	@echo "Julia libraries installed to ./julia/"

julia: install-julialibs

run: mainc
	./mainc


.PHONY: clean run install-julialibs

clean:
	-rm -f main mainc libposeest Manifest.toml
