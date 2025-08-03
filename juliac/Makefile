JULIA ?= $(shell which julia)
SRCDIR = ../src
SRC = $(shell find $(SRCDIR) -name "*.jl")

CC = gcc
CFLAGS = -Wall -O2
RPATH_FLAGS = -Wl,-rpath,\$$ORIGIN -Wl,-rpath,\$$ORIGIN/julia
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
	gcc -Wall -O2 -Wl,-rpath='$$ORIGIN' -Wl,-rpath='$$ORIGIN/julia' -o mainc main.c -L. -Ljulia -lposeest -ljulia

julia/.installed:
	@echo "Copying Julia libraries..."
	JULIA_LIB_DIR=$$($(JULIA) -e "print(joinpath(Sys.BINDIR, \"..\", \"lib\"))"); \
	cp -r "$$JULIA_LIB_DIR"/* .
	@touch julia/.installed
	@echo "Julia libraries installed to current directory"

julia/.cleaned: julia/.installed required_libraries.txt
	@echo "Cleaning julia/ directory based on required_libraries.txt"
	@echo "Before cleanup: $$(du -sh julia/ | cut -f1)"
	cd julia && for file in *.so*; do [ -f "$$file" ] && stem=$$(echo "$$file" | sed 's/\.so.*/.so/') && ! grep -q "$$stem" ../required_libraries.txt && rm -f "$$file" || true; done
	@echo "After cleanup: $$(du -sh julia/ | cut -f1)"
	@touch julia/.cleaned


julia: julia/.installed

julia-clean: julia/.cleaned

run: mainc
	./mainc


.PHONY: clean run install-julialibs julia-clean

clean:
	-rm -rf main mainc libposeest.so Manifest.toml julia abiinfo.json libjulia.so*
