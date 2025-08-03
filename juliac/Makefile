JULIA ?= $(shell which julia)
SRCDIR = ../src
SRC = $(shell find $(SRCDIR) -name "*.jl")

main: main.jl Project.toml Manifest.toml $(SRC)
	JULIAC="./juliac/juliac.jl" ; \
	$(JULIA) --project=. --depwarn=error "$$JULIAC" --experimental --trim=unsafe-warn --output-exe main main.jl

lib: main.jl Project.toml Manifest.toml $(SRC)
	JULIAC="./juliac/juliac.jl" ; \
	$(JULIA) --project=. --depwarn=error "$$JULIAC" --experimental --trim=unsafe-warn --compile-ccallable --output-lib libposeest lib.jl --export-abi abiinfo.json

Manifest.toml: Project.toml ../Project.toml
	$(JULIA) --project=. -e 'using Pkg; Pkg.instantiate()'
	@touch $@ # Pkg.instantiate doesn't update the mtime if there are no changes

clean:
	-rm -f main libposeest Manifest.toml
