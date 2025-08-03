JULIA ?= $(shell which julia)
SRCDIR = ../src
SRC = $(shell find $(SRCDIR) -name "*.jl")

CC = gcc
CFLAGS = -Wall -O2
JULIA_DIR = $(shell julia -e "print(Sys.BINDIR)")/../
LDFLAGS = -L. -L$(JULIA_DIR)/lib
LIBS = -lposeest -ljulia

main: main.jl Project.toml Manifest.toml $(SRC)
	JULIAC="./juliac/juliac.jl" ; \
	$(JULIA) --project=. --depwarn=error "$$JULIAC" --experimental --trim=unsafe-warn --output-exe main main.jl

libposeest.so: lib.jl Project.toml Manifest.toml $(SRC)
	JULIAC="./juliac/juliac.jl" ; \
	$(JULIA) --project=. --depwarn=error "$$JULIAC" --experimental --trim=unsafe-warn --compile-ccallable --output-lib libposeest lib.jl --export-abi abiinfo.json

Manifest.toml: Project.toml ../Project.toml
	$(JULIA) --project=. -e 'using Pkg; Pkg.instantiate()'
	@touch $@ # Pkg.instantiate doesn't update the mtime if there are no changes

mainc: main.c libposeest.h libposeest.so
	$(CC) $(CFLAGS) -o mainc main.c $(LDFLAGS) $(LIBS)

run: mainc
	LD_LIBRARY_PATH=.:$(JULIA_DIR)/lib:$$LD_LIBRARY_PATH ./mainc


.PHONY: clean run

clean:
	-rm -f main mainc libposeest Manifest.toml
