all:
	@$(MAKE) -f Makefile.lcmgen all || exit 2;
	@make -C lcmtypes
	@make src

clean:
	@make -C lcmtypes -s clean
	@make -C src -s clean

