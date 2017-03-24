# get user and architecture specific options
OS := $(shell uname -s)
TORCHDIR := $(shell cd ..; pwd)
include ../Makefile_options_$(OS)

CC_FILES := $(wildcard *.cc)
OBJS := $(foreach f,$(CC_FILES),$(OBJS_DIR)/$(patsubst %.cc,%.o,$(f)))

# redefinition of libtorch
LIBTORCH := $(LIBS_DIR)//libcmdline.a

all:	$(LIBTORCH)

$(LIBTORCH): $(OBJS)
	@echo "Archiving..."
	@$(AR) $(LIBTORCH) $(OBJS)

$(OBJS_DIR)/%.o: %.cc
	@echo $<
	@$(CC) $(CFLAGS_$(MODE)) $(INCS) -o $@ -c $<

distclean:
	@\rm -f .deps_*

clean:
	@echo "Remove objects file and dependencies..."
	@\rm -Rf $(OBJS) $(LIBTORCH)
	@\rm -f .deps_$(VERSION_KEY)

depend:
	@echo "Tracking dependencies..."
	@\rm -f .deps_$(VERSION_KEY)
	@for file in *.cc ; do printf "$(OBJS_DIR)/" >> .deps_$(VERSION_KEY); $(DEP)  $(DEPS_CFLAGS_$(FLOATING)) $(INCS) $$file >> .deps_$(VERSION_KEY); done

.deps_$(VERSION_KEY):
	@echo ">>> Please do a 'make depend' <<<"
	exit 10

ifneq ($(MAKECMDGOALS),distclean)
ifneq ($(MAKECMDGOALS),clean)
ifneq ($(MAKECMDGOALS),depend)
include .deps_$(VERSION_KEY)
endif
endif
endif
