
SUBDIRS = ant_code fortius_code

SUBCLEAN = $(addsuffix .clean,$(SUBDIRS))

SUBINSTALL = $(addsuffix .install,$(SUBDIRS))

.PHONY: clean $(SUBCLEAN) subdirs $(SUBDIRS) install $(SUBINSTALL) install_firmware

subdirs: $(SUBDIRS)

clean: $(SUBCLEAN)

install_firmware:
	cp -r firmware /usr/local

install: $(SUBINSTALL) install_firmware

$(SUBDIRS):
	$(MAKE) -C $@

$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean     

$(SUBINSTALL): %.install:
	$(MAKE) -C $* install

all: $(SUBDIRS)
	$(MAKE) -C $@	
       

