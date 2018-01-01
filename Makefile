
SUBDIRS = ant_code fortius_code

SUBCLEAN = $(addsuffix .clean,$(SUBDIRS))
        
.PHONY: clean $(SUBCLEAN) subdirs $(SUBDIRS)

subdirs: $(SUBDIRS)

clean: $(SUBCLEAN)
      
$(SUBDIRS):
	$(MAKE) -C $@

all: $(SUBDIRS)
	$(MAKE) -C $@	
       
$(SUBCLEAN): %.clean:
	$(MAKE) -C $* clean
