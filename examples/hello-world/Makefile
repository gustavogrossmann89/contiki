DEFINES+=PROJECT_CONF_H=\"project-conf.h\"
CFLAGS+=-g

CONTIKI_PROJECT = trabalho_lab
CONTIKI_WITH_IPV6 = 1
CFLAGS += -DUIP_CONF_ND6_SEND_NS=1
all: $(CONTIKI_PROJECT)

CONTIKI = ../..
include $(CONTIKI)/Makefile.include
