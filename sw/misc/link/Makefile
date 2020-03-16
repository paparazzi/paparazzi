# Quiet compilation
Q=@

all: pprzlink_router pprzlink_router_cc

pprzlink_router: pprzlink_router.c
	gcc -g -O2 -Wall -o $@ $^ -lpthread

pprzlink_router_cc:pprzlink_router.c
	arm-linux-gnueabi-gcc -static -g -O2 -Wall -o $@ $^ -lpthread
	
clean:
	$(Q)rm -f pprzlink_router pprzlink_router_cc
