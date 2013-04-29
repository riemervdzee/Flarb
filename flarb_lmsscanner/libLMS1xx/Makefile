all: installed

GIT_URL = https://github.com/riemervdzee/libLMS1xx.git
GIT_DIR = build/libLMS1xx

include $(shell rospack find mk)/git_checkout.mk

installed: $(GIT_DIR)
	cd $(GIT_DIR) && cmake -D CMAKE_INSTALL_PREFIX=`rospack find libLMS1xx`/libLMS1xx .
	cd $(GIT_DIR) && make && make install
	touch installed

clean:
	cd $(GIT_DIR) && make clean
	rm -rf libLMS1xx
	rm -f *~ installed
