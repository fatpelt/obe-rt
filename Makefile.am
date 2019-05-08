commit = $(shell git log --oneline -1 | awk '{print $$1'})
build = .0

AUTOMAKE_OPTIONS = foreign
SUBDIRS = obe
EXTRA_DIST = include doxygen/libklvanc.doxyconf doxygen/include

docs:
	cd doxygen && doxygen libklvanc.doxyconf

deb:
	mkdir -p obedeb/etc/systemd/system
	mkdir -p obedeb/etc/obe
	mkdir -p obedeb/DEBIAN
	mkdir -p obedeb/usr/local/bin

	cp obe@.service obedeb/etc/systemd/system/
	cp obe/obecli obedeb/usr/local/bin/
	strip obedeb/usr/local/bin/obecli

	cp decklink-sample.* obedeb/etc/obe/

	sed 's/COMMIT/${commit}/' control > obedeb/DEBIAN/control
	sed -i 's/BUILD/${build}/' obedeb/DEBIAN/control
	cd obedeb ; find etc/obe/ -type f > DEBIAN/conffiles

	fakeroot dpkg-deb --build obedeb .

