
all:
	rm -f *.so
	make libtank_control_plugin.so
	make libLedPlugin.so

libtank_control_plugin.so:
	(cd tank_control && make)
	cp tank_control/*.so .

libLedPlugin.so:
	(cd LedPlugin && make)
	cp LedPlugin/*.so .

clean:
	rm -f *.so
	(cd tank_control && make clean)
	(cd LedPlugin && make clean)

.PHONY: all clean
