TSC=node_modules/typescript/bin/tsc 

compilesim:
	rm -rf build
	mkdir build
	$(TSC) --outFile build/simulation.js src/simulation.ts &
	$(TSC) --outFile build/custom.js src/custom.ts