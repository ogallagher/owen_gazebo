/*

Owen Gallagher
28 noviembre 2018
Robótica 1

*/

// #include "ros/ros.h"

#include <SDL2/SDL.h>

#include "gnt_graph.h"
#include "drawer.h"

using namespace std;

int main() {
    Drawer drawer;
	GNTGraph gntGraph;
    
	if (drawer.init()) {
		//TODO: crear GNT
		
		drawer.setColor(0x00,0x00,0xAA,0xFF); //dibujar en azul

		while (!drawer.quit) {
			drawer.clear();
			
			while(SDL_PollEvent(&drawer.event) != 0) {
				if (drawer.event.type == SDL_QUIT) {
					drawer.quit = true;
				}
			}

			drawer.render();
		}

		return EXIT_SUCCESS;
	}
	else {
		return EXIT_FAILURE;
	}
}