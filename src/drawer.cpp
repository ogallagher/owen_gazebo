/*

Owen Gallagher
28 noviembre 2018
Robotica

*/

#include <iostream>
#include <SDL2/SDL.h> //libreria grafica

using namespace std;

int main() {
	bool quit = false;
	int windowWidth = 640;
	int windowHeight = 480;
	int canvasResolution = 32;
	
	SDL_Window* window = NULL;
	SDL_Renderer* renderer = NULL;
	
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		cout << "Drawer program failed to initialize SDL : " << SDL_GetError() << endl;
		return EXIT_FAILURE;
	}
	else {
		window = SDL_CreateWindow("Owen: Explorer GNT", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, windowWidth, windowHeight, SDL_WINDOW_SHOWN);
		
		if (window == NULL) {
			cout << "Drawer program failed to load window: " << SDL_GetError() << endl;
			return EXIT_FAILURE;
		}
		else {
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
			
			if (renderer == NULL) {
				cout << "Drawer program failed to load renderer: " << SDL_GetError() << endl;
				return EXIT_FAILURE;
			}
			else {
				SDL_Event event;
				while (!quit) {
					while(SDL_PollEvent(&event) != 0) {
						if (event.type == SDL_QUIT) {
							quit = true;
						}
					}
					
					//dibujar rectangulo rojo
					SDL_SetRenderDrawColor(renderer, 0xCC, 0x00, 0x00, 0xFF);
					SDL_Rect fillRect = {windowWidth/4,windowHeight/4,windowWidth/2,windowHeight/2};
					SDL_RenderFillRect(renderer,&fillRect);
					
					//dibujar linea azul
					SDL_SetRenderDrawColor(renderer, 0x00, 0x00, 0xCC, 0xFF);
					SDL_RenderDrawLine(renderer,0,windowHeight/2,windowWidth,windowHeight/2);
					
					SDL_RenderPresent(renderer);
					
					SDL_SetRenderDrawColor(renderer, 0xCC, 0xCC, 0xCC, 0xFF);
					SDL_RenderClear(renderer);
				}
				
				SDL_DestroyWindow(window);
				SDL_Quit();
				
				return EXIT_SUCCESS;
			}
		}
	}
}
