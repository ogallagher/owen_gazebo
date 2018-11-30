#ifndef DRAWER_H
#define DRAWER_H

/*

Owen Gallagher
28 noviembre 2018
Robotica

*/

#include <SDL2/SDL.h> //libreria grafica
#include "gnt_graph.h" //tiene las clases TreeNode

class Drawer {
	private:
		int windowResolution;
		SDL_Window* window;
		SDL_Renderer* renderer;
		
	public:
		SDL_Event event;
		bool quit;
		int width;
		int height;
		char backgroundColor[4];
		char paintColor[4];
		double strokeWidth;
		
		Drawer();
		Drawer(int w, int h);
		~Drawer();
		bool init();
		void render();
		void clear();
		void setColor(int r, int g, int b, int a);
		void setColor(const int c[4]);
		void line(double x1, double y1, double x2, double y2);
		void circle(double x, double y, double r);
		void node(TreeNode* node);
		void graph(GNTGraph* graph);
};

#endif
