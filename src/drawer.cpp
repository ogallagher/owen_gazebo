/*

Owen Gallagher
28 noviembre 2018
Robotica

*/

#include <iostream>
#include <cmath>

#include "drawer.h"
#include "owen_colors.h"

using namespace std;

Drawer::Drawer() {
	quit = false;
	windowResolution = 32;
	window = NULL;
	renderer = NULL;
	
	width = 640;
	height = 480;
	
	//gris
	backgroundColor[0] = 0xBB;
	backgroundColor[1] = 0xBB;
	backgroundColor[2] = 0xBB;
	backgroundColor[3] = 0xFF;
	
	//rojo
	paintColor[0] = 0xFF;
	paintColor[1] = 0x00;
	paintColor[2] = 0x00;
	paintColor[3] = 0xFF;
	
	strokeWidth = 1;
}

Drawer::Drawer(int w, int h) {
	Drawer();
	
	this->width = w;
	this->height = h;
};

Drawer::~Drawer() {
	SDL_DestroyWindow(window);
	SDL_Quit();
};

bool Drawer::init() {
	bool success = true;
	
	if (SDL_Init(SDL_INIT_EVERYTHING) < 0) {
		cout << "Drawer program failed to initialize SDL : " << SDL_GetError() << endl;
		success = false;
	}
	else {
		window = SDL_CreateWindow("Owen: Explorer GNT", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, width, height, SDL_WINDOW_SHOWN);
		
		if (window == NULL) {
			cout << "Drawer program failed to load window: " << SDL_GetError() << endl;
			success = false;
		}
		else {
			renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
		
			if (renderer == NULL) {
				cout << "Drawer program failed to load renderer: " << SDL_GetError() << endl;
				success = false;
			}
		}
	}
	
	return success;
}

void Drawer::setColor(int r, int g, int b, int a) {
	paintColor[0] = r;
	paintColor[1] = g;
	paintColor[2] = b;
	paintColor[3] = a;
}

void Drawer::setColor(const int c[4]) {
	paintColor[0] = c[0];
	paintColor[1] = c[1];
	paintColor[2] = c[2];
	paintColor[3] = c[3];
}

void Drawer::render() {
	SDL_RenderPresent(renderer);
}

void Drawer::clear() {
	SDL_SetRenderDrawColor(renderer, backgroundColor[0], backgroundColor[1], backgroundColor[2], backgroundColor[3]);
	SDL_RenderClear(renderer);
}

void Drawer::line(double x1, double y1, double x2, double y2) {
	SDL_SetRenderDrawColor(renderer, paintColor[0], paintColor[1], paintColor[2], paintColor[3]);
	SDL_RenderDrawLine(renderer,x1,y1,x2,y2);
}

void Drawer::circle(double x0, double y0, double r) {
	SDL_SetRenderDrawColor(renderer, paintColor[0], paintColor[1], paintColor[2], paintColor[3]);
	
	//algoritmo del punto medio, en 8 arcos
	int x = r-1;
    int y = 0;
    int dx = 1;
    int dy = 1;
	int d = r*2;
    int err = dx - d;

    while (x >= y) {
        SDL_RenderDrawPoint(renderer, x0 + x, y0 + y);
        SDL_RenderDrawPoint(renderer, x0 + y, y0 + x);
        SDL_RenderDrawPoint(renderer, x0 - y, y0 + x);
        SDL_RenderDrawPoint(renderer, x0 - x, y0 + y);
        SDL_RenderDrawPoint(renderer, x0 - x, y0 - y);
        SDL_RenderDrawPoint(renderer, x0 - y, y0 - x);
        SDL_RenderDrawPoint(renderer, x0 + y, y0 - x);
        SDL_RenderDrawPoint(renderer, x0 + x, y0 - y);

        if (err <= 0)
        {
            y++;
            err += dy;
            dy += 2;
        }
    
        if (err > 0)
        {
            x--;
            dx += 2;
            err += dx - d;
        }
    }
}

void Drawer::node(TreeNode* node) {
	switch(node->type) {
		case GAP:
		setColor(OwenColors::GAP_COLOR[0],OwenColors::GAP_COLOR[1],OwenColors::GAP_COLOR[2],OwenColors::GAP_COLOR[3]);
		break;
		case CLOUD:
		setColor(OwenColors::CLOUD_COLOR[0],OwenColors::CLOUD_COLOR[1],OwenColors::CLOUD_COLOR[2],OwenColors::CLOUD_COLOR[3]);
		break;
		
		case TRIO:
		setColor(OwenColors::TRIO_COLOR[0],OwenColors::TRIO_COLOR[1],OwenColors::TRIO_COLOR[2],OwenColors::TRIO_COLOR[3]);
		break;
		
		case OTHER:
		setColor(OwenColors::OTHER_COLOR[0],OwenColors::OTHER_COLOR[1],OwenColors::OTHER_COLOR[2],OwenColors::OTHER_COLOR[3]);
		break;
		
		default:
		setColor(OwenColors::GAP_COLOR[0],OwenColors::GAP_COLOR[1],OwenColors::GAP_COLOR[2],OwenColors::GAP_COLOR[3]);
		break;
	}
	
	circle(node->x,node->y,node->radius);
	
	setColor(OwenColors::OTHER_COLOR[0],OwenColors::OTHER_COLOR[1],OwenColors::OTHER_COLOR[2],OwenColors::OTHER_COLOR[3]);
	for (int i=0; i<node->children.size(); i++) {
		double x1 = node->x;
		double y1 = node->y;
		double x2 = node->children.at(i).x;
		double y2 = node->children.at(i).y;
		
		line(x1,y1,x2,y2);
	}
}

void Drawer::graph(GNTGraph* graph) {
	TreeNode* node = graph->root;
	vector<TreeNode>* children = &(node->children);
	
	this->node(node);
	
	if (children != NULL) {
		for (int i=0; i<children->size(); i++) {
			this->node(&(children->at(i)));
		}
	}
}
