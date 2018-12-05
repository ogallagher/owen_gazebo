#ifndef GNT_H
#define GNT_H

/*

Owen Gallagher
30 noviembre 2018

*/

#include <vector>

#include "owen_constants.h"

using namespace std;

class GNTNode {
	public:
		OwenConstants::NodeType type;
		bool explored;
		bool visible;
		
		GNTNode(OwenConstants::NodeType t);
		GNTNode(GNTNode* other);
};

class GNT {
	public:
		vector<GNTNode> nodes;
		
		GNT();
		void update(vector<uint8_t>* tags); //agregar nodos sin contexto del pasado; preparar etiquetas,
											//eliminando nubes repetidas y agregando trios invisibles
		void prune(GNT* pruned); //genera una copia de si mismo, sin nodos WALL y OTHER, y lo almacena en pruned
};

#endif
