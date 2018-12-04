/*

Owen Gallagher
30 noviembre 2018

*/

#include <iostream>

#include "gnt.h"

using namespace std;

//GNTNode

GNTNode::GNTNode(OwenConstants::NodeType t) {
	type = t;
	visible = true;
	explored = false;
}

GNTNode::GNTNode(GNTNode* other) {
	this->type = other->type;
	this->visible = other->visible;
	this->explored = other->explored;
}

//GNT

GNT::GNT() {
	//nada
}

//agregar nodos sin contexto del pasado; preparar etiquetas, eliminando nubes repetidas y agregando trios invisibles
void GNT::update(vector<uint8_t>* tags) {
	//cout << "GNT::update()..." << endl;
	
	nodes.clear();
	OwenConstants::NodeType type = OwenConstants::OTHER;
	bool gapSwitch = false; //brechas rodeadas por brechas: TRIOS
	int cloudIndex = -1;
	int gapCounter = 0;
	
	//cout << "Starting tags processing..." << endl;
	for (int i=0; i<tags->size(); i++) {
		type = static_cast<OwenConstants::NodeType>((*tags)[i]);
		//cout << "tags[" << i << "]: " << to_string(static_cast<int>(type)) << endl;
		
		if (type == OwenConstants::CLOUD) {
			gapSwitch = false;
			
			nodes.push_back(new GNTNode(OwenConstants::OTHER));
			if (cloudIndex < 0) { //inicio de la nube
				cloudIndex = i;
			}
		}
		else if (type == OwenConstants::GAP) { 
			if (cloudIndex >= 0) { //fin de la nube; calcula el punto medio
				cloudIndex = (i-1 + cloudIndex) / 2;
				nodes[cloudIndex].type = OwenConstants::CLOUD;
				
				cloudIndex = -1;
			}
			
			if (nodes.size() > 0) {
				if (gapSwitch) { //tres brechas seguidas; agrega un trio a la brecha previa
					nodes.back().type = OwenConstants::TRIO;
				}
				else if (nodes.back().type == OwenConstants::GAP) {
					gapSwitch = true;
				}
			}
			
			nodes.push_back(new GNTNode(OwenConstants::GAP));
			gapCounter++;
		}
		else if (type == OwenConstants::WALL) {
			gapSwitch = false;
			cloudIndex = -1;
			
			nodes.push_back(new GNTNode(OwenConstants::WALL));
		}
		else {
			gapSwitch = false;
			cloudIndex = -1;
			
			nodes.push_back(new GNTNode(OwenConstants::OTHER));
		}
	}
	
	cout << "#gaps: " << to_string(gapCounter) << endl;
}
