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
	nodes.clear();
	OwenConstants::NodeType type = OwenConstants::OTHER;
	bool gapSwitch = false; //brechas rodeadas por brechas: TRIOS
	int cloudIndex = -1;
	int gapCounter = 0;
	
	for (int i=0; i<tags->size(); i++) {
		if (nodes.size() > 0) {
			cout << to_string(i) << ": " << to_string(nodes[i-1].type) << endl;
		}
		
		type = static_cast<OwenConstants::NodeType>((*tags)[i]);
		
		if (type == OwenConstants::CLOUD) {
			nodes.push_back(new GNTNode(OwenConstants::OTHER));
			
			if (cloudIndex < 0 && gapCounter != 0) { //inicio de la nube
				cloudIndex = i;
			}
			else if (i+1 == tags->size()) { //completa nubes que abarcan los extremos del vector
				bool cloudDone = false;
				int j = 0;
				
				while (!cloudDone) {
					if (nodes[j].type == OwenConstants::GAP) { //fin de nube; calcula el punto medio
						j = j+i;
						cloudIndex = (cloudIndex+j)/2;
						
						if (cloudIndex > i) {
							cloudIndex -= i+1;
						}
						
						nodes[cloudIndex].type = OwenConstants::CLOUD;
						cloudIndex = -1;
						cloudDone = true;
					}
					j++;
				}
			}
			
			gapSwitch = false;
		}
		else if (type == OwenConstants::GAP) { 
			if (cloudIndex >= 0) { //fin de la nube; calcula el punto medio
				cloudIndex = (i-1 + cloudIndex) / 2;
				nodes[cloudIndex].type = OwenConstants::CLOUD;
				
				cloudIndex = -1;
			}
			
			if (nodes.size() > 0) {
				if (gapSwitch) { //tres brechas seguidas; agrega un trio a la brecha previa
					int h = i-1;
					if (h < 0) {
						h = nodes.size()-1;
					}
					bool trioDone = false;
					
					while (!trioDone && h != i) {
						if (nodes[h].type == OwenConstants::GAP || nodes[h].type == OwenConstants::TRIO) {
							nodes[h].type = OwenConstants::TRIO;
							trioDone = true;
						}
						else {
							h--;
							if (h < 0) {
								h = nodes.size()-1;
							}
						}
					}
				}
				else {
					int h = i-1;
					bool gapDone = false;
					if (h < 0) {
						h = nodes.size()-1;
					}
					
					while (!gapDone && h != i) {
						if (nodes[h].type == OwenConstants::GAP || nodes[h].type == OwenConstants::TRIO) {
							gapSwitch = true;
							gapDone = true;
						}
						else if (nodes[h].type == OwenConstants::CLOUD) {
							gapDone = true;
						}
						
						h--;
						if (h<0) {
							h = nodes.size()-1;
						}
					}
				}
			}
			
			nodes.push_back(new GNTNode(OwenConstants::GAP));
			gapCounter++;
		}
		else if (type == OwenConstants::WALL) {
			nodes.push_back(new GNTNode(OwenConstants::WALL));
		}
		else {
			nodes.push_back(new GNTNode(OwenConstants::OTHER));
		}
	}
	
	cout << "#gaps: " << to_string(gapCounter) << endl;
}
