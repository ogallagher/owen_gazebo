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

//agregar nodos sin contexto del pasado; preparar etiquetas, eliminando nubes repetidas y agregando trios invisibles.
//TODO: este metodo podria ser mejorado mucho si a la vez construye su forma recortada (sin WALL y OTHER). Asi no tendria
//		tantos ciclos para saltar por estos nodos generalmente insignificantes.
void GNT::update(vector<uint8_t>* tags) {
	nodes.clear();
	OwenConstants::NodeType type = OwenConstants::OTHER;
	bool gapSwitch = false; //brechas rodeadas por brechas: TRIOS
	int cloudIndex = -1;
	int gapCounter = 0;
	
	for (int i=0; i<tags->size(); i++) {
		type = static_cast<OwenConstants::NodeType>((*tags)[i]);
		
		if (type == OwenConstants::CLOUD) {
			nodes.push_back(new GNTNode(OwenConstants::OTHER));
			
			if (cloudIndex < 0 && gapCounter != 0) { //inicio de la nube
				cloudIndex = nodes.size()-1;
			}
			
			gapSwitch = false;
		}
		else if (type == OwenConstants::GAP) { 
			if (cloudIndex >= 0) { //fin de la nube; calcula el punto medio
				cloudIndex = (nodes.size()-1 + cloudIndex) / 2;
				nodes[cloudIndex].type = OwenConstants::CLOUD;
				
				cloudIndex = -1;
			}
			
			if (nodes.size() > 0) {
				if (gapSwitch) { //tres brechas seguidas; agrega un trio a la brecha previa
					int h = nodes.size()-2;
					bool trioDone = false;
					
					if (h < 0) {
						trioDone = true;
					}
					
					while (!trioDone) {
						if (nodes[h].type == OwenConstants::GAP || nodes[h].type == OwenConstants::TRIO) {
							nodes[h].type = OwenConstants::TRIO;
							trioDone = true;
						}
						else if (h == 0) {
							trioDone = true;
						}
						else {
							h--;
						}
					}
				}
				else {
					int h = nodes.size()-2;
					bool gapDone = false;
					if (h < 0) {
						gapDone = true;
					}
					
					while (!gapDone) {
						if (nodes[h].type == OwenConstants::GAP || nodes[h].type == OwenConstants::TRIO) {
							gapSwitch = true;
							gapDone = true;
						}
						else if (nodes[h].type == OwenConstants::CLOUD || h == 0) {
							gapDone = true;
						}
						
						h--;
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
	
	//casos a los extremos: nubes
	if (cloudIndex >= 0) {
		bool cloudDone = false;
		int j = 0;
		while (!cloudDone) {
			if (nodes[j].type == OwenConstants::GAP) { //fin de nube; calcula el punto medio
				j = j+nodes.size()-1;
				cloudIndex = (cloudIndex+j)/2;
				cloudIndex = cloudIndex % nodes.size();
				
				nodes[cloudIndex].type = OwenConstants::CLOUD;
				cloudIndex = -1;
				cloudDone = true;
			}
			j++;
			if (j == nodes.size()-1) {
				cloudDone = true;
			}
		}
	}
	
	//casos a los extremos: trios
	OwenConstants::NodeType last;
	OwenConstants::NodeType first;
	int lasti = -1;
	int firsti = -1;
	int ends = 0;
	bool endsDone = false;
	bool nextDone = false;
	
	while (!endsDone) {
		first = nodes[ends].type;
		
		if (first != OwenConstants::WALL && first != OwenConstants::OTHER) {
			if (first == OwenConstants::CLOUD) {
				nextDone = true; //no hay trio por los extremos
			}
			else {
				firsti = ends;
			}
			endsDone = true; //encontramos el primer nodo importante
		}
	
		ends++;
		if (ends == nodes.size()-1) {
			endsDone = true;
			nextDone = true;
		}
	}
	if (!nextDone) {
		ends = nodes.size()-1;
		endsDone = false;
		
		while (!endsDone) {
			last = nodes[ends].type;
			
			if (last != OwenConstants::WALL && last != OwenConstants::OTHER) {
				if (last == OwenConstants::CLOUD) {
					nextDone = true; //no hay trio por los extremos
				}
				else {
					lasti = ends;
				}
				endsDone = true; //encontramos el ultimo nodo importante
			}
		
			ends--;
			if (ends == 0) {
				endsDone = true;
				nextDone = true;
			}
		}
	}
	
	if (!nextDone) {
		int next = firsti+1;
		bool nextDone = false;
		while (!nextDone && next < nodes.size()-1) {
			if (nodes[next].type == OwenConstants::WALL || nodes[next].type == OwenConstants::OTHER) {
				next++;
			}
			else {
				if (nodes[next].type != OwenConstants::CLOUD) { //el segundo nodo hace que el primero sea un trio
					nodes[first].type = OwenConstants::TRIO;
				}
				nextDone = true;
			}
		}

		
		next = lasti-1;
		nextDone = false;
		while (!nextDone && next > 0) {
			if (nodes[next].type == OwenConstants::WALL || nodes[next].type == OwenConstants::OTHER) {
				next--;
			}
			else {
				if (nodes[next].type != OwenConstants::CLOUD) { //el penultimo nodo hace que el ultimo sea un trio
					nodes[last].type = OwenConstants::TRIO;
				}
				nextDone = true;
			}
		}
	}
	
	//cout << "#gaps: " << to_string(gapCounter) << endl;
}

void GNT::prune(GNT* pruned) { //genera una copia de si mismo, sin nodos WALL y OTHER
	pruned->nodes.clear();
	
	OwenConstants::NodeType type;
	for (int i=0; i<nodes.size(); i++) {
		type = nodes[i].type;
		
		if (type != OwenConstants::WALL && type != OwenConstants::OTHER) {
			pruned->nodes.push_back(new GNTNode(type));
		}
	}
}
