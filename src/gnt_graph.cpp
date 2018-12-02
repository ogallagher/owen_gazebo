/*

Owen Gallagher
28 noviembre 2018
Robotica

*/

#include <cmath>

#include "gnt_graph.h"

using namespace std;

TreeNode::TreeNode() {
	x = 0;
	y = 0;
	radius = 5;
	reach = 50;
	type = OwenConstants::OTHER;
	visible = true;
	explored = false;
}

TreeNode::TreeNode(OwenConstants::NodeType t, double x, double y, double rad, double rch) {
	TreeNode();
	
	type = t;
	this->x = x;
	this->y = y;
	radius = rad;
	reach = rch;
}

TreeNode::TreeNode(GNTNode* gntNode) {
	TreeNode();
	
	type = gntNode->type;
	visible = gntNode->visible;
	explored = gntNode->explored;
}

void TreeNode::addChild(TreeNode *child) {
	children.push_back(*child);
	//cout << "Nodo agregado al árbol: " << child->x << "," << child->y << endl;
}

void TreeNode::moveTo(double x, double y) {
	this->x = x;
	this->y = y;
}

void TreeNode::move(double x, double y) {
	this->x += x;
	this->y += y;
}

GNTGraph::GNTGraph() {
	branchLength = 10;
	branchWidth = 1;
	rootRadius = 5;
    root = new TreeNode(OwenConstants::OTHER,0,0,rootRadius,branchLength);
}

GNTGraph::GNTGraph(double x, double y) {
	branchLength = 100;
	branchWidth = 1;
	rootRadius = 5;
    root = new TreeNode(OwenConstants::OTHER,x,y,rootRadius,branchLength);
}

void GNTGraph::update(GNT* gnt) {
	root->children.clear();
	
	int nodesLength = static_cast<int>(gnt->nodes.size());
	double branchT = 0;
	double branchTD = 2*M_PI/(nodesLength/skip);
	double branchX = 0;
	double branchY = 0;
	
	for (int i=0; i<nodesLength; i++) {
		TreeNode treeNode(&(gnt->nodes[i])); //saca info de type,visible,explored
		
		branchX = root->reach * cos(branchT);
		branchY = root->reach * sin(branchT);
		
		treeNode.x = root->x + branchX;
		treeNode.y = root->y - branchY; //negativo porque en SDL y crece hacia abajo
		treeNode.radius = 3; //TODO: este renglón no debería ser necesario, pero por alguna razón los nuevos nodos
		
		root->addChild(&treeNode);
		
		branchT += branchTD;
	}
}

