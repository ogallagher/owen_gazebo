/*

Owen Gallagher
28 noviembre 2018
Robotica

*/

#include "gnt_graph.h"

using namespace std;

char TRIO_COLOR[4] = {0xDD,0x00,0xFF,0xFF}; //morado
char OTHER_COLOR[4] = {0xFF,0xFF,0xFF,0xFF}; //blanco
char GAP_COLOR[4] = {0x00,0x00,0xFF,0xFF}; //azul
char CLOUD_COLOR[4] = {0xFF,0x00,0x00,0xFF}; //rojo

TreeNode::TreeNode() {
	x = 0;
	y = 0;
	radius = 2;
	reach = 20;
	type = OTHER;
	visible = true;
	explored = false;
}

TreeNode::TreeNode(NodeType t, double x, double y, double rad, double rch) {
	TreeNode();
	
	type = t;
	this->x = x;
	this->y = y;
	radius = rad;
	reach = rch;
}

void TreeNode::addChild(TreeNode* child) {
	children.push_back(child);
	cout << "Nodo agregado al árbol: " << child->x << "," << child->y << endl;
}

GNTGraph::GNTGraph() {
	branchLength = 20;
	branchWidth = 1;
	nodeRadius = 5;
    root = new TreeNode(OTHER,0,0,nodeRadius,branchLength);
}

GNTGraph::GNTGraph(double x, double y) {
	branchLength = 20;
	branchWidth = 1;
	nodeRadius = 5;
    root = new TreeNode(OTHER,x,y,nodeRadius,branchLength);
}