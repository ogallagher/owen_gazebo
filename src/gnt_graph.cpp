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
	branchWidth = 1;
	double branchLength = 10;
	double rootRadius = 5;
    root = new TreeNode(OwenConstants::OTHER,0,0,rootRadius,branchLength);
}

GNTGraph::GNTGraph(double x, double y) {
	branchWidth = 1;
	double branchLength = 100;
	double rootRadius = 5;
    root = new TreeNode(OwenConstants::OTHER,x,y,rootRadius,branchLength);
}

void GNTGraph::update(GNT* gnt) {
	root->children.clear();
	
	int nodesLength = static_cast<int>(gnt->nodes.size());
	double branchT = -M_PI/2;
	double branchTD = 2*M_PI/nodesLength;
	double branchX = 0;
	double branchY = 0;
	
	for (int i=0; i<nodesLength; i++) {
		TreeNode treeNode(&(gnt->nodes[i])); //saca info de type,visible,explored
		
		branchX = root->reach * cos(branchT);
		branchY = root->reach * sin(branchT);
		
		treeNode.x = root->x + branchX;
		treeNode.y = root->y - branchY; //negativo porque en SDL y crece hacia abajo
		
		switch (treeNode.type) {
		case OwenConstants::TRIO:
			treeNode.radius = 6;
			break;
			
		case OwenConstants::GAP:
		case OwenConstants::CLOUD:
			treeNode.radius = 4;
			break;
			
		default:
			treeNode.radius = 2;
			break;
		}
		
		root->addChild(&treeNode);
		
		branchT += branchTD;
	}
}

