/*
 *
 * Implementación de GNT con rango acotado (psuedocódigo)
 *
 * Owen Gallagher
 * 22 noviembre 2018
 *
 * Prof. Murrieta Cid
 * Robótica 1
 * Univeridad de Guanajuato, GTO, México
 *
 */

GNT gntPrevious;
GNT gntCurrent;

final int INFINITY;
final int WALL = 0;
final int GAP = 1;
final int CLOUD = 2;

boolean done = false;
boolean finiteRange = true;
double angularUnit = 2;	//angulos se miden en grados
double noiseMax;			//si entre ciertas measures la superficie varía más que este valor, el robot considera que haya una brecha ahí. 
							//||m0-m1|-|m1-m2|| / |m0-m1|, donde m2 es la measure current. Cero es una frontera perfecta.				
Laser laser;
Robot robot;

class GNT {
	Node[] nodes;	//el orden de la lista es importante
					//TODO: también es importante los ángulos aproximados de los nodos para poder encontrar correspondencias entre gntPrevious y gntCurrent.
	
	void update(measures) {
		nodes = [];
		boolean cloudSwitch = false;
		
		for (int i=0; i<measures.length; i++) {
			if (measures[i].label != WALL) {
				if (measures[i] == CLOUD) {
					//TODO
				}
				else if (measures[i] == GAP) {
					//TODO: manejar el caso de brechas rodeadas por brechas, en que hay que insertar un trío de nodos BNB en vez de solo una brecha
					nodes.append(new Node(measures[i].label));
				}
			}
		}
	}
}

class Node {
	boolean isCloud;
	boolean explored;
	boolean visible;
	
	Node(isCloud) {
		if (isCloud == GAP) {
			this.isCloud = false;
		}
		else {
			this.isCloud = true;
		}
		this.explored = false;
		this.visible = true;
	}
}

class Measure {
	double measure;
	int label = WALL;
}

class Laser {
	double angularUnit = 2;	//grados entre cada medida
	double rangeMin = 0.4;	//distancia mínima robot-frontera
	double rangeMax = 4;	//qué tan lejos el robot puede ver
	double scope;	//campo de vista
	Measure[] measures;	//valores del sensor, y etiquetas (WALL,GAP,CLOUD)
	
	void read(double[] ranges) {
		for (int i=0; i<ranges.length; i++) {
			if (ranges[i] > rangeMax || ranges[i] == INFINITY) {
				measures[i] = INFINITY;
			}
			else if (ranges[i] < rangeMin) {
				measures[i] = 0
			}
			else {
				measures[i] = ranges[i]/rangeMax;
			}
		}
	}
}

class Robot {
	double radius; //se representa como un disco; este valor se determina por la parte más ancha del robot. Puede ayudar a escoger un noiseMax y también escoger un widthMin
	RosPublisher robotDriver; //publica comandos para mover el robot
	RosSubscriber robotReader; //escucha y lee la configuración del robot
}

class Explorer() {	//Aplicación ejecutable
	void main() {
		gntPrevious.root();	//crear el nodo raíz
		gntCurrent.root();

		while (!done) {
			laser.read();	//abre el láser y hace un escaneo
		
			labelMeasures(); // encontrar brechas y nubes
			
			gntPrevious = gntCurrent; //copia profunda; actualizar gntPrevious
			gntCurrent.update(laser.measures);	//crear nuevo GNT
			
			gntChanges(finiteRange); //ratrear cambios de nodes: uniones, divisiones, apariciones y desapariciones
		}
		
		print("Exploración completa");
	}
	
	void labelMeasures() {
		Measure current;
		Measure previous;
		Measure old;
		double noise;
		
		current.measure = laser.measures[-1];
		previous.measure = laser.measures[-2];
		
		//checar si medidas son brechas o nubes
		for (int i=0; i<laser.measures.length; i++) {
			//actualizar medidas
			old = previous;
			previous = current;
			current.measure = laser.measures[i];
			
			if (current.measure <= rangeMax) {	//brecha o frontera
				if (previous.label == CLOUD) {
					current.label = GAP; //fin de nube
				}
				else {
					//determinar ruido (función de error)
					noise = abs(old-previous);
					noise = abs(abs(previous-current)-noise) / noise;
					
					//determinar si actual es una brecha	
					if (noise > noiseMax) {	
						current.label = GAP;
					}
					else {
						current.label = WALL;
					}
				}
			}
			else {	//nube
				current.label = CLOUD;
				
				if (previous.label == WALL) { //inicio de nube
					previous.label = GAP;
				}
			}
		}
	}
	
	void gntChanges(finite) {
		int[] changes; //índices de unión, división, aparición, desaparición
		
		for (int i=0; i<gntCurrent.nodes.length; i++) {
			if (gntCurrent.nodes[i] != gntPrevious.nodes[i]) {
				changes.append(i);
			}
		}
		
		if (changes.length > 0) {
			if (finite) {
				//TODO: implementar según los apuntes en realtimeboard
			}
			else {
				//TODO: supongo que esta parte será usada primero, porque aunque he pensado más en la primera,
				//		el caso de rango infinito debería ser más sencillo.
				//		Implementar según los apuntes en realtimeboard
			}
		}
	}
}
