#ifndef OWEN_CONSTANTS_H
#define OWEN_CONSTANTS_H

/*

Owen Gallagher
30 noviembre 2018
Robotica 1

*/

#include <stdint.h>

using namespace std;

class OwenConstants {
	public:
		enum NodeType : uint8_t {GAP,CLOUD,WALL,TRIO,OTHER};
		static int constexpr TRIO_COLOR[4] = {0xDD,0x00,0xFF,0xFF}; //morado
		static int constexpr OTHER_COLOR[4] = {0xFF,0xFF,0xFF,0xFF}; //blanco
		static int constexpr GAP_COLOR[4] = {0x00,0x00,0xFF,0xFF}; //azul
		static int constexpr CLOUD_COLOR[4] = {0xFF,0x00,0x00,0xFF}; //rojo
		static int constexpr WALL_COLOR[4] = {0x00,0xFF,0x00,0xFF}; //verde
};


#endif
