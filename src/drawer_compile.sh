#Compilar drawer, ejemplo de graficacion con la libreria sdl2
g++ drawer.cpp $(pkg-config --cflags --libs sdl2) -o drawer

