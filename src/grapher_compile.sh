#Compilar owen_explorer_grapher, ejemplo de graficacion con la libreria sdl2
g++ owen_explorer_grapher.cpp drawer.cpp gnt_graph.cpp $(pkg-config --cflags --libs sdl2) -o owen_explorer_grapher;

