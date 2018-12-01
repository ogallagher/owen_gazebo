# gnt_raw
# Owen Gallagher
# 30 noviembre 2018

# Descripción de mensajes que se envían en el tópico /explorer/gnt_raw

# Después de que owen_gazebo_client haya leído las medidas del láser y
# las haya etiquetado con OwenConstants::NodeType{GAP,CLOUD,TRIO,OTHER}, 
# owen_explorer_grapher recibe los etiquetas y actualiza gntCurrent.
# Entonces, owen_explorer_grapher compara gntCurrent y gntPrevious para
# encontrar nodos aparecidos, desaparecidos, unidos y divididos.

int32 length
uint8[] labels
