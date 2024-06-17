#  Hecho por: 
# Hector Alejandro Ortega Gacria
# Registro: 21310248.
# Grupo: 6E
#--------------------------------------------------------------------------------------------------------------------
import tkinter as tk  # Importamos la biblioteca Tkinter para crear la interfaz gráfica
from tkinter import messagebox  # Importamos el módulo messagebox para mostrar mensajes emergentes
import heapq  # Importamos la biblioteca heapq para utilizar colas de prioridad

class GraphApp:
    def __init__(self, root):
        self.root = root  # Guardamos la referencia de la ventana principal
        self.root.title("Dijkstra's Algorithm Visualization")  # Establecemos el título de la ventana

        self.canvas = tk.Canvas(root, width=600, height=400, bg="white")  # Creamos un canvas para dibujar el grafo
        self.canvas.pack()  # Empaquetamos el canvas en la ventana

        self.nodes = []  # Lista para almacenar los nodos
        self.edges = []  # Lista para almacenar las aristas
        self.adjacency_list = {}  # Diccionario para almacenar la lista de adyacencia del grafo

        self.canvas.bind("<Button-1>", self.add_node)  # Vinculamos el evento de clic izquierdo para agregar nodos
        self.canvas.bind("<Button-3>", self.start_edge)  # Vinculamos el evento de clic derecho para iniciar una arista
        self.canvas.bind("<ButtonRelease-3>", self.end_edge)  # Vinculamos el evento de soltar el clic derecho para finalizar una arista

        self.calculate_button = tk.Button(root, text="Calculate Shortest Path", command=self.calculate_shortest_path)  # Creamos un botón para calcular el camino más corto
        self.calculate_button.pack()  # Empaquetamos el botón en la ventana

        self.start_node_label = tk.Label(root, text="Start Node:")  # Etiqueta para el nodo de inicio
        self.start_node_label.pack(side="left")  # Empaquetamos la etiqueta en la ventana
        self.start_node_entry = tk.Entry(root)  # Campo de entrada para el nodo de inicio
        self.start_node_entry.pack(side="left")  # Empaquetamos el campo de entrada en la ventana

        self.end_node_label = tk.Label(root, text="End Node:")  # Etiqueta para el nodo de fin
        self.end_node_label.pack(side="left")  # Empaquetamos la etiqueta en la ventana
        self.end_node_entry = tk.Entry(root)  # Campo de entrada para el nodo de fin
        self.end_node_entry.pack(side="left")  # Empaquetamos el campo de entrada en la ventana

        self.start_node = None  # Inicializamos la variable para el nodo de inicio de una arista

    def add_node(self, event):
        x, y = event.x, event.y  # Obtenemos las coordenadas del clic del mouse
        node_id = len(self.nodes)  # Asignamos un identificador único al nodo basado en el tamaño de la lista de nodos
        self.nodes.append((node_id, x, y))  # Agregamos el nodo a la lista de nodos
        self.canvas.create_oval(x-10, y-10, x+10, y+10, fill="yellow")  # Dibujamos el nodo como un óvalo amarillo
        self.canvas.create_text(x, y, text=str(node_id), fill="black")  # Dibujamos el identificador del nodo
        self.adjacency_list[node_id] = []  # Inicializamos la lista de adyacencia para el nodo

    def start_edge(self, event):
        self.x0, self.y0 = event.x, event.y  # Obtenemos las coordenadas del clic del mouse
        self.start_node = self.find_nearest_node(self.x0, self.y0)  # Encontramos el nodo más cercano al clic inicial

    def end_edge(self, event):
        x1, y1 = event.x, event.y  # Obtenemos las coordenadas del soltar el clic del mouse
        end_node = self.find_nearest_node(x1, y1)  # Encontramos el nodo más cercano al soltar el clic
        if self.start_node is not None and end_node is not None and self.start_node != end_node:  # Verificamos que los nodos son válidos y diferentes
            weight = ((self.x0 - x1)**2 + (self.y0 - y1)**2)**0.5  # Calculamos el peso de la arista basado en la distancia euclidiana
            self.edges.append((self.start_node, end_node, weight))  # Agregamos la arista a la lista de aristas
            self.edges.append((end_node, self.start_node, weight))  # Agregamos la arista inversa para un grafo no dirigido
            self.adjacency_list[self.start_node].append((end_node, weight))  # Actualizamos la lista de adyacencia
            self.adjacency_list[end_node].append((self.start_node, weight))  # Actualizamos la lista de adyacencia
            self.canvas.create_line(self.x0, self.y0, x1, y1)  # Dibujamos la arista en el canvas

    def find_nearest_node(self, x, y):
        min_distance = float("inf")  # Inicializamos la distancia mínima como infinito
        nearest_node = None  # Inicializamos el nodo más cercano como None
        for node_id, nx, ny in self.nodes:  # Iteramos sobre los nodos
            distance = ((nx - x)**2 + (ny - y)**2)**0.5  # Calculamos la distancia euclidiana al nodo
            if distance < min_distance:  # Si la distancia es menor que la mínima registrada
                min_distance = distance  # Actualizamos la distancia mínima
                nearest_node = node_id  # Actualizamos el nodo más cercano
        return nearest_node  # Devolvemos el nodo más cercano

    def calculate_shortest_path(self):
        try:
            start_node = int(self.start_node_entry.get())  # Obtenemos el nodo de inicio del campo de entrada
            end_node = int(self.end_node_entry.get())  # Obtenemos el nodo de fin del campo de entrada
        except ValueError:
            messagebox.showinfo("Error", "Please enter valid node numbers")  # Mostramos un mensaje de error si la entrada no es válida
            return

        if start_node not in self.adjacency_list or end_node not in self.adjacency_list:  # Verificamos que los nodos existen en la lista de adyacencia
            messagebox.showinfo("Error", "Node does not exist")  # Mostramos un mensaje de error si los nodos no existen
            return

        distances, previous_nodes = self.dijkstra(start_node)  # Ejecutamos el algoritmo de Dijkstra
        if distances[end_node] == float("inf"):  # Si no hay camino al nodo de fin
            messagebox.showinfo("Result", "No path found")  # Mostramos un mensaje de resultado
            return

        path = []  # Inicializamos la lista para almacenar el camino más corto
        current_node = end_node  # Empezamos desde el nodo de fin
        while current_node is not None:  # Seguimos hasta llegar al nodo de inicio
            path.append(current_node)  # Agregamos el nodo actual al camino
            current_node = previous_nodes[current_node]  # Movemos al nodo anterior en el camino
        path.reverse()  # Invertimos el camino para obtenerlo desde el inicio hasta el fin

        self.highlight_path(path)  # Resaltamos el camino más corto en el canvas

    def dijkstra(self, start_node):
        distances = {node: float("inf") for node in self.adjacency_list}  # Inicializamos todas las distancias como infinito
        distances[start_node] = 0  # La distancia al nodo de inicio es 0
        previous_nodes = {node: None for node in self.adjacency_list}  # Inicializamos todos los nodos anteriores como None
        priority_queue = [(0, start_node)]  # Inicializamos la cola de prioridad con el nodo de inicio

        while priority_queue:  # Mientras haya nodos en la cola de prioridad
            current_distance, current_node = heapq.heappop(priority_queue)  # Sacamos el nodo con la distancia mínima

            if current_distance > distances[current_node]:  # Si la distancia es mayor que la registrada, continuamos
                continue

            for neighbor, weight in self.adjacency_list[current_node]:  # Iteramos sobre los vecinos del nodo actual
                distance = current_distance + weight  # Calculamos la distancia al vecino
                if distance < distances[neighbor]:  # Si la distancia es menor que la registrada
                    distances[neighbor] = distance  # Actualizamos la distancia
                    previous_nodes[neighbor] = current_node  # Actualizamos el nodo anterior
                    heapq.heappush(priority_queue, (distance, neighbor))  # Agregamos el vecino a la cola de prioridad

        return distances, previous_nodes  # Devolvemos las distancias y los nodos anteriores

    def highlight_path(self, path):
        for i in range(len(path) - 1):  # Iteramos sobre el camino
            node1 = path[i]  # Nodo actual
            node2 = path[i + 1]  # Nodo siguiente
            x0, y0 = self.nodes[node1][1], self.nodes[node1][2]  # Coordenadas del nodo actual
            x1, y1 = self.nodes[node2][1], self.nodes[node2][2]  # Coordenadas del nodo siguiente
            self.canvas.create_line(x0, y0, x1, y1, fill="red", width=2)  # Dibujamos la línea del camino en rojo

if __name__ == "__main__":
    root = tk.Tk()  # Creamos la ventana principal
    app = GraphApp(root)  # Creamos una instancia de la clase GraphApp
    root.mainloop()  # Iniciamos el bucle principal de la interfaz gráfica