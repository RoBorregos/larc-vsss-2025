import numpy as np
from scipy.optimize import linear_sum_assignment

class Tracker:
    def __init__(self, predefined_ports):
        """
        Inicializa el tracker.

        Args:
            predefined_ports (list): Lista de IDs predefinidos disponibles.
        """
        self.predefined_ports = predefined_ports  # IDs disponibles
        self.active_tracks = {}  # Diccionario de seguimiento {ID: posición}
        self.next_id_index = 0  # Índice del próximo ID a asignar
        self.inactive_tracks = {}
        self.max_inactive_frames = 5

    def update(self, detections):
        """
        Actualiza el estado del tracker con las nuevas detecciones.

        Args:
            detections (list): Lista de detecciones actuales [(x, y), ...].

        Returns:
            dict: Diccionario con IDs asignados y sus posiciones correspondientes.
        """
        # Convertir detecciones a matriz bidimensional
        detections = np.atleast_2d(detections)
        
        # Combinar tracks activos e inactivos para la asignación
        all_tracks = {**self.active_tracks, **{k: v[0] for k, v in self.inactive_tracks.items()}}
        all_ids = list(all_tracks.keys())
        all_positions = np.atleast_2d(list(all_tracks.values()))

        # Si no hay tracks o detecciones, manejar los casos especiales
        if all_positions.size == 0:
            # No hay tracks activos ni inactivos, asignar nuevos IDs a todas las detecciones
            for detection in detections:
                if self.next_id_index < len(self.predefined_ports):
                    track_id = self.predefined_ports[self.next_id_index]
                    self.active_tracks[track_id] = detection
                    self.next_id_index += 1
            return self.active_tracks

        if detections.size == 0:
            # No hay detecciones, mover todos los tracks activos a inactivos
            for track_id, position in self.active_tracks.items():
                self.inactive_tracks[track_id] = (position, 1)
            self.active_tracks = {}
            return self.active_tracks

        # Calcular las distancias entre todos los tracks y las detecciones
        cost_matrix = np.linalg.norm(all_positions[:, None, :] - detections[None, :, :], axis=2)

        # Resolver el problema de asignación con el algoritmo de asignación húngaro
        row_ind, col_ind = linear_sum_assignment(cost_matrix)

        # Actualizar tracks con las detecciones asignadas
        unmatched_tracks = set(range(len(all_positions))) - set(row_ind)
        unmatched_detections = set(range(len(detections))) - set(col_ind)

        # Actualizar los tracks existentes
        self.active_tracks = {}
        for r, c in zip(row_ind, col_ind):
            track_id = all_ids[r]
            self.active_tracks[track_id] = detections[c]

            # Si el track estaba inactivo, eliminarlo de los inactivos
            if track_id in self.inactive_tracks:
                del self.inactive_tracks[track_id]

        # Manejar nuevas detecciones no asignadas
        for c in unmatched_detections:
            if self.next_id_index < len(self.predefined_ports):
                track_id = self.predefined_ports[self.next_id_index]
                self.active_tracks[track_id] = detections[c]
                self.next_id_index += 1

        # Manejar tracks desaparecidos
        for r in unmatched_tracks:
            track_id = all_ids[r]
            if track_id in self.active_tracks:
                del self.active_tracks[track_id]
            elif track_id in self.inactive_tracks:
                # Incrementar el contador de frames inactivos
                position, frames_inactive = self.inactive_tracks[track_id]
                if frames_inactive + 1 < self.max_inactive_frames:
                    self.inactive_tracks[track_id] = (position, frames_inactive + 1)
                else:
                    # Eliminar el track si supera el límite de frames inactivos
                    del self.inactive_tracks[track_id]
            else:
                # Mover el track a inactivos
                self.inactive_tracks[track_id] = (all_tracks[track_id], 1)

        return self.active_tracks

# Ejemplo de uso
if __name__ == "__main__":
    predefined_ports = [1001, 1002, 1003, 1004]  # IDs predefinidos
    tracker = Tracker(predefined_ports)

    # Ejemplo de detecciones en diferentes frames
    detections_frame_1 = [(100, 200), (150, 250)]
    detections_frame_2 = [(102, 202), (148, 248), (300, 400)]
    detections_frame_3 = [(105, 205), (299, 399)]
    detections_frame_4 = [(110, 212), (150, 255), (304, 390)]

    print("Frame 1:", tracker.update(detections_frame_1))
    print("Frame 2:", tracker.update(detections_frame_2))
    print("Frame 3:", tracker.update(detections_frame_3))
    print("Frame 4:", tracker.update(detections_frame_4))
