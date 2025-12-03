import math
import time
import cv2
import mediapipe as mp
import numpy as np
import os

def clearConsole():
    command = "clear"
    if os.name in ("nt", "dos"):
        command = "cls"
    os.system(command)

PARPADEO = False
CONTEO = 0
TIEMPO = 0
INICIO = 0
FINAL = 0
CONTEO_SUE = 0
MUESTRA = 0

mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(refine_landmarks=True, max_num_faces=1)

left_eye_idx = [362, 385, 387, 263, 373, 380]  # puntos ojo izquierdo
right_eye_idx = [33, 160, 158, 133, 153, 144]  # puntos ojo derecho


def calcular_ear(ojo):
    p1, p2, p3, p4, p5, p6 = ojo  # puntos de los ojos mediante un array
    # distancias euclidianas np.linalg.norm()
    distancia1 = math.sqrt((p2[0] - p6[0]) * 2 + (p2[1] - p6[1]) * 2)  # p2 -- p6
    distancia2 = math.sqrt((p3[0] - p5[0]) * 2 + (p3[1] - p5[1]) * 2)  # p3 -- p5
    distancia3 = math.sqrt((p1[0] - p4[0]) * 2 + (p1[1] - p4[1]) * 2)  # p1 -- p4

    ear = (distancia1 + distancia2) / (2.0 * distancia3)
    return ear


camara = cv2.VideoCapture(0)
while True:
    ok, frame = camara.read()
    if not ok:
        break
    frame = cv2.flip(frame, 1)
    alto, ancho, _ = frame.shape

    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    resultados = face_mesh.process(frame_rgb)
    """
    def punto_vista(ojos):
        nuevosPuntos = np.array([int(landmarks[i].x * ancho), 
                                 int(landmarks[i].y * alto)]
                              for i in ojos)
        return nuevosPuntos
    """
    if resultados.multi_face_landmarks:
        print(f"Rostro detectado = {True}", end="\r")
        landmarks = resultados.multi_face_landmarks[0].landmark
        ojo_izq = []
        ojo_der = []  # iran las coordenadas convertidas en puntos
        # usar los array de los ojos

        for i in left_eye_idx:
            x = int(landmarks[i].x * ancho)
            y = int(landmarks[i].y * alto)
            ojo_izq.append((x, y))

        for i in right_eye_idx:
            x = int(landmarks[i].x * ancho)
            y = int(landmarks[i].y * alto)
            ojo_der.append((x, y))

        EAR_izq = calcular_ear(ojo_izq)
        EAR_der = calcular_ear(ojo_der)

        EAR_promedio = (EAR_izq + EAR_der) / 2.0

        # for punto in ojo_izq:
        #     cv2.circle(frame, punto, 2, (0, 255, 0), -1)

        # for punto in ojo_der:
        #     cv2.circle(frame, punto, 2, (0, 255, 0), -1)
        """
        if EAR_promedio > 0.17:
            color = (0, 255, 0)  # verde si ojo abierto
        else:
            color = (0, 0, 255)  # rojo si cerrado
            texto = "Dormido"
            cv2.putText(frame, texto, (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        """
        # texto = f"EAR: {EAR_promedio:.2f}"
        # cv2.putText(frame, texto, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        
        #se puso en comentario el 25/11/25

        # cv2.putText(
        #     frame,
        #     f"parpadeos: {int(CONTEO)}",
        #     (20, 60),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     1,
        #     (0, 255, 0),
        #     2,
        # )
        # cv2.putText(
        #     frame,
        #     f"Micro suenos: {int(CONTEO_SUE)}",
        #     (350, 60),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     1,
        #     (0, 255, 0),
        #     2,
        # )
        # cv2.putText(
        #     frame,
        #     f"Duracion: {int(MUESTRA)}",
        #     (160, 100),
        #     cv2.FONT_HERSHEY_SIMPLEX,
        #     1,
        #     (0, 255, 0),
        #     2,
        # )
        
        if EAR_izq <= 0.17 and EAR_der <= 0.17 and PARPADEO is False:
            CONTEO += 1
            PARPADEO = True
            INICIO = time.time()
            clearConsole()
        elif EAR_izq > 0.17 and EAR_der > 0.17 and PARPADEO is True:
            PARPADEO = False
            FINAL = time.time()
        TIEMPO = round(FINAL - INICIO, 0)

        if TIEMPO >= 3:
            CONTEO_SUE += 1
            MUESTRA = TIEMPO
            INICIO = 0
            FINAL = 0
            print("Microsueño = True")

    #cv2.imshow("ojos", frame)
    if 0xFF == ord("q"):
        break
camara.release()
# cv2.destroyAllWindows()