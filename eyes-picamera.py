import math
import time
import os

# os.environ["OPENCV_VIDEOIO_MSMF_ENABLE_HW_TRANSFORMS"] = "0"
import cv2
import mediapipe as mp
import numpy as np
import RPi.GPIO as gpio

from picamera2 import Picamera2

# se usan los números físicos de los pines de la tarjeta
BLUE_LED = 16
RED_LED = 18
BUZZER = 22

# Configuración de los pines de entrada y salida
gpio.setmode(gpio.BOARD)
gpio.setwarnings(False)

# Se configuran los pines como entradas
gpio.setup(BLUE_LED, gpio.OUT)
gpio.setup(RED_LED, gpio.OUT)
gpio.setup(BUZZER, gpio.OUT)

# Variable para la detección de rostro
rostro_detectado = False
EAR_promedio = 0

# Variables para el control de los tiempos
parpadeo = False
tiempo_rostro_actual = 0
tiempo_rostro_anterior = time.time()
tiempo_inicio_ojos_cerrados = 0
tiempo_final_ojos_cerrados = 0
# conteo = 0
# tiempo = 0
# final = 0
# conteo_sue = 0
# muestra = 0

# Variable para el conteo de los fps
tiempo_actual_fps = 0
tiempo_anterior_fps = time.tiem()
fps = 0

face_mesh = mp.solutions.face_mesh.FaceMesh(
    max_num_faces=1,
    refine_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5,
)

left_eye_idx = [362, 385, 387, 263, 373, 380]  # puntos ojo izquierdo
right_eye_idx = [33, 160, 158, 133, 153, 144]  # puntos ojo derecho


def calcular_ear(ojo):
    p1, p2, p3, p4, p5, p6 = ojo  # puntos de los ojos mediante un array
    # distancias euclidianas np.linalg.norm()
    distancia1 = math.sqrt((p2[0] - p6[0]) ** 2 + (p2[1] - p6[1]) ** 2)  # p2 -- p6
    distancia2 = math.sqrt((p3[0] - p5[0]) ** 2 + (p3[1] - p5[1]) ** 2)  # p3 -- p5
    distancia3 = math.sqrt((p1[0] - p4[0]) ** 2 + (p1[1] - p4[1]) ** 2)  # p1 -- p4

    ear = (distancia1 + distancia2) / (2.0 * distancia3)
    # print(ear, end="\r")
    return ear


# Initialize PiCamera
picam2 = Picamera2()
# picam2.preview_configuration.main.size = (1280, 720)
# picam2.preview_configuration.main.format = "RGB888"
# picam2.preview_configuration.align()
# picam2.configure("preview")

camera_config = picam2.create_video_configuration(
    main={"format": "XRGB8888", "size": (640, 480)}
)
picam2.configure(camera_config)

picam2.start()

gpio.output(BLUE_LED, True)

time.sleep(0.1)

try:
    while True:
        frame = picam2.capture_array()

        if frame is not None:
            fps += 1

        tiempo_actual_fps = time.time()
        fps = 1 / (tiempo_actual_fps - tiempo_anterior_fps)
        tiempo_anterior_fps = tiempo_actual_fps

        alto, ancho, _ = frame.shape

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        resultados = face_mesh.process(frame_rgb)

        tiempo_rostro_actual = time.time()

        if tiempo_rostro_actual - tiempo_rostro_anterior >= 0.7:
            gpio.output(BLUE_LED, not gpio.input(BLUE_LED))

        # def punto_vista(ojos):
        #     nuevosPuntos = np.array(
        #         [int(landmarks[i].x * ancho), int(landmarks[i].y * alto)] for i in ojos
        #     )
        #     return nuevosPuntos

        if resultados.multi_face_landmarks:
            rostro_detectado = True
            landmarks = resultados.multi_face_landmarks[0].landmark
            ojo_izq = [] # iran las coordenadas convertidas en puntos
            ojo_der = [] # usar los array de los ojos
            

            gpio.output(BLUE_LED, True)

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

            # if EAR_promedio > 0.17:
            #     color = (0, 255, 0)  # verde si ojo abierto
            # else:
            #     color = (0, 0, 255)  # rojo si cerrado
            #     texto = "Dormido"
            #     cv2.putText(frame, texto, (30, 150), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            # texto = f"EAR: {EAR_promedio:.2f}"
            # cv2.putText(frame, texto, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)

            # cv2.putText(
            #     frame,
            #     f"parpadeos: {int(conteo)}",
            #     (20, 60),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 255, 0),
            #     2,
            # )
            # cv2.putText(
            #     frame,
            #     f"Micro suenos: {int(conteo_sue)}",
            #     (350, 60),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 255, 0),
            #     2,
            # )
            # cv2.putText(
            #     frame,
            #     f"Duracion: {int(muestra)}",
            #     (160, 100),
            #     cv2.FONT_HERSHEY_SIMPLEX,
            #     1,
            #     (0, 255, 0),
            #     2,
            # )

            if EAR_promedio <= 0.17 and parpadeo is False:
                tiempo_inicio_ojos_cerrados = time.time()
                parpadeo = True
            elif EAR_promedio > 0.17 and parpadeo is True:
                tiempo_final_ojos_cerrados = time.time()
                parpadeo = False

            if round(tiempo_final_ojos_cerrados - tiempo_inicio_ojos_cerrados) >= 3:
                gpio.output(RED_LED, True)
                gpio.output(BUZZER, True)

            # if EAR_izq <= 0.17 and EAR_der <= 0.17 and parpadeo is False:
            #     conteo += 1
            #     parpadeo = True
            #     tiempo_inicio_ojos_cerrados = time.time()
            # elif EAR_izq > 0.17 and EAR_der > 0.17 and parpadeo is True:
            #     parpadeo = False
            #     final = time.time()
            # tiempo = round(final - tiempo_inicio_ojos_cerrados, 0)

            # if tiempo >= 3:
            #     conteo_sue += 1
            #     muestra = tiempo
            #     tiempo_inicio_ojos_cerrados = 0
            #     final = 0
        else:
            rostro_detectado = False
            gpio.output(RED_LED, False)
            gpio.output(BUZZER, False)


        print(f"fps: {int(fps)}, rostro detectado: {rostro_detectado}, EAR promedio: {EAR_promedio}", end="\r")

except KeyboardInterrupt:
    print("Programa detenido por el usuario")
    # cv2.imshow("ojos", frame_rgb)
    # if cv2.waitKey(1) == ord("q"):
    picam2.stop()
    gpio.output(BLUE_LED, False)
    gpio.output(RED_LED, False)
    gpio.output(BUZZER, False)
    # break
