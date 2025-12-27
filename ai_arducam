from cvzone.HandTrackingModule import HandDetector
import cv2
import serial
import time
from math import atan2, pi
import math


# ==========================
# Config (TODO: sortir ça dans un fichier .json/.ini)
# ==========================
CAMERA_INDEX = 0
SERIAL_PORT = "COM8"
SERIAL_BAUDRATE = 115200

# Anti-bruit (combien de frames consécutives avant d'envoyer la commande)
STABLE_FRAMES_INDEX = 15
STABLE_FRAMES_MIDDLE = 10
STABLE_FRAMES_RING = 10
STABLE_FRAMES_PINKY = 10
STABLE_FRAMES_THUMB = 10

# Délimitation des angles (en degrés) pour 3 états (ouvert / semi / fermé)
# TODO: calibrer ces seuils selon ta caméra + ta main
THRESHOLDS = {
    "index":  [(165, 180), (110, 160), (0, 105)],
    "middle": [(170, 180), (115, 170), (0, 105)],
    "ring":   [(155, 180), (115, 145), (0, 105)],
    "pinky":  [(155, 180), (115, 145), (0, 105)],
    "thumb":  [(165, 180), (105, 160), (0, 100)],
}

# Codes envoyés au microcontrôleur (conservés tels quels)
SERIAL_CODES = {
    "index":  [b"\x01", b"\x02", b"\x04"],
    "middle": [b"\x21", b"\x22", b"\x24"],
    "ring":   [b"\x41", b"\x42", b"\x44"],
    "pinky":  [b"\x61", b"\x62", b"\x64"],
    "thumb":  [b"\x81", b"\x82", b"\x84"],
}

FINGER_LANDMARKS = {
    # (tip, pip, mcp) -> pour calculer l'angle au niveau de l'articulation
    "index":  (8,  6,  5),
    "middle": (12, 10, 9),
    "ring":   (16, 14, 13),
    "pinky":  (20, 18, 17),
    "thumb":  (4,  2,  1),
}

STABILITY_FRAMES = {
    "index":  STABLE_FRAMES_INDEX,
    "middle": STABLE_FRAMES_MIDDLE,
    "ring":   STABLE_FRAMES_RING,
    "pinky":  STABLE_FRAMES_PINKY,
    "thumb":  STABLE_FRAMES_THUMB,
}


def compute_angle_deg(point_a, point_b, point_c):
    """
    Calcule l'angle ABC (en degrés), où B est le point central.
    Retourne un angle entre 0 et 180.
    """
    ax, ay = point_a[0] - point_b[0], point_a[1] - point_b[1]
    cx, cy = point_c[0] - point_b[0], point_c[1] - point_b[1]

    ang_a = atan2(ay, ax)
    ang_c = atan2(cy, cx)

    if ang_a < 0:
        ang_a += 2 * pi
    if ang_c < 0:
        ang_c += 2 * pi

    delta = (2 * pi + ang_c - ang_a) if ang_a > ang_c else (ang_c - ang_a)
    delta_deg = delta * (180 / math.pi)

    if delta_deg > 180:
        delta_deg = 360 - delta_deg

    return delta_deg


def angle_to_state_idx(finger_name, angle_deg):
    """
    Mappe un angle vers l'index d'état (0/1/2) selon THRESHOLDS.
    0: range #1, 1: range #2, 2: range #3
    """
    ranges = THRESHOLDS[finger_name]
    for state_idx, (lo, hi) in enumerate(ranges):
        if lo < angle_deg <= hi:
            return state_idx
    return None  # hors plage


def main():
    # ==========================
    # Init caméra + détection main
    # ==========================
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    hand_detector = HandDetector(detectionCon=0.5, maxHands=2)

    # ==========================
    # Init série
    # ==========================
    serial_link = serial.Serial(
        port=SERIAL_PORT,
        baudrate=SERIAL_BAUDRATE,
        bytesize=8,
        stopbits=1,
        writeTimeout=0,
        timeout=0
    )
    time.sleep(2)
    serial_link.readline()

    # ==========================
    # Compteurs de stabilité (anti-spam UART)
    # clé = (finger_name, state_idx)
    # ==========================
    stable_count = {
        (fname, sidx): 0
        for fname in FINGER_LANDMARKS.keys()
        for sidx in range(3)
    }

    # TODO: ajouter une option pour logger/visualiser les angles en temps réel (debug/calibration)

    while True:
        ok, frame = cap.read()
        if not ok:
            # TODO: gérer proprement la caméra si elle ne répond pas
            continue

        hands, frame = hand_detector.findHands(frame)

        try:
            if hands:
                hand1 = hands[0]
                lm_list_1 = hand1["lmList"]

                # Calcule les angles des 5 doigts (en gardant une structure claire)
                finger_angles = {}
                for finger_name, (tip, pip, mcp) in FINGER_LANDMARKS.items():
                    a = lm_list_1[tip][:2]
                    b = lm_list_1[pip][:2]
                    c = lm_list_1[mcp][:2]
                    finger_angles[finger_name] = compute_angle_deg(a, b, c)

                # Flag de début de trame
                serial_link.write(b"\xFF")

                # Pour chaque doigt -> déterminer l'état -> appliquer anti-bruit -> envoyer code
                for finger_name, ang in finger_angles.items():
                    state_idx = angle_to_state_idx(finger_name, ang)
                    if state_idx is None:
                        # Angle hors seuils: on reset les compteurs du doigt
                        for s in range(3):
                            stable_count[(finger_name, s)] = 0
                        continue

                    # Incrémente seulement le compteur du state courant, reset les autres
                    for s in range(3):
                        if s == state_idx:
                            stable_count[(finger_name, s)] += 1
                        else:
                            stable_count[(finger_name, s)] = 0

                    required = STABILITY_FRAMES[finger_name]
                    if stable_count[(finger_name, state_idx)] >= required:
                        code = SERIAL_CODES[finger_name][state_idx]
                        serial_link.write(code)
                        print(f"[UART] doigt={finger_name} angle={ang:.1f} état={state_idx} code={code}")

                        # IMPORTANT: ton code original faisait `countX == 0` (comparaison),
                        # ici on remet vraiment à zéro après envoi.
                        stable_count[(finger_name, state_idx)] = 0

            # Si 2 mains détectées: exemple de distance (laissé comme dans ton code)
            if len(hands) == 2:
                hand2 = hands[1]
                lm_list_2 = hand2["lmList"]

                # Distance entre index (landmark 8) des deux mains
                length, info, frame = hand_detector.findDistance(lm_list_1[8], lm_list_2[8], frame)

                # TODO: utiliser `length` pour une fonctionnalité (ex: mode/gesture)
        except Exception:
            # TODO: logger l'exception au lieu de faire silence total
            continue

        cv2.imshow("Image", frame)
        if (cv2.waitKey(1) & 0xFF) == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
    serial_link.close()


if __name__ == "__main__":
    main()
