# Optimisation avec les threads
 
import cv2
import mediapipe as mp
import numpy as np
import time
import math
import HandTrackingModule as htm
import os
import subprocess
import threading
from queue import Queue
import signal

class VideoStreamWidget(object):
    def __init__(self, src=1):
        self.capture = cv2.VideoCapture(src)
        self.capture.set(3, 640)
        self.capture.set(4, 480)
        
        # On démare le thread pour lire les frames du flux vidéo
        self.thread = threading.Thread(target=self.update, args=())
        self.thread.daemon = True
        self.status = True
        self.thread.start()
        
        self.frame_queue = Queue(maxsize=2)  # Limiter la taille de la queue / c'est ici qu'on va stocker les frames
        
    def update(self): # Récupère l'image de la webacam
        while self.status:
            if self.capture.isOpened():
                ret, frame = self.capture.read()
                if ret:
                    if not self.frame_queue.full():
                        self.frame_queue.put(cv2.flip(frame, 1))
            time.sleep(0.005)  # Petit délai pour éviter de surcharger le CPU
                
    def read_frame(self):
        return False if self.frame_queue.empty() else self.frame_queue.get()
    
    def stop(self):
        self.status = False
        self.thread.join()
        self.capture.release()

class VolumeController(object):
    def __init__(self):
        self.volume_queue = Queue()
        self.status = True
        self.thread = threading.Thread(target=self.update_volume)
        self.thread.daemon = True
        self.thread.start()
    
    def update_volume(self):
        while self.status:
            if not self.volume_queue.empty():
                vol = self.volume_queue.get()
                os.system(f'osascript -e "set volume output volume {vol}"')
            time.sleep(0.05)  # Petit délai pour éviter de surcharger le CPU
    
    def set_volume(self, vol):
        if not self.volume_queue.full():
            self.volume_queue.put(vol)
    
    def stop(self):
        self.status = False
        self.thread.join()

def signal_handler(signum, frame):
    global running
    running = False
    print("Signal reçu, arrêt en cours...")

def main():
    global running
    running = True
    signal.signal(signal.SIGINT, signal_handler)
    
    # Initialisation des objets
    video_stream = VideoStreamWidget()
    volume_controller = VolumeController()
    detector = htm.handDetector(detectionCon=0.7)
    pTime = 0

    try:
        while running:
            img = video_stream.read_frame()
            if img is False:
                continue

            img = detector.findHands(img)
            lmList = detector.findPosition(img, draw=False)
            vol = os.popen('osascript -e "output volume of (get volume settings)"').read()

            if len(lmList) != 0:
                x1, y1 = lmList[4][1], lmList[4][2]
                x2, y2 = lmList[8][1], lmList[8][2]
                cx, cy = (x1+x2)//2, (y1+y2)//2

                cv2.circle(img, (x1, y1), 8, (255,0,255), cv2.FILLED)
                cv2.circle(img, (x2, y2), 8, (255,0,255), cv2.FILLED)

                length = math.hypot(x2-x1, y2-y1)

                if cx < 100 and length < 50: # Il faut être à gauche du volume
                    subprocess.run(["osascript", "-e", 'tell application "Spotify" to playpause'])

                if cx >= 100:
                    if not hasattr(detector, 'initial_cy'):
                        detector.initial_cy = None
                        detector.initial_vol = None

                    if length < 50:
                        if detector.initial_cy is None:
                            detector.initial_cy = cy
                            current_vol = float(vol)
                            detector.initial_vol = current_vol
                        
                        delta_y = detector.initial_cy - cy
                        new_vol = detector.initial_vol + (delta_y / 2)
                        new_vol = np.clip(new_vol, 0, 100)
                        
                        cv2.circle(img, (cx, cy), 8, (0,255,0), cv2.FILLED)
                        cv2.rectangle(img, (50,400-2*int(new_vol)), (85,400), (255,0,0), cv2.FILLED)
                        cv2.putText(img, f'{int(new_vol)}',(48,430), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0), 3)
                        
                        volume_controller.set_volume(new_vol)
                    else:
                        detector.initial_cy = None
                        detector.initial_vol = None

            # Affichage des éléments visuels
            cv2.rectangle(img, (50,200), (85,400), (255,0,0), 3)
            cv2.rectangle(img, (50,400-2*int(float(vol))), (85,400), (255,0,0), cv2.FILLED)
            cv2.putText(img, f'{int(float(vol))}',(48,430), cv2.FONT_HERSHEY_COMPLEX, 1, (255,0,0), 3)

            cTime = time.time()
            fps = 1/(cTime-pTime)
            pTime = cTime

            cv2.putText(img, f'FPS: {int(fps)}',(10,70), cv2.FONT_HERSHEY_COMPLEX, 3, (255,0,255), 3)

            cv2.imshow("Image", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        # Nettoyage
        video_stream.stop()
        volume_controller.stop()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
