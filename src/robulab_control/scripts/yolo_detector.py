#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import cv2
from ultralytics import YOLO
from std_msgs.msg import Bool

class BotellaSeguidor:
    def __init__(self, modelo_path='botella.pt', cam_index=0):
        rospy.init_node('seguidor_botella_node')
        self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.Subscriber('/modo_botella', Bool, self.callback_modo)

        self.cap = cv2.VideoCapture(cam_index)
        self.activo = False

        if not self.cap.isOpened():
            rospy.logerr("No se pudo abrir la cámara")
            exit(1)

        self.model = YOLO(modelo_path)
        rospy.loginfo("Modelo YOLOv8 cargado correctamente.")

    def callback_modo(self, msg):
        self.activo = msg.data
        estado = "ACTIVO" if self.activo else "INACTIVO"
        rospy.loginfo(f"[MODO BOTELLA] {estado}")

    def seguir_botella(self):
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if not self.activo:
                self.pub_cmd_vel.publish(Twist())  # parar robot
                rospy.sleep(0.1)
                continue

            ret, frame = self.cap.read()
            if not ret:
                rospy.logwarn("No se pudo leer el frame de la cámara")
                continue

            results = self.model.predict(frame, imgsz=640, conf=0.5)
            detections = results[0].boxes.cpu().numpy() if results[0].boxes else []

            msg = Twist()
            if len(detections) > 0:
                box = detections[0].xyxy[0]
                x1, y1, x2, y2 = box
                cx = (x1 + x2) / 2

                width = frame.shape[1]
                error_x = cx - width / 2

                msg.linear.x = 0.15
                msg.angular.z = -0.002 * error_x

                rospy.loginfo(f"[BOTELLA] Detectada - error_x: {error_x:.2f}")
            else:
                msg.linear.x = 0.0
                msg.angular.z = 0.0
                rospy.loginfo("[BOTELLA] No detectada")

            self.pub_cmd_vel.publish(msg)
            cv2.imshow("Detección de botella", frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            rate.sleep()

        self.cap.release()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    seguidor = BotellaSeguidor()
    seguidor.seguir_botella()
