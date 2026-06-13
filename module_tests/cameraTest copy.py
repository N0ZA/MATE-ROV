import cv2

cap = cv2.VideoCapture('rtsp://admin:admin@192.168.2.64:554')

if not cap.isOpened():
    print("Error opening RTSP stream. Check IP/port/firewall.")
    exit()

while True:
    ret, frame = cap.read()
    if not ret:
        print("Stream read failed.")
        break
    
    cv2.imshow('ROV Camera Stream', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()