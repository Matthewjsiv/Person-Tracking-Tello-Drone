import cv2

windowWidth = 640
windowHeight = 840
brightness = 100

cap = cv2.VideoCapture(0)
# cap.set(3,windowWidth)
# cap.set(4,windowHeight)
# cap.set(10,brightness)



while (cap.isOpened()):
    success, img = cap.read()
    if success== True:
        img = cv2.flip(img, 1)
        cv2.imshow("output", img)
        if cv2.waitKey(1)& 0xFF ==ord("q"):
            break
