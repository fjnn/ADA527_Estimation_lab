import cv2

def list_connected_cameras():
    # Iterate over camera indices until one fails to open
    connected_cameras = []
    index = 0
    while True:
        cap = cv2.VideoCapture(index)
        if not cap.isOpened():
            break
        else:
            connected_cameras.append(index)
            cap.release()
        index += 1

    return connected_cameras

if __name__ == "__main__":
    cameras = list_connected_cameras()
    if cameras:
        print("Connected cameras:")
        for camera in cameras:
            print(f"Camera {camera}")
    else:
        print("No cameras detected.")