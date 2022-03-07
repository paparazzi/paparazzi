import cv2
def find_edge_OF(image, gray_scale=False, blur=False, minTH=0, maxTH=200):
    img = cv2.rotate(image, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Convert to graycsale
    if gray_scale == True:
        img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        # cv2.imshow('Grayscale', img)

    # Blur the image for better edge detection
    if blur == True:
        img = cv2.GaussianBlur(img, (3, 3), 0)
        # cv2.imshow('Gaussian Blur', img)

    # Canny Edge Detection
    edges = cv2.Canny(image=img, threshold1=minTH, threshold2=maxTH)  # Canny Edge Detection

    return edges
