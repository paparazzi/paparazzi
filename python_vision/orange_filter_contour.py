import cv2
import numpy as np
import os

def bounding_rect_edge_to_perc(x,y,h,w,X,Y):
    """
    From a bounding rectangle, calculate the angle for the first horizontal and vertical edge.

    Supposes horizon is leveled.

    :param x: x coordinate of upper left corner of rectangle (px)
    :param y: y coordinate of upper left corner of rectangle (px)
    :param h: height of rectangle (px)
    :param w: width of rectangle(px)
    :param X: total image x size (px)
    :param Y: total image y size (px)
    :return:
     perc_x_p: from the center of the screen, percentage left (+) or percentage right (-) that the first edge from the rectangle is
     perc_x_m: from the center of the screen, percentage left (+) or percentage right (-) that the last edge from the rectangle is

     perc_y_p: from the center of the screen, percentage up (+) or percentage down (-) that the first edge from the rectangle is
     perc_y_m: from the center of the screen, percentage up (+) or percentage down (-) that the last edge from the rectangle is
    """

    # Get position of rectangle
    perc_y_p = 1 - (2 / Y) * (y)
    perc_y_m = 1 - (2 / Y) * (y + h)

    if x < X / 2:
        perc_x_m = 1 - (2 / X) * (x)
        perc_x_p = 1 - (2 / X) * (x + w)
    else:
        perc_x_p = 1 - (2 / X) * (x)
        perc_x_m = 1 - (2 / X) * (x + w)

    return [[perc_x_p,perc_y_p],[perc_x_m,perc_y_m]]

def area_rect_center_to_perc(minRect,X,Y):
    """

    :param minRect: ( center (x,y), (width, height), angle of rotation (deg) ) (px)
    :param X: total image x size (px)
    :param Y: total image y size (px)
    :return:
        perc_h: from the center of the screen, percentage left (+) or percentage right (-) that the center of the rectangle is
        perc_v: from the center of the screen, percentage up (+) or percentage down (-) that the center of the rectangle is
        perc_w: percentage of the width of the rectangle
        perc_h: percentage of the height of the rectangle
        rot: rotation angle of rectangle
    """

    # Extracting dimensions from minRect
    x = minRect[0][0]
    y = minRect[0][1]
    w = minRect[1][0]
    h = minRect[1][1]
    rot = minRect[2]

    perc_y = 1 - (2 / Y) * (y)
    perc_x = 1 - (2 / X) * (x)

    perc_w = w/X
    perc_h = h/Y

    return [[perc_x, perc_y],[perc_w,perc_h],rot]

def load_images_from_folder(folder):
    images = []
    for filename in sorted(os.listdir(folder)):
        img = cv2.imread(os.path.join(folder,filename))
        if img is not None:
            images.append(img)
    return images

def orange_obstacle_finder(image,complete_visualization,resize_factor,filter):

    # Extracting filter values
    y_low = filter[0]
    y_high = filter[1]
    u_low = filter[2]
    u_high = filter[3]
    v_low = filter[4]
    v_high = filter[5]

    # Read image
    ori_im = image

    # Resize
    im = cv2.resize(ori_im, (int(ori_im.shape[1]/resize_factor), int(ori_im.shape[0]/resize_factor)))

    # Rotate image
    im = cv2.rotate(im, cv2.ROTATE_90_COUNTERCLOCKWISE)

    # Filter
    YUV_im = cv2.cvtColor(im, cv2.COLOR_BGR2YUV)
    filt_im = np.zeros([YUV_im.shape[0], YUV_im.shape[1]])
    for y in range(YUV_im.shape[0]):
        for x in range(YUV_im.shape[1]):
            if(YUV_im[y,x,0] >= y_low and YUV_im[y,x,0] <= y_high and YUV_im[y,x,1] >= u_low and YUV_im[y,x,1] <= u_high and YUV_im[y,x,2] >= v_low and YUV_im[y,x,2] <= v_high):
                filt_im[y,x] = 1

    # Convert to uint8
    filt_im = (filt_im*255).astype(np.uint8)

    # Blur the image for better edge detection
    blur_im = cv2.GaussianBlur(filt_im, (3,3), 0)

    # Contours detection
    contours, hierarchy = cv2.findContours(image=blur_im, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)

    # Select biggest contours
    biggest_contours = [contour for contour in contours if cv2.contourArea(contour)>size_crit*im.shape[1]*im.shape[0]]

    # Calculate center of contour
    centroid_im = im.copy()
    centroids = []
    for c in biggest_contours:
        # calculate moments for each contour
        M = cv2.moments(c)
        # calculate x,y coordinate of center
        if M["m00"] != 0:
            cX = int(M["m10"] / M["m00"]) #cX
            cY = int(M["m01"] / M["m00"]) #cY
        else:
            cX,cY = 0,0

        centroids.append([cX,cY])
        cv2.circle(centroid_im, (cX, cY), 5, (255, 255, 255), -1)
    cv2.drawContours(image=centroid_im, contours=biggest_contours, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)

    # Verify if centroid or color inside edge is orange
    good_contours = []
    contours_im = im.copy()

    for i in range(len(centroids)):
        cX = centroids[i][0]
        cY = centroids[i][1]
        if (YUV_im[cY, cX, 0] >= y_low and YUV_im[cY, cX, 0] <= y_high and YUV_im[cY, cX, 1] >= u_low and YUV_im[cY, cX, 1] <= u_high and YUV_im[
            cY, cX, 2] >= v_low and YUV_im[cY, cX, 2] <= v_high):
            good_contours.append(biggest_contours[i])

            # Draw bounding box
            (x, y, w, h) = cv2.boundingRect(biggest_contours[i])
            cv2.rectangle(contours_im, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # Draw min area rectangle
            minRect = cv2.minAreaRect(biggest_contours[i])
            box = cv2.boxPoints(minRect)
            box = np.int0(box)
            cv2.drawContours(contours_im, [box], 0, (0, 0, 255), 2)

            # Draw Centroid
            cv2.circle(contours_im, (cX, cY), 5, (255, 255, 255), -1)
    # Draw contours
    cv2.drawContours(image=contours_im, contours=good_contours, contourIdx=-1, color=(0, 255, 0), thickness=1, lineType=cv2.LINE_AA)

    if good_contours:
        print('Good contour present')

        perc_edge = bounding_rect_edge_to_perc(x, y, h, w, im.shape[1], im.shape[0])
        perc_center = area_rect_center_to_perc(minRect, im.shape[1], im.shape[0])

    else:
        print('No good contour')

    if complete_visualization:
        # Plot image
        imS = cv2.resize(im, (960, 540))
        cv2.imshow('image window', imS)
        cv2.waitKey(0)

        imS = cv2.resize(filt_im, (960, 540))
        cv2.imshow('image window', imS)
        cv2.waitKey(0)

        imS = cv2.resize(blur_im, (960, 540))
        cv2.imshow('image window', imS)
        cv2.waitKey(0)

        imS = cv2.resize(centroid_im, (960, 540))
        cv2.imshow('image window', imS)
        cv2.waitKey(0)

        imS = cv2.resize(contours_im, (960, 540))
        cv2.imshow('image window', imS)
        cv2.waitKey(0)

        cv2.destroyAllWindows()

        return

    else:
        return contours_im


# Folder to do analysis from
folder_path = './AE4317_2019_datasets/cyberzoo_aggressive_flight/20190121-144646'
#folder_path = './AE4317_2019_datasets/cyberzoo_poles/20190121-135009'
#folder_path = './AE4317_2019_datasets/sim_poles/20190121-160844'

# Print all steps of processing
complete_visualization = False

# Resize image to facilitate processing
resize_factor = 4

# Filter parameters
y_low = 50
y_high = 200
u_low = 0
u_high = 120
v_low = 160
v_high = 220
filter = [y_low,y_high,u_low,u_high,v_low,v_high]

# Minimum size for obstacle (percentage of full image size)
size_crit = 0.01

images = load_images_from_folder(folder_path)

for image in images:
    contours_im = orange_obstacle_finder(image,complete_visualization,resize_factor,filter)

    imS = cv2.resize(contours_im, (960, 540))
    cv2.imshow('image window', imS)
    c = cv2.waitKey(1)
    if c == 27:
        break

cv2.destroyAllWindows()