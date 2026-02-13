import numpy as np
import cv2
from sklearn.cluster import KMeans
from collections import Counter
def main():
    image = cv2.imread('img.png')
    #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    #lower = np.array([35,82, 46])
    #upper = np.array([81, 255, 255])
    #mask = cv2.inRange(hsv, lower, upper)
    #result = cv2.bitwise_and(image, image, mask=mask)
    #b,g,r = cv2.split()
    #lower = np.array([6, 0, 0])
    #upper = np.array([77, 253, 224])
    ROI = image[258:280, 279:297]
    hsv2 = cv2.cvtColor(ROI, cv2.COLOR_BGR2HSV)
    print(hsv2[9,6])
    dom_color = get_dominant_color(ROI)
    dom_color_hsv = np.full(ROI.shape, dom_color, dtype='uint8')
    dom_color_bgr = cv2.cvtColor(dom_color_hsv, cv2.COLOR_HSV2BGR)
    output_image = np.hstack((ROI, dom_color_bgr))
    cv2.imshow('result', output_image)
    cv2.waitKey()

def get_dominant_color(image, k=3, image_processing_size = None):
    """
    takes an image as input
    returns the dominant color of the image as a list
    
    dominant color is found by running k means on the 
    pixels & returning the centroid of the largest cluster

    processing time is sped up by working with a smaller image; 
    this resizing can be done with the image_processing_size param 
    which takes a tuple of image dims as input

    >>> get_dominant_color(my_image, k=4, image_processing_size = (25, 25))
    [56.2423442, 34.0834233, 70.1234123]
    """
    #resize image if new dims provided
    if image_processing_size is not None:
        image = cv2.resize(image, image_processing_size, 
                            interpolation = cv2.INTER_AREA)
    
    #reshape the image to be a list of pixels
    image = image.reshape((image.shape[0] * image.shape[1], 3))

    #cluster and assign labels to the pixels 
    clt = KMeans(n_clusters = k)
    labels = clt.fit_predict(image)

    #count labels to find most popular
    label_counts = Counter(labels)

    #subset out most popular centroid
    dominant_color = clt.cluster_centers_[label_counts.most_common(1)[0][0]]

    return list(dominant_color)
if __name__=="__main__":
    main()