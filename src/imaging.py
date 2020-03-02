import copy
import os
import socket
import time
import cv2
import numpy as np

UDP_IP = "192.168.43.225"
UDP_PORT = 2390

class Image:
    def __init__(self, src, threshold, num_colours, kernel_size):
        self.img = src
        self.centres = []
        self.threshold = threshold
        self.num_colours = num_colours
        self.kernel_size = kernel_size

    def _draw_axis(self, p, q, colour, scale=0.2):
        angle = np.arctan2(p[1] - q[1], p[0] - q[0])
        hypotenuse = np.sqrt((p[1] - q[1]) ** 2 + (p[0] - q[0]) ** 2)
        q = np.zeros_like(q, dtype=np.int64)

        # Lengthen the arrow by a factor of scale
        q[0] = int(p[0] - scale * hypotenuse * np.cos(angle))
        q[1] = int(p[1] - scale * hypotenuse * np.sin(angle))
        cv2.line(self.img, (p[0], p[1]), (q[0], q[1]), colour, 1, cv2.LINE_AA)

        # Create the arrow hooks
        p[0] = int(q[0] + 9 * np.cos(angle + np.pi / 4))
        p[1] = int(q[1] + 9 * np.sin(angle + np.pi / 4))
        cv2.line(self.img, (p[0], p[1]), (q[0], q[1]), colour, 1, cv2.LINE_AA)

        p[0] = int(q[0] + 9 * np.cos(angle - np.pi / 4))
        p[1] = int(q[1] + 9 * np.sin(angle - np.pi / 4))
        cv2.line(self.img, (p[0], p[1]), (q[0], q[1]), colour, 1, cv2.LINE_AA)

    def _get_orientation(self, pts):
        # Construct a buffer used by the PCA analysis
        data_pts = np.squeeze(np.array(pts, dtype=np.float64))

        # Perform PCA analysis
        # https://stackoverflow.com/questions/22612828/python-opencv-pcacompute-eigenvalue
        covar, mean = cv2.calcCovarMatrix(data_pts, np.mean(data_pts, axis=0),
                                          cv2.COVAR_SCALE | cv2.COVAR_ROWS | cv2.COVAR_SCRAMBLED)
        eigenvalues, eigenvectors = cv2.eigen(covar)[1:]
        eigenvectors = cv2.gemm(eigenvectors, data_pts - mean, 1, None, 0)
        eigenvectors = np.apply_along_axis(lambda n: cv2.normalize(n, dst=None).flat, 1, eigenvectors)

        # Store the centre of the object
        cntr = np.array([int(mean[0, 0]), int(mean[0, 1])])
        self.centres.append(cntr)

        # Draw the principal components
        cv2.circle(self.img, (cntr[0], cntr[1]), 3, (255, 0, 255), 1)
        p1 = cntr + 0.02 * eigenvectors[0] * eigenvalues[0]
        p2 = cntr - 0.02 * eigenvectors[1] * eigenvalues[1]
        # self._draw_axis(copy.copy(cntr), p1, (0, 255, 0), 2)
        # self._draw_axis(copy.copy(cntr), p2, (255, 255, 0), 10)

        return np.arctan2(eigenvectors[0, 1], eigenvectors[0, 0])  # orientation in radians

    def _color_quantize(self):
        shape = self.img.shape
        img = self.img.reshape((-1, 3))
        img = np.float32(img)

        # Define criteria and apply kmeans
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 10, 1.0)
        ret, label, centre = cv2.kmeans(img, self.num_colours, None, criteria, 10, cv2.KMEANS_RANDOM_CENTERS)
        intensities = cv2.cvtColor(np.array([centre]), cv2.COLOR_BGR2GRAY)
        intensities = np.int64(np.sort(np.squeeze(intensities)))

        # Convert back into uint8 and make original image
        centre = np.uint8(centre)
        res = centre[label.flatten()]
        res = res.reshape(shape)

        return res, intensities

    def draw_contours(self):
        quantized, intensities = self._color_quantize()

        # Convert image to grayscale
        gray = cv2.cvtColor(quantized, cv2.COLOR_BGR2GRAY)

        # Dilate
        kernel = np.ones((self.kernel_size, self.kernel_size), np.uint8)
        dilation = cv2.dilate(gray, kernel, iterations=1)

        # Convert image to binary
        thresh = intensities[self.num_colours - self.threshold] - 2
        retval, bw = cv2.threshold(dilation, thresh, 255, cv2.THRESH_BINARY)

        # Apply mask
        mask = 255 * np.ones_like(bw)
        mask[:, :30] = 0
        mask[:, 620:] = 0
        bw = np.minimum(bw, mask)

        # Find all the contours in the threshold range
        _, contours, _ = cv2.findContours(bw, cv2.RETR_LIST, cv2.CHAIN_APPROX_NONE)

        for i in range(len(contours)):
            x, y, w, h = cv2.boundingRect(contours[i])
            area = cv2.contourArea(contours[i])
            # Ignore contours that are too small or too large
            if (w < 10) or (w > 100) or (h < 10) or (h > 100) or (area > 1000) or (area < 200):
                continue

            # Draw each contour only for visualisation purposes
            cv2.drawContours(self.img, contours, i, (0, 0, 255), 1)

            # Find the orientation of each shape
            self._get_orientation(contours[i])

    def show(self):
        cv2.namedWindow("output", cv2.WINDOW_NORMAL)
        cv2.imshow("output", self.img)
        cv2.waitKey()

    def get_centres(self):
        return np.array(self.centres)

def send_data(sock, centres):
    for coordinate in np.nditer(centres):
        # In the order x1, y1, x2, y2 etc.
        time.sleep(0.5)
        message = str(coordinate)
        sock.sendto(message.encode(), (UDP_IP, UDP_PORT))

def dist(x, y):
    x_centre = 378
    y_centre = 340
    return np.sqrt((x - x_centre) ** 2 + (y - y_centre) ** 2)

def sort_coordinates(centres):
    centres = list(centres)
    centres.sort(key=lambda centre: dist(centre[0], centre[1]))
    return np.asarray(centres)
    

if __name__ == '__main__':
    src = cv2.imread(os.path.join(os.getcwd(), 'images', 'test.jpg'))
    assert src is not None
    image = Image(copy.copy(src), threshold=2, num_colours=8, kernel_size=5)
    image.draw_contours()
    centres = sort_coordinates(image.get_centres())
    print(centres)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    send_data(sock, centres)
    image.show()