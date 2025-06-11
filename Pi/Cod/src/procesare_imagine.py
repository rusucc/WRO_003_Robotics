import cv2

def evidentiere_pereti(frame):
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.resize(frame, (160, 90))
    _, binFrame = cv2.threshold(frame, 100, 255, cv2.THRESH_BINARY)
    return binFrame

def identificare_linii_pereti(frame, DEBUG = 0):
    """se face erosion ca sa scapam de liniile spre centru"""
    kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
    erosion = cv2.erode(frame, kernel, iterations = 5)
    lines = cv2.HoughLinesP(erosion, 1, 3.14/180, 50, minLineLength = 20, maxLineGap = 20)
    
    longest_line = None
    max_len = 0
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            length = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** 0.5
            if length > max_len:
                max_len = length
                longest_line = line[0]
    if DEBUG == 1:
        imagine_mare = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        imagine_mare = cv2.resize(imagine_mare, (640, 480))
        if lines is not None:
            for line in lines:
                x1, y1, x2, y2 = line[0]
                cv2.line(imagine_mare, (x1 * 4 + 160, y1 * 4 + 60), (x2 * 4 + 160, y2 * 4 + 60), color=(0, 255, 0), thickness=2)
        cv2.imshow("Imagine linii", imagine_mare)
        cv2.waitKey(1)
    return longest_line