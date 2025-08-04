# lane_detector.py

import cv2
import numpy as np

def detect_lanes(image):

    # BGR을 HLS 색상 공간으로 변환
    hls = cv2.cvtColor(image, cv2.COLOR_BGR2HLS)

    lower_white = np.array([0, 200, 0])
    upper_white = np.array([255, 255, 255])

    # 색상 범위에 해당하는 마스크 생성
    white_mask = cv2.inRange(hls, lower_white, upper_white)

    # Canny 엣지 검출을 위한 전처리 
    blur = cv2.GaussianBlur(white_mask, (5, 5), 0)
    canny = cv2.Canny(blur, 50, 150)

    # 관심 영역(ROI) 설정
    height, width = canny.shape
    mask = np.zeros_like(canny)
    polygon = np.array([
        [(0, height), (width, height), (width // 2, int(height * 0.6))]
    ], np.int32)
    cv2.fillPoly(mask, polygon, 255)
    masked_edges = cv2.bitwise_and(canny, mask)

    # 허프 변환으로 선분 검출
    lines = cv2.HoughLinesP(
        masked_edges,
        rho=1,
        theta=np.pi / 180,
        threshold=40,
        minLineLength=30,
        maxLineGap=50
    )

    # 원본 이미지에 검출된 선 그리기 
    line_image = np.copy(image)
    if lines is not None:
        for line in lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(line_image, (x1, y1), (x2, y2), (0, 0, 255), 10)

    return line_image