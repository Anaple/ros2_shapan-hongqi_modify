#!/usr/bin/env python3
import math
import time

import cv2
import numpy as np


class LineDetect:

    def __init__(self):
        # 图像裁剪范围
        # 起始点 (clip_x, clip_y)  宽度 clip_w   高度 clip_h
        self.clip_x = 0
        self.clip_y = 180
        self.clip_w = 480
        self.clip_h = 90

        # 边缘检测参数设置
        self.canny_thresholds = [80, 200]
        self.canny_aperture_size = 3
        self.dilation_kernel_size = 3

        # 霍夫变换参数设置
        self.hough_threshold = 5
        self.hough_min_line_length = 5
        self.hough_max_line_gap = 10

        # 色彩平衡参数设置
        self.lower_thresholds = [80, 80, 80]
        self.higher_thresholds = [220, 220, 220]

        # 车辆中心点横坐标（0-480）
        self.view_center = 240

    # 图像预处理
    # 缩小图像，减小计算量，scale=1表示原始大小
    # 示例代码中不考虑性能，如缩小图像后，代码中的点坐标范围需要同步修改
    def prev_cb_image(self, image, scale=1):
        resized_image = cv2.resize(image, (0, 0), fx=scale, fy=scale)
        clip_image = resized_image[self.clip_y:self.clip_y + self.clip_h, :]
        # 色彩平衡调整
        color_balance_img = self.apply_color_balance(
            self.lower_thresholds, self.higher_thresholds, clip_image)
        cv2.imshow("src_img", image)
        cv2.imshow("color_balance_img", color_balance_img)
        cv2.waitKey(0)
        return color_balance_img

    # 色彩平衡, 减少环境光对图像的影响
    def apply_color_balance(self, lower_threshold, higher_threshold, image):
        if lower_threshold is None:
            return None
        channels = cv2.split(image)  # 将图像拆分为红、绿、蓝三通道
        out_channels = []
        # 遍历3个通道，取每个通道最大值做为白色参考值，取通道指定比例处的值做为黑色参考值
        for idx, channel in enumerate(channels):
            # 筛除掉范围以外的数据
            thresholded = self.apply_threshold(channel, lower_threshold[idx], higher_threshold[idx])
            # 指定将图片的值放缩到 0 - 255 之间
            normalized = cv2.normalize(thresholded, thresholded.copy(), 0, 255, cv2.NORM_MINMAX)
            out_channels.append(normalized)

        # 返回合并通道后的图像
        return cv2.merge(out_channels)

    # 标记不符合期望的点
    def apply_threshold(self, matrix, low_value, high_value):
        low_mask = matrix < low_value
        matrix = self.apply_mask(matrix, low_mask, low_value)
        high_mask = matrix > high_value
        matrix = self.apply_mask(matrix, high_mask, high_value)
        return matrix

    # 将标记的点替换成对应的值
    def apply_mask(self, matrix, mask, fill_value):
        masked = np.ma.array(matrix, mask=mask, fill_value=fill_value)
        return masked.filled()

    # 边缘查找
    def get_edges(self, image):
        # Canny边缘检测
        edges = cv2.Canny(image, self.canny_thresholds[0], self.canny_thresholds[1],
                          apertureSize=self.canny_aperture_size)
        # 设阈值，去除背景部分, 打开以下注释行，可将鼠标移到车道线上，查看其颜色，用inRange函数筛选出所需的颜色
        black_line = cv2.inRange(image, np.array([0, 0, 0]), np.array([80, 80, 80]))

        # 腐蚀处理，去除部分很小的噪声点
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.dilation_kernel_size, self.dilation_kernel_size))
        new_black_line = cv2.erode(black_line, kernel)

        # 膨胀处理，连通小的边缘
        kernel = cv2.getStructuringElement(
            cv2.MORPH_RECT, (self.dilation_kernel_size, self.dilation_kernel_size * 2))
        new_black_line = cv2.dilate(new_black_line, kernel)

        # 合并黑色边缘和整体边缘，取重合部分
        edge_black = cv2.bitwise_and(new_black_line, edges)
        # cv2.imshow('edges', edges)
        # cv2.imshow('edge_black', edge_black)
        # cv2.waitKey(0)
        return edge_black

    # 检测边缘中的线段，线段以起点终点坐标体现
    def get_lines(self, edges):
        # 统计概率霍夫线变换函数，该函数能输出检测到的直线的端点
        lines = cv2.HoughLinesP(edges, rho=1, theta=np.pi / 180, threshold=self.hough_threshold,
                                minLineLength=self.hough_min_line_length, maxLineGap=self.hough_max_line_gap)
        if lines is not None:
            # 重新整理数据格式[[x1,y1,x2,y2],[x1,y1,x2,y2],...]
            lines = lines.reshape((-1, 4))
            return lines
        else:
            return None

    # 计算色彩平衡阈值
    # 实际使用中无需每一帧都处理，可以定时5s执行一次
    def calculate_new_parameters(self, image):
        # 缩小图像，减少计算
        fx = 0.2
        fy = 0.2
        resized_image = cv2.resize(image, (0, 0), fx=fx, fy=fy)
        # 获取图像高度
        H = resized_image.shape[0]
        # 180-270是合适的车道线范围，只处理指定部分即可
        start_y = int(self.clip_y * fy)
        end_y = start_y + int(self.clip_h * fy)
        cropped_image = resized_image[start_y: end_y, :, :]
        # 将图像拆分为红、绿、蓝三通道
        channels = cv2.split(cropped_image)
        low = []
        hig = []
        # 遍历3个通道
        for idx, channel in enumerate(channels):
            height, width = channel.shape
            num_pixels = width * height
            # 不更改数据，将数据转换为一维数据
            flattened = channel.reshape(num_pixels)
            # 对数据进行排序
            flattened = np.sort(flattened)
            # rgb 颜色中，黑色为（0，0，0），尽量选取较低色度的范围
            # 处理黑色数据，由于光照等因素，黑色显示出来的可能并非标准黑色
            # 根据实际颜色，将一个范围内的数据统一转换为黑色，记录当前的黑色值
            low.append(flattened[10])
            # 白色处理和黑色一致
            hig.append(flattened[int(math.floor(num_pixels * 0.5))])

        self.lower_thresholds = low
        self.higher_thresholds = hig
        print(low, hig)

    # 车道线拟合（y=kx+b），计算k,b的值
    def cal_kb_linear(self, data_line):
        left = []  # 左侧坐标
        right = []  # 右侧坐标
        if data_line is not None:
            for line in data_line:
                x1, y1, x2, y2 = line
                # 0-300像素之间的线段噪声太多，直接不显示
                if 0 <= x1 < self.view_center:
                    left.append([x1, y1 + self.clip_y])
                    left.append([x2, y2 + self.clip_y])
                elif self.view_center <= x1 < (self.clip_x + self.clip_w):
                    right.append([x1, y1 + self.clip_y])
                    right.append([x2, y2 + self.clip_y])

        left = np.array(left)  # loc 必须为矩阵形式，且表示[x,y]坐标
        # 点太少，拟合结果容易出错，给定默认值，可以使用上次的计算结果
        if len(left) < 4:
            l_k = -1.38
            l_b = 300
        else:
            # 直线拟合
            l_output = cv2.fitLine(left, cv2.DIST_L2, 0, 0.01, 0.01)
            l_k = float(l_output[1] / l_output[0])
            l_b = int(l_output[3] - l_k * l_output[2])

        right = np.array(right)
        if len(right) < 4:
            r_k = 1.38
            r_b = -365
        else:
            r_output = cv2.fitLine(right, cv2.DIST_L2, 0, 0.01, 0.01)
            r_k = float(r_output[1] / r_output[0])
            r_b = int(r_output[3] - r_k * r_output[2])

        if l_k < -0.5:
            l_k = -1.38
            l_b = 300
        if r_k < 0.5:
            r_k = 1.38
            r_b = -365

        return l_k, l_b, r_k, r_b

    # 绘制车道线
    def draw_lines(self, img, data_line):
        # 获取车道线的直线拟合函数
        l_k, l_b, r_k, r_b = self.cal_kb_linear(data_line)
        # # 判断分母不为0
        # if l_k != 0:
        #     # 计算左侧线段的起点和终点坐标，y值是图像的纵坐标(180,270)，分别计算x值
        #     l_x1 = int((0 - l_b) / l_k)
        #     l_x2 = int((270 - l_b) / l_k)
        #     # 绘制左侧线段
        #     cv2.line(img, (l_x1, 0), (l_x2, 270), color=(0, 0, 255), thickness=5)
        # else:
        #     l_x1 = 0

        # 判断分母不为0
        if r_k != 0:
            # 右侧线段同左侧
            r_x1 = int((0 - r_b) / r_k)
            r_x2 = int((270 - r_b) / r_k)
            cv2.line(img, (r_x1, 0), (r_x2, 270), color=(0, 0, 255), thickness=5)
        else:
            r_x1 = 480

        # 记录线段的中心位置（两条车道线在180像素处相交，可以近似的认为此处的x坐标即为车辆中心值）
        self.view_center = r_x1 - 24
        # if abs(l_x1 - 240) < abs(r_x1 - 240):
        #     self.view_center = l_x1
        # else:
        #     self.view_center = r_x1
        # print(self.view_center)
        return img


if __name__ == '__main__':
    node = LineDetect()
    # capture = cv2.VideoCapture("images/video.mp4")
    for i in range(1, 67):
        # ret, frame = capture.read()
        # frame = cv2.imread("img/{}.jpg".format(i))
        frame = cv2.imread("img/{}.png".format(i))
        ret = True
        if ret:  # 如果有画面再执行
            # 计算色彩平衡阈值
            node.calculate_new_parameters(frame)
            # 获取色彩平衡后的图像
            color_balance = node.prev_cb_image(frame)
            # 查找白色边缘(轮廓)
            edge_white = node.get_edges(color_balance)
            # 查找白色线段
            white_lines = node.get_lines(edge_white)
            # 绘制车道线
            result = node.draw_lines(frame, white_lines)
            cv2.imshow('result', result)
            cv2.waitKey(0)

            time.sleep(0.1)
            if cv2.waitKey(1) & 0xff == ord('q'):
                break

    # capture.release()
    cv2.destroyAllWindows()
