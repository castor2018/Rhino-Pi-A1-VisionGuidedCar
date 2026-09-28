import os
# import sys
# import platform
import math
import cv2
import numpy as np
import random
from PIL import Image, ImageDraw, ImageFont

def get_rotate_crop_image(img, points):
    '''
    img_height, img_width = img.shape[0:2]
    left = int(np.min(points[:, 0]))
    right = int(np.max(points[:, 0]))
    top = int(np.min(points[:, 1]))
    bottom = int(np.max(points[:, 1]))
    img_crop = img[top:bottom, left:right, :].copy()
    points[:, 0] = points[:, 0] - left
    points[:, 1] = points[:, 1] - top
    '''
    assert len(points) == 4, "shape of points must be 4*2"
    img_crop_width = int(
        max(
            np.linalg.norm(points[0] - points[1]),
            np.linalg.norm(points[2] - points[3])))
    img_crop_height = int(
        max(
            np.linalg.norm(points[0] - points[3]),
            np.linalg.norm(points[1] - points[2])))
    pts_std = np.float32([[0, 0], [img_crop_width, 0],
                          [img_crop_width, img_crop_height],
                          [0, img_crop_height]])
    M = cv2.getPerspectiveTransform(points, pts_std)
    dst_img = cv2.warpPerspective(
        img,
        M, (img_crop_width, img_crop_height),
        borderMode=cv2.BORDER_REPLICATE,
        flags=cv2.INTER_CUBIC)
    dst_img_height, dst_img_width = dst_img.shape[0:2]
    if dst_img_height * 1.0 / dst_img_width >= 1.5:
        dst_img = np.rot90(dst_img)
    return dst_img


def get_minarea_rect_crop(img, points):
    bounding_box = cv2.minAreaRect(np.array(points).astype(np.int32))
    points = sorted(list(cv2.boxPoints(bounding_box)), key=lambda x: x[0])

    index_a, index_b, index_c, index_d = 0, 1, 2, 3
    if points[1][1] > points[0][1]:
        index_a = 0
        index_d = 1
    else:
        index_a = 1
        index_d = 0
    if points[3][1] > points[2][1]:
        index_b = 2
        index_c = 3
    else:
        index_b = 3
        index_c = 2

    box = [points[index_a], points[index_b], points[index_c], points[index_d]]
    crop_img = get_rotate_crop_image(img, np.array(box))
    return crop_img

def create_font(draw_text, txt, sz, font_path="./config/simfang.ttf"):
    font_size = int(sz[1] * 0.99)
    font = ImageFont.truetype(font_path, font_size, encoding="utf-8")
    # Pillow 7.0.0
    # length = font.getsize(txt)

    # Pillow 10.4.0
    left, top, right, bottom = draw_text.textbbox((0, 0), txt, font=font)
    width = right - left
    height = bottom - top
    # print("length: {}, sz: {}".format(length, sz))
    print("length: {}, sz: {}".format((width, height), sz))
    if width > sz[0]:
        font_size = int(font_size * sz[0] / width)
        font = ImageFont.truetype(font_path, font_size, encoding="utf-8")
    return font

def draw_box_txt_fine(img_size, box, txt, font_path="./config/simfang.ttf"):
    box_height = int(
        math.sqrt((box[0][0] - box[3][0])**2 + (box[0][1] - box[3][1])**2))
    box_width = int(
        math.sqrt((box[0][0] - box[1][0])**2 + (box[0][1] - box[1][1])**2))

    if box_height > 2 * box_width and box_height > 30:
        img_text = Image.new('RGB', (box_height, box_width), (255, 255, 255))
        draw_text = ImageDraw.Draw(img_text)
        if txt:
            font = create_font(draw_text, txt, (box_height, box_width), font_path)
            draw_text.text([0, 0], txt, fill=(0, 0, 0), font=font)
        img_text = img_text.transpose(Image.ROTATE_270)
    else:
        img_text = Image.new('RGB', (box_width, box_height), (255, 255, 255))
        draw_text = ImageDraw.Draw(img_text)
        if txt:
            font = create_font(draw_text, txt, (box_width, box_height), font_path)
            draw_text.text([0, 0], txt, fill=(0, 0, 0), font=font)

    pts1 = np.float32(
        [[0, 0], [box_width, 0], [box_width, box_height], [0, box_height]])
    pts2 = np.array(box, dtype=np.float32)
    M = cv2.getPerspectiveTransform(pts1, pts2)

    img_text = np.array(img_text, dtype=np.uint8)
    img_right_text = cv2.warpPerspective(
        img_text,
        M,
        img_size,
        flags=cv2.INTER_NEAREST,
        borderMode=cv2.BORDER_CONSTANT,
        borderValue=(255, 255, 255))
    return img_right_text

def draw_ocr_box_txt(image,
                     boxes,
                     txts=None,
                     scores=None,
                     drop_score=0.5,
                     font_path="./config/simfang.ttf"):
    h, w = image.height, image.width
    img_left = image.copy()
    img_right = np.ones((h, w, 3), dtype=np.uint8) * 255
    random.seed(0)

    draw_left = ImageDraw.Draw(img_left)
    if txts is None or len(txts) != len(boxes):
        txts = [None] * len(boxes)
    for idx, (box, txt) in enumerate(zip(boxes, txts)):
        if scores is not None and scores[idx] < drop_score:
            continue
        color = (random.randint(0, 255), random.randint(0, 255),
                 random.randint(0, 255))
        draw_left.polygon(box, fill=color)
        img_right_text = draw_box_txt_fine((w, h), box, txt, font_path)
        pts = np.array(box, np.int32).reshape((-1, 1, 2))
        cv2.polylines(img_right_text, [pts], True, color, 1)
        img_right = cv2.bitwise_and(img_right, img_right_text)
    img_left = Image.blend(image, img_left, 0.5)
    img_show = Image.new('RGB', (w * 2, h), (255, 255, 255))
    img_show.paste(img_left, (0, 0, w, h))
    img_show.paste(Image.fromarray(img_right), (w, 0, w * 2, h))
    return np.array(img_show)


def draw_ocr_box_txt_on_image(image,
                              boxes,
                              txts=None,
                              scores=None,
                              drop_score=0.5,
                              font_path="./config/simfang.ttf"):
    """
    在原图上画多边形框，并在框下方写文本，文本有背景色。

    Args:
        image (PIL.Image.Image): 原始图像（PIL Image）。
        boxes (List[List[float]]): 每个框的 4 个顶点 [[x1,y1],...[x4,y4]]。
        txts (List[str], optional): 每个框对应的文本标签。
        scores (List[float], optional): 每个框的置信度分数。
        drop_score (float): 小于该分数的框将被忽略。
        font_path (str): 字体文件路径，用于写中文/英文。
        font_size (int): 文本字号。

    Returns:
        numpy.ndarray: 标注后的图像（BGR 格式，适合后续用 cv2 显示或保存）。
    """
    # 复制一份原图并转为 PIL
    pil_img = image.copy() if isinstance(image, Image.Image) else Image.fromarray(image)
    draw = ImageDraw.Draw(pil_img)

    # 如果没有提供文本列表，则填充空
    if txts is None or len(txts) != len(boxes):
        txts = ["" for _ in boxes]

    # 载入字体
    try:
        font = ImageFont.truetype(font_path, 35)
    except IOError:
        font = ImageFont.load_default()

    for idx, box in enumerate(boxes):
        # 跳过低分框
        if scores is not None and scores[idx] < drop_score:
            continue

        # 随机生成一个鲜艳的颜色
        color = (255, 0, 0)

        # PIL 用 RGB
        pil_color = (color[0], color[1], color[2])

        # 使用 polylines 画框
        pts = np.array(box, np.int32).reshape((-1, 1, 2))
        draw.polygon(box, outline=pil_color, width=2)

        # 计算文本大小
        text = txts[idx] or ""
        if text:
            # text_w, text_h = draw.textsize(text, font=font)
            left, top, right, bottom = draw.textbbox((0, 0), text, font=font)
            text_w = right - left
            text_h = bottom - top

            # 找到 box 的左上顶点坐标
            xs = [pt[0] for pt in box]
            ys = [pt[1] for pt in box]
            x_min, y_min = int(min(xs)), int(min(ys))

            # 文本放置在框顶端上方
            text_x = x_min
            text_y = y_min - text_h - 5  # 留一点空间

            # 画背景矩形（文本底色）
            draw.rectangle([text_x, text_y,
                            text_x + text_w, text_y + text_h],
                           fill=(255, 255, 255))

            # 画文字
            draw.text((text_x, text_y), text, fill=pil_color, font=font)

    # 返回 BGR numpy，供 cv2.imshow 或 cv2.imwrite 使用
    return cv2.cvtColor(np.array(pil_img), cv2.COLOR_RGB2BGR)

class ClsPostProcess(object):
    """ Convert between text-label and text-index """

    def __init__(self, label_list=None, key=None, **kwargs):
        super(ClsPostProcess, self).__init__()
        self.label_list = label_list
        self.key = key

    def __call__(self, preds, label=None, *args, **kwargs):
        if self.key is not None:
            preds = preds[self.key]

        label_list = self.label_list
        if label_list is None:
            label_list = {idx: idx for idx in range(preds.shape[-1])}

        # if isinstance(preds, paddle.Tensor):
            # preds = preds.numpy()

        pred_idxs = preds.argmax(axis=1)
        decode_out = [(label_list[idx], preds[i, idx])
                      for i, idx in enumerate(pred_idxs)]
        if label is None:
            return decode_out
        label = [(label_list[idx], 1.0) for idx in label]
        return decode_out, label