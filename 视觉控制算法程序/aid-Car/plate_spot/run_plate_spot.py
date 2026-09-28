import os
import copy
import time
import numpy as np
import math
import utility_pt
import db_postprocess_pt
import onnxruntime as ort
import rec_postprocess_pt
from PIL import Image
import aidcv as cv2


def get_image_file_list(image_dir):
    imgs_lists=[]
    for single_file in os.listdir(image_dir):
        file_path = os.path.join(image_dir, single_file)
        if os.path.isfile(file_path) :
            imgs_lists.append(file_path)
    imgs_lists = sorted(imgs_lists)
    return imgs_lists

def create_predictor(model_dir):
    model_file_path = model_dir
    if not os.path.exists(model_file_path):
        raise ValueError("not find model file path {}".format(
            model_file_path))
    sess = ort.InferenceSession(model_file_path,providers=['CPUExecutionProvider'])
    return sess, sess.get_inputs()[0], None, None

def image_padding( im, value=0):
    h, w, c = im.shape
    im_pad = np.zeros((max(32, h), max(32, w), c), np.uint8) + value
    im_pad[:h, :w, :] = im
    return im_pad



def preprocess_img_new( img):
    scale = 1/255.
    mean = [0.485, 0.456, 0.406]
    std = [0.229, 0.224, 0.225]
    """
    resize image to a size multiple of 32 which is required by the network
    args:
        img(array): array with shape [h, w, c]
    return(tuple):
        img, (ratio_h, ratio_w)
    """
    limit_side_len = 960
    h, w, c = img.shape

    limit_type='max'
    # limit the max side
    if limit_type == 'max':
        if max(h, w) > limit_side_len:
            if h > w:
                ratio = float(limit_side_len) / h
            else:
                ratio = float(limit_side_len) / w
        else:
            ratio = 1.
    elif limit_type == 'min':
        if min(h, w) < limit_side_len:
            if h < w:
                ratio = float(limit_side_len) / h
            else:
                ratio = float(limit_side_len) / w
        else:
            ratio = 1.
    elif limit_type == 'resize_long':
        ratio = float(limit_side_len) / max(h, w)
    else:
        raise Exception('not support limit type, image ')
    resize_h = int(h * ratio)
    resize_w = int(w * ratio)

    resize_h = max(int(round(resize_h / 32) * 32), 32)
    resize_w = max(int(round(resize_w / 32) * 32), 32)

    try:
        if int(resize_w) <= 0 or int(resize_h) <= 0:
            return None, (None, None)
        img = cv2.resize(img, (int(resize_w), int(resize_h)))
    except:
        print(img.shape, resize_w, resize_h)
        # sys.exit(0)
    ratio_h = resize_h / float(h)
    ratio_w = resize_w / float(w)
    # return img, [ratio_h, ratio_w]
    float_img = img.astype('float32') 

    float_img = (float_img* scale - mean) / std
    float_img = np.transpose(float_img, (2, 0, 1))  
    return float_img, np.array([ h,w,ratio_h, ratio_w])

def preprocess_img(img):
    scale = 1/255.
    mean = [0.485, 0.456, 0.406]
    std = [0.229, 0.224, 0.225]

    src_h, src_w, _ = img.shape
    if sum([src_h, src_w]) < 64:
        img = image_padding(img)

    ratio =1
    h, w, c = img.shape
    resize_h = int(h * ratio)
    resize_w = int(w * ratio)

    resize_h = max(int(round(resize_h / 32) * 32), 32)
    resize_w = max(int(round(resize_w / 32) * 32), 32)


    if int(resize_w) <= 0 or int(resize_h) <= 0:
        return None, (None, None)
    img = cv2.resize(img, (int(resize_w), int(resize_h)))

    ratio_h = resize_h / float(h)
    ratio_w = resize_w / float(w)
    float_img = img.astype('float32') 

    float_img = (float_img* scale - mean) / std
    float_img = np.transpose(float_img, (2, 0, 1))  
    return float_img, np.array([ h,w,ratio_h, ratio_w])

class TextDetector(object):
    def __init__(self,model_dir ):
        self.postprocess_op = db_postprocess_pt.DBPostProcess(thresh=0.3,box_thresh=0.4,max_candidates=1000,
                                                              unclip_ratio=2.0,score_mode='fast',box_type='quad')
        self.predictor, self.input_tensor, self.output_tensors, self.config = create_predictor(model_dir)

    def order_points_clockwise(self, pts):
        rect = np.zeros((4, 2), dtype="float32")
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)]
        rect[2] = pts[np.argmax(s)]
        tmp = np.delete(pts, (np.argmin(s), np.argmax(s)), axis=0)
        diff = np.diff(np.array(tmp), axis=1)
        rect[1] = tmp[np.argmin(diff)]
        rect[3] = tmp[np.argmax(diff)]
        return rect

    def clip_det_res(self, points, img_height, img_width):
        for pno in range(points.shape[0]):
            points[pno, 0] = int(min(max(points[pno, 0], 0), img_width - 1))
            points[pno, 1] = int(min(max(points[pno, 1], 0), img_height - 1))
        return points

    def filter_tag_det_res(self, dt_boxes, image_shape):
        img_height, img_width = image_shape[0:2]
        dt_boxes_new = []
        for box in dt_boxes:
            if type(box) is list:
                box = np.array(box)
            box = self.order_points_clockwise(box)
            box = self.clip_det_res(box, img_height, img_width)
            rect_width = int(np.linalg.norm(box[0] - box[1]))
            rect_height = int(np.linalg.norm(box[0] - box[3]))
            if rect_width <= 3 or rect_height <= 3:
                continue
            dt_boxes_new.append(box)
        dt_boxes = np.array(dt_boxes_new)
        return dt_boxes

    def __call__(self, ori_img,t_rgb=True):
        if ori_img is None:
            return None, 0
        if False:
            img = cv2.cvtColor(ori_img, cv2.COLOR_BGR2RGB)  #[:, :, ::-1] 同理
        else:
            img=ori_img
        st = time.time()
        img, shape_list = preprocess_img(img)
        img = np.expand_dims(img, axis=0).astype(np.float32)
        shape_list = np.expand_dims(shape_list, axis=0)
        input_dict = {}
        input_dict[self.input_tensor.name] = img
        outputs = self.predictor.run(self.output_tensors, input_dict)

        preds = {}
        preds['maps'] = outputs[0]
        post_result = self.postprocess_op(preds, shape_list)
        dt_boxes = post_result[0]['points']
        dt_boxes = self.filter_tag_det_res(dt_boxes, ori_img.shape)
        et = time.time()
        return dt_boxes, et - st


class TextRecognizer(object):
    def __init__(self, model_dir,rec_char_dict_path,batch_num=6):
        self.rec_image_shape =[3,48,320]
        self.rec_char_dict_path = rec_char_dict_path
        self.use_space_char =True
        self.rec_batch_num =batch_num
        self.postprocess_op = rec_postprocess_pt.BaseRecLabelDecode(self.rec_char_dict_path,self.use_space_char)
        self.predictor, self.input_tensor, self.output_tensors, self.config = create_predictor(model_dir)
    def resize_norm_img(self, img, max_wh_ratio):
        imgC, imgH, imgW = self.rec_image_shape
        imgW = int((imgH * max_wh_ratio))
        # w = self.input_tensor.shape[3:][0]
        h, w = img.shape[:2]
        ratio = w / float(h)
        if math.ceil(imgH * ratio) > imgW:
            resized_w = imgW
        else:
            resized_w = int(math.ceil(imgH * ratio))
      
        resized_image = cv2.resize(img, (resized_w, imgH))
        resized_image = resized_image.astype('float32')
        resized_image = resized_image.transpose((2, 0, 1)) / 255
        resized_image -= 0.5
        resized_image /= 0.5
        padding_im = np.zeros((imgC, imgH, imgW), dtype=np.float32)
        padding_im[:, :, 0:resized_w] = resized_image
        return padding_im
    
    def __call__(self, img_list):
        img_num = len(img_list)
        # Calculate the aspect ratio of all text bars
        width_list = []
        for img in img_list:
            width_list.append(img.shape[1] / float(img.shape[0]))
        # Sorting can speed up the recognition process
        indices = np.argsort(np.array(width_list))
        rec_res = [['', 0.0]] * img_num
        batch_num = self.rec_batch_num
        st = time.time()
        for beg_img_no in range(0, img_num, batch_num):
            end_img_no = min(img_num, beg_img_no + batch_num)
            norm_img_batch = []
            imgC, imgH, imgW = self.rec_image_shape[:3]
            max_wh_ratio = imgW / imgH
            for ino in range(beg_img_no, end_img_no):
                h, w = img_list[indices[ino]].shape[0:2]
                wh_ratio = w * 1.0 / h
                max_wh_ratio = max(max_wh_ratio, wh_ratio)
            for ino in range(beg_img_no, end_img_no):
                norm_img = self.resize_norm_img(img_list[indices[ino]],
                                                    max_wh_ratio)
                norm_img = norm_img[np.newaxis, :]
                norm_img_batch.append(norm_img)
            norm_img_batch = np.concatenate(norm_img_batch)
            norm_img_batch = norm_img_batch.copy()
            infer_st = time.time()
            input_dict = {}
            input_dict[self.input_tensor.name] = norm_img_batch
            outputs = self.predictor.run(self.output_tensors,
                                            input_dict)
            preds = outputs[0]
            infer_et = time.time()
            print("rec cost time %f ms"%((infer_et-infer_st)*1000))
            rec_result = self.postprocess_op(preds)
            for rno in range(len(rec_result)):
                rec_res[indices[beg_img_no + rno]] = rec_result[rno]
        return rec_res, time.time() - st



class TextClassifier(object):
    def __init__(self, model_dir):
        self.cls_image_shape = [3,48,192]
        self.cls_batch_num = 6
        self.cls_thresh = 0.9
        label_list=['0','180']
        self.postprocess_op = utility_pt.ClsPostProcess(label_list)
        self.predictor, self.input_tensor, self.output_tensors, _ = create_predictor(model_dir)

    def resize_norm_img(self, img):
        imgC, imgH, imgW = self.cls_image_shape
        h = img.shape[0]
        w = img.shape[1]
        ratio = w / float(h)
        if math.ceil(imgH * ratio) > imgW:
            resized_w = imgW
        else:
            resized_w = int(math.ceil(imgH * ratio))
        resized_image = cv2.resize(img, (resized_w, imgH))
        resized_image = resized_image.astype('float32')
        if self.cls_image_shape[0] == 1:
            resized_image = resized_image / 255
            resized_image = resized_image[np.newaxis, :]
        else:
            resized_image = resized_image.transpose((2, 0, 1)) / 255
        resized_image -= 0.5
        resized_image /= 0.5
        padding_im = np.zeros((imgC, imgH, imgW), dtype=np.float32)
        padding_im[:, :, 0:resized_w] = resized_image
        return padding_im
    def __call__(self, img_list):
        img_list = copy.deepcopy(img_list)
        img_num = len(img_list)
        # Calculate the aspect ratio of all text bars
        width_list = []
        for img in img_list:
            width_list.append(img.shape[1] / float(img.shape[0]))
        # Sorting can speed up the cls process
        indices = np.argsort(np.array(width_list))

        cls_res = [['', 0.0]] * img_num
        batch_num = self.cls_batch_num
        elapse = 0
        for beg_img_no in range(0, img_num, batch_num):
            end_img_no = min(img_num, beg_img_no + batch_num)
            norm_img_batch = []
            max_wh_ratio = 0
            starttime = time.time()
            for ino in range(beg_img_no, end_img_no):
                h, w = img_list[indices[ino]].shape[0:2]
                wh_ratio = w * 1.0 / h
                max_wh_ratio = max(max_wh_ratio, wh_ratio)
            for ino in range(beg_img_no, end_img_no):
                norm_img = self.resize_norm_img(img_list[indices[ino]])
                norm_img = norm_img[np.newaxis, :]
                norm_img_batch.append(norm_img)
            norm_img_batch = np.concatenate(norm_img_batch)
            norm_img_batch = norm_img_batch.copy()
            input_dict = {}
            input_dict[self.input_tensor.name] = norm_img_batch
            outputs = self.predictor.run(self.output_tensors, input_dict)
            prob_out = outputs[0]
            cls_result = self.postprocess_op(prob_out)
            elapse += time.time() - starttime
            for rno in range(len(cls_result)):
                label, score = cls_result[rno]
                cls_res[indices[beg_img_no + rno]] = [label, score]
                if '180' in label and score > self.cls_thresh:
                    img_list[indices[beg_img_no + rno]] = cv2.rotate(
                        img_list[indices[beg_img_no + rno]], 1)
        return img_list, cls_res, elapse


def sorted_boxes(dt_boxes):
    """
    Sort text boxes in order from top to bottom, left to right
    args:
        dt_boxes(array):detected text boxes with shape [4, 2]
    return:
        sorted boxes(array) with shape [4, 2]
    """
    num_boxes = dt_boxes.shape[0]
    sorted_boxes = sorted(dt_boxes, key=lambda x: (x[0][1], x[0][0]))
    _boxes = list(sorted_boxes)

    for i in range(num_boxes - 1):
        for j in range(i, -1, -1):
            if abs(_boxes[j + 1][0][1] - _boxes[j][0][1]) < 10 and \
                    (_boxes[j + 1][0][0] < _boxes[j][0][0]):
                tmp = _boxes[j]
                _boxes[j] = _boxes[j + 1]
                _boxes[j + 1] = tmp
            else:
                break
    return _boxes


def show_img(img, dt_boxes, rec_res,drop_score):
    font_path='./config/simfang.ttf'
    image = Image.fromarray(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
    boxes = dt_boxes
    txts = [rec_res[i][0] for i in range(len(rec_res))]
    scores = [rec_res[i][1] for i in range(len(rec_res))]

    # draw_img = utility_pt.draw_ocr_box_txt(
    #     image,
    #     boxes,
    #     txts,
    #     scores,
    #     drop_score=drop_score,
    #     font_path=font_path)
    # cv2.imshow("", draw_img[:, :, ::-1])
    
    draw_img = utility_pt.draw_ocr_box_txt_on_image(
        image,
        boxes,
        txts,
        scores,
        drop_score=drop_score,
        font_path=font_path)
    
    draw_img = cv2.resize(draw_img, (360,550))
    cv2.namedWindow("", cv2.WINDOW_AUTOSIZE, size = (365,560))
    cv2.imshow("", draw_img)
  

def main(det_model_dir,rec_model_dir,cls_model_dir,image_dir,rec_char_dict_path):
    image_file_list = get_image_file_list(image_dir)
    detect = TextDetector(det_model_dir)
    text_recognizer =TextRecognizer(rec_model_dir,rec_char_dict_path)
    text_cls =TextClassifier(cls_model_dir)
    
    drop_score=0.5
    while True:
        for index , im_p in enumerate(image_file_list):
            time_dict = {'det': 0, 'rec': 0, 'cls': 0, 'all': 0}
            ori_img = cv2.imread(im_p)
            # img=cv2.resize(ori_img,(640,480))
            img = ori_img
            start = time.time()
            ori_im = img.copy()
            dt_boxes, elapse = detect(img)
            dt_boxes = sorted_boxes(dt_boxes)
            img_crop_list = []
            save_in =0
            for bno in range(len(dt_boxes)):
                tmp_box = copy.deepcopy(dt_boxes[bno])
                img_crop = utility_pt.get_rotate_crop_image(ori_im, tmp_box)
                # img_crop =  utility_pt.get_minarea_rect_crop(ori_im, tmp_box)
                img_crop_list.append(img_crop)
    
            rec_res, elapse = text_recognizer(img_crop_list)
            time_dict['rec'] = elapse
            print("rec_res num  : {}, elapsed : {}".format(len(rec_res), elapse))
            
            filter_boxes, filter_rec_res = [], []
            for box, rec_result in zip(dt_boxes, rec_res):
                text, score = rec_result
                if score >= drop_score:
                    filter_boxes.append(box)
                    filter_rec_res.append(rec_result)
            end = time.time()
            time_dict['all'] = end - start
            for rec in rec_res:
                print(rec)
            print("all cost time:",time_dict)
            print(im_p)
            show_img(img, dt_boxes, rec_res,drop_score)
        # 如果在循环完所有图片后用户没有按下'q'，则再次循环  
        if cv2.waitKey(1000) & 0xFF != ord('q'):  
            continue  
        else:  
            break


if __name__ == "__main__":
    det_model_dir="./model/mobile_ocrv4_det.onnx"
    rec_model_dir="./model/mobile_ocrv4_rec.onnx"
    cls_model_dir = "./model/model.onnx"
    image_dir="./img"
    rec_char_dict_path="./model/ppocr_keys_v1.txt"
    main(det_model_dir,rec_model_dir,cls_model_dir,image_dir,rec_char_dict_path)