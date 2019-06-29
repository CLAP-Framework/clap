import argparse
import time
from sys import platform

from models import *
from utils.datasets import *
from utils.utils import *

from skimage import io

import socket
import msgpack

def verify_image(img_file):
    try:
        img = io.imread(img_file)
    except:
        return False
    return img
    # return True

def detect(
        cfg,
        data_cfg,
        weights,
        images,
        output='carla_out',  # output folder
        img_size=416,
        conf_thres=0.5,
        nms_thres=0.5,
        save_txt=True,
        save_images=True,
        webcam=False
):
    device = torch_utils.select_device()
    if os.path.exists(output):
        shutil.rmtree(output)  # delete output folder
    os.makedirs(output)  # make new output folder

    # Initialize model
    model = Darknet(cfg, img_size)

    # Load weights
    if weights.endswith('.pt'):  # pytorch format
        model.load_state_dict(torch.load(weights, map_location=device)['model'])
    else:  # darknet format
        _ = load_darknet_weights(model, weights)

    model.to(device).eval()

    # Get classes and colors
    classes = load_classes(parse_data_cfg(data_cfg)['names'])
    colors = [[random.randint(0, 255) for _ in range(3)] for _ in range(len(classes))]

    # UDP
    _publisher = socket.socket(socket.AF_INET, # Internet
            socket.SOCK_DGRAM) # UDP

    frame_id = 0

    while True:
        path_to_img = 'carla_in/frame.jpg'
        # im0 = cv2.imread(path_to_img)
        im0 = verify_image(path_to_img)
        if im0 is None or im0 is False or im0.shape != (480, 640, 3):
            # print("skipping")
            time.sleep(.01)
            continue
        # else:
        #     print("reading")
        # assert im0 is not None, 'File Not Found ' + path_to_img
        try: 
            os.remove(path_to_img)
        except: pass

        # print(im0.shape)
        # print(im0.shape == (480, 640, 3))
        frame_id += 1
        im0 = im0[:, :, ::-1].astype(np.uint8).copy() 
        img, _, _, _ = letterbox(im0, height = img_size)

        # Normalize RGB
        # img = img[:, :, ::-1].transpose(2, 0, 1)  # RGB, channel first
        img = img[:, :, ::-1].transpose(2, 0, 1)  # BGR to RGB, channel first
        img = np.ascontiguousarray(img, dtype=np.float32)  # uint8 to float32
        img /= 255.0  # 0 - 255 to 0.0 - 1.0

        # print(img.shape)
        # cv2.imshow("original image", img0)
        # cv2.waitKey(0)

        path = 'frame%d.jpg'%(frame_id)

        t = time.time()
        save_path = str(Path(output) / Path(path).name)

        # Get detections
        img = torch.from_numpy(img).unsqueeze(0).to(device)
        if ONNX_EXPORT:
            torch.onnx.export(model, img, 'weights/model.onnx', verbose=True)
            return
        pred, _ = model(img)
        detections = non_max_suppression(pred, conf_thres, nms_thres)[0]

        print('Frame %d:' % (frame_id), end=' ')

        send_info = []
        if detections is not None and len(detections) > 0:
            # Rescale boxes from 416 to true image size
            scale_coords(img_size, detections[:, :4], im0.shape).round()

            # Print results to screen
            for c in detections[:, -1].unique():
                n = (detections[:, -1] == c).sum()
                print('%g %ss' % (n, classes[int(c)]), end=', ')

            # Draw bounding boxes and labels of detections
            # pos_list = [None]*detections.shape[0]
            i = 0
            for *xyxy, conf, cls_conf, cls in detections:
                if save_txt:  # Write to file
                    with open(save_path + '.txt', 'a') as file:
                        file.write(('%g ' * 6 + '\n') % (*xyxy, cls, conf))

                # Add bbox to the image
                label = '%s %.2f' % (classes[int(cls)], conf)
                pos_list = plot_one_box(xyxy, im0, label=label, color=colors[int(cls)]) # None or [lat, long]
                send_info.append((float(pos_list[0]),float(pos_list[1]), float(0) ) )
                i += 1
        _publisher.sendto(msgpack.packb(send_info), ("127.0.0.1", 6661))

        # print('(%.3fs)' % (time.time() - t))
        print(' ')
        cv2.imshow('output', im0)
        cv2.waitKey(1)
        # imgplot = plt.imshow(im0[:,:,::-1])
        # plt.pause(0.01)
        
        if save_images:  # Save generated image with detections
            cv2.imwrite(save_path, im0)

    if save_images and platform == 'darwin':  # macos
        os.system('open ' + output + ' ' + save_path)


if __name__ == '__main__':
    parser = argparse.ArgumentParser()
    parser.add_argument('--cfg', type=str, default='cfg/yolov3-spp.cfg', help='cfg file path')
    parser.add_argument('--data-cfg', type=str, default='data/coco.data', help='coco.data file path')
    parser.add_argument('--weights', type=str, default='weights/yolov3-spp.weights', help='path to weights file')
    parser.add_argument('--images', type=str, default='data/samples', help='path to images')
    parser.add_argument('--img-size', type=int, default=416, help='size of each image dimension')
    parser.add_argument('--conf-thres', type=float, default=0.5, help='object confidence threshold')
    parser.add_argument('--nms-thres', type=float, default=0.5, help='iou threshold for non-maximum suppression')
    opt = parser.parse_args()
    print(opt)

    with torch.no_grad():
        detect(
            opt.cfg,
            opt.data_cfg,
            opt.weights,
            opt.images,
            img_size=opt.img_size,
            conf_thres=opt.conf_thres,
            nms_thres=opt.nms_thres
        )
