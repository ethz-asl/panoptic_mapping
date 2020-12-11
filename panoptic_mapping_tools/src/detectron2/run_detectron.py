#!/usr/bin/env python

# import some common libraries
import numpy as np
import os, json, cv2, random, time

# Set target dir
target_dir = '/home/schmluk/panoptic_mapping/data/run1'
config_file = 'COCO-PanopticSegmentation/panoptic_fpn_R_101_3x.yaml'

# Setup
files = [
    o for o in os.listdir(target_dir)
    if os.path.isfile(os.path.join(target_dir, o)) and o.endswith('_color.png')
]
exec_times = []

# TEST
files = files[:10]

# Setup Network
cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file(config_file))
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url(config_file)
predictor = DefaultPredictor(cfg)

# Run
for im_file in files:
    im = cv2.imread(im_file)

    # Predict
    t1 = time.perf_counter()
    panoptic_seg, segments_info = predictor(im)["panoptic_seg"]
    t2 = time.perf_counter()
    exec_times.append(t2 - t1)

    # Store
    file_id = im_file[:6]
    id_img = panoptic_seg.numpy()
    cv2.imwrite(os.path.join(target_dir, file_id + "_predicted.png"))
    with open(os.path.join(target_dir, file_id + "_labels.json"),
              'w') as json_file:
        json.dump(segments_info, json_file)

# Timings
exec_times = np.array(exec_times)
print("Prediction times: %.3f +/- %.3f (min: %.3f, max: %.3f)." %
      (np.mean(exec_times), np.std(exec_times), np.min(exec_times),
       np.max(exec_times)))

#print("Seg")
#print(np.shape(panoptic_seg))
#print(panoptic_seg)
#print("Seg_info")
#print(segments_info[0])
#v = Visualizer(im[:, :, ::-1], MetadataCatalog.get(cfg.DATASETS.TRAIN[0]), scale=1.2)
#out = v.draw_panoptic_seg_predictions(panoptic_seg.to("cpu"), segments_info)
#cv2.imshow('test', out.get_image()[:, :, ::-1])
#cv2.waitKey()

if __name__ == '__main__':
    test()
