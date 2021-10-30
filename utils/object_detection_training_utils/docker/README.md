## How to run in dlbox

- Collect dataset from robot and task setting, including `test.zip`, `train.zip`, `label_map.pbtxt`, `pipeline.config`.
- Create your dataset compressed file `tar -czvf dataset.tar.gz label_map.pbtxt  pipeline.config  test.zip  train.zip`
- Launch training script `bash train_object_detection_dlbox.sh YOUR_PATH/dataset.tar.gz`
