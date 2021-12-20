source ~/anaconda3/bin/activate
conda activate aero_object_detection27

python data_augmentation_bbox_turntable.py
sh partition.sh

cd ../dataset/
mkdir dataset_training

cd aug_data/rgb

zip -r ../../dataset_training/test.zip test
zip -r ../../dataset_training/train.zip train

cd ../../
cp label_map.pbtxt dataset_training/
cp pipeline.config dataset_training/
