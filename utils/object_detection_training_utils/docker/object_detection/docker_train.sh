source /opt/anaconda3/bin/activate
conda activate aero_train

cd ~/Tensorflow/demo/
bash generate_record.sh
bash train.sh
