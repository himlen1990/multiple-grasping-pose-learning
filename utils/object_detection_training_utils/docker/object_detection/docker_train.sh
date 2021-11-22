source /opt/anaconda3/bin/activate
conda activate aero_train

cd ~/Tensorflow/demo/
bash generate_record.sh
python3 model_main-fine-tune.py --model_dir=models/my_ssd --pipeline_config_path=models/my_ssd/pipeline.config --num_train_steps=2000
cd ~/Tensorflow/demo/models
bash convert.sh
cp ~/Tensorflow/demo/models ~/Tensorflow/demo/models_test -r
