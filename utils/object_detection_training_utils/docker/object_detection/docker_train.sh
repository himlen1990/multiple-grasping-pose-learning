ITERATION_TIMES=$1

source /opt/anaconda3/bin/activate
conda activate aero_train

cd ~/Tensorflow/demo/
bash generate_record.sh
python3 model_main-fine-tune.py --model_dir=models/my_ssd --pipeline_config_path=models/my_ssd/pipeline.config --num_train_steps=$ITERATION_TIMES
cd ~/Tensorflow/demo/models
sed -i 's/2000/$ITERATION_TIMES/g' convert.sh
bash convert.sh
cd ~/Tensorflow/demo
cp models models_test -r
