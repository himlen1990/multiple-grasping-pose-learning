ITERATION_TIMES=ITERATIONS

source /opt/anaconda3/bin/activate
conda activate aero_train

cd ~/Tensorflow/demo/
bash generate_record.sh
python3 model_main-fine-tune.py --model_dir=models/my_ssd --pipeline_config_path=models/my_ssd/pipeline.config --num_train_steps=$ITERATION_TIMES
cd ~/Tensorflow/demo/models
bash convert.sh $ITERATION_TIMES
cd ~/Tensorflow/demo
cp models models_test -r
chmod 666 -R models_test
