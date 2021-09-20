python3 ./export_tflite_ssd_graph.py \
  --pipeline_config_path="my_ssd/pipeline.config" \
  --trained_checkpoint_prefix="my_ssd/model.ckpt-100000" \
  --output_directory="./" \


tflite_convert \
  --output_file="./my_object_detection.tflite" \
  --graph_def_file="./tflite_graph.pb" \
  --inference_type=QUANTIZED_UINT8 \
  --input_arrays="normalized_input_image_tensor" \
  --output_arrays='TFLite_Detection_PostProcess','TFLite_Detection_PostProcess:1','TFLite_Detection_PostProcess:2','TFLite_Detection_PostProcess:3' \
  --mean_values=128 \
  --std_dev_values=128 \
  --input_shapes=1,300,300,3 \
  --change_concat_input_ranges=false \
  --allow_nudging_weights_to_use_fast_gemm_kernel=true \
  --allow_custom_ops

edgetpu_compiler my_object_detection.tflite
rm -rf tflite_graph.pb*
rm -rf my_object_detection_edgetpu.log
rm -rf my_object_detection.tflite
mv my_object_detection_edgetpu.tflite my_object_detection.tflite
