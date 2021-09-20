python3 generate_tfrecord.py -x ./images/train -l ./annotations/label_map.pbtxt -o ./annotations/train.record

python3 generate_tfrecord.py -x ./images/test -l ./annotations/label_map.pbtxt -o ./annotations/test.record
