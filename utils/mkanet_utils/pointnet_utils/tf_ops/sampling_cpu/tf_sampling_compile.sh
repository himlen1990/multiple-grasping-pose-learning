TF_INC=$(python -c 'import tensorflow as tf; print(tf.sysconfig.get_include())')
TF_LIB=$(python -c 'import tensorflow as tf; print(tf.sysconfig.get_lib())')

g++ -std=c++11 tf_sampling.cpp  -o tf_sampling2_so.so -shared -fPIC -I $TF_INC/  -I $TF_INC/external/nsync/public -L$TF_LIB -ltensorflow_framework -O2 -D_GLIBCXX_USE_CXX11_ABI=0

